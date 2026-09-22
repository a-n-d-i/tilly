/*
 * Primers
 *
 * MAVLink writes are now length-checked (sendMavlink()) and ARM/mode-change
 * commands wait for COMMAND_ACK before pilotMode/the display believe they
 * took effect (see PendingModeCmd) - other commands (RC override, yaw
 * setpoints, stream-rate requests) are still fire-and-forget.
 * It also has no notion of multitasking or async. The slow display blocks everything.
 * Why am I doing this in arduino again?
 */

#define TILLY_DISPLAY
// Comment any of these out to compile that subsystem out entirely - handy
// for isolating a bug to one of them (each is otherwise independent: they
// only share ArduPilotSerial and, for web/opencpn, WiFi).
#define TILLY_NMEA_BRIDGE
#define TILLY_OPENCPN_BRIDGE
#define TILLY_WEB_TELEMETRY

#include "display.h"
#ifdef TILLY_NMEA_BRIDGE
#include "mavlink_nmea_bridge.h"
#endif
#ifdef TILLY_OPENCPN_BRIDGE
#include "opencpn_bridge.h"
#endif
#ifdef TILLY_WEB_TELEMETRY
#include "web_telemetry.h"
#endif
#include "board_pins.h"
#include "ButtonBox.h"
#include "applog.h"

#include <WiFi.h>
#include <WiFiUdp.h>
#include <MAVLink_ardupilotmega.h>
// Include MAVLink library - using ardupilotmega dialect for full ArduPilot support
// https://github.com/okalachev/mavlink-arduino

#include <ArduinoOTA.h>  // For enabling over the air updates
#include <SPIFFS.h>

// Include WiFi configuration from external file
#include "config.h"

WiFiUDP udp;

// ===== Hardware Serial Configuration =====
#define SERIAL_RX MAVLINK_RX_PIN
#define SERIAL_TX MAVLINK_TX_PIN
HardwareSerial ArduPilotSerial(2);  // Use UART2
#define NMEA_UDP_PORT 10110
#define OPENCPN_AP_UDP_PORT 10111

// ===== Button Configuration =====
// Button numbering (silkscreen): 1 Auto, 2 Standby, 3 -1, 4 +1, 5 -10, 6 +10
ButtonBox buttonBox;

// Broadcast Adress gets calculated automatically
IPAddress remoteIP;   // udp broadcast
uint16_t remotePort = 14550;        // destination port

// Display update timer
unsigned long lastDisplayUpdate = 0;
const unsigned long displayUpdateInterval = 500;

unsigned long lastMavlinkUpdate = 0;
const unsigned long mavlinkUpdateInterval = 1000;

unsigned long lastHeartbeatMs = 0;
const unsigned long heartbeatIntervalMs = 1000;  // standard 1 Hz GCS heartbeat

/* 
 *  In the RC world steering is value of
 *  1000-2000 with 1500 being midhsip. 
 *  Lets say a small incerement is 3mm, a large one is 10mm. The ram is 250mm atm, so 83 small increments or 25 big increments.
 *  So each small increment is 12 and each big increment is 40.
*/

const unsigned int small_increment = 12;
const unsigned int large_increment = 40;



unsigned int standby_ram_position = 1500;

// Variables for display data
int current_heading = 0;
int desired_heading = 0;

// NMEA: GUIDED steering like AUTO, but desired_heading is driven by the
// OpenCPN APB bridge instead of the +1/+10/-1/-10 buttons. Entered/left via
// the Auto+Standby combo (see kNmeaToggleComboMask) instead of a dedicated
// button, since there are only 6.
enum pilotModeType {STANDBY, AUTO, NMEA};

pilotModeType pilotMode = STANDBY;

// True only while NMEA mode is actually active - opencpn_bridge.cpp checks
// this before applying an APB-derived heading, so APB sentences are still
// parsed/logged but don't steer anything unless NMEA mode is on.
bool nmeaModeActive() { return pilotMode == NMEA; }

// pilotMode/the display only flip once ArduPilot's COMMAND_ACK confirms the
// mode change - entering GUIDED requires a good EKF position estimate
// whenever the vehicle is already armed (see Rover's Mode::enter()), so a
// mode request can be silently rejected and we don't want to lie about it.
enum class PendingModeCmd { NONE, GUIDED_AUTO, GUIDED_NMEA, MANUAL };
PendingModeCmd pendingModeCmd = PendingModeCmd::NONE;
uint32_t pendingModeCmdSentMs = 0;
const uint32_t commandAckTimeoutMs = 2000;

#ifdef TILLY_DISPLAY
TillyDisplayState displayState;
bool showLogScreen = false;  // toggled by holding buttons 5+6 together
#endif

bool rc_override_active = false;

bool wifi = false;


static void computeBroadcastAddress() {
  IPAddress ip = WiFi.localIP();
  IPAddress mask = WiFi.subnetMask();
  IPAddress bcast;
  for (int i = 0; i < 4; i++) bcast[i] = ip[i] | (~mask[i] & 0xFF);
  remoteIP = bcast;
  appLog("UDP broadcast target: %s:%d", remoteIP.toString().c_str(), NMEA_UDP_PORT);
}

// Serializes and writes a MAVLink message, logging (instead of silently
// dropping) if the UART couldn't take the full packet.
static bool sendMavlink(Stream &serial, const mavlink_message_t &msg) {
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  size_t written = serial.write(buf, len);
  if (written != len) {
    appLog("MAVLink write short: %u/%u bytes (msgid %u)", (unsigned)written, (unsigned)len, (unsigned)msg.msgid);
    return false;
  }
  return true;
}

// setup()-only: logs a line and immediately redraws the fullscreen log view,
// so boot progress is visible live on the display before the normal
// telemetry screen takes over. Don't use this outside setup() - it would
// yank whoever's looking at the normal screen or the PID/heading readout
// into the log view on every call. (TILLY_DISPLAY is always defined at the
// top of this file - this project has no non-display build.)
#define BOOT_LOG(...) do { appLog(__VA_ARGS__); updateTillyLogScreen(); } while (0)

void setup() {
  
  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH) {
        type = "sketch";
      } else {  // U_SPIFFS
        type = "filesystem";
        SPIFFS.end(); // unmount so the OTA write isn't racing our own mount
      }

      appLog("OTA: start updating %s", type.c_str());
    })
    .onEnd([]() {
      appLog("OTA: update finished");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      const char *reason = "Unknown";
      if (error == OTA_AUTH_ERROR) reason = "Auth Failed";
      else if (error == OTA_BEGIN_ERROR) reason = "Begin Failed";
      else if (error == OTA_CONNECT_ERROR) reason = "Connect Failed";
      else if (error == OTA_RECEIVE_ERROR) reason = "Receive Failed";
      else if (error == OTA_END_ERROR) reason = "End Failed";
      appLog("OTA error [%u]: %s", error, reason);
    });

  ArduinoOTA.setHostname("tilly-buttonbox");

  Serial.begin(115200);

  #ifdef TILLY_DISPLAY
  initTillyDisplay();
  #endif

  BOOT_LOG("Tillys little helper");

  // Initialize ArduPilot Serial
  ArduPilotSerial.begin(115200, SERIAL_8N1, SERIAL_RX, SERIAL_TX);
  BOOT_LOG("ArduPilot Serial initialized");

  buttonBox.begin();
  BOOT_LOG("Buttons initialized");


  // Connect to WiFi
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  BOOT_LOG("Connecting to WiFi...");

  int wifiTimeout = millis() + 30000;

  while ((WiFi.status() != WL_CONNECTED) and (millis() < wifiTimeout)){
    delay(500);
    //ESP.restart();
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    wifi = true;
    Serial.println();
    BOOT_LOG("WiFi connected, IP %s", WiFi.localIP().toString().c_str());
    computeBroadcastAddress();
    #ifdef TILLY_DISPLAY
    updateTillyLogScreen();
    #endif

    ArduinoOTA.begin();  // Starts OTA

    // Start UDP
    udp.begin(udpPort);
    BOOT_LOG("UDP listening on port %d", udpPort);
  } else {
    Serial.println();
    BOOT_LOG("WiFi connect timed out");
  }

  #ifdef TILLY_NMEA_BRIDGE
  mavNmeaBridge_setup(ArduPilotSerial, udp, remoteIP, NMEA_UDP_PORT);
  #ifdef TILLY_DISPLAY
  updateTillyLogScreen();
  #endif
  #endif

  if (wifi == true) {
    #ifdef TILLY_OPENCPN_BRIDGE
    opencpnBridge_setup(OPENCPN_AP_UDP_PORT);
    #endif
    #ifdef TILLY_WEB_TELEMETRY
    webTelemetry_setup(ArduPilotSerial);
    #endif
    #ifdef TILLY_DISPLAY
    updateTillyLogScreen();
    #endif
  }

  sendHeartbeat();
  lastHeartbeatMs = millis();

  requestMessageStream(MAVLINK_MSG_ID_GLOBAL_POSITION_INT);
  requestMessageStream(MAVLINK_MSG_ID_GPS_RAW_INT);
  requestMessageStream(MAVLINK_MSG_ID_SYSTEM_TIME);
  requestMessageStream(MAVLINK_MSG_ID_VFR_HUD);
  requestMessageStream(MAVLINK_MSG_ID_HEARTBEAT);
  requestMessageStream(MAVLINK_MSG_ID_ATTITUDE);
  requestMessageStream(MAVLINK_MSG_ID_PID_TUNING);
  requestMessageStream(MAVLINK_MSG_ID_SERVO_OUTPUT_RAW);


  sendArmCommand();

  #ifdef TILLY_DISPLAY
  updateTillyLogScreen();
  delay(1000);  // let the last boot lines stay readable for a moment
  displayState.wifiOk = wifi;
  updateTillyDisplay(displayState);  // hand off to the normal view
  #endif
}

// Sends a custom "event" as a STATUSTEXT. Text field is max 50 chars (MAVLink2).
void sendCustomEvent(const char* text, uint8_t severity = MAV_SEVERITY_NOTICE) {
  mavlink_message_t msg;

  mavlink_msg_statustext_pack(
    250, 1, &msg,
    severity,
    text,
    0,   // id (MAVLink2 chunking id, 0 = not chunked)
    0    // chunk_seq
  );

  sendMavlink(ArduPilotSerial, msg);
}



// Applies a +/-1 or +/-10 step: to standby_ram_position in STANDBY, to
// desired_heading in AUTO.
static void applyStep(int delta) {
  if (pilotMode == STANDBY) {
    standby_ram_position = constrain((int)standby_ram_position + delta, 1000, 2000);
    sendRcOverride(standby_ram_position);
  } else {
    desired_heading = ((desired_heading + delta) % 360 + 360) % 360;
    char buf[64];
    snprintf(buf, sizeof(buf), "Course %+d deg, Heading %d", delta, desired_heading);
    sendCustomEvent(buf);
  }
}

// Buttons 5+6 (-10 / +10) held together toggle the on-screen log view.
static const uint8_t kLogScreenComboMask = (1 << 4) | (1 << 5);
// Buttons 1+2 (Auto/Standby) held together toggle NMEA mode.
static const uint8_t kNmeaToggleComboMask = (1 << 0) | (1 << 1);
// AUTO mode only: 4+6 (+1/+10) held together = +100 deg, 3+5 (-1/-10) = -100 deg.
static const uint8_t kPlus100ComboMask = (1 << 3) | (1 << 5);
static const uint8_t kMinus100ComboMask = (1 << 2) | (1 << 4);

void handleButtons(){
  buttonBox.update();

  ButtonBox::Event ev;
  while (buttonBox.popEvent(ev)) {
    if (ev.type == ButtonBox::EventType::ComboStart) {
      #ifdef TILLY_DISPLAY
      if (ev.mask == kLogScreenComboMask) {
        showLogScreen = !showLogScreen;
        appLog(showLogScreen ? "Log screen on" : "Log screen off");
        continue;
      }
      #endif

      if (ev.mask == kNmeaToggleComboMask && pendingModeCmd == PendingModeCmd::NONE) {
        if (pilotMode == NMEA) {
          appLog("NMEA off: requesting MANUAL mode");
          setManualMode();
          pendingModeCmd = PendingModeCmd::MANUAL;
        } else {
          appLog("NMEA on: requesting GUIDED mode");
          setGuidedMode();
          sendArmCommand();
          pendingModeCmd = PendingModeCmd::GUIDED_NMEA;
        }
        pendingModeCmdSentMs = millis();
        continue;
      }

      if (ev.mask == kPlus100ComboMask && pilotMode == AUTO) {
        applyStep(100);
        continue;
      }

      if (ev.mask == kMinus100ComboMask && pilotMode == AUTO) {
        applyStep(-100);
        continue;
      }

      continue;
    }

    if (ev.type != ButtonBox::EventType::Click) continue;

    switch (ev.mask) {
      case 1 << 0:  // button 1: Auto
        if (pilotMode == STANDBY && pendingModeCmd == PendingModeCmd::NONE) {
          appLog("Auto: requesting GUIDED mode");
          setGuidedMode();
          // TODO: don't do this every time?
          sendArmCommand();
          pendingModeCmd = PendingModeCmd::GUIDED_AUTO;
          pendingModeCmdSentMs = millis();
        }
        break;

      case 1 << 1:  // button 2: Standby
        if ((pilotMode == AUTO || pilotMode == NMEA) && pendingModeCmd == PendingModeCmd::NONE) {
          appLog("Standby: requesting MANUAL mode");
          setManualMode();
          pendingModeCmd = PendingModeCmd::MANUAL;
          pendingModeCmdSentMs = millis();
        }
        break;

      case 1 << 2:  // button 3: -1
        applyStep(-1 * (int)((pilotMode == STANDBY) ? small_increment : 1));
        break;

      case 1 << 3:  // button 4: +1
        applyStep((int)((pilotMode == STANDBY) ? small_increment : 1));
        break;

      case 1 << 4:  // button 5: -10
        applyStep(-1 * (int)((pilotMode == STANDBY) ? large_increment : 10));
        break;

      case 1 << 5:  // button 6: +10
        applyStep((int)((pilotMode == STANDBY) ? large_increment : 10));
        break;

      default:
        break;
    }
  }
}

void loop() {
  if (wifi == true) ArduinoOTA.handle(); 
  handleButtons();
  
  // ---------------------------
  // Serial → UDP
  // ---------------------------
  mavlink_message_t msg;
  mavlink_status_t status;

  while (ArduPilotSerial.available() > 0) {
        uint8_t c = ArduPilotSerial.read();

        // Add charactar to message and try to parse / loop on until it parses/is complete
        if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
          // send to UDP
           
           // Build the raw MAVLink packet for forwarding
            uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
            uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
            
            if (msg.msgid == MAVLINK_MSG_ID_VFR_HUD) {
                mavlink_vfr_hud_t hud;
                mavlink_msg_vfr_hud_decode(&msg, &hud);

                current_heading = hud.heading;   // heading in degrees (0–360)
                #ifdef TILLY_DISPLAY
                displayState.curHeading = current_heading;
                displayState.speedKn = hud.groundspeed * 1.94384f;
                #endif
            }

            #ifdef TILLY_DISPLAY
            if (msg.msgid == MAVLINK_MSG_ID_SYS_STATUS) {
                mavlink_sys_status_t sys;
                mavlink_msg_sys_status_decode(&msg, &sys);
                displayState.magOk = (sys.onboard_control_sensors_health & MAV_SYS_STATUS_SENSOR_3D_MAG) != 0;
                if (sys.voltage_battery != UINT16_MAX) {
                  displayState.batteryVolts = sys.voltage_battery / 1000.0f;
                }
                if (sys.current_battery != -1) {
                  displayState.batteryAmps = sys.current_battery / 100.0f;
                }
            }

            // sysid 1 only - a GCS sharing this link (e.g. MAVProxy) sends its
            // own heartbeat too, with base_mode never carrying the armed bit.
            if (msg.msgid == MAVLINK_MSG_ID_HEARTBEAT && msg.sysid == 1) {
                mavlink_heartbeat_t hb;
                mavlink_msg_heartbeat_decode(&msg, &hb);
                displayState.armed = (hb.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) != 0;
            }
            #endif

            // target_system 250 matches the sysid we send commands under
            // (see sendModeCommand()/sendArmCommand()) - ignore acks aimed
            // at other senders sharing this link (e.g. MAVProxy).
            if (msg.msgid == MAVLINK_MSG_ID_COMMAND_ACK && msg.sysid == 1) {
                mavlink_command_ack_t ack;
                mavlink_msg_command_ack_decode(&msg, &ack);
                if (ack.target_system == 250) {
                  if (ack.command == MAV_CMD_DO_SET_MODE && pendingModeCmd != PendingModeCmd::NONE) {
                    if (ack.result == MAV_RESULT_ACCEPTED) {
                      if (pendingModeCmd == PendingModeCmd::GUIDED_AUTO) {
                        pilotMode = AUTO;
                        desired_heading = current_heading;
                        appLog("Auto: GUIDED mode confirmed");
                      } else if (pendingModeCmd == PendingModeCmd::GUIDED_NMEA) {
                        pilotMode = NMEA;
                        desired_heading = current_heading;
                        appLog("NMEA: GUIDED mode confirmed");
                      } else {
                        pilotMode = STANDBY;
                        standby_ram_position = 1500;
                        appLog("Standby: MANUAL mode confirmed");
                      }
                    } else {
                      appLog("Mode change rejected: MAV_RESULT %u", ack.result);
                    }
                    pendingModeCmd = PendingModeCmd::NONE;
                  } else if (ack.command == MAV_CMD_COMPONENT_ARM_DISARM) {
                    if (ack.result == MAV_RESULT_ACCEPTED) {
                      appLog("Arm/disarm confirmed");
                    } else {
                      appLog("Arm/disarm rejected: MAV_RESULT %u", ack.result);
                    }
                  }
                }
            }

            #ifdef TILLY_DISPLAY

            if (msg.msgid == MAVLINK_MSG_ID_GLOBAL_POSITION_INT) {
                mavlink_global_position_int_t pos;
                mavlink_msg_global_position_int_decode(&msg, &pos);
                displayState.lat = pos.lat / 1e7;
                displayState.lon = pos.lon / 1e7;
            }

            if (msg.msgid == MAVLINK_MSG_ID_PID_TUNING) {
                mavlink_pid_tuning_t pid;
                mavlink_msg_pid_tuning_decode(&msg, &pid);
                if (pid.axis == PID_TUNING_STEER) {
                  displayState.pidFF = pid.FF;
                  displayState.pidP = pid.P;
                  displayState.pidI = pid.I;
                  displayState.pidD = pid.D;
                  displayState.pidSRate = pid.SRate;
                }
            }
            #endif

          if (msg.msgid == MAVLINK_MSG_ID_EKF_STATUS_REPORT) {
            mavlink_ekf_status_report_t ekf_status;
            mavlink_msg_ekf_status_report_decode(&msg, &ekf_status);
            #ifdef TILLY_DISPLAY
            displayState.ekfOk = (ekf_status.flags & EKF_ATTITUDE) != 0;
            #endif
          }

          // this is raw gps data, non fused. maybe change?
          if (msg.msgid == MAVLINK_MSG_ID_GPS_RAW_INT) {
            mavlink_gps_raw_int_t gps_status;
            mavlink_msg_gps_raw_int_decode(&msg, &gps_status);
            #ifdef TILLY_DISPLAY
            displayState.sats = gps_status.satellites_visible;
            if (gps_status.cog != UINT16_MAX) {
              displayState.cogDeg = gps_status.cog / 100.0f;
            }
            if (gps_status.eph != UINT16_MAX) {
              displayState.hdop = gps_status.eph / 100.0f;
            }
            if (gps_status.epv != UINT16_MAX) {
              displayState.vdop = gps_status.epv / 100.0f;
            }
            #endif
          }

          #ifdef TILLY_DISPLAY
          if (msg.msgid == MAVLINK_MSG_ID_SYSTEM_TIME) {
            mavlink_system_time_t sysTime;
            mavlink_msg_system_time_decode(&msg, &sysTime);
            if (sysTime.time_unix_usec != 0) {
              displayState.unixTimeSec = sysTime.time_unix_usec / 1000000ULL;
            }
          }
          #endif
          // update the nmea bridge
          #ifdef TILLY_NMEA_BRIDGE
          handleMavMessage(msg);
          #endif
          #ifdef TILLY_WEB_TELEMETRY
          if (wifi == true) webTelemetry_handleMavMessage(msg);
          #endif
          break;

        }

    }

   yield();

  if (pendingModeCmd != PendingModeCmd::NONE && (millis() - pendingModeCmdSentMs > commandAckTimeoutMs)) {
    appLog("Mode change: no ack received, timed out");
    pendingModeCmd = PendingModeCmd::NONE;
  }

  // GUIDED Mode needs updates at least every three seconds or it stops
  if ((pilotMode == AUTO || pilotMode == NMEA) && (millis() - lastMavlinkUpdate >  mavlinkUpdateInterval)) {
      sendYawCommandDeg(ArduPilotSerial, 1, 1, desired_heading);
      lastMavlinkUpdate = millis();
  }

  if (millis() - lastHeartbeatMs > heartbeatIntervalMs) {
      sendHeartbeat();
      lastHeartbeatMs = millis();
  }

  #ifdef TILLY_NMEA_BRIDGE
  mavNmeaBridge_update();
  #endif
  #ifdef TILLY_OPENCPN_BRIDGE
  opencpnBridge_update();
  #endif
  #ifdef TILLY_WEB_TELEMETRY
  if (wifi == true) webTelemetry_update();
  #endif

  #ifdef TILLY_DISPLAY
  if (millis() - lastDisplayUpdate > displayUpdateInterval) {
    lastDisplayUpdate = millis();
    if (showLogScreen) {
      updateTillyLogScreen();
    } else {
      displayState.autoMode = (pilotMode == AUTO);
      displayState.nmeaMode = (pilotMode == NMEA);
      displayState.desHeading = desired_heading;
      displayState.wifiOk = wifi;
      updateTillyDisplay(displayState);
    }
  }
  #endif

}


// Announces this device on the MAVLink link, same sysid/compid (250/1) used
// for every command we send. Without this the vehicle has no way to know a
// GCS-like device is present on the link (relevant to GCS failsafe, and to
// any other MAVLink-aware tool sniffing the same UART).
void sendHeartbeat() {
    mavlink_message_t msg;
    mavlink_msg_heartbeat_pack(250, 1, &msg,
        MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID, 0, 0, MAV_STATE_ACTIVE);
    sendMavlink(ArduPilotSerial, msg);
}

void sendArmCommand(){

    mavlink_message_t msg;

    // Pack the MAVLink message directly
    mavlink_msg_command_long_pack(
        250,          // system ID
        1,       // component ID
        &msg,                   // message struct
        1,          // target system
        1,       // target component
        MAV_CMD_COMPONENT_ARM_DISARM,
            0, // confirmation
            1, // param1 (0 to indicate disarm)
            0, // param2 (all other params meaningless)
            0, // param3
            0, // param4
            0, // param5
            0, // param6
            0); // param7

    appLog("Sending ARM to tilly");
    sendMavlink(ArduPilotSerial, msg);
}

void setGuidedMode() {
  sendModeCommand(15, 1);
} 

void setManualMode() {
  sendModeCommand(0, 0);
} 


void sendModeCommand(int modeNumber, int subMode){

    mavlink_message_t msg;
  
    // Pack the MAVLink message directly
    mavlink_msg_command_long_pack(
        250,          // system ID
        1,       // component ID
        &msg,                   // message struct
        1,          // target system
        1,       // target component
        MAV_CMD_DO_SET_MODE,
            0, // confirmation
            MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, // param1 (0 to indicate disarm)
            modeNumber, // 15 -> guided sailboat, 0 manual
            subMode, // param3
            0, // param4
            0, // param5
            0, // param6
            0); // param7

    appLog("Sending Mode to tilly");
    sendMavlink(ArduPilotSerial, msg);
}


void sendYawCommandDeg(Stream &serial, uint8_t target_system, uint8_t target_component, float yaw_deg) {
    mavlink_message_t msg;

    // Convert degrees to radians
    float yaw_rad = yaw_deg * DEG_TO_RAD;

    // Pack the MAVLink message directly
    mavlink_msg_set_position_target_global_int_pack(
        250,          // system ID
        1,       // component ID
        &msg,                   // message struct
        millis(),               // timestamp (ms since boot)
        1,          // target system
        1,       // target component
        MAV_FRAME_GLOBAL,       // frame - matches send_course.py's set_position_target_global_int_send()
        0b100111111111,         // type_mask: ignore position, velocity, acceleration
        0, 0, 0,                // lat, lon, alt (ignored)
        0, 0, 0,                // vx, vy, vz (ignored)
        0, 0, 0,                // ax, ay, az (ignored)
        yaw_rad,                // yaw in radians
        0                       // yaw_rate
    );

    sendMavlink(serial, msg);
}


// Asks the autopilot to stream the given message at 10 Hz via
// MAV_CMD_SET_MESSAGE_INTERVAL (modern ArduPilot).
void requestMessageStream(uint8_t message_number) {
    mavlink_message_t msg;

    mavlink_msg_command_long_pack(
        250,          // system ID
        1,            // component ID
        &msg,
        1,            // target system
        1,            // target component
        MAV_CMD_SET_MESSAGE_INTERVAL,
        0,               // confirmation
        message_number,  // param1: message ID to configure
        100000,          // param2: interval in microseconds (10 Hz)
        0, 0, 0, 0, 0);  // param3-7: unused

    sendMavlink(ArduPilotSerial, msg);
}



void sendRcOverride(uint16_t value) {

    mavlink_message_t msg;

    // Send REQUEST_DATA_STREAM
    mavlink_msg_rc_channels_override_pack(
        255,  // sender system. Has to be 255 for rc_override, anything else seems to be ignored
        0,    // sender component
        &msg,
        1,
        1,
        value,  // channel 1 (servo1)
        0,      // channel 2 (0 = no change)
        0,      // channel 3
        0,      // channel 4
        0,      // channel 5
        0,      // channel 6
        0,      // channel 7
        0,0,0,0,0,0,0,0,0,0,0       // channel 8-18
    );

    sendMavlink(ArduPilotSerial, msg);
}



void sendCompassCalibrationCommand(){

    mavlink_message_t msg;
  
    // Pack the MAVLink message directly
    mavlink_msg_command_long_pack(
        250,          // system ID
        1,       // component ID
        &msg,                   // message struct
        1,          // target system
        1,       // target component
        MAV_CMD_DO_START_MAG_CAL,
         0,   // confirmation
         1,   // param1: auto retry
         1,   // param2: auto save
         0,   // param3: delay
         1,   // param4: LARGE VEHICLE MODE
         0,   // param5: motor compensation (0 = no)
         0,   // param6 (unused)
         0    // param7 (unused)
  );

    appLog("CompassCalibration");
    sendMavlink(ArduPilotSerial, msg);
}
