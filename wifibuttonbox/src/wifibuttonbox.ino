/*
 * Primers
 * 
 * This has no error handling yet. It's all fire and forget.
 * How long do the acks take, maybe just active wait since esp stuff continus running in the background?
 * It also has no notion of multitasking or async. The slow display blocks everything.
 * Why am I doing this in arduino again?
 */

#define TILLY_DISPLAY

#include "display.h"
#include "mavlink_nmea_bridge.h"
#include "opencpn_bridge.h"
#include "web_telemetry.h"
#include "board_pins.h"
#include "ButtonBox.h"

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

enum pilotModeType {STANDBY, AUTO};

pilotModeType pilotMode = STANDBY;

#ifdef TILLY_DISPLAY
TillyDisplayState displayState;
#endif

bool rc_override_active = false;

bool wifi = false;


static void computeBroadcastAddress() {
  IPAddress ip = WiFi.localIP();
  IPAddress mask = WiFi.subnetMask();
  IPAddress bcast;
  for (int i = 0; i < 4; i++) bcast[i] = ip[i] | (~mask[i] & 0xFF);
  remoteIP = bcast;
  Serial.printf("UDP broadcast target: %s:%d\n",
                remoteIP.toString().c_str(), NMEA_UDP_PORT);
}

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

      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) {
        Serial.println("Auth Failed");
      } else if (error == OTA_BEGIN_ERROR) {
        Serial.println("Begin Failed");
      } else if (error == OTA_CONNECT_ERROR) {
        Serial.println("Connect Failed");
      } else if (error == OTA_RECEIVE_ERROR) {
        Serial.println("Receive Failed");
      } else if (error == OTA_END_ERROR) {
        Serial.println("End Failed");
      }
    });
  
  ArduinoOTA.setHostname("tilly-buttonbox");

  Serial.begin(115200);
  Serial.println("Tillys little helper");

  // Initialize ArduPilot Serial
  ArduPilotSerial.begin(115200, SERIAL_8N1, SERIAL_RX, SERIAL_TX);
  Serial.println("ArduPilot Serial initialized");

  buttonBox.begin();
  Serial.println("Buttons initialized");
  
  
  // Connect to WiFi
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");

  int wifiTimeout = millis() + 30000;
  
  while ((WiFi.status() != WL_CONNECTED) and (millis() < wifiTimeout)){
    delay(500);
    //ESP.restart();
    Serial.print(".");
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    wifi = true;
    Serial.println("\nWiFi connected!");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    computeBroadcastAddress();
  
    ArduinoOTA.begin();  // Starts OTA
    
    // Start UDP
    udp.begin(udpPort);
    Serial.printf("UDP listening on port %d\n", udpPort);
  }

  mavNmeaBridge_setup(ArduPilotSerial, udp, remoteIP, NMEA_UDP_PORT);

  if (wifi == true) {
    opencpnBridge_setup(OPENCPN_AP_UDP_PORT);
    webTelemetry_setup(ArduPilotSerial);
  }

  #ifdef TILLY_DISPLAY
  initTillyDisplay();
  Serial.println("Display initialized");
  #endif

  requestMessageStream(MAVLINK_MSG_ID_GLOBAL_POSITION_INT);
  requestMessageStream(MAVLINK_MSG_ID_GPS_RAW_INT);
  requestMessageStream(MAVLINK_MSG_ID_SYSTEM_TIME);
  requestMessageStream(MAVLINK_MSG_ID_VFR_HUD);
  requestMessageStream(MAVLINK_MSG_ID_HEARTBEAT);
  requestMessageStream(MAVLINK_MSG_ID_ATTITUDE);
  requestMessageStream(MAVLINK_MSG_ID_PID_TUNING);
  requestMessageStream(MAVLINK_MSG_ID_SERVO_OUTPUT_RAW);
  
  
  sendArmCommand();
}

// Sends a custom "event" as a STATUSTEXT. Text field is max 50 chars (MAVLink2).
void sendCustomEvent(const char* text, uint8_t severity = MAV_SEVERITY_NOTICE) {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  mavlink_msg_statustext_pack(
    250, 1, &msg,
    severity,
    text,
    0,   // id (MAVLink2 chunking id, 0 = not chunked)
    0    // chunk_seq
  );

  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
   ArduPilotSerial.write(buf, len);
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

void handleButtons(){
  buttonBox.update();

  ButtonBox::Event ev;
  while (buttonBox.popEvent(ev)) {
    if (ev.type != ButtonBox::EventType::Click) continue;

    switch (ev.mask) {
      case 1 << 0:  // button 1: Auto
        if (pilotMode == STANDBY) {
          pilotMode = AUTO;
          desired_heading = current_heading;
          Serial.println("Auto");
          setGuidedMode();
          // TODO: don't do this every time?
          sendArmCommand();
        }
        break;

      case 1 << 1:  // button 2: Standby
        if (pilotMode == AUTO) {
          pilotMode = STANDBY;
          standby_ram_position = 1500;
          Serial.println("Standby");
          setManualMode();
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

        //Serial.println("Char received");

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
            }

            // sysid 1 only - a GCS sharing this link (e.g. MAVProxy) sends its
            // own heartbeat too, with base_mode never carrying the armed bit.
            if (msg.msgid == MAVLINK_MSG_ID_HEARTBEAT && msg.sysid == 1) {
                mavlink_heartbeat_t hb;
                mavlink_msg_heartbeat_decode(&msg, &hb);
                displayState.armed = (hb.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) != 0;
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
            #endif
          }
          // update the nmea bridge
          handleMavMessage(msg);
          if (wifi == true) webTelemetry_handleMavMessage(msg);
          break;

        }

    }

   yield();
  
  // GUIDED Mode needs updates at least every three seconds or it stops
  if ((pilotMode == AUTO) && (millis() - lastMavlinkUpdate >  mavlinkUpdateInterval)) {
      sendYawCommandDeg(ArduPilotSerial, 1, 1, desired_heading);
      lastMavlinkUpdate = millis();
  }

  mavNmeaBridge_update();
  opencpnBridge_update();
  if (wifi == true) webTelemetry_update();

  #ifdef TILLY_DISPLAY
  if (millis() - lastDisplayUpdate > displayUpdateInterval) {
    lastDisplayUpdate = millis();
    displayState.autoMode = (pilotMode == AUTO);
    displayState.desHeading = desired_heading;
    updateTillyDisplay(displayState);
  }
  #endif

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

    // Serialize and send over serial
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    Serial.println("Sending ARM to tilly");
    ArduPilotSerial.write(buf, len);
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

    // Serialize and send over serial
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    Serial.println("Sending Mode to tilly");
    ArduPilotSerial.write(buf, len);
}


void sendYawCommandDeg(Stream &serial, uint8_t target_system, uint8_t target_component, float yaw_deg) {
    mavlink_message_t msg;

    // Convert degrees to radians
    float yaw_rad = yaw_deg * DEG_TO_RAD;

    // Pack the MAVLink message directly
    mavlink_msg_set_position_target_local_ned_pack(
        250,          // system ID
        1,       // component ID
        &msg,                   // message struct
        millis(),               // timestamp (ms since boot)
        1,          // target system
        1,       // target component
        MAV_FRAME_LOCAL_NED,    // frame
        0b100111111111,         // type_mask: ignore position, velocity, acceleration
        0, 0, 0,                // x, y, z (ignored)
        0, 0, 0,                // vx, vy, vz (ignored)
        0, 0, 0,                // ax, ay, az (ignored)
        yaw_rad,                // yaw in radians
        0                       // yaw_rate
    );

    // Serialize and send over serial
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    //Serial.println("writing to tilly");
    serial.write(buf, len);
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

    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    ArduPilotSerial.write(buf, len);
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

    // Serialize and send
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    ArduPilotSerial.write(buf, len);
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

    // Serialize and send over serial
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    Serial.println("CompassCalibration");
    ArduPilotSerial.write(buf, len);
}
