
/* Configuration:
 * - Create a file named "config.h" in the same folder as this sketch
 * - See config.h.example for template
 */

#define TILLY_DISPLAY

#include "display.c"

#include <WiFi.h>
#include <WiFiUdp.h>
#include <ezButton.h>
#include <MAVLink.h>
// Include MAVLink library - using ardupilotmega dialect for full ArduPilot support
// https://github.com/okalachev/mavlink-arduino

#include <ArduinoOTA.h>  // For enabling over the air updates

#define TFT_BACKGROUND 5

// Include WiFi configuration from external file
#include "config.h"

WiFiUDP udp;

// ===== Hardware Serial Configuration =====
#define SERIAL_RX 35  // RX2
#define SERIAL_TX 15  // TX2
HardwareSerial ArduPilotSerial(2);  // Use UART2

// ===== Button Configuration =====
ezButton plus1Button(14, INPUT_PULLUP);
ezButton plus10Button(26, INPUT_PULLUP);
ezButton autoButton(12, INPUT_PULLUP);
ezButton minus1Button(27, INPUT_PULLUP);
ezButton minus10Button(22, INPUT_PULLUP);
ezButton standbyButton(13, INPUT_PULLUP);

IPAddress remoteIP(192,168,1,255);   // destination
uint16_t remotePort = 14550;        // destination port

// Display update timer
unsigned long lastDisplayUpdate = 0;
const unsigned long displayUpdateInterval = 500;

unsigned long lastMavlinkUpdate = 0;
const unsigned long mavlinkUpdateInterval = 1000;

// Variables for display data
int current_heading = 0;
int current_heading_old = 0;
int desired_heading = 0;
int desired_heading_old = 0;

enum pilotModeType {STANDBY, AUTO};

pilotModeType pilotMode = STANDBY;

void setup() {
  
  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH) {
        type = "sketch";
      } else {  // U_SPIFFS
        type = "filesystem";
      }

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
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
  
  pinMode(TFT_BACKGROUND, OUTPUT);    // sets the digital pin 13 as output
  analogWrite(5, 200); // 0 -> full brightness/whiteout, 1024 -> off
  Serial.begin(115200);
  Serial.println("Tillys little helper");
  
  // Initialize ArduPilot Serial
  ArduPilotSerial.begin(115200, SERIAL_8N1, SERIAL_RX, SERIAL_TX);
  Serial.println("ArduPilot Serial initialized");

  plus1Button.setDebounceTime(50);
  plus10Button.setDebounceTime(50);
  autoButton.setDebounceTime(50);
  minus1Button.setDebounceTime(50);
  minus10Button.setDebounceTime(50);
  standbyButton.setDebounceTime(50);  
  Serial.println("Buttons initialized");
  
  // Initialize displays
  Serial.println("Display initialized");
  
  // Connect to WiFi
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    //ESP.restart();
    Serial.print(".");
  }
  
  Serial.println("\nWiFi connected!");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  ArduinoOTA.begin();  // Starts OTA
  
  // Start UDP
  udp.begin(udpPort);
  //Serial.printf("UDP listening on port %d\n", udpPort);
  requestNavControllerOutput(ArduPilotSerial, 1, 1);

  #ifdef TILLY_DISPLAY
  lower.curText = String(current_heading);
  lower.desText = String(desired_heading);
  initTillyDisplay();
  #endif
  
  sendArmCommand();
}

void handleButtons(){
  plus1Button.loop();
  plus10Button.loop();
  autoButton.loop();
  minus1Button.loop();
  minus10Button.loop();
  standbyButton.loop();

  if (pilotMode == STANDBY) {
      if(plus1Button.isPressed()){        
        Serial.println("+1 step");
        
      }
    

    // TODO: Send RC override
    
  } else {
    
    if(plus1Button.isPressed()){
      desired_heading += 1;
      Serial.println("+1 deg");
    }
  
    if(plus10Button.isPressed()){
      desired_heading += 10;
      Serial.println("+10 deg");
    }

    if(minus1Button.isPressed()){
      desired_heading -= 1;
      Serial.println("-1 deg");
    }
  
    if(minus10Button.isPressed()){
      desired_heading -= 10;
      Serial.println("-10 deg");
    }

    if (desired_heading > 359) {
      desired_heading -= 360;
    }

    if (desired_heading < 0) {
      desired_heading +=360;
    }
  }


  // bc of the slow display, there's bouncing in the debounce lib. Dirty fix for now. TODO, I guess...
  if(standbyButton.isPressed() && pilotMode == AUTO){
    pilotMode = STANDBY;
    Serial.println("Standby");
    setManualMode();
    #ifdef TILLY_DISPLAY
    lower.bgColor = ILI9341_RED;
    drawLowerDisplay();
    #endif
  }
  
  
  if(autoButton.isPressed() && pilotMode == STANDBY){
    pilotMode = AUTO;
    desired_heading = current_heading;
    Serial.println("Auto");  
    setGuidedMode();
    // TODO: don't do this every time?
    sendArmCommand();
    #ifdef TILLY_DISPLAY
    lower.bgColor = ILI9341_GREEN;
    drawLowerDisplay();
    #endif
  }
}


void loop() {
  ArduinoOTA.handle(); 
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

            // TODO: decide wether package is for us or someone else?...

            // Send via UDP
            udp.beginPacket(remoteIP, remotePort);
            udp.write(buffer, len);
            udp.endPacket();

            
             if (msg.msgid == MAVLINK_MSG_ID_VFR_HUD) {
                mavlink_vfr_hud_t hud;
                mavlink_msg_vfr_hud_decode(&msg, &hud);
        
                current_heading = hud.heading;   // heading in degrees (0–360)               
                Serial.println("Heading: " + String(current_heading));
            }

           if (msg.msgid == MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT) {
              mavlink_nav_controller_output_t nav;
              mavlink_msg_nav_controller_output_decode(&msg, &nav);
      
              Serial.print("Nav Controller Output - ");
              Serial.print("Nav_roll: "); Serial.print(nav.nav_roll);
              Serial.print(" | Nav_pitch: "); Serial.print(nav.nav_pitch);
              Serial.print(" | Alt_error: "); Serial.print(nav.alt_error);
              Serial.print(" | Aspd_error: "); Serial.print(nav.aspd_error);
              Serial.print(" | Xtrack_error: "); Serial.print(nav.xtrack_error);
              Serial.print(" | Target_bearing: "); Serial.print(nav.target_bearing);
              Serial.print(" | WP_dist: "); Serial.println(nav.wp_dist);
          }
                  
            break;
        }
    }

  // ---------------------------
  // UDP → Serial
  // ---------------------------
  
  int packetSize = udp.parsePacket();
  if (packetSize) {

    // Read UDP packet
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    udp.read(buf, MAVLINK_MAX_PACKET_LEN);
    ArduPilotSerial.write(buf, packetSize);
  }

  // GUIDED Mode needs updates at least every three seconds or it stops
  if (millis() - lastMavlinkUpdate >  mavlinkUpdateInterval) {
      sendYawCommandDeg(ArduPilotSerial, 1, 1, desired_heading);
      lastMavlinkUpdate = millis();
  }
  
  #ifdef TILLY_DISPLAY
  // Update displays
  if (millis() - lastDisplayUpdate > displayUpdateInterval) {
    lastDisplayUpdate = millis();
    if (desired_heading_old != desired_heading) {     
      updateDeg(70, desired_heading, desired_heading_old);
    }
    if (current_heading != current_heading_old) {
      updateDeg(10, current_heading, current_heading_old);
    }
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
    Serial.println("writing to tilly");
    serial.write(buf, len);
}

// -------------------------------------------
// Request NAV_CONTROLLER_OUTPUT message
// -------------------------------------------
void requestNavControllerOutput(Stream &serial, uint8_t target_sys, uint8_t target_comp) {
    mavlink_message_t msg;

    // Send REQUEST_DATA_STREAM
    mavlink_msg_request_data_stream_pack(
        255,  // sender system (255 = GCS/this device)
        0,    // sender component
        &msg,
        target_sys,
        target_comp,
        MAV_DATA_STREAM_EXTRA1, // NAV_CONTROLLER_OUTPUT is part of EXTRA1
        1,    // message rate (1 Hz)
        1     // start streaming
    );

    // Serialize and send
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    serial.write(buf, len);
}
