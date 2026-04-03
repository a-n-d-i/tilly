/*
 * Since we're tyring to get the most of cheap and slow controllers, this one follows the "race driver" approach. 
 * Dunno if this is a thing and/or already has a name, I made it up on the spot but don't believe I'm the first one who came up with this.
 * 
 * The "race driver" approach on breaking: We remember the distance to the corner and slam
 * the brakes, there's no feedback control loop for breaking itself. We might make it self learning though...
 * Parameter: breakDistance, unit rad
 * If this leads to too much banging, this should be switched to rampdown.
 * 
 * Acceleration follows the same approach, just with an adjustable ramp up which should simulate the motors acceleration. Adjust it by ear, 
 * you don't want banging an shaking, just a smooth rise in pitch. Parameters: rampSize (amount that is added to PWM per step) rampDuration (ms between additions)
 * 
 * Theory: This Algorith has almost no feedback. Only on direct reversals, we look at the actual velocity. 
 * We might switch that to self learning as well...
 * 
 * So we're pretty much going open loop..
 * 
 * TODO: Measure how long it takes to ramp up/down with velocity
 * TODO: Auto-Home at startup is a risky thing.
 * TODO: Stall detection via Sensor, detect broken / fallen off sensor
 * TODO: Mavlink, get Servo Position and DC Current
 * TODO: Stall detection on Current
 * TODO: i2c error handling
 * 
 * 
 * Test Listener for udp: nc -u -kl 42424
 * 
 */
#include "SimpleFOC.h"
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h> 
#include "config.h"
#include <MAVLink_ardupilotmega.h>

MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);

// TODO: Switch between brake and ramp down

const int PWM_PIN = 16;   // PWM output
// 33 -> IN1, ... per Datasheet, IN1 High, IN2 Low is forward
const int DIR1  = 33;   // direction
const int DIR2 = 13;   // second direction pin 

int rampSize = 10;
int rampDelay = 30;

int curentRamp = 0;
unsigned long lastRampTime = 0;

unsigned long lastVelocityTime = 0;

int minVelocity = 20; // rads / s 
int minPWM = 100;
int outputPWM = 0;

long lastSerial = 0;
const int serialPeriod = 1000;

// test speed control
// go forward 2 sec
// go backward 2 sec

// green scl gpio22
// yellow sda gpio21

// Backwards sucks the ram in, Forward pushes it out.

enum state_t {FORWARD, FORWARD_RAMP_UP, BACKWARD, BACKWARD_RAMP_UP, BRAKE, STOPPED};

state_t currentState = STOPPED;

state_t nextDirection = STOPPED;

struct testEvent {
  int startTimeOffset;
  state_t currentState;
  
};

testEvent events[] = {
  {2000, FORWARD},
  {2000, BACKWARD}
 };

int eventCount = sizeof(events) / sizeof(testEvent);

int currentEvent = -1;
long nextEventTime;

bool wifi = false;

WiFiUDP udp;

IPAddress remoteIP(192,168,1,255);   // udp broadcast
uint16_t remotePort = 42424;        // destination port

HardwareSerial ArduPilotSerial(0);


int targetPosition = 0; // mm
int homePosition = 80; //
int maxTravel = 150; // mm


int current_heading = 0;

// Change to just go to 100mm and then random 0-50mm steps returning to 100mm
// oder iterativ gegen den messschieber drücken, immer 1cm vorwärts und random zurück...
void runTest(){
  if ((currentEvent == -1) || (currentEvent == eventCount)) {
    currentEvent = 0;   
    nextDirection = events[0].currentState;
    nextEventTime = millis() + events[currentEvent].startTimeOffset;    
  }
  
  if (nextEventTime <= millis()) {
    currentEvent +=1;
    nextDirection = events[currentEvent].currentState;
    nextEventTime = millis() + events[currentEvent].startTimeOffset;
  }   
}

bool homing = false;
bool homed = false;
int sensorOffset = 0;

void homeActuator() {

  // move ram in 
  if (homing == false) {
    nextDirection = BACKWARD;
    homing = true;
    logThis(String(millis()) + " Homing");
    // wait at least 1.5s to decide if homed in case the actuator is at the far endstop
    lastVelocityTime = millis();
  }

  // TODO: Home for at least 2 seconds or so, just in case the actuator is at the far end stop
  
  // are we stalling?
  if (((millis() - lastVelocityTime) > 500) && (fabs(sensor.getVelocity()) < 100)) {
     // We're home, set offset
     // resets the turn counter, hopefully without sideeffects
     //sensor.init();
     // seems there is no way to set the acutal rotatinal offset
     sensorOffset = sensor.getAngle();
     logThis(String(millis()) + " Homed by stalling, Sensor offset " + String(sensorOffset));
     homing = false;
     homed = true;          
     currentState = BRAKE;     
  }  
  targetPosition = homePosition;
}

int currentPosition(){
  // convert radiants to mm
  return (sensor.getAngle() - sensorOffset) / (2 * PI) * 0.24;
}

// TODO: https://ardupilot.org/dev/docs/mavlink-requesting-data.html
// mavproxy / sitl sets message rates, this can be disabled
// after that, disable all messages, enable messages we actually want to save bandwidth
// make the requested rate actually achievable

// void requestMessageAtRate(uint16_t message_id, uint16_t interval)
void setupMavlinkMessages(){

  // disable all message stream packs
  for (int x=0; x<12; x++) {
    mavlink_message_t msg;
    mavlink_msg_request_data_stream_pack(255,1,&msg,1,1,x,0,0);


    // Serialize and send over serial
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  
    ArduPilotSerial.write(buf, len);
  }


  mavlink_message_t msg;

  // get servo output raw
  mavlink_msg_command_long_pack(
      250,          // system ID
      1,       // component ID
      &msg,                   // message struct
      1,          // target system
      1,       // target component
      MAV_CMD_SET_MESSAGE_INTERVAL,
          0, // confirmation
          MAVLINK_MSG_ID_SERVO_OUTPUT_RAW, // param1 (0 to indicate disarm)
          250000, // param2: Interval in microseconds
          0, // param3
          0, // param4
          0, // param5
          0, // param6
          0); // param7

  // Serialize and send over serial
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  ArduPilotSerial.write(buf, len);
}



void setup() {
  ArduPilotSerial.begin(115200);
  //Serial.begin(115200);  
  
  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH) {
        type = "sketch";
      } else {  // U_SPIFFS
        type = "filesystem";
      }

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      logThis("Start updating " + type);
    })
    .onEnd([]() {
      logThis("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      //Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      //Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) {
        logThis("Auth Failed");
      } else if (error == OTA_BEGIN_ERROR) {
        logThis("Begin Failed");
      } else if (error == OTA_CONNECT_ERROR) {
        logThis("Connect Failed");
      } else if (error == OTA_RECEIVE_ERROR) {
        logThis("Receive Failed");
      } else if (error == OTA_END_ERROR) {
        logThis("End Failed");
      }
    });
  
  ArduinoOTA.setHostname("tilly-actuator");

  int wifiTimeout = millis() + 10000;

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
    //while ((WiFi.status() != WL_CONNECTED) and (millis() < wifiTimeout)){
    while ((WiFi.status() != WL_CONNECTED)){
      delay(500);
      //ESP.restart();
      //Serial.print(".");
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    wifi = true;
    logThis("\nWiFi connected!");
    //Serial.print("IP address: ");
    //Serial.println(WiFi.localIP());
  
    ArduinoOTA.begin();  // Starts OTA
    
    // Start UDP
    udp.begin(remotePort);
  }
  
  pinMode(DIR1, OUTPUT);
  pinMode(DIR2, OUTPUT);
  ledcAttach(PWM_PIN, 1000, 8); 
  //pinMode(PWM_PIN, OUTPUT);
  //Wire.begin(25, 26);
  //Wire.setClock(400000);
  sensor.init();
  //requestMessageStream(MAVLINK_MSG_ID_SERVO_OUTPUT_RAW);
  setupMavlinkMessages();  
}

// The Controller only accepts commands when in forward, back or stop'ed state. 
void rampUp(){
  if (outputPWM < minPWM) {
    outputPWM = minPWM;
  } 
  
  if ((outputPWM < 255) && (millis() >= lastRampTime + rampDelay)) {
    outputPWM += rampSize;
    outputPWM = constrain(outputPWM, minPWM, 255);
    lastRampTime = millis();
  }
}

void sendLogToUdp(String message) {
  udp.beginPacket(remoteIP, 42424);
  udp.print(message + "\n");
  udp.endPacket();
}

void logThis(String message) {
  sendLogToUdp(message);
  // or serial.println...
}

void loop() {
  if (wifi == true) ArduinoOTA.handle(); 
  sensor.update();

  // remember the last time we saw the motor actually moving
  if (fabs(sensor.getVelocity()) > 100) lastVelocityTime = millis();
  
  if (homed == false) {
    // TODO: make this more safe
    homeActuator();
  } else {
    // determine if we're there-ish
    if (fabs(targetPosition - currentPosition()) < 3) {
      if (fabs(sensor.getVelocity()) < minVelocity) {
        currentState = STOPPED;
        nextDirection = STOPPED;     
      } else {
        currentState = BRAKE;  
      }
    } else {
      // determine if / where to go
      if (targetPosition > currentPosition()) {
        nextDirection = FORWARD;
      } else {
        nextDirection = BACKWARD;
      }
    }
  }
  
  switch (currentState) {
 
    case STOPPED:
      if (nextDirection == FORWARD) {
        currentState = FORWARD_RAMP_UP;
        logThis("changing state from stopped to forward ramp up");
      }
      if (nextDirection == BACKWARD) {
        currentState = BACKWARD_RAMP_UP;
        logThis("changing state from stopped to backward ramp up");
      }
      //Serial.println("STOPPED");
      break;

    case FORWARD_RAMP_UP:
      digitalWrite(DIR1, HIGH);
      digitalWrite(DIR2, LOW);
      rampUp();
      if(outputPWM == 255) {
        currentState = FORWARD;        
        logThis("changing state from forward ramp up to forward");
      }
      //logThis("FORWARD_RAMP_UP");

      break;

    case FORWARD:
      if (nextDirection == BACKWARD) {
        currentState = BRAKE;
        logThis("changing state from backward ramp up to brake");
      }
      //Serial.println("FORWARD");
      break;

    case BACKWARD_RAMP_UP:
      digitalWrite(DIR1, LOW);
      digitalWrite(DIR2, HIGH);
      rampUp();
      if(outputPWM == 255) {
        currentState = BACKWARD;        
        logThis("changing state from backward ramp up to backward");
      }      
      //Serial.println("BACKWARD_RAMP_UP");

    break;

    case BACKWARD:
      if (nextDirection == FORWARD) {
        currentState = BRAKE;
        logThis("changing state from forward up to brake");
      }
      //Serial.println("BACKWARD");

      break;

    case BRAKE:
      outputPWM = 0;
      digitalWrite(DIR1, HIGH);
      digitalWrite(DIR2, HIGH);
      if (fabs(sensor.getVelocity()) < minVelocity) {
        currentState = STOPPED;
        nextDirection = STOPPED;
        //logThis("changing state from brake to stopped");
      }
     //Serial.println("BRAKE");

      break;
      
    default:
      logThis("dafuq");
  }

  // read command from serial
  //if (homed == true) runTest();

  ledcWrite(PWM_PIN, outputPWM);
  
  if (lastSerial + serialPeriod < millis()) { 
    
    //Serial.println(String(outputPWM) + ";" + String(sensor.getVelocity()) + ";" + String(currentPosition())+ ";" + String(rampSize) + ";" + String(rampDelay) + ";" + String(minVelocity));
    //logThis(String(outputPWM) + ";" + String(sensor.getVelocity()) + ";" + String(currentPosition())+ ";" + String(rampSize) + ";" + String(rampDelay) + ";" + String(minVelocity) + "\n");

    //logThis("Target Position: " + String(targetPosition) + " Current Position " + String(currentPosition()) + " State: " + String(currentState) + " Speed " + String(sensor.getVelocity()));  
    logThis(String(targetPosition) + ";" + String(currentPosition()) + ";" + String(currentState) + ";" + String(sensor.getVelocity()) +";" + String(outputPWM)  );  
    
    lastSerial = millis();
  }

  mavlink_message_t msg;
  mavlink_status_t status;

  while (ArduPilotSerial.available() > 0) {
    
    uint8_t c = ArduPilotSerial.read();

    // Add charactar to message and try to parse / loop on until it parses/is complete
    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {

      //logThis(String(msg.msgid));
             
      // Build the raw MAVLink packet for forwarding
      uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
      uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);

      // receive servo raw
      // receive current consumption
      if (msg.msgid == MAVLINK_MSG_ID_VFR_HUD) {
          mavlink_vfr_hud_t hud;
          mavlink_msg_vfr_hud_decode(&msg, &hud);
  
          current_heading = hud.heading;   // heading in degrees (0–360)               
          logThis("Heading: " + String(current_heading));
      }

      if (msg.msgid == MAVLINK_MSG_ID_SERVO_OUTPUT_RAW){
        mavlink_servo_output_raw_t servo_msg;
        mavlink_msg_servo_output_raw_decode(&msg, &servo_msg);

        // Servo 1 output (PWM µs)
        uint16_t servo1_pwm = servo_msg.servo1_raw;

        //logThis("Servo 1 PWM: " + String(servo1_pwm));    
        
        float pos = (float) maxTravel * ((float)(servo1_pwm - 1000) / 1000.0f);
        targetPosition = round(pos);
        //logThis("Target Position: " + String(pos));            
      }           
      break; // only read max one message per main loop 
    }           
  }  
}
