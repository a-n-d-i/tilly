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
 * TODO: Stall detection via Sensor
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

MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);

// TODO: Switch between brake and ramp down

const int PWM_PIN = 16;   // PWM output
const int DIR1  = 13;   // direction
const int DIR2 = 33;   // second direction pin 

int rampSize = 10;
int rampDelay = 30;

int curentRamp = 0;
unsigned long lastRampTime = 0;

unsigned long lastVelocityTime = 0;

int minVelocity = 20; // rads / s 
int minPWM = 100;
int outputPWM = 0;

long lastSerial = 0;
const int serialPeriod = 100;

// test speed control
// go forward 2 sec
// go backward 2 sec

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
    //Serial.println("Homing");
  }

  // are we stalling?
  if ((millis() - lastVelocityTime > 500) && (fabs(sensor.getVelocity()) < 100)) {
     // We're home, set offset
     // resets the turn counter, hopefully without sideeffects
     //sensor.init();
     // seems there is no way to set the acutal rotatinal offset
     sensorOffset = sensor.getAngle();
     //Serial.println("Homed, Sensor offset " + String(sensorOffset));
     homing = false;
     homed = true;          
     currentState = BRAKE;
  }  
}

int currentPosition(){
  // convert radiants to mm
  return (sensor.getAngle() - sensorOffset) / (2 * PI) * 0.24;
}

void setup() {
  Serial.begin(115200);  
  
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
  
  ArduinoOTA.setHostname("tilly-actuator");

  int wifiTimeout = millis() + 10000;

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
    //while ((WiFi.status() != WL_CONNECTED) and (millis() < wifiTimeout)){
    while ((WiFi.status() != WL_CONNECTED)){
      delay(500);
      //ESP.restart();
      Serial.print(".");
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    wifi = true;
    Serial.println("\nWiFi connected!");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
  
    ArduinoOTA.begin();  // Starts OTA
    
    // Start UDP
    udp.begin(remotePort);
  }
  
  pinMode(DIR1, OUTPUT);
  pinMode(DIR2, OUTPUT);
  ledcAttach(PWM_PIN, 1000, 8); 
  //pinMode(PWM_PIN, OUTPUT);
  Wire.begin(25, 26);
  Wire.setClock(400000);
  sensor.init(&Wire);

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

int targetPosition = 42;

void sendLogToUdp(String message) {
  udp.beginPacket(remoteIP, 42424);
  udp.print(message);
  udp.endPacket();
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
    if (fabs(currentPosition() - targetPosition) < 3) {
      // stop
      currentState = BRAKE;
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
      if (nextDirection == FORWARD) currentState = FORWARD_RAMP_UP;
      if (nextDirection == BACKWARD) currentState = BACKWARD_RAMP_UP;
      //Serial.println("STOPPED");
      break;

    case FORWARD_RAMP_UP:
      digitalWrite(DIR1, HIGH);
      digitalWrite(DIR2, LOW);
      rampUp();
      if(outputPWM == 255) {
        currentState = FORWARD;        
      }
      //Serial.println("FORWARD_RAMP_UP");

      break;

    case FORWARD:
      if (nextDirection == BACKWARD) currentState = BRAKE;
      //Serial.println("FORWARD");
      break;

    case BACKWARD_RAMP_UP:
      digitalWrite(DIR1, LOW);
      digitalWrite(DIR2, HIGH);
      rampUp();
      if(outputPWM == 255) {
        currentState = BACKWARD;        
      }      
      //Serial.println("BACKWARD_RAMP_UP");

    break;

    case BACKWARD:
      if (nextDirection == FORWARD) currentState = BRAKE;
      //Serial.println("BACKWARD");

      break;

    case BRAKE:
      outputPWM = 0;
      digitalWrite(DIR1, HIGH);
      digitalWrite(DIR2, HIGH);
      if (sensor.getVelocity() < minVelocity) {
        currentState = STOPPED;
      }
     //Serial.println("BRAKE");

      break;
      
    default:
      Serial.println("dafuq");
  }

  // read command from serial
  //if (homed == true) runTest();

  ledcWrite(PWM_PIN, outputPWM);
  
  if (lastSerial + serialPeriod < millis()) { 
    
    //Serial.println(String(outputPWM) + ";" + String(sensor.getVelocity()) + ";" + String(currentPosition())+ ";" + String(rampSize) + ";" + String(rampDelay) + ";" + String(minVelocity));
    sendLogToUdp(String(outputPWM) + ";" + String(sensor.getVelocity()) + ";" + String(currentPosition())+ ";" + String(rampSize) + ";" + String(rampDelay) + ";" + String(minVelocity) + "\n");
    lastSerial = millis();
  }
}
