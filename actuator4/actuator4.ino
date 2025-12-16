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
 */


#include "SimpleFOC.h"

MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);

// TODO: Switch between brake and ramp down

const int PWM_PIN  = 3;   // PWM output
const int DIR1  = 5;   // direction
const int DIR2 = 6;   // second direction pin (or LOW if driver only needs 1)

int rampSize = 10;
int rampDelay = 30;

int curentRamp = 0;
unsigned long lastRampTime = 0;

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



void setup() {
  Serial.begin(115200);  
  pinMode(DIR1, OUTPUT);
  pinMode(DIR2, OUTPUT);
  pinMode(PWM_PIN, OUTPUT);
  sensor.init();
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


void loop() {
      sensor.update();


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
  runTest();

  analogWrite(PWM_PIN, outputPWM);
  
  if (lastSerial + serialPeriod < millis()) { 
    Serial.println(String(outputPWM) + ";" + String(sensor.getVelocity()) + ";" + String(sensor.getAngle())+ ";" + String(rampSize) + ";" + String(rampDelay) + ";" + String(minVelocity));
    lastSerial = millis();
  }

  // check commanded position, slam the brake if nearby

}
