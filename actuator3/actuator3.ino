
#include "QuickPID.h"
#include "SimpleFOC.h"

MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);

// -------------------------- MOTOR PINS -----------------------------
const int PWM_PIN  = 3;   // PWM output
const int DIR_PIN  = 5;   // direction
const int DIR_PIN2 = 6;   // second direction pin (or LOW if driver only needs 1)


// ------------------------ POSITION CONTROL -------------------------
float targetPositionMM = 30.0f;  // [mm]
float currentPositionMM = 0.0f; // [mm]


// ------------------------- SERIAL DEBUGGING -------------------------
long lastSerial = millis();
const int serialPeriod = 1000;

// Conversion: 0.24 mm per full 360° turn
const float MM_PER_TURN = 0.24f;

//Define Variables we'll be connecting to
float Input, Output;

//Specify the links and initial tuning parameters
float Kp = 0.2, Ki = 0.1, Kd = 0;

bool testrun = true;

// controller state
float commandedSpeed = 0;
float plannedSpeed = 0;
int plannedDirection = CW;
int currentDirection = CW;


QuickPID velocityPID(&Input, &Output, &commandedSpeed, Kp, Ki, Kd, QuickPID::DIRECT);

void setup()
{
  Serial.begin(115200);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(DIR_PIN2, OUTPUT);
  pinMode(PWM_PIN, OUTPUT);

  sensor.init();
  //turn the PID on
  velocityPID.SetMode(QuickPID::AUTOMATIC);

  digitalWrite(DIR_PIN, LOW);
  digitalWrite(DIR_PIN2, HIGH);
}

struct testEvent {
  int startTimeOffset;
  int targetSpeed;
  
};

void setTarget(float speed) {
  if (speed < 0) {
    plannedDirection = CCW;
  } else {
    plannedDirection = CW;
  }  
  plannedSpeed = fabs(speed);  
}


testEvent events[] = {
  {2000, 400},
  {1000, 0},
  {2000, 300},
  {1000, 0},
  {5000, -400},
  {1000, 400},
  {1000, -400},
  {1000, 300},
  {1000, -300},
  {1000, 0}
 };

int eventCount = sizeof(events) / sizeof(testEvent);

int currentEvent = -1;
long nextEventTime;

// TODO: stall check for end switches and broken/fallen off sensor

void runTest(){
  if ((currentEvent == -1) || (currentEvent == eventCount)) {
    currentEvent = 0;   
    nextEventTime = millis() + events[currentEvent].startTimeOffset;
  }
  
  setTarget(events[currentEvent].targetSpeed);

  if (nextEventTime <= millis()) {
    currentEvent +=1;
    nextEventTime = millis() + events[currentEvent].startTimeOffset;
  }
}

// mavlink erst mal über softserial?
// nicht servo sondern speed aus mavlink lesen?


String inputString = "";         
bool stringComplete = false;  

void loop()
{  
  sensor.update();
  Input = fabs(sensor.getVelocity());
  velocityPID.Compute();

  Output = constrain(commandedSpeed, 0, 255);
  if (Output < 128) Output = 0;

  analogWrite(PWM_PIN, Output);

  // update controller state
  // do we need to reverse direction?
  if (plannedDirection != currentDirection) {
    commandedSpeed -= 30;
  } else {
    commandedSpeed = plannedSpeed;
  }
  commandedSpeed = constrain(commandedSpeed, 0, 400);
  if (commandedSpeed < 200) commandedSpeed = 0;

  // dont just slam it in reverse, let it slow down a bit first...
  if ((plannedDirection != currentDirection) && Input < 100) {
    if (plannedDirection == CW) {
      digitalWrite(DIR_PIN, LOW);
      digitalWrite(DIR_PIN2, HIGH);
    } else {
      digitalWrite(DIR_PIN, HIGH);
      digitalWrite(DIR_PIN2, LOW);
    }
    currentDirection = plannedDirection;
  }
  

  if (lastSerial + serialPeriod < millis()) {
    Serial.println("Commanded " + String(commandedSpeed)+ " Planned " + String(plannedSpeed) + " Direction " + String(currentDirection) + " Input " + String(Input) + " Output " + String(Output));
    lastSerial = millis();
  }
  if (testrun == true) runTest();

  while (Serial.available()) {    
    char inChar = Serial.read();
    inputString += inChar; 

    if (inChar == '\n') {
      stringComplete = true;
      break; 
    }
  }

  if (stringComplete) {
    float value = inputString.toFloat();   
    setTarget(value);    
    inputString = ""; 
    stringComplete = false; 
  }  
}
