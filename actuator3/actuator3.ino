
#include "QuickPID.h"
#include "SimpleFOC.h"

MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);

// -------------------------- MOTOR PINS -----------------------------
const int PWM_PIN  = 3;   // PWM output
const int DIR_PIN  = 5;   // direction
const int DIR_PIN2 = 6;   // second direction pin (or LOW if driver only needs 1)

// ----------------------- SPEED CALCULATION -------------------------
float currentSpeedDegS = 0.0f;  // [deg/s]

#define NOT_SET 100

int currentDirection = NOT_SET; // CW; CCW; NOT_SET

// ------------------------ POSITION CONTROL -------------------------
float targetPositionMM = 30.0f;  // [mm]
float currentPositionMM = 0.0f; // [mm]


// ------------------------- SERIAL DEBUGGING -------------------------
long lastSerial = millis();
const int serialPeriod = 1000;

// Conversion: 0.24 mm per full 360° turn
const float MM_PER_TURN = 0.24f;


//Define Variables we'll be connecting to
float Setpoint, Input, Output;

//Specify the links and initial tuning parameters
float Kp = 0.2, Ki = 0.1, Kd = 0;

bool testrun = true;

// ===================================================================
// SAFE DIRECTION CONTROL
// ===================================================================
void setDirectionSafe(bool dir) {
  if (dir == currentDirection) return;

  if (abs(currentSpeedDegS) > 10.0f) return;

  currentDirection = dir;
  digitalWrite(DIR_PIN,  currentDirection ? HIGH : LOW);
  digitalWrite(DIR_PIN2, currentDirection ? LOW : HIGH);
}


// ===================================================================
// MOTOR POWER OUTPUT
// ===================================================================
void setMotorPower(float pwmVal) {
  pwmVal = constrain(pwmVal, 0, 255);
  analogWrite(PWM_PIN, pwmVal);
}


QuickPID myQuickPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, QuickPID::DIRECT);

void setup()
{
  Serial.begin(115200);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(DIR_PIN2, OUTPUT);
  pinMode(PWM_PIN, OUTPUT);
  
  
  Setpoint = 300;
  sensor.init();
  //turn the PID on
  myQuickPID.SetMode(QuickPID::AUTOMATIC);

      digitalWrite(DIR_PIN, LOW);
    digitalWrite(DIR_PIN2, HIGH);
}


void setDirection(int direction){
  if (direction == currentDirection) return;
  if ((direction != NOT_SET) && (currentDirection != direction)) {
    // ramp down
  }
  
  if (direction == CW) {
    currentDirection = CW;
    digitalWrite(DIR_PIN, LOW);
    digitalWrite(DIR_PIN2, HIGH);
  } else {
    currentDirection = CCW;
    digitalWrite(DIR_PIN, HIGH);
    digitalWrite(DIR_PIN2, LOW);
  }
}


struct target {
  // -1 if inactive, current speed if < speed. 
  int currentRamp = -1;
  int speed = 0;
  int direction = CW;
};


struct testEvent {
  int startTimeOffset;
  int direction;
  int targetSpeed;
  
};


testEvent events[] = {
  {2000, CW, 400},
  {1000, CW, 0},
  {2000, CW, 300},
  {1000, CW, 0},
  {4000, CCW, 400},
  {1000, CW, 400},
  {1000, CW, 0}
 };

int eventCount = sizeof(events) / sizeof(testEvent);


// Do a little test jig: accel up down and reversal in infinite loop and csv output...

int currentEvent = -1;
long nextEventTime;

// stall check for end switches and broken/fallen off sensor

void runTest(){
  if ((currentEvent == -1) || (currentEvent == eventCount)) {
    currentEvent = 0;   
    nextEventTime = millis() + events[currentEvent].startTimeOffset;
  }
  
  Setpoint = events[currentEvent].targetSpeed;
  setDirection(events[currentEvent].direction);

  if (nextEventTime <= millis()) {
    currentEvent +=1;
    nextEventTime = millis() + events[currentEvent].startTimeOffset;
  }
}

// TODO: direction nach unten durchdrücken, +-speed obenrum, PID nur +

// mavlink erst mal über softserial?
// nicht servo sondern speed aus mavlink lesen?


String inputString = "";         // Eine String-Variable, um die empfangenen Daten zu speichern
bool stringComplete = false;  

void loop()
{  
  sensor.update();
  Input = sensor.getVelocity();
  myQuickPID.Compute();

  if (lastSerial + serialPeriod < millis()) {
    Serial.println("Target " + String(Setpoint) + " Direction " + currentDirection + " Input " + String(Input) + " Output " + String(Output));
    lastSerial = millis();
  }
  // analogWrite(PWM_PIN, Output);
  if (testrun == true) runTest();

  analogWrite(PWM_PIN, Output);
  
  while (Serial.available()) {
    // Liest Zeichen, bis ein Zeilenumbruch '\n' gefunden wird
    char inChar = Serial.read();
    inputString += inChar; // Fügt das Zeichen zum String hinzu

    if (inChar == '\n') { // Wenn das Zeichen ein Zeilenumbruch ist...
      stringComplete = true;
      break; // Verlässt die Schleife
    }
  }

  if (stringComplete) {
    Serial.print("Empfangen: ");
    Serial.println(inputString);
    float value = inputString.toFloat();   
    Setpoint = fabs(value);
    // Hier kann die 'inputString' verarbeitet werden
    inputString = ""; // String für die nächste Eingabe leeren
    stringComplete = false; // Flag zurücksetzen
  }  

  // do interrupt based timing
  
}
