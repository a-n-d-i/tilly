
#include "QuickPID.h"
#include "SimpleFOC.h"

MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);

// -------------------------- MOTOR PINS -----------------------------
const int PWM_PIN  = 3;   // PWM output
const int DIR_PIN  = 5;   // direction
const int DIR_PIN2 = 6;   // second direction pin (or LOW if driver only needs 1)

// ----------------------- SPEED CALCULATION -------------------------
float currentSpeedDegS = 0.0f;  // [deg/s]

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


QuickPID myQuickPID(&Input, &Output, &commandedSpeed, Kp, Ki, Kd, QuickPID::DIRECT);

void setup()
{
  Serial.begin(115200);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(DIR_PIN2, OUTPUT);
  pinMode(PWM_PIN, OUTPUT);
  

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


// Do a little test jig: accel up down and reversal in infinite loop and csv output...

int currentEvent = -1;
long nextEventTime;

// stall check for end switches and broken/fallen off sensor

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

// TODO: direction nach unten durchdrücken, +-speed obenrum, PID nur +

// mavlink erst mal über softserial?
// nicht servo sondern speed aus mavlink lesen?


String inputString = "";         // Eine String-Variable, um die empfangenen Daten zu speichern
bool stringComplete = false;  

void loop()
{  
  sensor.update();
  Input = fabs(sensor.getVelocity());
  myQuickPID.Compute();

  Output = constrain(commandedSpeed, 0, 255);
  if (Output < 128) Output = 0;

  analogWrite(PWM_PIN, Output);

  // update controller state
  // do we need to reverse direction
  if (plannedDirection != currentDirection) {
    commandedSpeed -= 30;
  } else {
    commandedSpeed = plannedSpeed;
  }
  commandedSpeed = constrain(commandedSpeed, 0, 400);
  if (commandedSpeed < 200) commandedSpeed = 0;

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
    setTarget(value);    
    // Hier kann die 'inputString' verarbeitet werden
    inputString = ""; // String für die nächste Eingabe leeren
    stringComplete = false; // Flag zurücksetzen
  }  

  // do interrupt based timing
  
}
