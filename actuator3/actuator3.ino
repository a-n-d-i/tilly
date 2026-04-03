
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
const int serialPeriod = 100;

// Conversion: 0.24 mm per full 360° turn
const float MM_PER_TURN = 0.24f;

//Define Variables we'll be connecting to
float Input, Output;

//Specify the links and initial tuning parameters
float Kp = 0.6, Ki = 0.0, Kd = 0.00, Pon = 0.5, Don = 0.0;
bool testrun = true;

// controller state
float commandedSpeed = 0;
float plannedSpeed = 0;
int plannedDirection = CW;
int currentDirection = CW;

// P on measurement?
QuickPID velocityPID(&Input, &Output, &commandedSpeed, Kp, Ki, Kd, Pon, Don, QuickPID::DIRECT);

void setup()
{
  Serial.begin(115200);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(DIR_PIN2, OUTPUT);
  pinMode(PWM_PIN, OUTPUT);

  sensor.init();
  //turn the PID on
  velocityPID.SetMode(QuickPID::AUTOMATIC);
  velocityPID.SetSampleTimeUs(100000); // Default is 100000 µs / 10ms.
  velocityPID.SetControllerDirection(QuickPID::DIRECT);
  velocityPID.SetOutputLimits(-255, 255);

  digitalWrite(DIR_PIN, LOW);
  digitalWrite(DIR_PIN2, HIGH);


}

struct testEvent {
  int startTimeOffset;
  int targetSpeed;
  
};

/*
void setTarget(float speed) {
  if (speed < 0) {
    plannedDirection = CCW;
  } else {
    plannedDirection = CW;
  }  
  plannedSpeed = fabs(speed);  
}*/


void setTarget(float speed) {   
  if (speed > 0) {
    digitalWrite(DIR_PIN, LOW);
    digitalWrite(DIR_PIN2, HIGH);
    currentDirection = CW;
    velocityPID.SetControllerDirection(QuickPID::DIRECT);
    
  } else {
    digitalWrite(DIR_PIN, HIGH);
    digitalWrite(DIR_PIN2, LOW);
    currentDirection = CCW;
    velocityPID.SetControllerDirection(QuickPID::REVERSE);
  }    
  commandedSpeed = fabs(speed);
}

/*
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
*/


testEvent events[] = {
  {2000, 600},
  {1000, 0},
  {2000, -600},
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
  
  // dafuq? ist das mein fehler?
  setTarget(events[currentEvent].targetSpeed);

  if (nextEventTime <= millis()) {
    currentEvent +=1;
    nextEventTime = millis() + events[currentEvent].startTimeOffset;
  }
}

void printFloat(float value, int width, int decimals) {
  char buf[20];
  dtostrf(value, width, decimals, buf);
  Serial.print(buf);
}

// mavlink erst mal über softserial?
// nicht servo sondern speed aus mavlink lesen?


String inputString = "";         
bool stringComplete = false;  
float v[5];
LowPassFilter filter = LowPassFilter(0.050); // Tf = 1ms

void loop()
{  
  sensor.update();
  //Input = filter(fabs(sensor.getVelocity()));
  Input = filter(sensor.getVelocity());
  velocityPID.Compute();

  //Output = constrain(commandedSpeed, 0, 255);
  //if (Output < 128) Output = 0;

  analogWrite(PWM_PIN, Output);

  // update controller state
  // do we need to reverse direction?
 
  /*if (plannedDirection != currentDirection) {
    commandedSpeed -= 30;
  } else {
    commandedSpeed = plannedSpeed;
  }*/
    commandedSpeed = plannedSpeed;
  //commandedSpeed = constrain(commandedSpeed, 0, 600);
  //if (commandedSpeed < 200) commandedSpeed = 0;

  // dont just slam it in reverse, let it slow down a bit first...

  

  if (lastSerial + serialPeriod < millis()) {
    //Serial.println("Commanded " + String(commandedSpeed)+ " Planned " + String(plannedSpeed) + " Direction " + String(currentDirection) + " Input " + String(Input) + " Output " + String(Output));

    // convert direction to speed neg/pos
    //Serial.print(String(millis()) + ";" + String(commandedSpeed * currentDirection)+ ";" + String(plannedSpeed * currentDirection) + ";" + String(Input) + ";" + String(Output));
    //Serial.println(";" + String(velocityPID.GetPterm())+ ";" + String(velocityPID.GetIterm()) + ";" + String(velocityPID.GetDterm()));

    Serial.print( String(commandedSpeed * currentDirection)+ "," + String(plannedSpeed * currentDirection) + "," + String(Input) + "," + String(Output));
    Serial.println("," + String(velocityPID.GetPterm())+ "," + String(velocityPID.GetIterm()) + "," + String(velocityPID.GetDterm()));
   
    /*Serial.print(millis());
    Serial.print(" ; ");
    
    printFloat(commandedSpeed * currentDirection, 6, 2);
    Serial.print(" ; ");
    
    printFloat(plannedSpeed * currentDirection, 6, 2);
    Serial.print(" ; ");
    
    printFloat(Input, 6, 2);
    Serial.print(" ; ");
    
    printFloat(Output, 6, 2);
    Serial.print(" ; ");
    
    printFloat(velocityPID.GetPterm(), 6, 2);
    Serial.print(" ; ");
    
    printFloat(velocityPID.GetIterm(), 6, 2);
    Serial.print(" ; ");
    
    printFloat(velocityPID.GetDterm(), 6, 2);
    
    Serial.println();*/
    
    lastSerial = millis();
  }
  if (testrun == true) runTest();
  
  if (Serial.available()) {
    char buf[64];
    int n = Serial.readBytesUntil('\n', buf, sizeof(buf) - 1);
    buf[n] = '\0';

    char *tok = strtok(buf, ",");
    int i = 0;

    while (tok && i < 5) {
      v[i++] = atof(tok);
      tok = strtok(NULL, ",");
    }

    if (i == 5) {
      velocityPID.SetTunings(v[0], v[1], v[2], v[3], v[4]);
    }
  }
}
