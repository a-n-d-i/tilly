/**
 * Position control example.
 * This example controls a DC motor via a 2PWM-compatible driver hardware,
 * such as a L298N based driver.
 * The example is using a SC60228 magnetic position sensor to monitor the rotor position.
 * 
 * SimpleFOC's position mode is used to move the motor to specific positions (angles)
 * chosen by the user. Positive or negative values determine the direction of
 * rotation. The angle is expressed in radians.
 * 
 * SimpleFOC's commander object is used to enable serial control of the
 * desired angle. After connecting the serial console, type "M100" to
 * set the postion to 100 rad, or "M-2.5" to set it to negative 2.5
 * rad. 
 * Many other motor parameters can be set via the commander, please see 
 * our documentation for details on how to use it.
 */


/* Vevor 500mm travel 14mm/s 1000N IP54 Actuator
 * At 12.3V: 2084 Turns end-to-end in 34seconds -> approx 3600RPM / 14.7mm/s
 * 0.24mm/turn
 * 
 * Motor Torque 4-9 mNm as per chatgpt
 * 
 * some bits and pices taken from: 
// https://curiousscientist.tech/blog/as5600-nema17-speed-and-position
 * 
 * */


// TODO Copy over PID implementation, add logic to stop and maybe even disable drive
// Determine min and max speed, maybe just stop action if speed is less than min_speed?
// or min_voltage? Where is stuff converted to voltage anyways?
// do own dcdriver with min voltage? might cause 

#include <Arduino.h>

#include "SimpleFOC.h"
#include "SimpleFOCDrivers.h"
#include "SimpleDCMotor.h"
#include "DCLinearActuatorDriver1PWM2Dir.h"
//#include "sloppypid.h"

// DCDriver object
// there are different types to choose from, please select the correct one
// that matches your motor driver hardware.
// DCDriver2PWM driver = DCDriver2PWM(5, 6);
// Sensor object

#define PIN_PWM 3
#define PIN_DIRA 5
#define PIN_DIRB 6





class MyDCDriver : public DCDriver1PWM2Dir {
public:
    // Forward the base-class constructor
    MyDCDriver(int pwm_pin, int dir1_pin, int dir2_pin)
        : DCDriver1PWM2Dir(pwm_pin, dir1_pin, dir2_pin, NOT_SET) {}

    virtual void setPwm(float pwm) override {
        if (fabsf(pwm) < 5) {          
          //Serial.print("Set PWM to zero");
          DCDriver1PWM2Dir::setPwm(0);
        } else {
          DCDriver1PWM2Dir::setPwm(pwm);
        }        
    }
};

MyDCDriver driver = MyDCDriver(PIN_PWM, PIN_DIRA, PIN_DIRB);


//DCDriver1PWM2Dir driver = DCDriver1PWM2Dir(PIN_PWM, PIN_DIRA, PIN_DIRB, NOT_SET);

class PIDWithDeadband : public PIDController {
public:
    float deadband = 0.00f;   // default deadband (adjust as needed)

    PIDWithDeadband(float P, float I, float D, float output_ramp = 100000.0f, float limit=0) : PIDController(P, I, D, output_ramp, limit)
    {
        this->P = P;
        this->I = I;
        this->D = D;
        this->output_ramp = output_ramp;
    }


    // Override the PID compute operator
    float operator()(float error) {
      Serial.print("foo");
        // Apply deadband BEFORE PID math
        if (fabsf(error) < deadband) {
            return 0.0f;
        }

        Serial.println("Error " + String(error));

        // Call the base PID logic
        float out = PIDController::operator()(error);

        return out;
    }
};


class CustomDCMotor : public DCMotor {
public:

  /*  CustomDCMotor() : DCMotor() {
      Serial.println("custom");
      }*/

    PIDWithDeadband PID_velocity{DEF_PID_VEL_P,DEF_PID_VEL_I,DEF_PID_VEL_D,DEF_PID_VEL_RAMP,DEF_PID_VEL_LIMIT};//!< parameter determining the velocity PID configuration
    PIDWithDeadband P_angle{DEF_P_ANGLE_P,0,0,0,DEF_VEL_LIM};  //!< parameter determining the position PID configuration 


};


// DCMotor object
DCMotor motor = DCMotor();
//CustomDCMotor motor = CustomDCMotor();



PIDWithDeadband p = PIDWithDeadband(DEF_P_ANGLE_P,0,0,0,DEF_VEL_LIM);  //!< parameter determining the position PID configuration 
//PIDWithDeadband v = PIDWithDeadband(DEF_PID_VEL_P,DEF_PID_VEL_I,DEF_PID_VEL_D,DEF_PID_VEL_RAMP,DEF_PID_VEL_LIMIT);  //!< parameter determining the position PID configuration 

MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
// Commander object, used for serial control
Commander commander = Commander(Serial);
// motor control function - this is needed to link the incoming commands 
// to the motor object
void doMotor(char* cmd){ 
  commander.motor(&motor, cmd);
}

  

//void onScalar(char* cmd){ commander.scalar(&p.P,cmd); }
/**
 * Setup function, in which you should intialize sensor, driver and motor,
 * and the serial communications and commander object.
 * Before calling the init() methods of these objects you can set relevant
 * parameters on them. 
 */
void setup() {

  // to use serial control we have to initialize the serial port
  Serial.begin(115200); // init serial communication
  // wait for serial connection - doesn't work with all hardware setups
  // depending on your application, you may not want to wait
  while (!Serial) {};   // wait for serial connection
  // enable debug output to the serial port
  SimpleFOCDebug::enable();

  motor.P_angle = p;
  
  // basic driver setup - set power supply voltage
  driver.voltage_power_supply = 12.0f;
  // if you want, you can limit the voltage used by the driver.
  // This value has to be same as or lower than the power supply voltage.
  driver.voltage_limit = 12.0f;
  // Optionally set the PWM frequency.
  driver.pwm_frequency = 5000;
  // init driver
  driver.init();
  // init sensor
  sensor.init();
  // link driver to motor
  motor.linkDriver(&driver);
  // link sensor to motor
  motor.linkSensor(&sensor);


  // set a voltage limit on the motor, optional. The value set here
  // has to be lower than the power supply voltage.
  motor.voltage_limit = 12.0f;
  //motor.velocity_limit = 500.0f;
  // control type - for this example we use position mode.
  motor.controller = MotionControlType::angle;
  //motor.controller = MotionControlType::velocity;
  motor.torque_controller = TorqueControlType::voltage;
  // init motor
  motor.init();

  //motor.PID_velocity = void;
  
  // set the PID parameters for velocity control. Velocity PID is the basis also
  // for position mode. If you have not yet tested the velocity mode we strongly
  // suggest you do this first, and find the PID parameters for that mode as
  // the initial values to use here. 
  // Please consult our documentation and forums for tips on PID tuning. The values
  // can be different depending on your PSU voltage, the driver, the sensor
  // and the motor used.
  motor.PID_velocity.P = 0.3f;
  motor.PID_velocity.I = 0.1f;
  motor.PID_velocity.D = 0.0f;
  // output ramp limits the rate of change of the velocity, e.g. limits the
  // accelleration.
  motor.PID_velocity.output_ramp = 200.0f;
  //motor.PID_velocity.output_ramp = 0.0f;
  // low pass filter time constant. higher values smooth the velocity measured
  // by the sensor, at the cost of latency and control responsiveness.
  // Generally speaking, the lower this value can be while still producing good
  // response, the better.
  motor.LPF_velocity.Tf = 0.01f;
  motor.LPF_angle.Tf = 0.01f;
  // angle P-controller P parameter setting. Normally this can
  // be set to a fairly high value.
  motor.P_angle.P = 15.0f;  
  // set the target velocity to 0, we use the commander to set it later+
  motor.target = 0.0f;
  // enable motor
  motor.enable();
  // add the motor and its control function to the commander
  commander.add('M', doMotor, "motor");
  //commander.add('A',onScalar,"my variable");
  // enable monitoring on Serial port
  motor.useMonitoring(Serial);
  motor.monitor_downsample = 1000; // disable monitor at first - optional
  Serial.println("Initialization complete.");
}




void loop() {
  // call motor.move() once per iteration, ideally at a rate of 1kHz or more.
  // rates of more than 10kHz might need a delay, as the sensor may not be able to
  // update quickly enough (depends on sensor)
  motor.move(); // target position can be set via commander input

  // call commander.run() once per loop iteration, it will process incoming commands
  commander.run();

  // call motor.monitor() once per loop iteration, it will print the motor state
  motor.monitor();
}
