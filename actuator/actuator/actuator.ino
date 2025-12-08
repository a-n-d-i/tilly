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



// TODO Copy over PID implementation, add logic to stop and maybe even disable drive
// Determine min and max speed, maybe just stop action if speed is less than min_speed?
// or min_voltage? Where is stuff converted to voltage anyways?
// do own dcdriver with min voltage? might cause 

#include <Arduino.h>

#include "SimpleFOC.h"
#include "SimpleFOCDrivers.h"
#include "SimpleDCMotor.h"
//#include "sloppypid.h"

// DCMotor object
DCMotor motor = DCMotor();
// DCDriver object
// there are different types to choose from, please select the correct one
// that matches your motor driver hardware.
// DCDriver2PWM driver = DCDriver2PWM(5, 6);
// Sensor object

#define PIN_PWM 3
#define PIN_DIRA 5
#define PIN_DIRB 6

DCDriver1PWM2Dir driver = DCDriver1PWM2Dir(PIN_PWM, PIN_DIRA, PIN_DIRB, NOT_SET);


class MyDCDriver : public DCDriver1PWM2Dir {
public:
    // Forward the base-class constructor
    MyDCDriver(int pwm_pin, int dir1_pin, int dir2_pin)
        : DCDriver1PWM2Dir(pwm_pin, dir1_pin, dir2_pin) {}

    // Override setPwm
    virtual void setPwm(float pwm) override {
        // ---- custom logic start ----
        // Example: clamp and print
        float modifiedPwm = constrain(pwm, -1.0f, 1.0f);
        Serial.print("Custom PWM: ");
        Serial.println(modifiedPwm);
        // ---- custom logic end ----

        // Call the original method if you still want its behavior
        DCDriver1PWM2Dir::setPwm(modifiedPwm);
    }
};


class PIDWithDeadband : public PIDController {
public:
    float deadband = 0.05f;   // default deadband (adjust as needed)

    PIDWithDeadband(float P, float I, float D, float output_ramp = 100000.0f, float limit=0) : PIDController(P, I, D, output_ramp, limit)
    {
        this->P = P;
        this->I = I;
        this->D = D;
        this->output_ramp = output_ramp;
    }


    // Override the PID compute operator
    float operator()(float error) {
        // Apply deadband BEFORE PID math
        if (fabsf(error) < deadband) {
            return 0.0f;
        }

        // Call the base PID logic
        float out = PIDController::operator()(error);

        return out;
    }
};



PIDController p = PIDWithDeadband(DEF_P_ANGLE_P,0,0,0,DEF_VEL_LIM);  //!< parameter determining the position PID configuration 

MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
// Commander object, used for serial control
Commander commander = Commander(Serial);
// motor control function - this is needed to link the incoming commands 
// to the motor object
void doMotor(char* cmd){ 
  
  byte mesgLen = strlen(cmd);
  //cmd[mesgLen - 1] = '\0';
  Serial.println("len "+ String(mesgLen) +" "+cmd);
  commander.motor(&motor, cmd);
}

  

void onScalar(char* cmd){ commander.scalar(&p.P,cmd); }
/**
 * Setup function, in which you should intialize sensor, driver and motor,
 * and the serial communications and commander object.
 * Before calling the init() methods of these objects you can set relevant
 * parameters on them. 
 */
void setup() {
  motor.P_angle = p;
  // to use serial control we have to initialize the serial port
  Serial.begin(115200); // init serial communication
  // wait for serial connection - doesn't work with all hardware setups
  // depending on your application, you may not want to wait
  while (!Serial) {};   // wait for serial connection
  // enable debug output to the serial port
  // SimpleFOCDebug::enable();
  
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
  //motor.controller = MotionControlType::angle;
  motor.controller = MotionControlType::velocity;
  motor.torque_controller = TorqueControlType::voltage;
  // init motor
  motor.init();
  // set the PID parameters for velocity control. Velocity PID is the basis also
  // for position mode. If you have not yet tested the velocity mode we strongly
  // suggest you do this first, and find the PID parameters for that mode as
  // the initial values to use here. 
  // Please consult our documentation and forums for tips on PID tuning. The values
  // can be different depending on your PSU voltage, the driver, the sensor
  // and the motor used.
  motor.PID_velocity.P = 0.5f;
  motor.PID_velocity.I = 0.2f;
  motor.PID_velocity.D = 0.0f;
  // output ramp limits the rate of change of the velocity, e.g. limits the
  // accelleration.
  motor.PID_velocity.output_ramp = 200.0f;
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
  commander.add('A',onScalar,"my variable");
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
