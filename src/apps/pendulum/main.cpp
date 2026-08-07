#include "serial_setup.h"

#include <SimpleFOC.h>

MagneticSensorI2C wheel = MagneticSensorI2C(AS5600_I2C);
MagneticSensorSPI pendulum = MagneticSensorSPI(10, 14, 0x3FFF);

BLDCMotor motor = BLDCMotor(7);

BLDCDriver3PWM driver = BLDCDriver3PWM(5, 6, 3, 8);

float angle_offset = 0; // offset to be added to the pendulum angle in order to set the zero position

// function constraining the angle in between -pi and pi, in degrees -180 and 180
float constrainAngle(float x){
    x = fmod(x + M_PI, _2PI);
    if (x < 0)
        x += _2PI;
    return x - M_PI;
}

// LQR stabilization controller functions
// calculating the voltage that needs to be set to the motor in order to stabilize the pendulum
float controllerLQR(float p_angle, float p_vel, float m_vel){
  // if angle controllable
  // calculate the control law 
  // LQR controller u = k*x
  // k = [-400, -7, 0.4]
  // x = [pendulum angle, pendulum velocity, motor velocity]' 
  float u =  -400*p_angle -6*p_vel + .4*m_vel;
  
  // limit the voltage set to the motor
  if(abs(u) > motor.voltage_limit*0.7) u = _sign(u)*motor.voltage_limit*0.7;
  
  return u;
}



void setup() {

  serialSetup(APP_NAME_STR);
  // enable more verbose output for debugging
  // comment out if not needed
  SimpleFOCDebug::enable(&Serial);

  // initialise motor encoder hardware
  wheel.init();
  
  // init the pendulum encoder
  pendulum.init();
  
  // set control loop type to be used
  motor.controller = MotionControlType::torque;

  // link the motor to the encoder
  motor.linkSensor(&wheel);
  
  // driver
  driver.voltage_power_supply = 12; 
  driver.init();
  // link the driver and the motor
  motor.linkDriver(&driver);

  // initialize motor
  motor.init();

  motor.voltage_limit = 6; // limit the voltage set to the motor to 6V

  // align encoder and start FOC
  motor.initFOC();
  
  delay(1000);
  angle_offset = pendulum.getAngle() + M_PI; // set the zero position of the pendulum to be at the bottom
  delay(1000);

}



// loop downsampling counter
long loop_count = 0;

void loop() {
  
  // ~1ms 
  motor.loopFOC();
  loop_count++;
  // control loop each 10ms
  if(loop_count > 10){
    // updating the pendulum angle sensor
    // NECESSARY for library versions > v2.2 
    pendulum.update();
    // calculate the pendulum angle 
    float pendulum_angle = constrainAngle(pendulum.getAngle() - angle_offset);

    float target_voltage;
    if( abs(pendulum_angle) < .5 ) // if angle small enough stabilize
      target_voltage = controllerLQR(pendulum_angle, pendulum.getVelocity(), motor.shaftVelocity());
    else // else do swing-up
      
      target_voltage = _sign(pendulum.getVelocity())*motor.voltage_limit*0.3; // sets 40% of the maximal voltage to the motor in order to swing up

    // set the target voltage to the motor

    /*
    Serial.print(F("Wheel angle is: "));
    Serial.print(wheel.getAngle());
    Serial.print(F("Wheel velocity is: "));
    Serial.print(wheel.getVelocity());
    Serial.print(F(" Pendulum angle is: "));       
    Serial.print(pendulum_angle);
    Serial.print(F(" Pendulum velocity is: "));       
    Serial.print(pendulum.getVelocity());

    Serial.print(F(" Target voltage is: "));
    Serial.println(target_voltage);
    */

    motor.move(target_voltage);
    
    // restart the counter
    loop_count=0;
  }
   

}

