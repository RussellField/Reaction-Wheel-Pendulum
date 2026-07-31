#include "serial_setup.h"

#include <SimpleFOC.h>

MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);

BLDCMotor motor = BLDCMotor(7);

BLDCDriver3PWM driver = BLDCDriver3PWM(5, 6, 3, 8);

// instantiate the commander
Commander command = Commander(Serial);
void doMotor(char* cmd) { command.motor(&motor, cmd); }

void setup() {
  serialSetup(APP_NAME_STR);
  // enable more verbose output for debugging
  // comment out if not needed
  SimpleFOCDebug::enable(&Serial);
  
  // initialize encoder sensor hardware
  sensor.init();
  // link the motor to the sensor
  motor.linkSensor(&sensor);

  // driver config
  // power supply voltage [V]
  driver.voltage_limit = 12;
  // driver init
  if(!driver.init()){
    Serial.println("Driver init failed!");
    return;
  }
  // link driver
  motor.linkDriver(&driver);

  // aligning voltage
  motor.voltage_sensor_align = 5;

  // set motion control loop to be used
  motor.torque_controller = TorqueControlType::voltage;
  motor.controller = MotionControlType::torque;

  // comment out if not needed
  motor.useMonitoring(Serial);

  // initialize motor
  if(!motor.init()){
    Serial.println("Motor init failed!");
    return;
  }
  // align sensor and start FOC
  if(!motor.initFOC()){
    Serial.println("FOC init failed!");
    return;
  }

  // set the initial motor target
  motor.target = 2; // Volts 

  // add target command M
  command.add('M', doMotor, "Motor");

  Serial.println(F("Motor ready."));
  Serial.println(F("Set the target using serial terminal and command M:"));
  _delay(1000);
}

void loop() {
  // main FOC algorithm function
  motor.loopFOC();

  // Motion control function
  motor.move();

  // user communication
  command.run();
}