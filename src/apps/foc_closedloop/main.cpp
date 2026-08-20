#include "serial_setup.h"

#include <SimpleFOC.h>

//testing encoders
#include <SPI.h>

const int CS_MOTOR = PB9;
const int CS_PENDULUM = PB6;

MagneticSensorSPI sensor_motor = MagneticSensorSPI(AS5047_SPI, PB9);
MagneticSensorSPI sensor_pendulum = MagneticSensorSPI(AS5047_SPI, PB6);

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
  
  pinMode(CS_MOTOR, OUTPUT); digitalWrite(CS_MOTOR, HIGH);
  pinMode(CS_PENDULUM, OUTPUT); digitalWrite(CS_PENDULUM, HIGH);

  // initialize encoder sensor hardware
  sensor_pendulum.init();
  sensor_motor.init();
  // link the motor to the sensor
  motor.linkSensor(&sensor_motor);

  // driver config
  // power supply voltage [V]
  driver.voltage_limit = 12;
  // driver init
  if(!driver.init()){
    Serial.println("Driver init failed!");
    return;
  }

  // comment out if not needed
  motor.useMonitoring(Serial);

  // link driver
  motor.linkDriver(&driver);

  // aligning voltage
  motor.voltage_sensor_align = 3;

  // set motion control loop to be used
  motor.torque_controller = TorqueControlType::voltage;
  motor.controller = MotionControlType::torque;

  

  // initialize motor
  if(!motor.init()){
    Serial.println("Motor init failed!");
    return;
  }

  Serial.println(motor.voltage_limit);
  Serial.println(motor.voltage_sensor_align);

  // align sensor and start FOC
  if(!motor.initFOC()){
    Serial.println("FOC init failed!");
    return;
  }

  // set the initial motor target
  motor.target = 1.0f; // Volts 

  motor.PID_velocity.P = 0.05f;    // default is 0.5 — start 10x lower
  motor.PID_velocity.I = 1.0f;     // default is 10
  motor.PID_velocity.D = 0.0f;     // leave at zero
  motor.PID_velocity.output_ramp = 200;   // V/s, limits di/dt
  motor.LPF_velocity.Tf = 0.02f;   // default 0.005 — more filtering
  motor.voltage_limit = 3.0f;      // keep low while tuning
  motor.phase_resistance = 3.0f;   // measured
  motor.current_limit = 2.0f;

  // add target command M
  command.add('M', doMotor, "Motor");

  Serial.println(F("Motor ready."));
  Serial.println(F("Enter MC0 for torque control"));
  Serial.println(F("Enter MC1 for velocity control"));
  Serial.println(F("Enter MC2 for angle control"));
  Serial.println(F("Within each mode enter M and an integer value to set the target"));
  _delay(1000);
}

void loop() {
  // main FOC algorithm function
  motor.loopFOC();

  // Motion control function
  motor.move();

  command.run();

  //test control frequency
  //static uint32_t n = 0; static uint32_t t0 = millis();
  //if (++n >= 1000) {
  //Serial.print("loop Hz: "); Serial.println(1000000.0f / (millis() - t0));
  //n = 0; t0 = millis();
  //}
}