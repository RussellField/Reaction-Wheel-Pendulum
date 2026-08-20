//Only tests functionality of encoders
//Expects AS5600 connected to I2C bus and AS5047 connected to SPI bus.

#include "serial_setup.h"
#include <SimpleFOC.h>

MagneticSensorSPI motorSensor = MagneticSensorSPI(AS5047_SPI, PB9);
MagneticSensorSPI pendulumSensor = MagneticSensorSPI(AS5047_SPI, PB6);

void setup() {
  serialSetup(APP_NAME_STR);
  // enable more verbose output for debugging
  // comment out if not needed
  SimpleFOCDebug::enable(&Serial);
  
  // initialize encoder sensor hardware
  motorSensor.init();
  pendulumSensor.init();

  _delay(1000);
}

void loop() {
  // update the sensor values 
  motorSensor.update();
  pendulumSensor.update();
  // display the angle and the angular velocity to the terminal
  Serial.print(F("motor angle: "));
  Serial.print(motorSensor.getAngle());
  Serial.print(F(" motor velocity: "));
  Serial.print(motorSensor.getVelocity());
  Serial.print(F(" pendulum angle: "));       
  Serial.println(pendulumSensor.getAngle());
}