//Only tests functionality of encoders
//Expects AS5600 connected to I2C bus and AS5047 connected to SPI bus.

#include "serial_setup.h"
#include <SimpleFOC.h>

MagneticSensorI2C sensor5600 = MagneticSensorI2C(AS5600_I2C);
MagneticSensorSPI sensor5047 = MagneticSensorSPI(10, 14, 0x3FFF);

void setup() {
  serialSetup(APP_NAME_STR);
  // enable more verbose output for debugging
  // comment out if not needed
  SimpleFOCDebug::enable(&Serial);
  
  // initialize encoder sensor hardware
  sensor5600.init();
  sensor5047.init();

  _delay(1000);
}

void loop() {
  // update the sensor values 
  sensor5600.update();
  sensor5047.update();
  // display the angle and the angular velocity to the terminal
  Serial.print(F("AS5600 angle: "));
  Serial.print(sensor5600.getAngle());
  Serial.print(F(" AS5600 velocity: "));
  Serial.print(sensor5600.getVelocity());
  Serial.print(F(" AS5047 angle: "));       
  Serial.println(sensor5047.getAngle());
}