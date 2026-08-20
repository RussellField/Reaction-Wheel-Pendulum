//Only tests functionality of encoders
//Expects AS5600 connected to I2C bus and AS5047 connected to SPI bus.

#include "serial_setup.h"
#include <SimpleFOC.h>
#include <SPI.h>

SPIClass SPI_3(PC12, PC11, PC10);   // MOSI, MISO, SCK

MagneticSensorSPI sensor_motor = MagneticSensorSPI(AS5047_SPI, PB9);
MagneticSensorSPI sensor_pendulum = MagneticSensorSPI(AS5047_SPI, PB6);

void setup() {
  serialSetup(APP_NAME_STR);
  // enable more verbose output for debugging
  // comment out if not needed
  SimpleFOCDebug::enable(&Serial);
  
  pinMode(PB9, OUTPUT); digitalWrite(PB9, HIGH);
  pinMode(PB6, OUTPUT); digitalWrite(PB6, HIGH);

  SPI.begin();
  SPI_3.begin();

  // initialize encoder sensor hardware
  sensor_motor.clock_speed = 1000000;
  sensor_motor.init(&SPI_3);       // long cable, own bus
  sensor_pendulum.init(&SPI); 

  _delay(1000);
}

void loop() {
  // update the sensor values 
  sensor_motor.update();
  sensor_pendulum.update();
  // display the angle and the angular velocity to the terminal
  Serial.print(F("motor angle: "));
  Serial.print(sensor_motor.getAngle());
  Serial.print(F(" motor velocity: "));
  Serial.print(sensor_motor.getVelocity());
  Serial.print(F(" pendulum angle: "));       
  Serial.println(sensor_pendulum.getAngle());
}