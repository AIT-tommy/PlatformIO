/*#include <Arduino.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_Sensor.h>
#include "LIDARLite_v4LED.h"
#include <Wire.h>
#include <SPI.h>

LIDARLite_v4LED myLIDAR; //Click here to get the library: http://librarymanager/All#SparkFun_LIDARLitev4 by SparkFun

void setup() {

  Serial.begin(115200);
  Serial.println("Qwiic LIDARLite_v4 examples");
  Wire.begin(); //Join I2C bus

  //check if LIDAR will acknowledge over I2C
  if (myLIDAR.begin() == false) {
    Serial.println("Device did not acknowledge! Freezing.");
    while(1);
  }
  Serial.println("LIDAR acknowledged!");
}

void loop() {
  Serial.println("Testing");
  float newDistance;

  //getDistance() returns the distance reading in cm
  newDistance = myLIDAR.getDistance();

  //Print to Serial port
  Serial.print("New distance: ");
  Serial.print(newDistance/100);
  Serial.println(" m");

  delay(20);  //Don't hammer too hard on the I2C bus
}*/