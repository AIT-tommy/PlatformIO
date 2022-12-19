#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "FS.h"
#include <SD.h>
#include <SPI.h>
#include <Adafruit_I2CDevice.h>
#include "LIDARLite_v4LED.h"

#define LEDR 25 // RED pin of rgb led is connected to 25 gpio pin
#define LEDG 26 // green pin is connected to 26 gpio
#define LEDB 27 //
#define sdChipSelect 21
LIDARLite_v4LED myLIDAR;
File myFile;

uint16_t BNO055_SAMPLERATE_DELAY_MS = 100; /* Set the delay between fresh samples */
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28); //I2C address
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200); //turn serial port on
  while (!Serial);
  Serial.println();
  Serial.print("Initializing SD card...");
  if (!SD.begin(sdChipSelect)){
    Serial.print("Initialization failed.");
    return;
  }
  Serial.println("Initialization done.");
  Wire.begin(); 
  bno.begin();
  int8_t temp = bno.getTemp(); //compact data type
  bno.setExtCrystalUse(true);
  pinMode(LEDR, OUTPUT);
  pinMode(LEDG, OUTPUT);
  pinMode(LEDB, OUTPUT);
myFile = SD.open("/SDtest.txt", "a"); //append to file
if (myFile) {
  Serial.print("Writing to SD..");
  myFile.print("Acc(x), Acc(y), Acc(z),");
  myFile.print("Gyro(x), Gyro(y), Gyro(z),");
  myFile.print("Mag(x), Mag(y), Mag(z),");
  myFile.print("LinAcc(x), LinAcc(y), LinAcc(z),");
  myFile.print("Eul(x), Eul(y), Eul(z),");
  myFile.print("Grav(x), Grav(y), Grav(z)");
  myFile.println();
  myFile.close();
  Serial.print("File column setup done");
}
if (myLIDAR.begin() == false) {
    Serial.println("Device did not acknowledge! Freezing.");
    while(1);
}
  Serial.println("LIDAR acknowledged!");
delay(500);
}
void loop() {
  // put your main code here, to run repeatedly:
  imu::Vector<3> acc = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER); //accelerometer vector named imu
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE); //gyroscope vector named imu
  imu::Vector<3> mag = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER); //magnetometer vector named mag
  imu::Vector<3> linAcc = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL); //linear accelerometer  vector named mag
  imu::Vector<3> eul = bno.getVector(Adafruit_BNO055::VECTOR_EULER); //magnetometer vector named mag
  imu::Vector<3> grav = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY); //magnetometer vector named mag

  float newDistance; 
  newDistance = myLIDAR.getDistance();


myFile = SD.open("/test2.txt", "a"); //append to file
while(!myFile){
    delay(100);
    digitalWrite(LEDG,LOW);
    Serial.println("File failed to open...");
    return;
} 
/*PRINTING TO THE SERIAL MONITOR*/
  digitalWrite(LEDG, HIGH);
  Serial.print("Distance(m): ");
  Serial.print(newDistance/100); Serial.println();
  Serial.print("Acceleration: ");
  Serial.print(acc.x()); Serial.print(", ");
  Serial.print(acc.y()); Serial.print(", ");
  Serial.print(acc.z()); Serial.println();
  Serial.print("Gyroscope: ");
  Serial.print(gyro.x()); Serial.print(", ");
  Serial.print(gyro.y()); Serial.print(", ");
  Serial.print(gyro.z()); Serial.println();
  Serial.print("Magnetometer: ");
  Serial.print(mag.x()); Serial.print(", ");
  Serial.print(mag.y()); Serial.print(", ");
  Serial.print(mag.z()); Serial.println();
  Serial.print("Linear Acceleration: ");
  Serial.print(linAcc.x()); Serial.print(", ");
  Serial.print(linAcc.y()); Serial.print(", ");
  Serial.print(linAcc.z()); Serial.println();
  Serial.print("Euler: ");
  Serial.print(eul.x()); Serial.print(", ");
  Serial.print(eul.y()); Serial.print(", ");
  Serial.print(eul.z()); Serial.println();
  Serial.print("Gravity: ");
  Serial.print(grav.x()); Serial.print(", ");
  Serial.print(grav.y()); Serial.print(", ");
  Serial.print(grav.z()); Serial.println();
/*
  PRINTING TO THE MICROSD CARD
*/
  myFile.print(newDistance); myFile.print(", ");
  myFile.print(acc.x()); myFile.print(", ");
  myFile.print(acc.y()); myFile.print(", ");
  myFile.print(acc.z()); myFile.print(", ");
  myFile.print(gyro.x()); myFile.print(", ");
  myFile.print(gyro.y()); myFile.print(", ");
  myFile.print(gyro.z()); myFile.print(", ");
  myFile.print(mag.x()); myFile.print(", ");
  myFile.print(mag.y()); myFile.print(", ");
  myFile.print(mag.z()); myFile.print(", ");
  myFile.print(linAcc.x()); myFile.print(", ");
  myFile.print(linAcc.y()); myFile.print(", ");
  myFile.print(linAcc.z()); myFile.print(", ");
  myFile.print(eul.x()); myFile.print(", ");
  myFile.print(eul.y()); myFile.print(", ");
  myFile.print(eul.z()); myFile.print(", ");
  myFile.print(grav.x()); myFile.print(", ");
  myFile.print(grav.y()); myFile.print(", ");
  myFile.print(grav.z());
  myFile.println();
  myFile.close();

delay(BNO055_SAMPLERATE_DELAY_MS);
}