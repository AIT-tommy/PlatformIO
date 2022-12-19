/*
  Average Thermocouple

  Reads a temperature from a thermocouple based
  on the MAX6675 driver and displays it in the default Serial.

  https://github.com/YuriiSalimov/MAX6675_Thermocouple

  Created by Yurii Salimov, May, 2019.
  Released into the public domain.
*/
#include <Arduino.h>
#include <Thermocouple.h>
#include <MAX6675_Thermocouple.h>

#define SCK_PIN 5
#define CS_PIN 18
#define SO_PIN 19

Thermocouple* thermocouple;

void setup() {
  Serial.begin(9600);

  thermocouple = new MAX6675_Thermocouple(SCK_PIN, CS_PIN, SO_PIN);
}

void loop() {

  float celsius = thermocouple->readCelsius();
  float kelvin = thermocouple->readKelvin();
  float fahrenheit = thermocouple->readFahrenheit();

  Serial.print("Temperature: ");
  Serial.print(celsius);
  Serial.print(" C, ");
  Serial.print(kelvin);
  Serial.print(" K, ");
  Serial.print(fahrenheit);
  Serial.println(" F");

  delay(500); 
}