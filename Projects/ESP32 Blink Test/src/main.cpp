#include <Arduino.h> //using Arduino framework, must always be included
#include "Wire.h" //using Arduino framework, must always be included
#define LED_BUILTIN 27

void setup() {
  // put your setup code here, to run once:
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.print("LED ON");
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);
  Serial.print("LED OFF");
  delay(1000);
}