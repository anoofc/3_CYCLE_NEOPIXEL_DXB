#define DEBUG         1
#define SENSOR_1      25
#define SENSOR_2      26
#define SENSOR_3      27

#define LED_1         18
#define LED_2         19
#define LED_3         21

#define NUMPIXELS     30

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

Adafruit_NeoPixel strip1 = Adafruit_NeoPixel(NUMPIXELS, LED_1, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel strip2 = Adafruit_NeoPixel(NUMPIXELS, LED_2, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel strip3 = Adafruit_NeoPixel(NUMPIXELS, LED_3, NEO_GRB + NEO_KHZ800); 

void processData(char data){
  // TODO: Implement this function
}

void readBTSerial () {
  if (SerialBT.available()) {
    char incoming = SerialBT.read();
    if (DEBUG){ Serial.println(incoming); }
    processData(incoming);
  }
}

void readSerial () {
  if (Serial.available()) {
    char incoming = Serial.read();
    if (DEBUG){ Serial.println(incoming); }
    processData(incoming);
  }
}

void io_init() {
  pinMode(SENSOR_1, INPUT);
  pinMode(SENSOR_2, INPUT);
  pinMode(SENSOR_3, INPUT);

  strip1.begin(); strip1.clear(); strip1.show();
  strip2.begin(); strip2.clear(); strip2.show();
  strip3.begin(); strip3.clear(); strip3.show();
}

void setup() {
  Serial.begin(9600);
  io_init();

}

void loop() {
  readBTSerial();
  readSerial();
}

