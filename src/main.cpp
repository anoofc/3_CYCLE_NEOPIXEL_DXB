#define DEBUG         1

#define SENSOR_1      25
#define SENSOR_2      26
#define SENSOR_3      27

#define LED_1         18
#define LED_2         19
#define LED_3         21

#define STRIP1_NUMPIXELS     30
#define STRIP2_NUMPIXELS     30
#define STRIP3_NUMPIXELS     30

#define TIMOUT_1      10000
#define TIMOUT_2      10000
#define TIMOUT_3      10000

#define MULT_FACTOR   10.5

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include "BluetoothSerial.h"

bool    cylcle1_active    = false;
bool    cylcle2_active    = false;
bool    cylcle3_active    = false;

bool    sens1state        = false;
bool    sens2state        = false;
bool    sens3state        = false;

float   distance1         = 0;
float   distance2         = 0;
float   distance3         = 0;

uint8_t   cycleOneCounter        = 0;
uint8_t   cycleTwoCounter        = 0;
uint8_t   cycleThreeCounter      = 0;

uint32_t  cycleOneUpdateMillis   = 0;
uint32_t  cycleTwoUpdateMillis   = 0;
uint32_t  cycleThreeUpdateMillis = 0;

BluetoothSerial SerialBT;

Adafruit_NeoPixel strip1 = Adafruit_NeoPixel(STRIP1_NUMPIXELS, LED_1, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel strip2 = Adafruit_NeoPixel(STRIP2_NUMPIXELS, LED_2, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel strip3 = Adafruit_NeoPixel(STRIP3_NUMPIXELS, LED_3, NEO_GRB + NEO_KHZ800); 


void cycleHandler(){

}

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

void debugPins(){
  Serial.println("S1: " + (String(digitalRead(SENSOR_1))) + "\t S2: " + (String(digitalRead(SENSOR_2))) + "\t S3: " + String((digitalRead(SENSOR_3)))); 
}

void io_init() {
  pinMode(SENSOR_1, INPUT_PULLUP);
  pinMode(SENSOR_2, INPUT_PULLUP);
  pinMode(SENSOR_3, INPUT_PULLUP);

  strip1.begin(); strip1.clear(); strip1.show();
  strip2.begin(); strip2.clear(); strip2.show();
  strip3.begin(); strip3.clear(); strip3.show();
}

void setup() {
  Serial.begin(9600);
  io_init();
}

void loop() {
  debugPins();
  readBTSerial();
  readSerial();
}

