#define DEBUG         1

#define SENSOR_1      18
#define SENSOR_2      19
#define SENSOR_3      21

#define LED_1         27
#define LED_2         26
#define LED_3         25

#define STRIP1_NUMPIXELS     70
#define STRIP2_NUMPIXELS     70
#define STRIP3_NUMPIXELS     70

#define TIMOUT_1             15000
#define TIMOUT_2             15000
#define TIMOUT_3             15000

#define MULT_FACTOR   10.5

#define LED_BLINK_DELAY  500

#define RED           255,  0,    0
#define BLUE          0,    0,    255
#define PURPLE        255,  0,    255

#define SENSOR_1_DETECTED  digitalRead(SENSOR_1) == LOW
#define SENSOR_2_DETECTED  digitalRead(SENSOR_2) == LOW
#define SENSOR_3_DETECTED  digitalRead(SENSOR_3) == LOW

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include "BluetoothSerial.h"

bool      cylcle1_active         = false;
bool      cylcle2_active         = false;
bool      cylcle3_active         = false;

bool      led1Blink              = false;
bool      led2Blink              = false;
bool      led3Blink              = false;

bool      sens1LastState         = false;
bool      sens2LastState         = false;
bool      sens3LastState         = false;

uint8_t   cycleOneCounter        = 0;
uint8_t   cycleTwoCounter        = 0;
uint8_t   cycleThreeCounter      = 0;

uint32_t  cycleOneUpdateMillis   = 0;
uint32_t  cycleTwoUpdateMillis   = 0;
uint32_t  cycleThreeUpdateMillis = 0;

uint32_t  cycleOneTimer          = 0;
uint32_t  cycleTwoTimer          = 0;
uint32_t  cycleThreeTimer        = 0;


BluetoothSerial SerialBT;

Adafruit_NeoPixel strip1 = Adafruit_NeoPixel(STRIP1_NUMPIXELS, LED_1, NEO_BRG + NEO_KHZ800);
Adafruit_NeoPixel strip2 = Adafruit_NeoPixel(STRIP2_NUMPIXELS, LED_2, NEO_BRG + NEO_KHZ800);
Adafruit_NeoPixel strip3 = Adafruit_NeoPixel(STRIP3_NUMPIXELS, LED_3, NEO_BRG + NEO_KHZ800); 

void stripTest(){
  for(int i=0; i<STRIP1_NUMPIXELS; i++){
    strip1.setPixelColor(i, strip1.Color(255, 0, 0));
    strip2.setPixelColor(i, strip2.Color(255, 0, 0));
    strip3.setPixelColor(i, strip3.Color(255, 0, 0));
    strip1.show(); strip2.show(); strip3.show();
    delay(50);
  }
  for(int i=0; i<STRIP1_NUMPIXELS; i++){
    strip1.setPixelColor(i, strip1.Color(0, 255, 0));
    strip2.setPixelColor(i, strip2.Color(0, 255, 0));
    strip3.setPixelColor(i, strip3.Color(0, 255, 0));
    strip1.show(); strip2.show(); strip3.show();
    delay(50);
  }
  for(int i=0; i<STRIP1_NUMPIXELS; i++){
    strip1.setPixelColor(i, strip1.Color(0, 0, 255));
    strip2.setPixelColor(i, strip2.Color(0, 0, 255));
    strip3.setPixelColor(i, strip3.Color(0, 0, 255));
    strip1.show(); strip2.show(); strip3.show();
    delay(50);
  }
}

void blinkLED1() {
  static uint8_t blinkCount = 0;
  static uint32_t lastMillis = 0;
  static bool ledState = false;

  if (led1Blink) {
    uint32_t currentMillis = millis();
    if (currentMillis - lastMillis >= LED_BLINK_DELAY) {
      lastMillis = currentMillis;
      ledState = !ledState;

      if (ledState) {
        for (uint8_t i = 0; i < STRIP1_NUMPIXELS; i++) {
          strip1.setPixelColor(i, RED);
        }
      } else {
        strip1.clear();
        blinkCount++;
      }
      strip1.show();

      if (blinkCount >= 5) {
        led1Blink = false; // Reset the flag after blinking
        blinkCount = 0;
      }
    }
  }
}

void blinkLED2() {
  static uint8_t blinkCount = 0;
  static uint32_t lastMillis = 0;
  static bool ledState = false;

  if (led2Blink) {
    uint32_t currentMillis = millis();
    if (currentMillis - lastMillis >= LED_BLINK_DELAY) {
      lastMillis = currentMillis;
      ledState = !ledState;

      if (ledState) {
        for (uint8_t i = 0; i < STRIP2_NUMPIXELS; i++) {
          strip2.setPixelColor(i, RED);
        }
      } else {
        strip2.clear();
        blinkCount++;
      }
      strip2.show();

      if (blinkCount >= 5) {
        led2Blink = false; // Reset the flag after blinking
        blinkCount = 0;
      }
    }
  }
}

void blinkLED3() {
  static uint8_t blinkCount = 0;
  static uint32_t lastMillis = 0;
  static bool ledState = false;

  if (led3Blink) {
    uint32_t currentMillis = millis();
    if (currentMillis - lastMillis >= LED_BLINK_DELAY) {
      lastMillis = currentMillis;
      ledState = !ledState;

      if (ledState) {
        for (uint8_t i = 0; i < STRIP3_NUMPIXELS; i++) {
          strip3.setPixelColor(i, RED);
        }
      } else {
        strip3.clear();
        blinkCount++;
      }
      strip3.show();

      if (blinkCount >= 5) {
        led3Blink = false; // Reset the flag after blinking
        blinkCount = 0;
      }
    }
  }
}

void cycle1Handler(){
  if (millis() - cycleOneUpdateMillis < TIMOUT_1){ return; }
  if (SENSOR_1_DETECTED && sens1LastState){
    Serial.println("A_1");
    for (int i=0; i<cycleOneCounter; i++){
      strip1.setPixelColor(i, RED);
    }
    strip1.show();
    cycleOneCounter++;
    if (cycleOneCounter == STRIP1_NUMPIXELS){
      Serial.println("A_0");
      cycleOneCounter = 0;
      led1Blink = true;
      cycleOneUpdateMillis = millis();
    }
  } 
}

void cycle2Handler(){
  if (millis() - cycleTwoUpdateMillis < TIMOUT_2){ return; }
  if (SENSOR_2_DETECTED && sens2LastState){
    Serial.println("B_1");
    for (int i=0; i<cycleTwoCounter; i++){
      strip2.setPixelColor(i, BLUE);
    }
    strip2.show();
    cycleTwoCounter++;
    if (cycleTwoCounter == STRIP2_NUMPIXELS){
      Serial.println("B_0");
      cycleTwoCounter = 0;
      led2Blink = true;
      cycleTwoUpdateMillis = millis();
    }
  } 
}

void cycle3Handler(){
  if (millis() - cycleThreeUpdateMillis < TIMOUT_3){ return; }
  if (SENSOR_3_DETECTED && sens3LastState){
    Serial.println("C_1");
    for (int i=0; i<cycleThreeCounter; i++){
      strip3.setPixelColor(i, PURPLE);
    }
    strip3.show();
    cycleThreeCounter++;
    if (cycleThreeCounter == STRIP3_NUMPIXELS){
      Serial.println("C_0");
      cycleThreeCounter = 0;
      led3Blink = true;
      cycleThreeUpdateMillis = millis();
    }
  } 
}

void updateSensorState(){
  sens1LastState = digitalRead(SENSOR_1);
  sens2LastState = digitalRead(SENSOR_2);
  sens3LastState = digitalRead(SENSOR_3);
}

/**
 * CMD A_1 - START A_0 - STOP
 * CMD B_1 - START B_0 - STOP
 * CMD C_1 - START C_0 - STOP
 */


void processData(char data){
  if (data == 'A'){
    led1Blink = true;
  } else if (data == 'B'){
    led2Blink = true;
  } else if (data == 'C'){
    led3Blink = true;
  }
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
  SerialBT.begin("BSL_CYCLE_GAME");
  io_init();
}

void loop() {
  // stripTest();
  // debugPins();
  
  blinkLED1();
  blinkLED2();
  blinkLED3();

  cycle1Handler();
  cycle2Handler();
  cycle3Handler();

  updateSensorState();

  readBTSerial();
  // readSerial();
}

