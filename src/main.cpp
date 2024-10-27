#define DEBUG         1

#define SENSOR_1      18
#define SENSOR_2      19
#define SENSOR_3      21

#define LED_1         27
#define LED_2         26
#define LED_3         25

#define STRIP1_NUMPIXELS      364
#define STRIP2_NUMPIXELS      345      
#define STRIP3_NUMPIXELS      328   

#define STRIP1ENDPIXEL        145
#define STRIP2ENDPIXEL        138
#define STRIP3ENDPIXEL        132

#define TIMOUT_1              15000
#define TIMOUT_2              15000
#define TIMOUT_3              15000

#define RESET_TIMOUT          45000

#define LED_BLINK_DELAY       500

#define RED                   255,  0,    0
#define BLUE                  0,    0,    255
#define PURPLE                255,  0,    255
#define WHITE                 255,  255 , 255

#define SENSOR_1_DETECTED     digitalRead(SENSOR_1) == LOW
#define SENSOR_2_DETECTED     digitalRead(SENSOR_2) == LOW
#define SENSOR_3_DETECTED     digitalRead(SENSOR_3) == LOW

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include "BluetoothSerial.h"

bool      led1Blink              = false;
bool      led2Blink              = false;
bool      led3Blink              = false;

bool      sens1LastState         = false;
bool      sens2LastState         = false;
bool      sens3LastState         = false;

bool      cycleOneActive         = false;  
bool      cycleTwoActive         = false;
bool      cycleThreeActive       = false;

uint16_t  cycleOneCounter        = STRIP1ENDPIXEL - 1;
uint16_t  cycleTwoCounter        = STRIP2ENDPIXEL - 1;
uint16_t  cycleThreeCounter      = STRIP3ENDPIXEL - 1;

uint32_t  cycleOneUpdateMillis   = 0;
uint32_t  cycleTwoUpdateMillis   = 0;
uint32_t  cycleThreeUpdateMillis = 0;

uint32_t  cycleOneResetMillis    = 0;
uint32_t  cycleTwoResetMillis    = 0;
uint32_t  cycleThreeResetMillis  = 0;



BluetoothSerial SerialBT;

Adafruit_NeoPixel strip1 = Adafruit_NeoPixel(STRIP1_NUMPIXELS, LED_1, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel strip2 = Adafruit_NeoPixel(STRIP2_NUMPIXELS, LED_2, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel strip3 = Adafruit_NeoPixel(STRIP3_NUMPIXELS, LED_3, NEO_GRB + NEO_KHZ800); 

int pixelNumber = 320;

void stripPixelNumber(){
  strip1.clear(); strip2.clear(); strip3.clear();
  for (int i = 0; i < pixelNumber; i++) {
    strip1.setPixelColor(i, strip1.Color(255, 0, 0));
    strip2.setPixelColor(i, strip2.Color(0, 255, 0));
    strip3.setPixelColor(i, strip3.Color(0, 0, 255));
  } strip1.show(); strip2.show(); strip3.show();
}

void stripTestRed(){
  for(int i=0; i<STRIP1_NUMPIXELS; i++){
    strip1.setPixelColor(i, strip1.Color(255, 0, 0));
    strip2.setPixelColor(i, strip2.Color(255, 0, 0));
    strip3.setPixelColor(i, strip3.Color(255, 0, 0));
    strip1.show(); strip2.show(); strip3.show();
    delay(1);
  }
}

void stripTestGreen(){
  for(int i=0; i<STRIP1_NUMPIXELS; i++){
    strip1.setPixelColor(i, strip1.Color(0, 255, 0));
    strip2.setPixelColor(i, strip2.Color(0, 255, 0));
    strip3.setPixelColor(i, strip3.Color(0, 255, 0));
    strip1.show(); strip2.show(); strip3.show();
    delay(1);
  }
}
void stripTestBlue(){
  for(int i=0; i<STRIP1_NUMPIXELS; i++){
    strip1.setPixelColor(i, strip1.Color(0, 0, 255));
    strip2.setPixelColor(i, strip2.Color(0, 0, 255));
    strip3.setPixelColor(i, strip3.Color(0, 0, 255));
    strip1.show(); strip2.show(); strip3.show();
    delay(1);
  }
}

void stripTestWhite(){
  for (int i = 0; i < STRIP1_NUMPIXELS; i++) {
    strip1.setPixelColor(i, WHITE);
    strip2.setPixelColor(i, WHITE);
    strip3.setPixelColor(i, WHITE);
    strip1.show(); strip2.show(); strip3.show();
    delay(1);
  }
}
void stripTest(){
  for(int i=0; i<STRIP1_NUMPIXELS; i++){
    strip1.setPixelColor(i, strip1.Color(255, 0, 0));
    strip2.setPixelColor(i, strip2.Color(255, 0, 0));
    strip3.setPixelColor(i, strip3.Color(255, 0, 0));
    strip1.show(); strip2.show(); strip3.show();
    delay(1);
  }
  for(int i=0; i<STRIP1_NUMPIXELS; i++){
    strip1.setPixelColor(i, strip1.Color(0, 255, 0));
    strip2.setPixelColor(i, strip2.Color(0, 255, 0));
    strip3.setPixelColor(i, strip3.Color(0, 255, 0));
    strip1.show(); strip2.show(); strip3.show();
    delay(1);
  }
  for(int i=0; i<STRIP1_NUMPIXELS; i++){
    strip1.setPixelColor(i, strip1.Color(0, 0, 255));
    strip2.setPixelColor(i, strip2.Color(0, 0, 255));
    strip3.setPixelColor(i, strip3.Color(0, 0, 255));
    strip1.show(); strip2.show(); strip3.show();
    delay(1);
  }
  for (int i = 0; i < STRIP1_NUMPIXELS; i++) {
    strip1.setPixelColor(i, WHITE);
    strip2.setPixelColor(i, WHITE);
    strip3.setPixelColor(i, WHITE);
    strip1.show(); strip2.show(); strip3.show();
    delay(1);
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
        for (uint16_t i = 0; i < STRIP1_NUMPIXELS; i++) {
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
        for (uint16_t i = 0; i < STRIP2_NUMPIXELS; i++) {
          strip2.setPixelColor(i, BLUE);
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
        for (uint16_t i = 0; i < STRIP3_NUMPIXELS; i++) {
          strip3.setPixelColor(i, WHITE);
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
    strip1.setPixelColor(cycleOneCounter, RED);
    strip1.show();
    cycleOneCounter--;
    cycleOneResetMillis = millis();
    if (cycleOneCounter == 0){
      cycleOneCounter = STRIP1_NUMPIXELS;
      strip1.setPixelColor(cycleOneCounter, RED);
      strip1.show();
    }
    if (cycleOneCounter == STRIP1ENDPIXEL){
      Serial.println("A_0");
      led1Blink = true;
      cycleOneCounter = STRIP1ENDPIXEL-1;
      cycleOneActive = false;
      cycleOneUpdateMillis = millis();
    }
  } 
}

void cycle2Handler(){
  if (millis() - cycleTwoUpdateMillis < TIMOUT_2){ return; }
  if (SENSOR_2_DETECTED && sens2LastState){
    Serial.println("B_1");
    strip2.setPixelColor(cycleTwoCounter, BLUE);
    strip2.show();
    cycleTwoCounter--;
    cycleTwoResetMillis = millis();
    if (cycleTwoCounter == 0){
      cycleTwoCounter = STRIP2_NUMPIXELS;
      strip2.setPixelColor(cycleTwoCounter, BLUE);
      strip2.show();
    }
    if (cycleTwoCounter == STRIP2ENDPIXEL){
      Serial.println("B_0");
      led2Blink = true;
      cycleTwoCounter = STRIP2ENDPIXEL-1;
      cycleTwoActive = false;
      cycleTwoUpdateMillis = millis();
    }
  } 
}

void cycle3Handler(){
  if (millis() - cycleThreeUpdateMillis < TIMOUT_3){ return; }
  if (SENSOR_3_DETECTED && sens3LastState){
    Serial.println("C_1");
    strip3.setPixelColor(cycleThreeCounter, WHITE);
    strip3.show();
    cycleThreeCounter--;
    cycleThreeResetMillis = millis();
    if (cycleThreeCounter == 0){
      cycleThreeCounter = STRIP3_NUMPIXELS;
      strip3.setPixelColor(cycleThreeCounter, WHITE);
      strip3.show();
    }
    if (cycleThreeCounter == STRIP3ENDPIXEL){
      Serial.println("C_0");
      led3Blink = true;
      cycleThreeCounter = STRIP3ENDPIXEL-1;
      cycleThreeActive = false;
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
    cycleOneResetMillis = millis();
    cycleOneActive = true;
  } else if (data == 'B'){
    cycleTwoResetMillis = millis();
    cycleTwoActive = true;
  } else if (data == 'C'){
    cycleThreeResetMillis = millis();
    cycleThreeActive = true;
  } else if (data == 'Z'){
    strip1.clear(); strip1.show();
    strip2.clear(); strip2.show();
    strip3.clear(); strip3.show();
    cycleOneCounter   = STRIP1ENDPIXEL-1;  cycleOneUpdateMillis   = millis();  Serial.println("A_0");
    cycleTwoCounter   = STRIP2ENDPIXEL-1;  cycleTwoUpdateMillis   = millis();  Serial.println("B_0");
    cycleThreeCounter = STRIP3ENDPIXEL-1;  cycleThreeUpdateMillis = millis();  Serial.println("C_0");
  }
  else if (data == 'T'){
    stripTest();
  } else if (data == '1'){
    stripTestRed();
  } else if (data == '2'){
    stripTestGreen();
  } else if (data == '3'){
    stripTestBlue();
  } else if (data == '4'){
    stripTestWhite();
  } else if (data == '+'){
    pixelNumber++;
    SerialBT.println(pixelNumber);
    stripPixelNumber();
  } else if (data == '-'){
    pixelNumber--;
    SerialBT.println(pixelNumber);
    stripPixelNumber();
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
    if (DEBUG){ Serial.println  (incoming); }
    if (DEBUG){ SerialBT.println(incoming); }
    processData(incoming);
  }
}

void debugPins(){
  Serial.println("S1: " + (String(digitalRead(SENSOR_1))) + "\t S2: " + (String(digitalRead(SENSOR_2))) + "\t S3: " + String((digitalRead(SENSOR_3)))); 
}

void resetCycles(){
  if (millis() - cycleOneResetMillis > RESET_TIMOUT && cycleOneActive){
    cycleOneCounter = STRIP1ENDPIXEL - 1;
    cycleOneActive = false;
    strip1.clear(); strip1.show();
    Serial.println("A_0");
  }
  if (millis() - cycleTwoResetMillis > RESET_TIMOUT && cycleTwoActive){
    cycleTwoCounter = STRIP2ENDPIXEL - 1;
    cycleTwoActive = false;
    strip2.clear(); strip2.show();
    Serial.println("B_0");
  }
  if (millis() - cycleThreeResetMillis > RESET_TIMOUT && cycleThreeActive){
    cycleThreeCounter = STRIP3ENDPIXEL - 1;
    cycleThreeActive = false;
    strip3.clear(); strip3.show();
    Serial.println("C_0");
  }
}

void io_init() {
  pinMode(SENSOR_1, INPUT_PULLUP);
  pinMode(SENSOR_2, INPUT_PULLUP);
  pinMode(SENSOR_3, INPUT_PULLUP);

  strip1.begin(); strip1.clear(); strip3.setBrightness(200); strip1.show();
  strip2.begin(); strip2.clear(); strip3.setBrightness(200); strip2.show();
  strip3.begin(); strip3.clear(); strip3.setBrightness(128); strip3.show();
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

  if (cycleOneActive)   { cycle1Handler(); }
  if (cycleTwoActive)   { cycle2Handler(); }
  if (cycleThreeActive) { cycle3Handler(); }

  updateSensorState();
  resetCycles();
  readBTSerial();
  readSerial();
}

