#include <Arduino.h>

// put function declarations here:
int RGB_LED(int R, int G, int B);

// RGB LED Color
#define BLUE LOW, HIGH, HIGH
#define RED HIGH, LOW, HIGH
#define GREEN HIGH, HIGH, LOW
#define WHITE LOW, LOW, LOW
#define OFF HIGH, HIGH, HIGH

void setup() {
  // put your setup code here, to run once:
  pinMode(PIN_D0, OUTPUT); // LED4B
  pinMode(PIN_D1, OUTPUT); // LED4R
  pinMode(PIN_D2, OUTPUT); // LED4G
}

void loop() {
  // put your main code here, to run repeatedly:
  // digitalWrite(5, LOW);
  // digitalWrite(6, HIGH);
  // digitalWrite(7, HIGH);
  RGB_LED(OFF);
}

// put function definitions here:
int RGB_LED(int R, int G, int B){
  digitalWrite(PIN_D0, R);
  digitalWrite(PIN_D1, G);
  digitalWrite(PIN_D2, B);
  // Options
  // - BLUE 
  // - RED 
  // - GREEN
  // - WHITE 
  // - OFF 
}