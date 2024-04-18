#include <Arduino.h>

// put function declarations here:
int RGB_LED(int R, int G, int B);

#define BLUE LOW, HIGH, HIGH
#define RED HIGH, LOW, HIGH
#define GREEN HIGH, HIGH, LOW
#define WHITE LOW, LOW, LOW
#define OFF HIGH, HIGH, HIGH

void setup() {
  // put your setup code here, to run once:
  pinMode(5, OUTPUT); // LED4B
  pinMode(6, OUTPUT); // LED4R
  pinMode(7, OUTPUT); // LED4G
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
  digitalWrite(5, R);
  digitalWrite(6, G);
  digitalWrite(7, B);
}