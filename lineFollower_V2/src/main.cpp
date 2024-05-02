#include <Arduino.h>
#include <avr/interrupt.h>
#include <avr/io.h>

// function declarations
int RGB_LED(int R, int G, int B);
void colorSensor();
int process_red_value();
int process_green_value();
int process_blue_value();
int process_clear_value();

// Gloal variables declarations
int CS_red, CS_green, CS_blue, CS_clear;

// PIN Definitions
#define LED1_PIN 0
#define LED2_PIN 1
#define LED3_PIN 2
#define LED4B_PIN 5
#define LED4R_PIN 6
#define LED4G_PIN 7
#define LED5_PIN 10
#define CS_S3_PIN 23
#define CS_S2_PIN 22
#define S1_PIN 21
#define S2_PIN 20
#define S3_PIN 19
#define S4_PIN 18
#define S5_PIN 17
#define S6_PIN 16
#define S7_PIN 15
#define S8_PIN 14
#define S9_PIN 13
#define S10_PIN 12
#define S11_PIN 11
#define SW1_PIN 8
#define SW2_PIN 9
#define MT1_PIN 3
#define MT2_PIN 4

// RGB LED Color
#define BLUE LOW, HIGH, HIGH
#define RED HIGH, LOW, HIGH
#define GREEN HIGH, HIGH, LOW
#define WHITE LOW, LOW, LOW
#define OFF HIGH, HIGH, HIGH

void setup() {
  // put your setup code here, to run once:
  pinMode(LED1_PIN, OUTPUT); // LED1
  pinMode(LED2_PIN, OUTPUT); // LED2
  pinMode(LED3_PIN, OUTPUT); // LED3
  pinMode(LED4B_PIN, OUTPUT); // LED4B
  pinMode(LED4R_PIN, OUTPUT); // LED4R
  pinMode(LED4G_PIN, OUTPUT); // LED4G
  pinMode(LED5_PIN, OUTPUT); // LED5 Rear light
  pinMode(CS_S3_PIN, OUTPUT); // Color Sensor S3 Ref. Col 
  pinMode(CS_S2_PIN, OUTPUT); // Color Sensor S2
  pinMode(MT1_PIN, OUTPUT); // Motor 1
  pinMode(MT2_PIN, OUTPUT); // Motor 2
  pinMode(S1_PIN, INPUT); // S1
  pinMode(S2_PIN, INPUT); // S2
  pinMode(S3_PIN, INPUT); // S3
  pinMode(S4_PIN, INPUT); // S4
  pinMode(S5_PIN, INPUT); // S5
  pinMode(S6_PIN, INPUT); // S6
  pinMode(S7_PIN, INPUT); // S7
  pinMode(S8_PIN, INPUT); // S8 
  pinMode(S9_PIN, INPUT); // B4 - S9 - Obstacle sensor
  pinMode(S10_PIN, INPUT); // D7 - S10 - Marker sensor
  pinMode(S11_PIN, INPUT); // D6 - S11 - Color sensor
  pinMode(SW1_PIN, INPUT); // SW1 - Active HIGH
  pinMode(SW2_PIN, INPUT); // SW2 - ACtive HIGH
  
  // PWM for Rear LED, OC4A
  TCCR4A = 0b10000010; // COM4A1:0 = 10, WGM41:0 = 10
  TCCR4B = 0b00011001; // WGM43:2 = 10, CS42:0 = 001
  OCR4A = 0; // Set the duty cycle to 0
  // PWM for Motor OC0A MT1
  
  // PWM for Motor OC0B MT2
  TCCR0A = 0b10100011; // COM0A1:0 = 10, COM0B1:0 = 10, WGM01:0 = 11
  TCCR0B = 0b00000001; // WGM02 = 0, CS02:0 = 001

  OCR0A = 200; // Set the duty cycle to 0

  Serial.begin(9600); // Initialize serial communication at 9600 baud
}


void loop() {
  // put your main code here, to run repeatedly:
  //Check button state
  //int SW1 = digitalRead(PIN_D3);
  //int SW2 = digitalRead(PIN_C6);
  //if (SW1 == 1)
  //{
  //  //RGB_LED(RED);
  //  OCR4A = 0; // Set the duty cycle to 0%
  //}
  //else if (SW2 == 1)
  //{
  //  //RGB_LED(GREEN);
  //  OCR4A = 64; // Set the duty cycle to 50%
  //}
  //else
  //{
  //  //RGB_LED(WHITE);
  //  OCR4A = 255; // Set the duty cycle to 100%
  //}
  
  RGB_LED(OFF);
  digitalWrite(LED5_PIN, LOW); // Rear light
  digitalWrite(PIN_B0, LOW);
  digitalWrite(PIN_B1, LOW);
  digitalWrite(PIN_B2, LOW);
  int s1Value = analogRead(PIN_F0);
  int s2Value = analogRead(PIN_F1);
  int s3Value = analogRead(PIN_F4);
  int s4Value = analogRead(PIN_F5);
  int s5Value = analogRead(PIN_F6);
  int s6Value = analogRead(PIN_F7);
  int s7Value = analogRead(PIN_B6);
  int s8Value = analogRead(PIN_B5);
  CS_red = process_red_value();delay(10);
  CS_green = process_green_value();delay(10);
  CS_blue = process_blue_value();delay(10);
  CS_clear = process_clear_value();
  

  Serial.print("s1: ");
  Serial.print(s1Value);
  Serial.print(", s2: ");
  Serial.print(s2Value);
  Serial.print(", s3: ");
  Serial.print(s3Value);
  Serial.print(", s4: ");
  Serial.print(s4Value);
  Serial.print(", s5: ");
  Serial.print(s5Value);
  Serial.print(", s6: ");
  Serial.print(s6Value);
  Serial.print(", s7: ");
  Serial.print(s7Value);
  Serial.print(", s8: ");
  Serial.print(s8Value);
  Serial.print(", RED: ");
  Serial.print(CS_red);
  Serial.print(", BLUE: ");
  Serial.print(CS_blue);
  Serial.print(", GREEN: ");
  Serial.print(CS_green);
  Serial.print(", CLEAR: ");
  Serial.print(CS_clear);
  Serial.println();

    //Serial.println(s10Value); // Print the value of S10 to the serial monitor
}

// put function definitions here:
int RGB_LED(int R, int G, int B){
  digitalWrite(PIN_D0, R);
  digitalWrite(PIN_D1, G);
  digitalWrite(PIN_D2, B);
  // Options
  // - BLUE - OBSTACLE
  // - RED - STOP
  // - GREEN - STRAIGHT
  // - WHITE 
  // - OFF 
}

// Color sensor readings
  //S2 | S3 | Output 
  // L | L | Red
  // L | H | Blue
  // H | L | Clean
  // H | H | Green
int process_red_value()
{
  digitalWrite(CS_S2_PIN, LOW);
  digitalWrite(CS_S3_PIN, LOW);
  int pulse_length = pulseIn(S11_PIN, LOW);
  return pulse_length;
}
int process_green_value()
{
  digitalWrite(CS_S2_PIN, HIGH);
  digitalWrite(CS_S3_PIN, HIGH);
  int pulse_length = pulseIn(S11_PIN, LOW);
  return pulse_length;
}
int process_blue_value()
{
  digitalWrite(CS_S2_PIN, LOW);
  digitalWrite(CS_S3_PIN, HIGH);
  int pulse_length = pulseIn(S11_PIN, LOW);
  return pulse_length;
}
int process_clear_value()
{
  digitalWrite(CS_S2_PIN, HIGH);
  digitalWrite(CS_S3_PIN, LOW);
  int pulse_length = pulseIn(S11_PIN, LOW);
  return pulse_length;
}
