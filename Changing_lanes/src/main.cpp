#include <Arduino.h>
#include <avr/interrupt.h>
#include <avr/io.h>

// function declarations
int RGB_LED(int R, int G, int B);
int colorsensing();
int colorsensing2();
// void colorSensor();
// int process_red_value();
// int process_green_value();
// int process_blue_value();
// int process_clear_value();


// PIN Definitions
#define LED1_PIN 2
#define LED2_PIN 1
#define LED3_PIN 0
#define LED4B_PIN 3
#define LED4R_PIN 6
#define LED4G_PIN 7
#define LED5_PIN 10
#define CS_S3_PIN 23
#define CS_S2_PIN 11
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
#define S11_PIN 22
#define SW1_PIN 8
#define SW2_PIN 9
#define MT1_PIN 5
#define MT2_PIN 4

// RGB LED Color
#define RED LOW, HIGH, HIGH
#define GREEN HIGH, LOW, HIGH
#define BLUE HIGH, HIGH, LOW
#define WHITE LOW, LOW, LOW
#define OFF HIGH, HIGH, HIGH

// State Machine
typedef enum{
  INIT,
  LINE_FOLLOWING, 
  SLOWZONE,
  OBSTACLE, 
  MARKER,
  ERROR
} State;

// Gloal variables declarations
int colorDetected;
int colorDetected_2;
const int initbaseSpeed = 90; // Base speed for both motors
const int initmaxSpeed = 130; // Maximum speed
int baseSpeed = initbaseSpeed; // Base speed for both motors
int maxSpeed = initmaxSpeed; // Maximum speed
char obstacle;
int obstacleThershold = 170;
int markerThershold = 235;
int errorThreshold = 200;
//int RedThreshold = 180; // Red color threshold on CLEAR value green 150
//int WhiteThreshold = 150; // WHITE color threshold on CLEAR value
int RedThreshold = 230; // Red color threshold on CLEAR value green 150
int WhiteThreshold = 197; // WHITE color threshold on CLEAR value
int GreenThreshold = 230; // Green color threshold on CLEAR value
volatile State state = INIT;
int lap = 0;
int lapCount = 0;
int count = 0;
int makerdetected;
int stoppingMarker = 0;
int stopzoneflag = 0;
int executionflag = 0;

uint8_t pb_sample = 0;

int mode = 0;	// mode 0 is line following mode ; mode 1 is move forward

volatile uint8_t sensorOutput[11]; //ADC sensor value array

void getSensorReading()
{	
	int sens_num;
  int Value;
	for (sens_num = 0; sens_num < 11; sens_num++)
	{
    if (sens_num == 0) Value = analogRead(S1_PIN);
    if (sens_num == 1) Value = analogRead(S2_PIN);
    if (sens_num == 2) Value = analogRead(S3_PIN);
    if (sens_num == 3) Value = analogRead(S4_PIN);
    if (sens_num == 4) Value = analogRead(S5_PIN);
    if (sens_num == 5) Value = analogRead(S6_PIN);
    if (sens_num == 6) Value = analogRead(S7_PIN);
    if (sens_num == 7) Value = analogRead(S8_PIN);
    if (sens_num == 8) Value = analogRead(S9_PIN); // Obstacle
    if (sens_num == 9) Value = analogRead(S10_PIN); // Marker
    if (sens_num == 10) Value = analogRead(S11_PIN); // Color sensor

    sensorOutput[sens_num] = Value>>2; // 10 bit ADC to 8 bit
    // sensorOutput[sens_num] = ((Value<<8)/1000);
  }
}

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
  pinMode(CS_S2_PIN, INPUT); // Color Sensor
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
  TCCR4B = 0b0001001; // WGM43:2 = 10, CS42:0 = 001
  OCR4A = 0; // Set the duty cycle to 0

  // PWM for Motor OC0A MT1
  TCCR1A = 0b10100001; // COM0A1:0 = 10, COM0B1:0 = 10, WGM01:0 = 01
  TCCR1B = 0b00001001; // WGM12 = 1, CS02:0 = 001
  
  // PWM for Motor OC0B MT2
  TCCR0A = 0b10100011; // COM0A1:0 = 10, COM0B1:0 = 10, WGM01:0 = 11
  TCCR0B = 0b00000001; // WGM02 = 0, CS02:0 = 001

  OCR0A = 0; // Set the duty cycle to 0
  OCR0B = 0;

  Serial.begin(9600); // Initialize serial communication at 9600 baud
}


void loop() {
  // put your main code here, to run repeatedly:
  getSensorReading();
  //mode = 0;
  switch (state){
		case INIT: {
      state = LINE_FOLLOWING;
      }
      break;
		case LINE_FOLLOWING: {
      digitalWrite(LED1_PIN, LOW); // Turn on LED1
      RGB_LED(WHITE); 
      maxSpeed = initmaxSpeed;
      // Slow Zone Mode
      if ((sensorOutput[10] < GreenThreshold) && (stopzoneflag == 1))
      {
        colorsensing();
        //stopzoneflag++;
        if ((colorDetected < RedThreshold) && (colorDetected > WhiteThreshold)){
          state = SLOWZONE;
        }
      }
      // Stopping Zone Mode
      if ((sensorOutput[10] < GreenThreshold) && (stopzoneflag == 0)){
        colorsensing();
        if ((colorDetected < WhiteThreshold)){
          stoppingMarker = ~stoppingMarker;
          state = MARKER;
        }
      }
      if(sensorOutput[8] < obstacleThershold){
        obstacle = 'Y';
        state = OBSTACLE; // obstacle mode
        Serial.println("Obstacle Detected");
      }
      break;
    }
    case SLOWZONE:{
      baseSpeed = 45;
      maxSpeed = 70;
      // Serial.println("SLOW ZONE MODE");
      RGB_LED(RED);
      if ( lap == 0 ) //left
      {
        if (count > 1000){
          if (sensorOutput[10] < GreenThreshold)
          {
            colorsensing();
            if ((colorDetected > 100) && (colorDetected < 250)){
              lap = 1;
              lapCount++;
              makerdetected = 0;
              stopzoneflag = 0;
              state = LINE_FOLLOWING;
            }
          }
        }
        if (count < 1000)
        {        
          //sensorOutput[2] = 250;
          sensorOutput[0] = 230;
          sensorOutput[1] = 230;
        }
        if(sensorOutput[8] < obstacleThershold){
          obstacle = 'Y';
          state = OBSTACLE; // obstacle mode
          //Serial.println("Obstacle Detected");
        }
      }
      if (lap == 1) //right
      {
        // Should be able to change to GREEN now
        if (count > 1000){
          if (sensorOutput[10] < GreenThreshold)
          {
            colorsensing();
            if ((colorDetected > 100) && (colorDetected < 250)){
              lap = 1;
              lapCount++;
              makerdetected = 0;
              stopzoneflag = 0;
              state = LINE_FOLLOWING;
            }
          }
        }
        if (count < 1000)
        {
          //sensorOutput[5] = 250;
          sensorOutput[6] = 230;
          sensorOutput[7] = 230;
        }
        if(sensorOutput[8] < obstacleThershold){
          obstacle = 'Y';
          state = OBSTACLE; // obstacle mode
          //Serial.println("Obstacle Detected");
        }
      }
        count++;
    }
    break;
    case OBSTACLE: {
      Serial.println("Enter obstacle state");
      digitalWrite(LED3_PIN, HIGH); //Blue LED
      baseSpeed = 25;
      maxSpeed = 45;
      if(sensorOutput[8] > obstacleThershold){
        digitalWrite(LED3_PIN, LOW); // 
        count++;
        if (count > 1000) {
          baseSpeed = initbaseSpeed;
          maxSpeed = initmaxSpeed;
          count = 0;
          state = LINE_FOLLOWING;
          //Serial.println("Exit obstacle state");
        }
      } else {
        count = 0;
      }
    }
    break;
    case MARKER:{ 
      // Stop zone
      //delay(10000);
      //delay(10000);
      //delay(1000);
      if (stoppingMarker == 0){
        // Need to instead make it slow to a stop
        // Turn on RED led
        delay(10000);
        delay(10000);
        digitalWrite(LED2_PIN,HIGH);
        OCR0A = 0;
        OCR0B = 0;
        delay(200000);
      }
      stopzoneflag = 1;
      state = LINE_FOLLOWING;
    }
    break;
    case ERROR:{
      maxSpeed = 0;
      RGB_LED(OFF);
      OCR4A = 0; // Set the duty cycle to 0%
      digitalWrite(LED1_PIN, HIGH); // Turn off LED1
      digitalWrite(LED2_PIN, LOW); // Turn on LED2
      if((sensorOutput[0] <= errorThreshold) | (sensorOutput[1] <= errorThreshold) | (sensorOutput[2] <= errorThreshold) | (sensorOutput[3] <= errorThreshold) | (sensorOutput[4] <= errorThreshold) | (sensorOutput[5] <= errorThreshold) | (sensorOutput[6] <= errorThreshold) | (sensorOutput[7] <= errorThreshold)){
      lap = 0;
      lapCount = 0;
      count = 0;
      stoppingMarker = 0;
      stopzoneflag = 0;
      delay(100000);
      state = LINE_FOLLOWING; // line following
      }
    }
    break;
		default:
			state = INIT;
	}
  
  //---------Straight Line behavior---------
  if ((sensorOutput[3] + sensorOutput[4]) < 70)
  {                   // Straight
    OCR4A = 32; // Set the duty cycle to 25%
    digitalWrite(LED2_PIN, HIGH); // Turn on LED2
    baseSpeed = initbaseSpeed;
  } else {            // Turning
    OCR4A = 255; // Set the duty cycle to 100%
    digitalWrite(LED2_PIN, LOW); // Turn off LED2
    baseSpeed = initbaseSpeed - 20;
  }
  
  // Error Mode
  if((sensorOutput[0] >= errorThreshold) && (sensorOutput[1] >= errorThreshold) && (sensorOutput[2] >= errorThreshold) && (sensorOutput[3] >= errorThreshold) && (sensorOutput[4] >= errorThreshold) && (sensorOutput[5] >= errorThreshold) && (sensorOutput[6] >= errorThreshold) && (sensorOutput[7] >= errorThreshold)){
  // Stops the motors when the robot detects black
  OCR0A = 0;
  OCR0B = 0;
  state = ERROR; // error mode
  }

      //---------P Control---------
      float position = (sensorOutput[0] * -2) + (sensorOutput[1] * -1.5) + (sensorOutput[2] * -0.9) + (sensorOutput[3] * -0.9)+ (sensorOutput[4] * 0.9) + (sensorOutput[5] * 0.9) + (sensorOutput[6] * 1.5) + (sensorOutput[7] * 2);
      //Serial.print(sensorOutput[3]);
      // Serial.println();
      int controlSignal = (position * 0.1); // Proportional gain of 1.0, adjust as needed
    
      // Adjust motor speeds based on control signal
      int leftMotorSpeed = baseSpeed + controlSignal;
      int rightMotorSpeed = baseSpeed - controlSignal;

      // Constrain speeds to allowable range
      leftMotorSpeed = constrain(leftMotorSpeed, 0, maxSpeed);
      rightMotorSpeed = constrain(rightMotorSpeed, 0, maxSpeed);

      OCR0A = leftMotorSpeed; //left motor
      OCR0B = rightMotorSpeed; //right motor
      
      delay(5);

  // Serial.print("s1: ");
  // Serial.print(sensorOutput[0]);
  // Serial.print(", s2: ");
  // Serial.print(sensorOutput[1]);
  // Serial.print(", s3: ");
  // Serial.print(sensorOutput[2]);
  // Serial.print(", s4: ");
  // Serial.print(sensorOutput[3]);
  // Serial.print(", s5: ");
  // Serial.print(sensorOutput[4]);
  // Serial.print(", s6: ");
  // Serial.print(sensorOutput[5]);
  // Serial.print(", s7: ");
  // Serial.print(sensorOutput[6]);
  // Serial.print(", s8: ");
  // Serial.print(sensorOutput[7]);
  Serial.print(", Obstacle: ");
  Serial.print(sensorOutput[8]);
  Serial.print(", Marker: ");
  Serial.print(sensorOutput[9]);
  Serial.print(", Color: ");
  Serial.print(sensorOutput[10]);
  Serial.println();
}

// put function definitions here:
int RGB_LED(int R, int G, int B){
  digitalWrite(LED4R_PIN, R);
  digitalWrite(LED4G_PIN, G);
  digitalWrite(LED4B_PIN, B);
}


int colorsensing(){
  // Color sensor
  colorDetected = 240; // Initialize the minimum value
  int sampleLastRight = 240; 
  int sampleLastS1 = 240;
  int sampleLastS8 = 240;
  int sampleCount = 500; // Number of samples to take
  //int sampleCount = 1100; // Number of samples to take
  for (int i = 0; i < sampleCount; i++)
  {
    int sampleRight = sensorOutput[10]; // Read the sensor value
    int sampleS1 = sensorOutput[0]; // Read the sensor value
    int sampleS8 = sensorOutput[7]; // Read the sensor value
    if (sampleLastRight > sampleRight) // Find the minimum value
    {
      sampleLastRight = sampleRight;
    }
    if (sampleLastS1 > sampleS1) // Find the minimum value of S1
    {
      sampleLastS1 = sampleS1;
    }
    if (sampleLastS8 > sampleS8) // Find the minimum value of S8
    {
      sampleLastS8 = sampleS8;
    }
    delay(1000);
  }
  // Serial.print("Color Detected: ");
  if ((sampleLastS1 < errorThreshold) && (sampleLastS8 < errorThreshold)) // if White detected on S1 and S8
  {
    colorDetected = 255;
    // Serial.println("Crossline deteted");
  } else {
    colorDetected = sampleLastRight;
    // Serial.println(colorDetected);
  }
  return colorDetected;
}