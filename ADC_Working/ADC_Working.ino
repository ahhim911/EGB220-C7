
const char MUXMASK = 0b00011111;
const char MUXMASK_MUX5 = 0b00100000;

void setup() {
  #define SENSOR8 (1<<MUX5)|(0<<MUX4)|(0<<MUX3)|(0<<MUX2)|(0<<MUX1)|(0<<MUX0) //Sets up each 
  #define SENSOR7 (0<<MUX4)|(0<<MUX3)|(0<<MUX2)|(0<<MUX1)|(1<<MUX0)
  #define SENSOR6 (1<<MUX5)|(0<<MUX4)|(0<<MUX3)|(0<<MUX2)|(1<<MUX1)|(0<<MUX0)
  #define SENSOR5 (0<<MUX4)|(0<<MUX3)|(0<<MUX2)|(1<<MUX1)|(1<<MUX0)
  #define SENSOR4 (0<<MUX4)|(0<<MUX3)|(1<<MUX2)|(1<<MUX1)|(1<<MUX0)
  #define SENSOR3 (0<<MUX4)|(0<<MUX3)|(1<<MUX2)|(1<<MUX1)|(0<<MUX0)
  #define SENSOR2 (0<<MUX4)|(0<<MUX3)|(1<<MUX2)|(0<<MUX1)|(1<<MUX0)
  #define SENSOR1 (0<<MUX4)|(0<<MUX3)|(1<<MUX2)|(0<<MUX1)|(0<<MUX0)

  //Set Output Direction for the LED 0,1,2,3
  DDRE |= (1<<PE6);
  DDRB |= (1<<PB0)|(1<<PB1)|(1<<PB2);


}
uint8_t sensorOutput[8]; //ADC sensor value array
void setupButton()
{
  DDRC &= ~((1<<PC6) | (1<<PC7));
  PORTB &= ~((1<<PC6) | (1<<PC7));
}
void setupMotors()
{
  DDRB |= (1<<PB7);
  DDRD |= (1<<PD0);
  
	//Timer 1 settings (8-bit fast pwm, ignore on compare match, clear at top, 256 prescaler)
	TCCR1A |= (1<<WGM10)|(1<<COM1A1)|(1<<COM1B1);
	TCCR1B |= (1<<WGM12)|(1<<CS12); 
	//Timer 1 interrupt enable
	TIMSK1 |= (1<<0);
	//Timer 0 settings (8-bit fast pwm, ignore on compare match, clear at top, 256 prescaler) 
  
  TCCR0A = (1<<COM0A1) | (0<<COM0A0) | (1<<COM0B1) | (0<<COM0B0) | (1<<WGM01) | (1<<WGM00);
  TCCR0B = (0<<WGM02) | (1<<CS02) | (0<<CS01) | (1<<CS00);
	//Timer 0 interrupt enable
	TIMSK0 |= (1<<0);
	//Max PWM cycle
	OCR0A = 0;
	OCR0B = 0;
}
void setupADC()
{
	//Using internal 2.56V reference, left adjusted
	ADMUX |= (1<<REFS1)|(1<<REFS0)|(1<<ADLAR); 
	//Enabling ADC, 128 prescaler, no auto-triggering (1<<ADATE to auto trigger) (1<<ADIE for conversion complete interrupt enable)
	ADCSRA |= (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);
	//Free running mode
	ADCSRB = 0; 
}
void getSensorReading()
{	
	///When changing number of sensors - change length of for loop and size of global sensorOutput[]
	int sens_num;
	for (sens_num = 0; sens_num < 4; sens_num++)
	{
		//Clearing current ADC
		// ADMUX |= (0<<MUX4)|(0<<MUX3)|(0<<MUX2)|(0<<MUX1)|(0<<MUX0);
    ADMUX = (ADMUX & ~MUXMASK);
    ADCSRB = (ADCSRB & ~MUXMASK_MUX5);
		
		//Cycling through each sensor
		if (sens_num == 0) {ADMUX |= SENSOR1; ADCSRB |= SENSOR1;}//Does not work for sensors that require MUX5 set high (some problem with clearing this bit..?)
		if (sens_num == 1) {ADMUX |= SENSOR2; ADCSRB |= SENSOR2;}
		if (sens_num == 2) {ADMUX |= SENSOR3; ADCSRB |= SENSOR3;}
		if (sens_num == 3) {ADMUX |= SENSOR4; ADCSRB |= SENSOR4;}
		
		//Starting conversion
		ADCSRA |= (1<<ADSC);
		//Waiting till conversion finished
		while(~ADCSRA&(1<<ADIF)){}
		//Setting sensed value in array
		sensorOutput[sens_num] = ADCH;
	}
}
void loop() {
  // put your main code here, to run repeatedly:
  setupADC();
  setupMotors();
  setupButton();
	///sensorOutput[] tests (//higher when over black(low reflectance) //smaller when over white(high reflectance))
	//From 0.5 cm above line
	//Anything smaller than 30-40 is white (Untested)
	//Anything above 30-40 is grey to black (Untested)
  int sensitivity = 100;
  const int baseSpeed = 60; // Base speed for both motors
  const int maxSpeed = 100; // Maximum speed
  int error = 0; // Difference in reflectance readings between left and right sensors
  float ts = 80.0; // Modify ts to adjust turning sharpness

  int testMode = 1;	// Set test mode to print the sensor reading on Serial Monitor
  int mode = 0;	// mode 0 is line following mode ; mode 1 is move forward
  
	while(1)
	{
		getSensorReading();
		if ( PINC & (1<<PC7) )
    {
      mode = 1; //move forward
      OCR0A = 100;
      OCR0B = 100;
    } else if  ( PINC & (1<<PC6) ){
      mode = 0;
    }
    if ( mode == 1)
    {
      PORTB |= (1<<2); // Turn on LED3
      PORTB &= ~(1<<1); // Turn off LED2
    } else if ( mode == 0 ) {
      PORTB |= (1<<1); // Turn on LED2
      PORTB &= ~(1<<2); // Turn off LED3
      //Testing 
      if ( testMode == 1 )
      {
        Serial.print("S1  S2  S3  S4  \n");
        //Active mode

        int sensor_0 = (sensorOutput[0] - 0);
        int sensor_1 = (sensorOutput[1] - 0);
        int sensor_2 = (sensorOutput[2] - 0);
        int sensor_3 = (sensorOutput[3] - 0);
        Serial.print(sensor_3); Serial.print(" ");
        Serial.print(sensor_2); Serial.print(" ");
        Serial.print(sensor_1); Serial.print(" ");
        Serial.print(sensor_0); Serial.print(" ");
        Serial.println();
      }
      //P Control 0
      float position = (sensorOutput[0] * -2) + (sensorOutput[1] * -0.9) + (sensorOutput[2] * 0.9) + (sensorOutput[3] * 2);
      Serial.print(position);
      Serial.println();
      int controlSignal = (int)(position * 0.1); // Proportional gain of 1.0, adjust as needed
    
      // Adjust motor speeds based on control signal
      int leftMotorSpeed = baseSpeed + controlSignal;
      int rightMotorSpeed = baseSpeed - controlSignal;

      // Constrain speeds to allowable range
      leftMotorSpeed = constrain(leftMotorSpeed, 0, maxSpeed);
      rightMotorSpeed = constrain(rightMotorSpeed, 0, maxSpeed);
    

      /*
      // P Control 1
      int leftSensorAverage = (sensorOutput[0] * 5 + sensorOutput[1]) / 6;
      int rightSensorAverage = (sensorOutput[2] + sensorOutput[3] * 5) / 6;
      // Calculate ratio
      float leftRatio = leftSensorAverage / rightSensorAverage;
      float rightRatio = rightSensorAverage / leftSensorAverage;

      int leftMotorSpeed = baseSpeed - (leftRatio * ts);
      int rightMotorSpeed = baseSpeed - (rightRatio * ts);
      leftMotorSpeed = constrain(leftMotorSpeed, 0, 255);
      rightMotorSpeed = constrain(rightMotorSpeed, 0, 255);
      
      //Serial.print("left:"); Serial.print(leftMotorSpeed);
      //Serial.print("  right:"); Serial.print(rightMotorSpeed); Serial.println();
      */

      OCR0A = leftMotorSpeed; //left motor
      OCR0B = rightMotorSpeed; //right motor
      
      delay(10);
    }
} 
}
