void setup() {
  // put your setup code here, to run once:
  #define SENSOR8 (0<<MUX4)|(0<<MUX3)|(0<<MUX2)|(0<<MUX1)|(0<<MUX0)
  #define SENSOR7 (0<<MUX4)|(0<<MUX3)|(0<<MUX2)|(0<<MUX1)|(1<<MUX0)
  #define SENSOR6 (0<<MUX4)|(0<<MUX3)|(0<<MUX2)|(1<<MUX1)|(0<<MUX0)
  #define SENSOR5 (0<<MUX4)|(0<<MUX3)|(0<<MUX2)|(1<<MUX1)|(1<<MUX0)
  #define SENSOR4 (0<<MUX4)|(0<<MUX3)|(1<<MUX2)|(1<<MUX1)|(1<<MUX0)
  #define SENSOR3 (0<<MUX4)|(0<<MUX3)|(1<<MUX2)|(1<<MUX1)|(0<<MUX0)
  #define SENSOR2 (0<<MUX4)|(0<<MUX3)|(1<<MUX2)|(0<<MUX1)|(1<<MUX0)
  #define SENSOR1 (0<<MUX4)|(0<<MUX3)|(1<<MUX2)|(0<<MUX1)|(0<<MUX0)

}
uint8_t sensorOutput[8]; //ADC sensor value array
void setupMotors()
{
	//Timer 1 settings (8-bit fast pwm, ignore on compare match, clear at top, 256 prescaler)
	TCCR1A |= (1<<WGM10)|(1<<COM1A1)|(1<<COM1B1);
	TCCR1B |= (1<<WGM12)|(1<<CS12); 
	//Timer 1 interrupt enable
	TIMSK1 |= (1<<0);
	
	//Motor 2 output pins
	DDRD |= (1<<PD0);
	DDRB |= (1<<PB7);
	//Timer 0 settings (8-bit fast pwm, ignore on compare match, clear at top, 256 prescaler)
	TCCR0A |= (1<<COM0A1)|(1<<COM0B1)|(1<<WGM01)|(1<<WGM00);
	TCCR0B |= (1<<CS02);
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
		ADMUX |= (0<<MUX4)|(0<<MUX3)|(0<<MUX2)|(0<<MUX1)|(0<<MUX0);
		
		//Cycling through each sensor
		if (sens_num == 0) {ADMUX |= SENSOR1; ADCSRB |= (0<<MUX5);}//Does not work for sensors that require MUX5 set high (some problem with clearing this bit..?)
		if (sens_num == 1) {ADMUX |= SENSOR2; ADCSRB |= (0<<MUX5);}
		if (sens_num == 2) {ADMUX |= SENSOR3; ADCSRB |= (0<<MUX5);}
		if (sens_num == 3) {ADMUX |= SENSOR4; ADCSRB |= (0<<MUX5);}
		
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
	///sensorOutput[] tests (//higher when over black(low reflectance) //smaller when over white(high reflectance))
	//From 0.5 cm above line
	//Anything smaller than 30-40 is white (high reflectance)
	//Anything above 30-40 is grey to black (low reflectance)
		
	while(1)
	{
		getSensorReading();
		//Testing 
		if (sensorOutput[2] < 30)//30 value here needs to be calibrated with actual robot conditions
		{
			Serial.print("f");//Do something when high reflectance
		}
		else
		{
      Serial.print("l");
			//Do something else when low reflectance
		}
}
}
