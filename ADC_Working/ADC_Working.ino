
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
  //DDRD = (0<<5) | (0<<8);
  //DDRF = (0<<5) | (0<<6);
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
		if (sens_num == 0) {ADMUX |= SENSOR2; ADCSRB |= SENSOR2;}//Does not work for sensors that require MUX5 set high (some problem with clearing this bit..?)
		if (sens_num == 1) {ADMUX |= SENSOR4; ADCSRB |= SENSOR4;}
		if (sens_num == 2) {ADMUX |= SENSOR6; ADCSRB |= SENSOR6;}
		if (sens_num == 3) {ADMUX |= SENSOR8; ADCSRB |= SENSOR8;}
		
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
	//Anything smaller than 30-40 is white (Untested)
	//Anything above 30-40 is grey to black (Untested)
		
	while(1)
	{
		getSensorReading();
		//Testing 
    Serial.print("S1  S2  S3  S4  \n");
    //Active mode
    int sersor_0 = sensorOutput[0];
    int sersor_1 = sensorOutput[1];
    int sersor_2 = sensorOutput[2];
    int sersor_3 = sensorOutput[3];
    Serial.print(sersor_0); Serial.print(" ");
    Serial.print(sersor_1); Serial.print(" ");
    Serial.print(sersor_2); Serial.print(" ");
    Serial.print(sersor_3); Serial.println();

    // Switch mode
    /*
		if (sensorOutput[0] < 30)//30 value here needs to be calibrated with actual robot conditions
		{
			Serial.print("f ");//Do something when high reflectance
		}
		else
		{
      Serial.print("l ");
			//Do something else when low reflectance
		}
		if (sensorOutput[1] < 30)//30 value here needs to be calibrated with actual robot conditions
		{
			Serial.print("f ");//Do something when high reflectance
		}
		else
		{
      Serial.print("l ");
			//Do something else when low reflectance
		}
		if (sensorOutput[2] < 30)//30 value here needs to be calibrated with actual robot conditions
		{
			Serial.print("f ");//Do something when high reflectance
		}
		else
		{
      Serial.print("l ");
			//Do something else when low reflectance
		}
		if (sensorOutput[3] < 30)//30 value here needs to be calibrated with actual robot conditions
		{
			Serial.print("f \n");//Do something when high reflectance
		}
		else
		{
      Serial.print("l \n");
			//Do something else when low reflectance
		}
    */
}
}
