ISR(ADC_vect){
  uint16_t sensor_value = ADCH;
  //OCR0A = ADCH;
}

void setup() {
  Serial.begin(9600); //Starts the serial monitor
  // put your setup code here, to run once:
  ADMUX = (0<<REFS1) | (1<<REFS0) | (1<<ADLAR) | 0;
  ADCSRA = (1<<ADEN) | (1<<ADATE) | (1<<ADIE) | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0);
  ADCSRB = 0;
  ADCSRA |= (1<<ADSC);
  sei();
}

void motor_setup(){
  DDRB |= (1<<7);
  DDRD |= 1;
  TCCR0A = 0b10100011;
  TCCR0B = 1;
}

void loop() {
  // put your main code here, to run repeatedly:
  motor_setup();
  Serial.print(ADCH);
}
