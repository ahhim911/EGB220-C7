#include <avr/interrupt.h>
//#include <QTRSensors.h>


void setup() {
  // put your setup code here, to run once:
  DDRB |= (1<<PB7);
  DDRD |= (1<<PD0);
  TCCR0A = (1<<COM0A1) | (0<<COM0A0) | (1<<COM0B1) | (0<<COM0B0) | (1<<WGM01) | (1<<WGM00);
  
  TCCR0B = (0<<WGM02) | (1<<CS02) | (0<<CS01) | (1<<CS00);
  OCR0A = 0.75*255;
  OCR0B = 0.75*255;

  ADMUX |= (1<<6) |(1<<5);
  ADCSRA |= (1<<7)|(1<<5)|(1<<3)|(1<<2)|(1<<1)|1;
  ADCSRB = 0;
  ADCSRA |= (1 << 6);

  Serial.begin(9600);


  //pinMode()
}

void loop() {

}











