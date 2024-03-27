void setup() {
  // Turn on the Motor Pins
  DDRB |= (1<<PB7);
  DDRD |= (1<<PD0);
  //Timer set up
  TCCR0A = (1<<COM0A1) | (0<<COM0A0) | (1<<COM0B1) | (0<<COM0B0) | (1<<WGM01) | (1<<WGM00);
  TCCR0B = (0<<WGM02) | (1<<CS02) | (0<<CS01) | (1<<CS00);

  Serial.begin(9600); //Starts the serial monitor
}
void move_forward_max(){
  OCR0A = 255;
  OCR0B = 255;
}
void move_forward_25(){
  OCR0A = 0.25*255; //0.25 represents a 25% duty cycle
  OCR0B = 0.25*255;
}
void turn_left(){
  OCR0A = 0.75*255;
  OCR0B = 0.25*255;
}
void turn_right(){
  OCR0A = 0.25*255;
  OCR0B = 0.75*255;
}

void loop() {
  move_forward_25();
  delay(2000);
  turn_left();
  delay(2000);
  turn_right();
  delay(2000);
}

