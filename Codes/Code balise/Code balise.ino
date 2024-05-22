#include <avr/sleep.h>
#include <avr/interrupt.h>

//pins
const int switchPin = 3;
const int pinBatMon = 4;


uint8_t adress = 0x12;
uint8_t command = 0x00;

int battery = 0;
int adcReadNb = 0;

void sleep() {

    GIMSK |= _BV(PCIE);                     // Enable Pin Change Interrupts
    PCMSK |= _BV(PCINT3);                   // Use PB3 as interrupt pin
    ADCSRA &= ~_BV(ADEN);                   // ADC off
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);    // replaces above statement

    sleep_enable();                         // Sets the Sleep Enable bit in the MCUCR Register (SE BIT)
    sei();                                  // Enable interrupts
    sleep_cpu();                            // sleep

    cli();                                  // Disable interrupts //wake-ups here
    PCMSK &= ~_BV(PCINT3);                  // Turn off PB3 as interrupt pin
    sleep_disable();                        // Clear SE bit
    ADCSRA |= _BV(ADEN);                    // ADC on

    sei();                                  // Enable interrupts
    } // sleep

ISR(PCINT0_vect) {
    
    }

void sendNEC(){
  //start
  TCCR0A = 0b01010010;
  OCR0A = 13; //pour regler 38khz
  _delay_us(9500);
  TCCR0A = 0b00000000;
  _delay_us(2950);

  //adress
  for (int i = 0; i < 8; i++) {
    if (bitRead(adress, i) == 1) { //1
      TCCR0A = 0b01010010;
      _delay_us(575);
      TCCR0A = 0b00000000;
      _delay_us(1050);
    }
    else {                        //0
      TCCR0A = 0b01010010;
      _delay_us(575);
      TCCR0A = 0b00000000;
      _delay_us(365);
    }
  }
  for (int i = 0; i < 8; i++) { //inverted
    if (bitRead(adress, i) == 0) { //0
      TCCR0A = 0b01010010;
      _delay_us(575);
      TCCR0A = 0b00000000;
      _delay_us(1050);
    }
    else {                        //1
      TCCR0A = 0b01010010;
      _delay_us(575);
      TCCR0A = 0b00000000;
      _delay_us(365);
    }
  }

  //command
  for (int i = 0; i < 8; i++) {
    if (bitRead(command, i) == 1) { //1
      TCCR0A = 0b01010010;
      _delay_us(575);
      TCCR0A = 0b00000000;
      _delay_us(1050);
    }
    else {                        //0
      TCCR0A = 0b01010010;
      _delay_us(575);
      TCCR0A = 0b00000000;
      _delay_us(365);
    }
  }
  for (int i = 0; i < 8; i++) { //inverted
    if (bitRead(command, i) == 0) { //0
      TCCR0A = 0b01010010;
      _delay_us(575);
      TCCR0A = 0b00000000;
      _delay_us(1050);
    }
    else {                        //1
      TCCR0A = 0b01010010;
      _delay_us(575);
      TCCR0A = 0b00000000;
      _delay_us(365);
    }
  }

  //stop
  TCCR0A = 0b01010010;
  _delay_us(600);
  TCCR0A = 0b00000000;
  _delay_us(1000);
}

/*appeler juste sur chg de pile*/
void setup() {
  //wakeup piŉ
  pinMode(switchPin, INPUT);
  digitalWrite(switchPin, HIGH);
  //digitalWrite(switchPin, HIGH);
  pinMode(pinBatMon, INPUT);
  
  //PWM 38KHZ on PB0
  TCCR0B = TCCR0B & 0b11111000 | 0b001; //set timer prescaler
  //TCCR0A = 0b01010010; //changer le counter
  //OCR0A = 13; //pour regler 38khz
  DDRB = DDRB & 0b11111110 | 0b00000001; //set pin 0 as output

  //ADC battery
  battery = analogRead(pinBatMon) - 568; //0 à 398 parce que batterie est tjrs plus que 1.8V
  command = ((float)battery / 456) * 255;
  adcReadNb++;

  sendNEC(); //send on battery change
}

void loop() {
  sleep();
  _delay_ms(600); //on attend que RX est fini
  sendNEC();
  _delay_ms(25);
  sendNEC();
  
  //ADC battery
  if(adcReadNb%5 == 0){ // au 5 fois on lit la batterie
  battery = analogRead(pinBatMon) - 568; //0 à 398 parce que batterie est tjrs plus que 1.8V
  command = ((float)battery / 456) * 255;
  }
  adcReadNb++;

}
