#include <Arduino.h> 

// setup
// functions
// loop that runs functions
#define MEAS A0
#define REF A1
#define SOUND 10
#define SOUNDGND 9
#define BUTTON 8



void setup() {
  // put your setup code here, to run once:
  pinMode(BUTTON, INPUT);
  pinMode(SOUND,OUTPUT);
  pinMode(SOUNDGND, OUTPUT);
  analogReference(INTERNAL); // Set ADC to 1.1V internal reference
}

void buzz(int dcycle, int bzzlen){
  // buzz the buzzer
  // dcycle between 0 and 1
  int t = millis();
  while(millis() - t < bzzlen){
    analogWrite(SOUNDGND, 0);
    analogWrite(SOUND, dcycle);
  }
  analogWrite(SOUND, 0);
}

// https://www.intorobotics.com/how-to-make-accurate-adc-readings-with-arduino/?msclkid=03c3eb10d08d11ec88ef55df2fef17e3
double getADCVoltage(int pin){
  // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA,ADSC));
  long result = ADCL;
  result |= ADCH<<8;
  result = 1125300L / result; // Back-calculate AVcc in mV
  double vcc = result/1000.0;
  unsigned int ADCValue = analogRead(pin);
  double voltage  = (ADCValue / 1024.0) * vcc;
  return voltage;
}

void loop() {
  // put your main code here, to run repeatedly:
}