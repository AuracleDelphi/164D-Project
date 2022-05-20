#include <Arduino.h> 
#include <Wire.h>
#include <Adafruit_SSD1306.h>

// setup
// functions
// loop that runs functions
#define MEAS A0
#define REF A1
#define SOUND 10
#define SOUNDGND 9
#define BUTTON 8
double refVoltage = 0;
double measVoltage = 0;
int bpm = 0;
int t = 0;
int temp = 0;

// OLED Stuff
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels
#define OLED_RESET    -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET); //Declaring the OLED name

void setup() {
  // put your setup code here, to run once:
  pinMode(BUTTON, INPUT);
  pinMode(SOUND,OUTPUT);
  pinMode(SOUNDGND, OUTPUT);
  pinMode(BUTTON, INPUT);
  analogReference(INTERNAL); // Set ADC to 1.1V internal reference
}

void buzz(int dcycle, int bzzlen){
  // buzz the buzzer
  // dcycle between 0 and 1
  t = millis();
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

bool buttonPress(){
  // Button pulled high, push brings low
  if (digitalRead(BUTTON) == LOW){
    return true;
  }
  return false;
}

void oledDisplay(String msg){
  // I'll finish copying this over from class code soon
}

double getBPM(){
  // I'll finish copying this over from class code soon
  bpm = 0;
  return bpm;
}

void loop() {
  // put your main code here, to run repeatedly:
  if(buttonPress()){
    //read ambient adc voltage
    refVoltage = getADCVoltage(REF);
    //read measurement adc voltage
    measVoltage = getADCVoltage(MEAS);
    //process these into temperatures
    // TODO: SOME ACTUAL FORMULAS
    temp = measVoltage * 10000004000
    //display on oled
    oledDisplay("Temp: " + String(temp))
    //send over bluetooth
  }
}