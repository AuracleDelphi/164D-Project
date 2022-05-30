#include <Arduino.h> 
#include <Wire.h>
#include <Adafruit_SSD1306.h>

// setup
// functions
// loop that runs functions
#define OBJ A0
#define AMB A1
#define SOUND 10
#define SOUNDGND 9
#define BUTTON 8
double ambVoltage = 0;
double objVoltage = 0;
int bpm = 0;
int t = 0;
int temp = 0;

//Constants for temp equations
const double p1 = -0.02613;
const double p2 = 16.55;
const double p2 = 5889;
const double q1 = 44.18;
const double SE = .01;
const double x = 2.50;

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

double getAmbTemp() {
  //Get ambient temperature in units of Celsius
  double ambVoltage = getADCVoltage(AMB);
  double ambTemp = (p1*ambVoltage*ambVoltage + p2*ambVoltage + p3)/(ambVoltage + q1);
  return(ambTemp);
}

double getObjTemp() {
  //Get object temperature in units of Celsius
  double objVoltage = getADCVoltage(OBJ);
  double ambTemp = getAmbTemp();
  double objTemp = pow(((objVoltage/SE)+pow(ambTemp, 4-x)), (1/4-x));
  return(objTemp);
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
}