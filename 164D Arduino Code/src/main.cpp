#include <Arduino.h> 
#include <Wire.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include "SSD1306Ascii.h"
#include "SSD1306AsciiWire.h"
#include <Adafruit_I2CDevice.h> //Might need so oled display libraries compile

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
const double p3 = 5889;
const double q1 = 44.18;
const double seeb = .01;
const double x = 2.50;
const double gain = 47.51;

// OLED Stuff
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels
#define OLED_RESET    -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET); //Declaring the OLED name

void BPM_Display_setup() {  
  Serial.begin(19200); // initialize serial communication at 9600 bits per second 

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C); //Start the OLED display
  display.display();
  delay(3000);
}

void setup() {
  // put your setup code here, to run once:
  BPM_Display_setup();
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(BUTTON, INPUT);
  pinMode(SOUND,OUTPUT);
  pinMode(SOUNDGND, OUTPUT);
  pinMode(BUTTON, INPUT);
  analogReference(INTERNAL); // Set ADC to 1.1V internal reference
}

void oledDisplay(String msg) {
  display.clearDisplay(); 
  display.setTextSize(1);
  display.setTextColor(WHITE); 
  display.setCursor(0,0);  
  display.println(msg);
  display.display();
}

// https://www.intorobotics.com/how-to-make-accurate-adc-readings-with-arduino/?msclkid=03c3eb10d08d11ec88ef55df2fef17e3
double getADCVoltage(int pin){
  analogReference(INTERNAL);
  unsigned int adcVal = analogRead(pin);
  double voltage = adcVal/1024.0 * 1.1 * 1000; // Return voltage in mV
  return voltage;
  /*
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
  */
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
  double objTemp = pow((((objVoltage/gain)/seeb)+pow(ambTemp, 4-x)), (1/4-x));
  return(objTemp);
}

bool buttonShortPress(){
  // Button pulled high, push brings low
  if (digitalRead(BUTTON) == LOW){
    delay(5); //Debouncing
    if(digitalRead(BUTTON) == LOW) {
      return true;
    }
  }
  return false;
}

bool buttonLongPress(){
  // Button pulled high, push brings low
  if (digitalRead(BUTTON) == LOW){
    delay(1000); //Wait a long time before checking again
    if(digitalRead(BUTTON) == LOW) {
      return true;
    }
  }
  return false;
}

double getBPM(){
  // I'll finish copying this over from class code soon
  bpm = 0;
  return bpm;
}

void loop() {
  // put your main code here, to run repeatedly:
 
  // test the microcontroller (buzzer)
  bool test_buzz = false;
  if(test_buzz){
    tone(SOUND, 500); // 500 khz
    delay(100);
    noTone(SOUND);
    delay(100);
  }

  if(buttonShortPress()){
    tone(SOUND, 500); // 500 khz
    delay(100);
    noTone(SOUND);
    //read ambient adc voltage
    ambVoltage = getADCVoltage(AMB);
    //read measurement adc voltage
    objVoltage = getADCVoltage(OBJ);
    //get temps
    double objTemp = getObjTemp();
    double ambTemp = getAmbTemp();
    //display on oled
    oledDisplay("Ambient temp is " + String(ambTemp, 4) + " & obj temp is " + String(objTemp, 4));    
    //send over bluetooth
    delay(100);
  }
}