#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "MAX30105.h"  //MAX3010x library
#include "heartRate.h" //Heart rate calculation algorithm
#include "SSD1306Ascii.h"
#include "SSD1306AsciiWire.h"
#include <Adafruit_I2CDevice.h> //Might need so oled display libraries compile
#include <SoftwareSerial.h>

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
bool buttonMode = true;

// Constants for temp equations
const double p1 = -25.23;
const double p2 = 4.318*10000;
const double p3 = 5.48*1000000;
const double q1 = 939.7;
const double q2 = 3.531*10000;
const double seeb = .01;
const double x = 2.50;
const double gain = 101;
const int voltageOffset = 1000;
const double ambTempOffset = 2.2; //Celsius offset for ambient temp
const double objTempOffset = -11.8; //Celsius offset for object temp

//Constants for ADC calculation
const double adc_mV_offset = 12;
const int integrations = 5;

// OLED Stuff
#define SCREEN_WIDTH 128                                                  // OLED display width, in pixels
#define SCREEN_HEIGHT 32                                                  // OLED display height, in pixels
#define OLED_RESET -1                                                     // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET); // Declaring the OLED name

// BT Stuff
SoftwareSerial HC06(0, 1); // BEWARE: Cannot upload to uC w/ BT module connected

// BPM Stuff
MAX30105 particleSensor;
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
// Arduino Uno doesn't have enough SRAM to store 100 samples of IR led data and red led data in 32-bit format
// To solve this problem, 16-bit MSB of the sampled data needs to be truncated. Samples become 16-bit data.
static uint16_t irBuffer[50];  // infrared LED sensor data
static uint16_t redBuffer[50]; // red LED sensor data
#else
static uint32_t irBuffer[50];  // infrared LED sensor data
static uint32_t redBuffer[50]; // red LED sensor data
#endif

static int32_t bufferLength;  // data length
static int32_t spo2;          // SPO2 value
static int8_t validSPO2;      // indicator to show if the SPO2 calculation is valid
static int32_t heartRate;     // heart rate value
static int8_t validHeartRate; // indicator to show if the heart rate calculation is vali

static const byte RATE_SIZE = 4; // Increase this for more averaging. 4 is good.
static byte rates[RATE_SIZE];    // Array of heart rates
static byte rateSpot = 0;
static long lastBeat = 0; // Time at which the last beat occurred
static float beatsPerMinute;
static int beatAvg;
static int threshold = 7000;

//Declaration
void oledDisplay(String msg, int textsize = 1, int posHorz = 25, int posVert=10);

void BPM_setup()
{
  // Initialize sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) // Use default I2C port, 400kHz speed
  {
    oledDisplay("MAX30105 was not found. Please check wiring/power.", 1, 0, 0);
    while (1);
  }
  byte ledBrightness = 55; // Options: 0=Off to 255=50mA
  byte sampleAverage = 4;  // Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2;        // Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  byte sampleRate = 200;   // Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411;    // Options: 69, 118, 215, 411
  int adcRange = 4096;     // Options: 2048, 4096, 8192, 16384

  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); // Configure sensor with these settings

  /** Your code goes below here: Start **/
  particleSensor.setPulseAmplitudeRed(0x1F); // Turn Red LED to low to indicate sensor is running //Options for amplitude values: 0x00 = 0mA, 0x7F = 25.4mA, 0xFF = 50mA (typical)
  /** Your code goes above here: End **/
}

void Display_setup()
{
  //Serial.begin(9600); // initialize serial communication at 9600 bits per second

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C); // Start the OLED display
  display.display();
  delay(1500);
}

void setup()
{
  // put your setup code here, to run once:
  HC06.begin(9600);
  Display_setup();
  BPM_setup();
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(BUTTON, INPUT);
  pinMode(SOUND, OUTPUT);
  pinMode(SOUNDGND, OUTPUT);
  pinMode(BUTTON, INPUT);
  analogReference(EXTERNAL); // Set ADC to external reference
  
}

void oledDisplay(String msg, int textsize = 1, int posHorz = 25, int posVert=10)
{
  display.clearDisplay();
  display.setTextSize(textsize);
  display.setTextColor(WHITE);
  display.setCursor(posHorz, posVert);
  int screenlen = floor(21/textsize);
  int msglen = msg.length();
  int lastSpace = -1;
  int splitSpace = 0;
  while(lastSpace < msglen){
    while (msg.indexOf(" ", splitSpace+1) - lastSpace < screenlen){
      splitSpace = msg.indexOf(" ", splitSpace+1);
      if (splitSpace == -1){
        splitSpace = msglen;
        break;
      }
    }
    String a = msg.substring(lastSpace+1, splitSpace);
    a.replace("^", " ");
    display.println(a);
    lastSpace = splitSpace;
  }
  display.display();
}

// TODO: MAKE THIS FUNCTION WORK AS INTENDED
void getAndDisplayBPM()
{
  oledDisplay("Switching into BPM mode", 1, 0, 10);
  delay(1000);
  oledDisplay("Please press firmly on the sensor and wait for a hot sec!", 1.5, 0, 5);
  /** Your code goes below here: Start **/
  particleSensor.check();
  long irValue = particleSensor.getIR(); // Reading the IR value to detect heartbeat and finger's placement on the sensor
  /** Your code goes above here: End **/

  while (irValue > threshold) // If finger detected
  { // If finger is placed and detected
    particleSensor.check();
    irValue = particleSensor.getIR();
    /** Your code goes below here: Start **/
    Serial.print("BPM: ");
    Serial.println(beatAvg); // Print on serial monitor
    /** Your code goes above here: End **/

    if (checkForBeat(irValue) == true) // If a heart beat is detected
    {
      /** Your code goes below here: Start **/
      // Sensed a beat!
      long delta = millis() - lastBeat; // Measure duration between two beats
      lastBeat = millis();

      beatsPerMinute = 60 / (delta / 1000.0); // Calculate BPM

      //DEBUGGING PRINT STATEMENT        
      if (beatsPerMinute < 255 && beatsPerMinute > 20) // Strore some values (4) and then calculate the average
      {
        rates[rateSpot++] = (byte)beatsPerMinute; // Store this reading in the array
        rateSpot %= RATE_SIZE;                    // Wrap variable

        // Take average of readings
        beatAvg = 0;
        
        for (byte x = 0; x < RATE_SIZE; x++)
        {
          beatAvg += rates[x];
        }
        beatAvg /= RATE_SIZE;
        oledDisplay("BPM: " + String(beatAvg), 2);
      }
      Serial.print("BPM: ");
      Serial.println(beatAvg);
      /** Your code goes above here: End **/
    }
  }
  // If no finger is detected it inform the user and put the average BPM to 0 or it will be stored for the next measure
  beatAvg = 0;
  oledDisplay("No finger detected, returning to previous mode!", 1.5, 0, 5);
  delay(1000);

  Serial.println(irValue);
  /** Your code goes above here: End **/
}

bool writeBT(double objTemp, double ambTemp, double BPM, bool BPMMode)
{
  // Function to send values over bluetooth. Returns true if executed correctly.
  String sendString;
  if (BPMMode)
  {
    sendString = String(BPM);
  }
  else
  {
    String sendString_obj = String(objTemp, 2);
    String sendString_amb = String(ambTemp, 2);
    sendString = sendString_amb + "," + sendString_obj;
  }
  if(HC06.available() > 0)
    {
      char receive = HC06.read();
      if(receive == '1')
      {
        int data = 3;
        HC06.print(sendString);
      }
    }
  return (true);
}

int readBT()
{
  // Recieves a string value over BT.
  if (HC06.available())
  {
    return (HC06.read());
  }
}

// https://www.intorobotics.com/how-to-make-accurate-adc-readings-with-arduino/?msclkid=03c3eb10d08d11ec88ef55df2fef17e3
double getADCVoltage(int pin)
{
  double adcValIntegrated = 0;
  for(int i = 0; i < integrations; i++) {
    unsigned int adcVal = analogRead(pin);
    adcValIntegrated = adcValIntegrated + adcVal;
  }
  adcValIntegrated = adcValIntegrated / integrations;
  double voltage = adcValIntegrated / 1024.0 * 2.226 * 1000; // Return voltage in mV
  return (voltage+adc_mV_offset);
}

double getAmbTemp()
{
  // Get ambient temperature in units of Celsius
  double ambVoltage = getADCVoltage(AMB);
  double ambTemp = (p1 * ambVoltage * ambVoltage + p2 * ambVoltage + p3) / (ambVoltage * ambVoltage + ambVoltage*q1 + q2);
  return (ambTemp+ambTempOffset);
}

double getObjTemp()
{
  // Get object temperature in units of Celsius
  double objVoltage = getADCVoltage(OBJ);
  double ambTemp = getAmbTemp();
  double objTemp = pow(((((objVoltage - voltageOffset) / gain) / seeb) + pow(ambTemp, 4 - x)), (1 / (4 - x)));
  return (objTemp+objTempOffset);
}

bool buttonShortPress()
{
  // Button pulled high, push brings low
  if (digitalRead(BUTTON) == LOW)
  {
    delay(5); // Debouncing
    if (digitalRead(BUTTON) == LOW)
    {
      tone(SOUND, 500); // 500hz
      delay(100);
      noTone(SOUND);
      return true;
    }
  }
  return false;
}

bool buttonLongPress()
{
  // Button pulled high, push brings low
  if (digitalRead(BUTTON) == LOW)
  {
    delay(1000); // Wait a long time before checking again
    if (digitalRead(BUTTON) == LOW)
    {
      tone(SOUND, 2000);
      delay(100);
      noTone(SOUND);
      return true;
    }
  }
  return false;
}

void loop()
{
  // put your main code here, to run repeatedly:
  if(buttonLongPress()) // Switch modes
  {
    buttonMode = !buttonMode;
  }
  particleSensor.check();
  long irValue = particleSensor.getIR(); // Reading the IR value to detect heartbeat and finger's placement on the sensor
  if (irValue > threshold)
  { // If finger detected
    // TODO: SKIP BPM UNTIL ITS WORKING
    getAndDisplayBPM();
  }

  else if(buttonMode){
    if (buttonShortPress())
    {
      // get temps
      double objTemp = getObjTemp();
      double ambTemp = getAmbTemp();
      display.clearDisplay();
      display.setTextSize(1.5);
      display.setTextColor(WHITE);
      display.setCursor(0, 0);
      display.println("Ambient temp:");
      display.println("    " + String(ambTemp, 4));
      display.println("Object temp:");
      display.println("    " + String(objTemp, 4));
      display.display();
      // display on oled
    }
  }
  else{
    // get temps
    double objTemp = getObjTemp();
    double ambTemp = getAmbTemp();
    // display on oled
    display.clearDisplay();
    display.setTextSize(1.5);
    display.setTextColor(WHITE);
    display.setCursor(0, 0);
    display.println("Ambient temp:");
    display.println("    " + String(ambTemp, 4));
    display.println("Object temp:");
    display.println("    " + String(objTemp, 4));
    display.display();
    // send over bluetooth
    writeBT(objTemp, ambTemp, 0, false);
  }
}