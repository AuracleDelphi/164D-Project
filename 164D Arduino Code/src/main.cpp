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
const double p1 = -0.02613;
const double p2 = 16.55;
const int p3 = 5889;
const double q1 = 44.18;
const double seeb = .01;
const double x = 2.50;
const double gain = 47.51;
const int voltageOffset = 495;

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

void BPM_setup()
{
  // Initialize sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) // Use default I2C port, 400kHz speed
  {
    oledDisplay("MAX30105 was not found. Please check wiring/power.");
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
  delay(3000);
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
  analogReference(INTERNAL); // Set ADC to 1.1V internal reference
  
}

void oledDisplay(String msg)
{
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.println(msg);
  display.display();
}

// TODO: MAKE THIS FUNCTION WORK AS INTENDED
void getAndDisplayBPM()
{
  /** Your code goes below here: Start **/
  particleSensor.check();
  long irValue = particleSensor.getIR(); // Reading the IR value to detect heartbeat and finger's placement on the sensor
  /** Your code goes above here: End **/

  if (irValue > threshold)
  { // If finger is placed and detected
    /** Your code goes below here: Start **/
    display.clearDisplay(); // Clear the display
    /** Your code goes above here: End **/
    display.setTextSize(2); // Display the average BPM next to it
    display.setTextColor(WHITE);
    display.setCursor(50, 0);
    display.println("BPM");
    /** Your code goes below here: Start **/
    display.setCursor(40, 20); // Put the coordinates for the beatAvg to be displayed on your OLED
    display.setTextSize(1);    // Display the average BPM next to it
    display.print(F("Beat Avg: "));
    display.println(String(beatAvg));
    ;                  // Print beatAvg on display
    display.display(); // display the componenets mapped out in this segment
    Serial.print("BPM: ");
    Serial.println(beatAvg); // Print on serial monitor
    /** Your code goes above here: End **/

    if (checkForBeat(irValue) == true) // If a heart beat is detected
    {
      /** Your code goes below here: Start **/
      display.clearDisplay(); // Clear the display

      // Sensed a beat!
      long delta = millis() - lastBeat; // Measure duration between two beats
      lastBeat = millis();

      beatsPerMinute = 60 / (delta / 1000.0); // Calculate BPM

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
      }
      display.setTextSize(2); // And still displays the average BPM
      display.setTextColor(WHITE);
      display.setCursor(50, 0);
      display.println("BPM");
      /** Your code goes below here: Start **/
      display.setCursor(40, 20); // Put the coordinates for the beatAvg to be displayed on your OLED
      display.setTextSize(1);
      display.print(F("Beat Avg: "));
      display.println(String(beatAvg));
      ;                  // Print beatAvg on display
      display.display(); // display the componenets mapped out in this segment
      Serial.print("BPM: ");
      Serial.println(beatAvg);
      /** Your code goes above here: End **/
    }
  }
  if (irValue < threshold)
  { // If no finger is detected it inform the user and put the average BPM to 0 or it will be stored for the next measure
    beatAvg = 0;
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    /** Your code goes below here: Start **/
    beatAvg = 0;
    display.setCursor(10, 10); // command placement of finger
    display.println("No finger detected");
    display.setCursor(40, 20);
    display.print(F("Beat Avg: "));
    display.print(beatAvg); // Print beatAvg on display
    display.display();

    Serial.println(irValue);
    /** Your code goes above here: End **/
  }
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
  analogReference(INTERNAL);
  unsigned int adcVal = analogRead(pin);
  double voltage = adcVal / 1024.0 * 1.1 * 1000; // Return voltage in mV
  return voltage;
  /*
  // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA,ADSC));
  long result = ADCL;
  result |= ADCH<<8;`
  result = 1125300L / result; // Back-calculate AVcc in mV
  double vcc = result/1000.0;
  unsigned int ADCValue = analogRead(pin);
  double voltage  = (ADCValue / 1024.0) * vcc;
  return voltage;
  */
}

double getAmbTemp()
{
  // Get ambient temperature in units of Celsius
  double ambVoltage = getADCVoltage(AMB);
  double ambTemp = (p1 * ambVoltage * ambVoltage + p2 * ambVoltage + p3) / (ambVoltage + q1);
  return (ambTemp);
}

double getObjTemp()
{
  // Get object temperature in units of Celsius
  double objVoltage = getADCVoltage(OBJ);
  double ambTemp = getAmbTemp();
  double objTemp = pow(((((objVoltage - voltageOffset) / gain) / seeb) + pow(ambTemp, 4 - x)), (1 / (4 - x)));
  ;
  return (objTemp);
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
  if(buttonLongPress())
  {
    buttonMode = !buttonMode;
  }
  particleSensor.check();
  long irValue = particleSensor.getIR(); // Reading the IR value to detect heartbeat and finger's placement on the sensor
  if (irValue > threshold)
  { // If finger detected
    // TODO: SKIP BPM UNTIL ITS WORKING
    // getAndDisplayBPM();
  }

  if(buttonMode){
    if (buttonShortPress())
    {
      // get temps
      double objTemp = getObjTemp();
      double ambTemp = getAmbTemp();
      // display on oled
      oledDisplay("Ambient temp is " + String(ambTemp, 4) + " & obj temp is " + String(objTemp, 4));
    }
  }

  else{
    // get temps
    double objTemp = getObjTemp();
    double ambTemp = getAmbTemp();
    // display on oled
    oledDisplay("Ambient temp is " + String(ambTemp, 4) + " & obj temp is " + String(objTemp, 4));
    // send over bluetooth
    writeBT(objTemp, ambTemp, 0, false);
    
  }
}