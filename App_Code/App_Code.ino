#include <SPI.h>
#include <SD.h>
#include <RTCZero.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility\imumaths.h>
#include <Wire.h>

// PIN DEFINITIONS
#define PIN_LED       13
#define PIN_BUZZER    7
#define SPI_CS        10
#define PIN_BUTTON    5
#define PIN_DEBUG     8
#define PIN_R         2
#define PIN_G         3
#define PIN_B         4

// STATE DEFINITIONS
#define S_STARTUP       0
#define S_IDLE          1
#define S_GREEN         2
#define S_YELLOW        3
#define S_RED           4
#define S_FBK_DELAY     5

// COEFFICIENTS
#define SAMPLE_DELAY  200     // mS

// COLORS
#define OFF    0x00
#define RED    0x01
#define GREEN  0x02
#define BLUE   0x04
#define CYAN   0x06
#define PURPLE 0x05
#define YELLOW 0X03 

// Global Variables
uint state;
long ledTimer;
const uint ledDelay = 70;
const char* filename = "data.txt";
bool isLogging;

// Real-Time Clock
RTCZero rtc;

// Variables using the SD utility library functions:
Sd2Card card;
SdVolume volume;
SdFile root;

// IMU SENSOR DECLARATIONS
Adafruit_BNO055 bno1 = Adafruit_BNO055(1, BNO055_ADDRESS_A);
Adafruit_BNO055 bno2 = Adafruit_BNO055(2, BNO055_ADDRESS_B);

void setup() {
  // put your setup code here, to run once:
  state = S_STARTUP;
  rgbLED(PURPLE);
  
  rtc.begin();
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_DEBUG, OUTPUT);

  digitalWrite(PIN_LED, HIGH); 
  digitalWrite(PIN_DEBUG, HIGH);
  delay(1);
}

void loop() {
  // put your main code here, to run repeatedly:

  switch(state) {
    case S_STARTUP:{
      digitalWrite(PIN_DEBUG, LOW);
      initSDlogging();
      digitalWrite(PIN_DEBUG, HIGH);

      state = S_IDLE;
      }
      break;

    case S_IDLE:
      rgbLED(YELLOW);
      // Sensors recording measurement. Waiting for user Zero. No feedback given; print values for debug?
      if (isLogging) {
        logData("reh");
      }
      break;
    
    case S_GREEN:
      // Once curvature zeroed, reading values, comparing to treshold. Within range
      break;
    
    case S_YELLOW:
      // Threshold passed: give some form of feedback
      break;
    
    default:

      break; 
  } 

}

// FUNCTIONS
/**************************************************************************/
/* void blinkLED()
Turn LED on for ledTimer time in ms
*/
/**************************************************************************/
void blinkLED(){
  ledTimer = millis();
  digitalWrite(PIN_LED, HIGH);

  while(digitalRead(PIN_LED) == HIGH){
    if(millis() - ledTimer > ledDelay) {
      
      digitalWrite(PIN_LED, LOW);
    }
  }
}

void initSDlogging(){
  SerialUSB.print("Initializing SD card...");

  // see if the card is present and can be initialized:
  if (!SD.begin(SPI_CS)) {
    SerialUSB.println("Card failed, or not present");
    // don't do anything more:
    isLogging = false;
    return;
  }
  SerialUSB.println("card initialized.");     

  String dataString = "Device_Startup, Data, Time, Value";
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  File dataFile = SD.open(filename, FILE_WRITE);

  // if the file is available, write to it:
  if (dataFile) {
    dataFile.println(dataString);
    dataFile.close();
    // print to the serial port too:
    SerialUSB.println(dataString);
  }
  // if the file isn't open, pop up an error:
  else {
    SerialUSB.println("error opening datalog.txt");
  }

  isLogging = true;
}

// Write a string as a line to the SD card file filename 
void logData(char* dataString){

  File dataFile = SD.open(filename, FILE_WRITE);
  // if the file is available, write to it:
  if (dataFile) {
    dataFile.println(dataString);
    dataFile.close();
    // print to the serial port too:
    SerialUSB.println(dataString);
  }
  // if the file isn't open, pop up an error:
  else {
    SerialUSB.println("error opening datalog.txt");
  }
}

// Control RGB LED to either: Red, Green, Blue, Purple, Cyan, Yellow, OFF
void rgbLED(unsigned char color){
  digitalWrite(PIN_R, HIGH);
  digitalWrite(PIN_G, HIGH);
  digitalWrite(PIN_B, HIGH);

  if(color && BLUE == BLUE){
    digitalWrite(PIN_B, LOW);
  }
  if(color && GREEN == GREEN){
    digitalWrite(PIN_G, LOW);
  }
  if(color && RED == RED){
    digitalWrite(PIN_R, LOW);
  }
}