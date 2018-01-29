#include <SPI.h>
#include <SD.h>
#include <RTCZero.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility\imumaths.h>
#include <string.h>
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
#define S_ERROR         6

// COEFFICIENTS
#define SAMPLE_DELAY   50     // mS
#define BLINK_DELAY    10     // mS      

// COLORS
#define OFF    0x00
#define RED    0x01
#define GREEN  0x02
#define BLUE   0x04
#define CYAN   0x06
#define PURPLE 0x05
#define YELLOW 0X03

// ERROR CODES
#define BNO_A_ERROR   0x01
#define BNO_B_ERROR   0x02

// Global Variables
uint state;
uint errorcode = 0x00;
float zero_delta;
const char* filename = "data.txt";
bool isLogging;
volatile bool buttonPressed = false;

// Real-Time Clock
RTCZero rtc;

// IMU SENSOR DECLARATIONS
Adafruit_BNO055 bno_a = Adafruit_BNO055(1, BNO055_ADDRESS_A);
Adafruit_BNO055 bno_b = Adafruit_BNO055(2, BNO055_ADDRESS_B);

// STRUCTURE DEFINITIONS
struct IMU_Sample{
    float a;
    float b;
    float delta;
};

// FUNCTIONS
void blinkLED(){
  long ledTimer = millis();
  digitalWrite(PIN_LED, HIGH);

  while(digitalRead(PIN_LED) == HIGH){
    if((millis() - ledTimer) > BLINK_DELAY) {
      
      digitalWrite(PIN_LED, LOW);
    }
  }
}

// Initializes SD card. Sets isLogging to true if successful. 
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
void logData(String dataString){

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

// Returns a struct with both IMU pitch readings, and delta = A - B
struct IMU_Sample sensorRead(Adafruit_BNO055 bno_a, Adafruit_BNO055 bno_b){
  struct IMU_Sample sample;

  sensors_event_t event_a;
  sensors_event_t event_b;

  bno_a.getEvent(&event_b);
  bno_b.getEvent(&event_b);

  // extract the pitch angles of interest as samples  
  sample.a = event_a.orientation.z;
  sample.b = event_b.orientation.z;
  sample.delta = sample.a - sample.b;

  return sample;
}

// Log a data sample (sensor pitches, delta, state) to the SD
void logSample(struct IMU_Sample sample){
  String dataline = "";
  
  dataline += "A, ";
  dataline += String(sample.a);
  dataline += ", ";
  dataline += "B, ";
  dataline += sample.b;
  dataline += ", ";  
  dataline += "Delta, ";
  dataline += sample.delta;
  dataline += ", ";
  dataline += "State, ";
  dataline += state;
  
  logData(dataline);
}

// Control RGB LED to either: Red, Green, Blue, Purple, Cyan, Yellow, OFF
void rgbLED(byte color){
  digitalWrite(PIN_R, LOW);
  digitalWrite(PIN_G, LOW);
  digitalWrite(PIN_B, LOW);

  if((color & BLUE) == BLUE){
    digitalWrite(PIN_B, HIGH);
  }
  if((color & GREEN) == GREEN){
    digitalWrite(PIN_G, HIGH);
  }
  if((color & RED) == RED){
    digitalWrite(PIN_R, HIGH);
  }
}

// Button 1 Interrupt Service Routine
void button1_isr(){
  buttonPressed = true;
}