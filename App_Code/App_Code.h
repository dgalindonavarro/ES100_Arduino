#include <SPI.h>
#include <SD.h>
#include <RTCZero.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility\imumaths.h>
#include <string.h>
#include <Wire.h>

// PIN DEFINITIONS
#define PIN_DEBUG     13      // also attached to M0 on-board LED
#define PIN_BUZZER    10
#define SPI_CS        7
#define PIN_BUTTON    5
#define PIN_R         4
#define PIN_G         3
#define PIN_B         2
#define PIN_HAP_A     8
#define PIN_HAP_B     9

// STATE DEFINITIONS
#define S_STARTUP       0
#define S_IDLE          1
#define S_GREEN         2
#define S_YELLOW        3
#define S_RED           4
#define S_FBK_DELAY     5
#define S_ERROR         6
#define S_DEFAULT       7
#define S_HOLD          8

// TIMING
#define SAMPLE_DELAY_MS   40     // mS
#define BLINK_DELAY_MS    10     // mS
#define START_BEEP        100    // mS
#define HELD_BEEP         50     // mS 
#define HOLD_TIME         1000   // mS

// COEFFICIENTS
#define G_THRESHOLD    6      // (float) degrees       

// CALIBRATION DATA
#define A_ax           6      // (uint16) offset
#define A_ay           0      // (uint16) offset
#define A_az           26     // (uint16) offset
#define A_gx           65536  // (uint16) offset
#define A_gy           65536  // (uint16) offset
#define A_gz           0      // (uint16) offset
#define A_ar           1000
#define B_ax           2      // (uint16) offset
#define B_ay           65491  // (uint16) offset
#define B_az           29     // (uint16) offset
#define B_gx           1      // (uint16) offset
#define B_gy           65535  // (uint16) offset
#define B_gz           0      // (uint16) offset
#define B_ar           1000 

// COLORS
#define OFF    0x00
#define RED    0x01
#define GREEN  0x02
#define BLUE   0x04
#define CYAN   0x06
#define PURPLE 0x05
#define YELLOW 0X03

// HAPTICS
#define A      0x01
#define B      0x02
#define BOTH   0x03

// BUZZER
#define ON     0x01
 
// ERROR CODES
#define BNO_A_ERROR   0x01
#define BNO_B_ERROR   0x02
#define FILE_ERROR    0x04
#define GET_A_FAIL    0x08
#define GET_B_FAIL    0x10

// STRUCTURE DEFINITIONS
#define BUFF_SIZE           1200    // struct data_samples of size 20 bytes (only 32kB RAM)
#define WRITE_REST_PERIOD     40    // length, in samples, of stable activity before writing
#define P2P_MAX               15    // degrees, max allowable P2P to write data

typedef struct IMU_Sample{
    float a;
    float b;
    float delta;
} IMU_Sample;

typedef struct data_sample{
    IMU_Sample angles;
    long time;
    byte state;
    byte statA;
    byte statB;
} data_sample;

// FORWARD DECLARATIONS
float minarray(float *arr);
float maxarray(float *arr);
bool calculateActivity(float nextA);
void writeData();
void rgbLED(byte color);

// Global Variables
unsigned long cycle_count;
volatile unsigned long hold_timer;
byte state;
uint errorcode = 0x00;
float zero_delta;
const char* filename = "data.txt";
bool isLogging;
bool isFeedbck = true; // by default true so startup beeps occur
volatile bool buttonPressed = false;
volatile bool buttonReleased = false;
bool waiting;
byte haptic_status = 0x00;

// Real-Time Clock
RTCZero rtc;

// IMU SENSOR DECLARATIONS
Adafruit_BNO055 bno_a = Adafruit_BNO055(1, BNO055_ADDRESS_A);
Adafruit_BNO055 bno_b = Adafruit_BNO055(2, BNO055_ADDRESS_B);

// DATA BUFFERS and PTRS
static data_sample databuff[BUFF_SIZE] = {0};
static data_sample *buffptr = databuff;
static data_sample *buffend = databuff + BUFF_SIZE;
bool buff_overflow = false;

// FOR CALCULATING MAX AND MIN of A data
static float a_data[WRITE_REST_PERIOD];
byte a_data_pos = 0;
byte rest_counter = 0;

float a_min = INFINITY;
float a_max = -INFINITY;


// FUNCTIONS
void blinkLED(){
  long ledTimer = millis();
  digitalWrite(PIN_DEBUG, HIGH);

  while(digitalRead(PIN_DEBUG) == HIGH){
    if((millis() - ledTimer) > BLINK_DELAY_MS) {
      
      digitalWrite(PIN_DEBUG, LOW);
    }
  }
}

void buzzer(byte cmd){
  if(cmd & isFeedbck){
    digitalWrite(PIN_BUZZER, HIGH);
  }
  else
  {
    digitalWrite(PIN_BUZZER, LOW);
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

  String dataString = "Time (mS), Sensor_A, Sensor_B, A-B, State, Hap_A, Hap_B";
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
    errorcode = FILE_ERROR;
  }

  isLogging = true;
}

// Write a string as a line to the SD card file filename. Should check isLogging before calling. 
void logString(String dataString){

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
IMU_Sample sensorRead(Adafruit_BNO055 bno_a, Adafruit_BNO055 bno_b){
  IMU_Sample sample;

  sensors_event_t event_a;
  sensors_event_t event_b;

  bno_a.getEvent(&event_a);
  bno_b.getEvent(&event_b);

  // extract the pitch angles of interest as samples  
  sample.a = event_a.orientation.z;
  sample.b = -(event_b.orientation.z); // ensure signs are both positive
  sample.delta = sample.a - sample.b;

  return sample;
}

// Log a data sample (sensor pitches, delta, state) to the data buffer's current position. If full, trigger writeData().
// Move buffer pointer forward by one.
void logSample(IMU_Sample sample){
  if(!buff_overflow){
    buffptr->time = millis();
    buffptr->angles.a = sample.a; 
    buffptr->angles.b = sample.b;
    buffptr->angles.delta = sample.delta;
    buffptr->state = state;
    buffptr->statA = (haptic_status & A);
    buffptr->statB = ((haptic_status & B) >> 1);

    // increment buffer position to next available
    buffptr++;  
    // error check for end of buffer
    if(buffptr == buffend){
      buff_overflow = true;  
    }
  }

  // have we gone N samples with no activity? OR filled buffer
  if(calculateActivity(sample.a) || buff_overflow){
    rgbLED(CYAN);
    writeData();
  }
}

// given a new data pt, see if peak-to-peak values of A indicate user activity
// update the activity detect buffer, buffer pos
bool calculateActivity(float nextA){

  // put data in buffer
  a_data[a_data_pos] = nextA;  
  if(++a_data_pos >= WRITE_REST_PERIOD){
    a_data_pos = 0;
  }

  // don't do anything if first N data
  if(cycle_count < WRITE_REST_PERIOD){
    a_max = nextA;
    a_min = nextA;   
    rest_counter++;

    return false;  
  }
  else{

    // calculate new max/min
    a_max = maxarray(a_data);
    a_min = maxarray(a_data);

    // calculate peak to peak, see if trips
    if((a_max - a_min) >= (float) P2P_MAX){
      // reset rest counter, still active.
      rest_counter = 0;
    }
    else{
      rest_counter++;
    }

    // return true if rest counter reaches MAX
    if(rest_counter >= WRITE_REST_PERIOD){
      rest_counter = 0;
      return true;
    }
    else{
      return false;
    }
  }
}

// Write all data in buffer to SD (if logging) and clear buffer, move pointer back to beginning.
// determine if data should be written to SD
void writeData(){
  if(isLogging){
    // move ptr to beginning of buffer
    buffptr = databuff;
    File dataFile = SD.open(filename, FILE_WRITE);

    if (dataFile){
      // while data_Sample.time != 0 write it all out
      while(!buffptr->time){
        String dataline = "";
        
        dataline += String(buffptr->time);
        dataline += ", ";
        dataline += String(buffptr->angles.a);
        dataline += ", ";
        dataline += String(buffptr->angles.b);
        dataline += ", ";  
        dataline += String(buffptr->angles.delta);
        dataline += ", ";
        dataline += String(buffptr->state);
        dataline += ", ";
        dataline += String(buffptr->statA);
        dataline += ", ";
        dataline += String(buffptr->statB);
        
        dataFile.println(dataline);
        buffptr++;
      }
      dataFile.close();
    }  
    else{
      // there was an error opening the data.txt file.
    }  
  memset(databuff, 0, sizeof(databuff));
  buffptr = databuff;
  buff_overflow = false;
  }
}

// stupid linear search, only 40 values
// calculate, return minimum value in a float array
float minarray(float *arr){
  float m = INFINITY;
  for(int i; i < sizeof(arr); i++){
    if(arr[i] < m){
      m = arr[i];    
    }
  }

  return m;  
}

// calculate, return max value in a float array
float maxarray(float *arr){
  float m = -INFINITY;
  for(int i; i < sizeof(arr); i++){
    if(arr[i] > m){
      m = arr[i];    
    }
  }

  return m;    
}

// Control RGB LED to either: Red, Green, Blue, Purple, CYAN, Yellow, OFF
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

// Haptic Motor Controller for both A and B (A corresponds to thoracic, B corresponds to lumbar)
// for now, only on or OFF
// OFF, A, B, BOTH
// isFeedbck must be true for haptics to be switched on
void haptics(byte code){
  digitalWrite(PIN_HAP_A, LOW); 
  digitalWrite(PIN_HAP_B, HIGH);  // haptic B is now active LOW

  if(isFeedbck){
    if((code & A) == A){
      digitalWrite(PIN_HAP_A, HIGH);

    }
    if((code & B) == B){
      digitalWrite(PIN_HAP_B, LOW);
    }                   
  }

  haptic_status = code;
}

// Button 1 Interrupt Service Routine
void button1_isr(){
  if(digitalRead(PIN_BUTTON) == LOW){
    buttonPressed = true;
  } 
  else{
    buttonReleased = true;
  }
  
}

// Sets global zero point to passed in float. (does not reset flag)
void zero(float delta){
  if(isLogging){
    String zeroed = "Posture delta Zeroed at ";
    zeroed += String(delta);
    zeroed += " degrees.";
    logString(zeroed);
  }
  zero_delta = delta;  
}

// Button down ISR flag handler in Idle, Green, or Yellow states.
// takes a sample as input
// state must be sent to S_HOLD afterwards
void button_handle(float delta){
  hold_timer = millis();
  buttonPressed = false;
  waiting = true;

  // Store angle delta as zero point for posture analysis.
  zero(delta);  
}