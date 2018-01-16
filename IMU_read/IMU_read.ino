#include <SPI.h>
#include <RTCZero.h>
#include <FlashAsEEPROM.h>
#include <FlashStorage.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility\imumaths.h>
#include <Wire.h>
//extern "C"{
//  #include <utility\twi.h>  // screw it we don't need this
//}  

// LED HEARTBEAT and TIMER DEFINITIONS
long ledTimer;
bool LEDstatus = HIGH;
#define PIN_LED 13
uint ledDelay = 70;
int counter;
RTCZero rtc;

// PWM and OUTPUT DEFINITIONS
#define PIN_BUZZER 3
#define BUZZ_PERIOD 488   // uS

// IMU SENSOR DECLARATIONS
#define TCAADDR 0x70
Adafruit_BNO055 bno1 = Adafruit_BNO055(1, BNO055_ADDRESS_B);
//Adafruit_BNO055 bno2 = Adafruit_BNO055(2, BNO055_ADDRESS_B);

// DATA SAMPLING
#define SAMPLERATE_DELAY_MS 200 // sample period in mS. 1000/x -> Hz

void setup() {

  Serial.begin(115200);
  rtc.begin();  
  Wire.begin();
  
  // pin instantiations
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_LED, HIGH); 

  SerialUSB.println("Orientation Sensor Test"); SerialUSB.println("");
  
  // Initialise both sensors 
  //tcaselect(0);
  delay(500);
  if(!bno1.begin())
  {
    // There was a problem detecting the BNO055 ID1 ... check your connections 
    SerialUSB.print("Reh, first BNO055 not detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  /*tcaselect(1);
  if(!bno2.begin())
  {
    // There was a problem detecting the BNO055 ID2 ... check your connections 
    SerialUSB.print("Reh, second BNO055 not detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  //delay(1000);*/
  bno1.setExtCrystalUse(true);
  //bno2.setExtCrystalUse(true);
  
  SerialUSB.print("BNO 1:\n");
  displayCalStatus(bno1);
  //SerialUSB.print("BNO 2:\n");
  //displayCalStatus(bno2);
}

void loop() {
  //heartbeat();
  
  // sampleData(SAMPLERATE_DELAY_MS);

  for(int i = 0; i < 256; i++){
    digitalWrite(PIN_BUZZER, HIGH);
    delayMicroseconds(BUZZ_PERIOD/2);
    digitalWrite(PIN_BUZZER, LOW);
    delayMicroseconds(BUZZ_PERIOD/2);
  }

  for(int i = 0; i < 256; i++){
    delayMicroseconds(BUZZ_PERIOD);
  }

  sensorPing(bno1);
}

// blink LED, print RTC clock every one second (called in state loop)
// takes current time in ms
void heartbeat(){
  unsigned long mtime = millis();
  if (mtime% 1000 == 0){
    SerialUSB.print(rtc.getHours());
    SerialUSB.print(":");
    SerialUSB.print(rtc.getMinutes());
    SerialUSB.print(":");
    SerialUSB.print(rtc.getSeconds());
    SerialUSB.print('\n');

    blinkLED();
    
  }  
}   

/* REWRITE TO ACCOUNT FOR MULTIPLE bno objects
void sampleData(int period){
  unsigned long mtime = millis();
  if (mtime% period == 0){
    sensorPing();
    displayCalStatus(bno);
  }  
}
*/

// does nothing so far, figure out later
/*
void sensorDetails(Adafruit_BNO055 bno){
  adafruit_bno055_rev_info_t info;
  bno.getRevInfo(&info);

}
*/

// print IMU sensor Euler angles at time called
void sensorPing(Adafruit_BNO055 bno){
  sensors_event_t event; 
  bno.getEvent(&event);

  /* Display the floating point data */
  SerialUSB.print("X: ");
  SerialUSB.print(event.orientation.x, 4);
  SerialUSB.print("\tY: ");
  SerialUSB.print(event.orientation.y, 4);
  SerialUSB.print("\tZ: ");
  SerialUSB.print(event.orientation.z, 4);
  //SerialUSB.println("");
}

/**************************************************************************/
/*
Display sensor calibration status
*/
/**************************************************************************/
void displayCalStatus(Adafruit_BNO055 bno)
{
/* Get the four calibration values (0..3) */
/* Any sensor data reporting 0 should be ignored, */
/* 3 means 'fully calibrated" */
uint8_t system, gyro, accel, mag;
system = gyro = accel = mag = 0;
bno.getCalibration(&system, &gyro, &accel, &mag);
/* The data should be ignored until the system calibration is > 0 */
SerialUSB.print("\t");
if (!system)
{
SerialUSB.print("! ");
}
/* Display the individual values */
SerialUSB.print("Sys:");
SerialUSB.print(system, DEC);
SerialUSB.print(" G:");
SerialUSB.print(gyro, DEC);
SerialUSB.print(" A:");
SerialUSB.print(accel, DEC);
SerialUSB.print(" M:");
//© Adafruit Industries https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor Page 26 of 33
SerialUSB.println(mag, DEC);
}

/**************************************************************************/
/*
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

/**************************************************************************/
/*
TCA9548A Function
Select an I2C device to communicate with , 0-7
*/
/**************************************************************************/
void tcaselect(uint8_t i) {
  if (i > 7) return;
 
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}

// I2C BUS SCANNING Function.
// BROKEN, REQUIRES twi.h file not used in Wire.h of SAMD implementation
/*
void testTCA(){
  while (!SerialUSB);
  delay(1000);

  Wire.begin();
  
  SerialUSB.begin(115200);
  SerialUSB.println("\nTCAScanner ready!");
  
  for (uint8_t t=0; t<8; t++) {
    tcaselect(t);
    SerialUSB.print("TCA Port #"); SerialUSB.println(t);

    for (uint8_t addr = 0; addr<=127; addr++) {
      if (addr == TCAADDR) continue;
    
      uint8_t data;
      if (! twi_writeTo(addr, &data, 0, 1, 1)) {
         SerialUSB.print("Found I2C 0x");  SerialUSB.println(addr,HEX);
      }
    }
  }
  SerialUSB.println("\ndone");
}
*/
