#include <SPI.h>
#include <RTCZero.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

long ledTimer;
bool LEDstatus = HIGH;

int pinLED = 13;
int aSelect = 2;
int bSelect = 3;
int reset = 4;
uint ledDelay = 70;
int counter;

RTCZero rtc;
Adafruit_BNO055 bno = Adafruit_BNO055(-1, BNO055_ADDRESS_B);

void setup() {

  Serial.begin(115200);
  rtc.begin();  
  
  pinMode(pinLED, OUTPUT);
  pinMode(aSelect, OUTPUT);
  pinMode(bSelect, OUTPUT);
  pinMode(reset, OUTPUT);

  digitalWrite(pinLED, HIGH); 

  // power cycle both IMUs with ADDR pin set high
  digitalWrite(aSelect, HIGH);
  digitalWrite(reset, LOW);
  delayMicroseconds(1);
  digitalWrite(reset, HIGH);

  Serial.println("Orientation Sensor Test"); Serial.println("");
  
  /* Initialise both sensors */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Reh, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  //delay(1000);
  bno.setExtCrystalUse(true);

  displayCalStatus(bno);


}

void loop() {
  heartbeat();
  
  
}

// blink LED, print RTC clock every one second (called in state loop)
// takes current time in ms
void heartbeat(){
  unsigned long mtime = millis();
  if (mtime% 1000 == 0){
    Serial.print(rtc.getHours());
    Serial.print(":");
    Serial.print(rtc.getMinutes());
    Serial.print(":");
    Serial.print(rtc.getSeconds());
    Serial.print('\n');

    blinkLED();
    sensorPing();
  }  
}

// print IMU sensor Euler angles at time called
void sensorPing(){
  sensors_event_t event; 
  bno.getEvent(&event);

  /* Display the floating point data */
  Serial.print("X: ");
  Serial.print(event.orientation.x, 4);
  Serial.print("\tY: ");
  Serial.print(event.orientation.y, 4);
  Serial.print("\tZ: ");
  Serial.print(event.orientation.z, 4);
  Serial.println("");
}

/**************************************************************************/
/*
Turn LED on for ledTimer time in ms
*/
/**************************************************************************/
void blinkLED(){
  ledTimer = millis();
  digitalWrite(pinLED, HIGH);

  while(digitalRead(pinLED) == HIGH){
    if(millis() - ledTimer > ledDelay) {
      
      digitalWrite(pinLED, LOW);
    }
  }
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
Serial.print("\t");
if (!system)
{
Serial.print("! ");
}
/* Display the individual values */
Serial.print("Sys:");
Serial.print(system, DEC);
Serial.print(" G:");
Serial.print(gyro, DEC);
Serial.print(" A:");
Serial.print(accel, DEC);
Serial.print(" M:");
//© Adafruit Industries https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor Page 26 of 33
Serial.println(mag, DEC);
}