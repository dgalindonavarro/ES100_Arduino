#include <SPI.h>
#include <RTCZero.h>
#include <FlashAsEEPROM.h>
#include <FlashStorage.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Wire.h>
extern "C" { 
  //#include "utility\twi.h"
  }
  

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
Adafruit_BNO055 bno = Adafruit_BNO055(-1, BNO055_ADDRESS_B);

// DATA SAMPLING
#define SAMPLERATE_DELAY_MS 200 // sample period in mS. 1000/x -> Hz

void setup() {

  Serial.begin(115200);
  rtc.begin();  
  
  // pin instantiations
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_LED, HIGH); 

  Serial.println("Orientation Sensor Test"); Serial.println("");
  
  /* Initialise both sensors */
  /*if(!bno.begin())
  {
    // There was a problem detecting the BNO055 ... check your connections 
    Serial.print("Reh, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  //delay(1000);
  bno.setExtCrystalUse(true);

  displayCalStatus(bno);
  */

  //testTCA();
  
  
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
    
  }  
}   

void sampleData(int period){
  unsigned long mtime = millis();
  if (mtime% period == 0){
    sensorPing();
    displayCalStatus(bno);
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
  //Serial.println("");
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
/*
void testTCA(){
  while (!Serial);
  delay(1000);

  Wire.begin();
  
  Serial.begin(115200);
  Serial.println("\nTCAScanner ready!");
  
  for (uint8_t t=0; t<8; t++) {
    tcaselect(t);
    Serial.print("TCA Port #"); Serial.println(t);

    for (uint8_t addr = 0; addr<=127; addr++) {
      if (addr == TCAADDR) continue;
    
      uint8_t data;
      if (! twi_writeTo(addr, &data, 0, 1, 1)) {
         Serial.print("Found I2C 0x");  Serial.println(addr,HEX);
      }
    }
  }
  Serial.println("\ndone");

}
*/

/**************************************************************************/
/*
Set SAMD21E Dual Slope PWM
https://forum.arduino.cc/index.php?topic=346731.0

Output 100Hz PWM on timer TCC0 digital pin D13
*/
/**************************************************************************/
void setupPWM()
{ 
  REG_GCLK_GENDIV = GCLK_GENDIV_DIV(3) |          // Divide the 48MHz clock source by divisor 3: 48MHz/3=16MHz
                    GCLK_GENDIV_ID(4);            // Select Generic Clock (GCLK) 4
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  REG_GCLK_GENCTRL = GCLK_GENCTRL_IDC |           // Set the duty cycle to 50/50 HIGH/LOW
                     GCLK_GENCTRL_GENEN |         // Enable GCLK4
                     GCLK_GENCTRL_SRC_DFLL48M |   // Set the 48MHz clock source
                     GCLK_GENCTRL_ID(4);          // Select GCLK4
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  // Enable the port multiplexer for digital pin 13 (D13): timer TCC0 output
  PORT->Group[g_APinDescription[13].ulPort].PINCFG[g_APinDescription[13].ulPin].bit.PMUXEN = 1;
  
  // Connect the TCC0 timer to the port output - port pins are paired odd PMUO and even PMUXE
  // F & E specify the timers: TCC0, TCC1 and TCC2
  PORT->Group[g_APinDescription[11].ulPort].PMUX[g_APinDescription[11].ulPin >> 1].reg = PORT_PMUX_PMUXO_F;
  
  // Feed GCLK4 to TCC0 and TCC1
  REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |         // Enable GCLK4 to TCC0 and TCC1
                     GCLK_CLKCTRL_GEN_GCLK4 |     // Select GCLK4
                     GCLK_CLKCTRL_ID_TCC0_TCC1;   // Feed GCLK4 to TCC0 and TCC1
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  // Dual slope PWM operation: timers countinuously count up to PER register value then down 0
  REG_TCC0_WAVE |= TCC_WAVE_POL(0xF) |         // Reverse the output polarity on all TCC0 outputs
                    TCC_WAVE_WAVEGEN_DSBOTTOM;    // Setup dual slope PWM on TCC0
  while (TCC0->SYNCBUSY.bit.WAVE);               // Wait for synchronization

  // Each timer counts up to a maximum or TOP value set by the PER register,
  // this determines the frequency of the PWM operation:
  // 20000 = 50Hz, 10000 = 100Hz, 2500  = 400Hz,    ... 500 = 2000Hz
  REG_TCC0_PER = 500;      // Set the frequency of the PWM on TCC0 to 100Hz
  while(TCC0->SYNCBUSY.bit.PER);

  // The CCBx register value corresponds to the pulsewidth in microseconds (us)
  REG_TCC0_CCB3 = 1500;       // TCC0 CCB3 - center the servo on D13
  while(TCC0->SYNCBUSY.bit.CCB3);

  // Divide the 16MHz signal by 8 giving 2MHz (0.5us) TCC0 timer tick and enable the outputs
  REG_TCC0_CTRLA |= TCC_CTRLA_PRESCALER_DIV8 |    // Divide GCLK4 by 8
                    TCC_CTRLA_ENABLE;             // Enable the TCC0 output
  while (TCC0->SYNCBUSY.bit.ENABLE);              // Wait for synchronization
}