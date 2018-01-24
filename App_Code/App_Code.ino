#include <SPI.h>
#include <SD.h>
#include <RTCZero.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility\imumaths.h>
#include <Wire.h>

// PIN DEFINITIONS
#define PIN_LED       13
#define PIN_BUZZER    3
#define SPI_CS        10
#define PIN_BUTTON    5

// STATE DEFINITIONS
#define S_STARTUP       0
#define S_IDLE          1
#define S_GREEN         2
#define S_YELLOW        3
#define S_RED           4
#define S_FBK_DELAY     5

// COEFFICIENTS
#define SAMPLE_DELAY  200     // mS

// Global Variables
uint state;
long ledTimer;
uint ledDelay = 70;

// IMU SENSOR DECLARATIONS
Adafruit_BNO055 bno1 = Adafruit_BNO055(1, BNO055_ADDRESS_A);
Adafruit_BNO055 bno2 = Adafruit_BNO055(2, BNO055_ADDRESS_B);

void setup() {
  // put your setup code here, to run once:
  state = STARTUP;
  Serial.begin(115200);

  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_LED, HIGH); 

}

void loop() {
  // put your main code here, to run repeatedly:

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