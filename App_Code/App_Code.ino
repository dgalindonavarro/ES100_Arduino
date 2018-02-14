/*#include <SPI.h>
#include <SD.h>
#include <RTCZero.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility\imumaths.h>
#include <Wire.h>*/
#include "App_Code.h"

void setup() {
  // put your setup code here, to run once:
  state = S_STARTUP;
  
  rtc.begin();
  Wire.begin();
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_HAP_A, OUTPUT);
  pinMode(PIN_HAP_B, OUTPUT);
  pinMode(PIN_DEBUG, OUTPUT);
  pinMode(PIN_R, OUTPUT);
  pinMode(PIN_G, OUTPUT);
  pinMode(PIN_B, OUTPUT);
  pinMode(PIN_BUTTON, INPUT_PULLUP);

  digitalWrite(PIN_DEBUG, HIGH);

  attachInterrupt(digitalPinToInterrupt(PIN_BUTTON), button1_isr, FALLING);
}

void loop() {
  // put your main code here, to run repeatedly:

  switch(state) {
    case S_STARTUP:{
      // Ensure Sensors Working Properly, Calibrated. Initiate SD logging.
      rgbLED(PURPLE);
      haptics(OFF);
      digitalWrite(PIN_DEBUG, LOW);
      initSDlogging();
      digitalWrite(PIN_DEBUG, HIGH);

      // SETUP SENSORS
      if(!bno_a.begin()){
        // There was a problem detecting the BNO055 ID1 ... check your connections 
        errorcode = errorcode | BNO_A_ERROR;
      }
      if(!bno_b.begin()){
        // There was a problem detecting the BNO055 ID2 ... check your connections 
        errorcode = errorcode | BNO_B_ERROR;
      }      
      bno_a.setExtCrystalUse(true);
      bno_b.setExtCrystalUse(true);
      
      if (errorcode){state = S_ERROR;} else {
        state = S_IDLE;
      }
      }
      break;

    case S_IDLE:{
      rgbLED(BLUE);  
      
      // Sensors recording measurement. Waiting for user Zero. No feedback given; print values for debug?
      struct IMU_Sample sample_idle = sensorRead(bno_a, bno_b); 

      if (isLogging) {
        logSample(sample_idle);
      }

      if (buttonPressed){
        buttonPressed = false;
        // Store angle delta as zero point for posture analysis. Switch to active monitoring Green state.
        zero(sample_idle.delta);
        state = S_GREEN;
      }
      }
      break;
    
    case S_GREEN:{
      rgbLED(GREEN);
      haptics(OFF);

      // Once curvature zeroed, reading values, comparing to treshold. Within range
      struct IMU_Sample sample_green = sensorRead(bno_a, bno_b); 
      float delta_m = sample_green.delta - zero_delta;
      if(isLogging){
        logSample(sample_green);   
      }

      // Deviation of sample exceeds first threshold. Move to next state.   
      if(abs(delta_m) > (float) G_THRESHOLD){
        state = S_YELLOW;  
      }

      if (buttonPressed){
        buttonPressed = false;
        // Store angle delta as zero point for posture analysis. Switch to active monitoring Green state.
        zero(sample_green.delta);
        state = S_GREEN;
      }

      }
      break;
    
    case S_YELLOW:{
      rgbLED(YELLOW);
      // Take new sample.
      struct IMU_Sample sample_yellow = sensorRead(bno_a, bno_b); 
      float delta_m = sample_yellow.delta - zero_delta;
      if(isLogging){
        logSample(sample_yellow);   
      }

      if(abs(delta_m) > (float) G_THRESHOLD){
        // PIN_BUZZER

        if(delta_m > 0){
          haptics(A);      
        }
        else{
          haptics(B);
        }
      }
      else{
        // returned to within normal
        state = S_GREEN;
      }

      // check for new Zero
      if (buttonPressed){
        buttonPressed = false;
        // Store angle delta as zero point for posture analysis. Switch to active monitoring Green state.
        zero(sample_yellow.delta);
        state = S_GREEN;
      }
      }
      break;
    
    case S_ERROR:
      rgbLED(RED);
      haptics(OFF);
      // Error logging state. Document error.
      if(!isLogging){
        initSDlogging();
      }
      logData("Error State Reached");
      // Print identifying error message
      if ((errorcode & BNO_A_ERROR) == BNO_A_ERROR){
        logData("Sensor_A disconnected");
      }
      if ((errorcode & BNO_B_ERROR) == BNO_B_ERROR){
        logData("Sensor_B disconnected");
      }
      if ((errorcode & FILE_ERROR) == FILE_ERROR){
        SerialUSB.println("Error writing to specified file.");
      }
      state = S_DEFAULT;
      break;

    default:

      break; 
  } 

  delay(SAMPLE_DELAY);
  blinkLED();
}