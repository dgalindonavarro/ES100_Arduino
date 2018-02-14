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

  attachInterrupt(digitalPinToInterrupt(PIN_BUTTON), button1_isr, CHANGE);
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
        cycle_count = 0;
        buzzer(ON);
        delay(START_BEEP); 
        buzzer(OFF);  
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

      // Zero set point.
      if (buttonPressed){
        button_handle(sample_idle.delta);
        state = S_HOLD;
      }
      }
      break;

    case S_HOLD:{
      haptics(OFF);
      buzzer(OFF);
      // if zero button is pressed down, contiuously determine length of hold. For feedback/non feedback input.
      // time since button down detected
      bool held;

      if(waiting){
        held = ((millis() - hold_timer) > HOLD_TIME);
      }
      if(held & waiting){
        buzzer(ON);
        delay(HELD_BEEP);
        buzzer(OFF);
        waiting = false;
      }
  
      // button gets released. Move to next state.
      if(buttonReleased){
        buttonReleased = false;
        if(held){
          isFeedbck = false;
        }
        else{
          isFeedbck = true;
        }
        state = S_GREEN;
      }
      }
      break;  
    
    case S_GREEN:{
      rgbLED(GREEN);
      haptics(OFF);


      // Once curvature zeroed, reading values, comparing to treshold. Within range
      struct IMU_Sample sample_green = sensorRead(bno_a, bno_b); 
      float dev_delta = sample_green.delta - zero_delta;
      if(isLogging){
        logSample(sample_green);   
      }

      // Deviation of sample exceeds first threshold. Move to next state.   
      if(abs(dev_delta) > (float) G_THRESHOLD){
        state = S_YELLOW;  
      }

      if (buttonPressed){
        button_handle(sample_green.delta);
        state = S_HOLD;
      }

      }
      break;
    
    case S_YELLOW:{
      rgbLED(YELLOW);
      buzzer(ON);
      // Take new sample.
      struct IMU_Sample sample_yellow = sensorRead(bno_a, bno_b); 
      float dev_delta = sample_yellow.delta - zero_delta;
      if(isLogging){
        logSample(sample_yellow);   
      }

      if(abs(dev_delta) > (float) G_THRESHOLD){

        if(dev_delta > 0){
          haptics(B);      
        }
        else{
          haptics(A);
        }
      }
      else{
        // returned to within normal
        state = S_GREEN;
      }

      // check for new Zero
      if (buttonPressed){
        button_handle(sample_yellow.delta);
        state = S_HOLD;
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

  // total cycle period = state_Execution_time + SAMPLE_DELAY + BLINK_DELAY
  delay(SAMPLE_DELAY);
  blinkLED();
  buzzer(OFF);
  cycle_count++;
}