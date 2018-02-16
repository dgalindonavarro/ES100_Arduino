/*#include <SPI.h>
#include <SD.h>
#include <RTCZero.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility\imumaths.h>
#include <Wire.h>*/
#include "Calib_Sensors_Script.h"

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
        state = S_A_CAL;
        cycle_count = 0;
        buzzer(ON);
        delay(START_BEEP); 
        buzzer(OFF);  
      }
      }
      break;

    case S_A_CAL:{
      // stream data from sensor A until it is calibrated. when calibrated, store the coeff
      rgbLED(RED);
      
      if(bno_a.isFullyCalibrated()){
        state = S_A_DONE;
      }

      }
      break;

    case S_A_DONE:{
      rgbLED(GREEN);
      // write A sensor's calibration offsets to SD
      adafruit_bno055_offsets_t A_calibration;
      if(bno_a.getSensorOffsets(A_calibration)) {
        String cal_data = "A_Cal, ";

        cal_data += "Accel_offset_x, ";
        cal_data += String(A_calibration.accel_offset_x);
        cal_data += ", ";
        cal_data += "Accel_offset_y, ";
        cal_data += String(A_calibration.accel_offset_y);
        cal_data += ", ";
        cal_data += "Accel_offset_z, ";
        cal_data += String(A_calibration.accel_offset_z);
        cal_data += ", ";
        cal_data += "Gyro_offset_x, ";
        cal_data += String(A_calibration.gyro_offset_x);
        cal_data += ", ";
        cal_data += "Gyro_offset_y, ";
        cal_data += String(A_calibration.gyro_offset_y);
        cal_data += ", ";       
        cal_data += "Gyro_offset_z, ";
        cal_data += String(A_calibration.gyro_offset_z);
        cal_data += ", ";
        cal_data += "Accel_radius, ";
        cal_data += String(A_calibration.accel_radius);
        

        logData(cal_data);

        buzzer(ON);
        delay(50);
        buzzer(OFF);
        state = S_B_CAL;
      }
      }
      break;  
    
    case S_B_CAL:{
      rgbLED(BLUE);
      
      if(bno_b.isFullyCalibrated()){
        state = S_B_DONE;
      }

      }
      break;
    
    case S_B_DONE:{
      rgbLED(GREEN);
      // write A sensor's calibration offsets to SD
      adafruit_bno055_offsets_t B_calibration;
      if(bno_b.getSensorOffsets(B_calibration)) {
        String cal_data = "B_Cal, ";

        cal_data += "Accel_offset_x, ";
        cal_data += String(B_calibration.accel_offset_x);
        cal_data += ", ";
        cal_data += "Accel_offset_y, ";
        cal_data += String(B_calibration.accel_offset_y);
        cal_data += ", ";
        cal_data += "Accel_offset_z, ";
        cal_data += String(B_calibration.accel_offset_z);
        cal_data += ", ";
        cal_data += "Gyro_offset_x, ";
        cal_data += String(B_calibration.gyro_offset_x);
        cal_data += ", ";
        cal_data += "Gyro_offset_y, ";
        cal_data += String(B_calibration.gyro_offset_y);
        cal_data += ", ";       
        cal_data += "Gyro_offset_z, ";
        cal_data += String(B_calibration.gyro_offset_z);
        cal_data += ", ";
        cal_data += "Accel_radius, ";
        cal_data += String(B_calibration.accel_radius);
        

        logData(cal_data);

        buzzer(ON);
        delay(50);
        buzzer(OFF);
        state = S_ERROR;
      
      }
      }
      break;
    
    case S_ERROR:{
      rgbLED(PURPLE);
      }
      break;

    default:

      break; 
  } 

  // total cycle period = state_Execution_time + SAMPLE_DELAY + BLINK_DELAY
  delay(30);
  }