#include <SPI.h>
#include <RTCZero.h>

long ledTimer;
bool LEDstatus = HIGH;

int pinLED = 13;
uint ledDelay = 70;
int counter;

RTCZero rtc;

void setup() {

  Serial.begin(115200);
  rtc.begin();  
  
  pinMode(pinLED, OUTPUT);
  digitalWrite(pinLED, HIGH); 
}

void loop() {
  secClock(millis());
  
}

// blink LED, print RTC clock every one second 
// takes current time in ms
void secClock(unsigned long mtime){
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


void blinkLED(){
  ledTimer = millis();
  digitalWrite(pinLED, HIGH);

  while(digitalRead(pinLED) == HIGH){
    if(millis() - ledTimer > ledDelay) {
      
      digitalWrite(pinLED, LOW);
    }
  }
}
