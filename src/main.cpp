#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include "MotorControl.hpp"
#include <Wire.h>
#include "TWIfunctions.hpp"





MotorControl motor;



void wheelCounttoByte(uint8_t * byteWheelCount,  int32_t wheelEncoderCount);
void onStarbPulse() ;
void  onPortPulse() ;
void  calculateStarbFrequency() ;
void  calculatePortFrequency() ;




/**************motor control variables***************/
bool StarbDir ;
uint8_t StarbPwr;
bool PortDir ; 
uint8_t PortPwr; 

/************Encoder Variables**************/
#define EncdStarbA 25   //Pink
#define EncdPortA 32   //Green
volatile int starbCount = 0;
volatile int portCount = 0;
volatile int starbInterrupt ;
int starbFreq = 0 ;
int portFreq = 0 ;
hw_timer_t *starbTimer = NULL;
hw_timer_t *portTimer = NULL;
const int intervalStarbMs = 1000;
const int intervalPortMs = 1000;

/*******data received from RPI****** */
//uint8_t receivedBytes[8] ;


 


void setup(){
  Serial.begin(115200);

/***********Encoder setup******* */
  pinMode(EncdStarbA, INPUT_PULLUP);
  pinMode(EncdPortA, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(EncdStarbA), onStarbPulse, RISING);
  attachInterrupt(digitalPinToInterrupt(EncdPortA), onPortPulse, RISING);
  starbTimer = timerBegin(0, 80, true);
  portTimer = timerBegin(1, 80, true);
  timerAttachInterrupt(starbTimer, &calculateStarbFrequency, true); 
  timerAttachInterrupt(portTimer, &calculatePortFrequency, true); 
  timerAlarmWrite(starbTimer, intervalStarbMs * 1000, true);       
  timerAlarmWrite(portTimer, intervalPortMs * 1000, true);   
  timerAlarmEnable(starbTimer);  
  timerAlarmEnable(portTimer);    
  
  /************TWIfunctions setup********** */
  twiFunctions::begin ;
  twiFunctions::DataRequest;
  twiFunctions::DataReceived;

/************Motor controller***********/
motor.MotorSetup();


/************initial values for motor*********** */
StarbDir = HIGH; // High is forward  ++ count
StarbPwr = 120;
ledcWrite(STBDCHANNEL, StarbPwr);
digitalWrite(STBDIODIR, StarbDir) ;

PortDir = LOW; // LOW is forward  
PortPwr = 120;
ledcWrite(PORTCHANNEL, PortPwr);
digitalWrite(PORTIODIR, PortDir) ;
}



void loop(){
            
    Serial.print("   starbFrequency: ");
    Serial.print(starbFreq);
    Serial.print("    portFrequency:  ");
    Serial.print(portFreq);
    Serial.println(" Hz");

    delay(500); // Adjust as needed for your display frequency


 

}



void wheelCounttoByte(uint8_t * byteWheelCount,  int32_t wheelEncoderCount){
    byteWheelCount[0] = wheelEncoderCount & 0xFF;
    byteWheelCount[1] = (wheelEncoderCount >> 8) & 0xFF;
    byteWheelCount[2] = (wheelEncoderCount >> 16) & 0xFF;
    byteWheelCount[3] = (wheelEncoderCount >> 24) & 0xFF;
}


/***************Encoder functions************* */

void IRAM_ATTR onStarbPulse() {    
    starbCount++;
}

void IRAM_ATTR onPortPulse() {    
    portCount++;
}
void IRAM_ATTR calculateStarbFrequency() {
  //Serial.println("Starboard Timer Triggered");
    starbFreq = starbCount * (1000 / intervalStarbMs); // pulses per second (Hz)
    starbCount = 0; // Reset count for the next interval
}

void IRAM_ATTR calculatePortFrequency() {
 // Serial.println("Port Timer Triggered");
    portFreq = portCount * (1000 / intervalPortMs); // pulses per second (Hz)
    portCount = 0; // Reset count for the next interval
}








  
