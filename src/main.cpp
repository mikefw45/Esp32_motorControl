#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include "MotorControl.hpp"
#include <Wire.h>
#include "TWIfunctions.hpp"
#include <iostream>





MotorControl motor;



void wheelFreqtoByte(uint8_t * byteWheelFreq,  int wheelFreq) ;
void onStarbPulse() ;
void  onPortPulse() ;
void  calculateStarbFrequency() ;
void  calculatePortFrequency() ;
void motorRheostat(uint8_t *rx_data);

/***********TWI pins********** */
#define SDA_PIN 21
#define SCL_PIN 22
#define I2C_ADDRESS 0X49 





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
//uint8_t twiFunctions::rx_data[4]
//rx_data[0]  0x08  = LOW  ,  0x0F = HIGH StarbDir
//rx_data[1]  0x00 - 0xFF  StarbPwr
//rx_data[2]  0x08  = LOW  ,  0x0F = HIGH PortDir
//rx_data[3]  0x00 - 0xFF  PortPwr
 


void setup(){
  Serial.begin(115200);
 Wire.begin(I2C_ADDRESS) ;


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
 // twiFunctions::begin ;
  Wire.onRequest(twiFunctions::DataRequest); 
  Wire.onReceive(twiFunctions::DataReceived);

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

    motorRheostat(twiFunctions::rx_data) ;


    std::cerr << std::hex << static_cast<int>(starbFreq) << " Starboard frequency hz"<< std::endl;
    std::cerr << std::hex << static_cast<int>(portFreq) << " Port frequency hz"<< std::endl;     
    /*   
    Serial.print("   starbFrequency: ");
    Serial.print(starbFreq);
    Serial.print("    portFrequency:  ");
    Serial.print(portFreq);
    Serial.println(" Hz");
    */
    uint8_t BstarbFreq[4] ;
    uint8_t BportFreq[4] ;

    wheelFreqtoByte(BstarbFreq , starbFreq) ; 
    memcpy(twiFunctions::tx_data, BstarbFreq, 4);
    wheelFreqtoByte(BportFreq , portFreq) ;
    memcpy(twiFunctions::tx_data + 4, BportFreq, 4);



    delay(500); // Adjust as needed for your display frequency


 

}



void wheelFreqtoByte(uint8_t * byteWheelFreq,  int wheelFreq){
    byteWheelFreq[0] = wheelFreq & 0xFF;
    byteWheelFreq[1] = (wheelFreq >> 8) & 0xFF;
    byteWheelFreq[2] = (wheelFreq >> 16) & 0xFF;
    byteWheelFreq[3] = (wheelFreq >> 24) & 0xFF;
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








  
