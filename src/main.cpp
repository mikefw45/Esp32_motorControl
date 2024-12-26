#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include "MotorControl.hpp"
#include <Wire.h>
#include "TWIfunctions.hpp"
#include <iostream>






MotorControl motor;




void onStarbPulse() ;
void  onPortPulse() ;
void  calculateStarbFrequency() ;
void  calculatePortFrequency() ;
void motorRheostat(uint8_t *rx_data);
void int32ToBytes(int32_t val, uint8_t* bytes_array) ;


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
#define StarbdWhlRev 14 // Yellow
#define PortWhlRev 12 //Blue

volatile unsigned long portEncdCount = 0;
volatile unsigned long starbEncdCount = 0;


//volatile unsigned long portPulsesPerRev = 0; // Pulses per revolution (Port)
//volatile unsigned long starbPulsesPerRev = 0; // Pulses per revolution (Starboard)

unsigned long previousMillis = 0;     // Tracks time for frequency calculation
const unsigned long interval = 1000;  // Measurement interval in ms (1 second)

void IRAM_ATTR onStarbPulse() ;
void IRAM_ATTR onPortPulse() ;
//void IRAM_ATTR onStarbRev() ;
//void IRAM_ATTR onPortRev() ;




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
 // pinMode(StarbdWhlRev, INPUT_PULLUP);
 // pinMode(PortWhlRev, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(EncdStarbA), onStarbPulse, RISING);
  attachInterrupt(digitalPinToInterrupt(EncdPortA), onPortPulse, RISING);
 // attachInterrupt(digitalPinToInterrupt(StarbdWhlRev), onStarbRev, FALLING);
 // attachInterrupt(digitalPinToInterrupt(PortWhlRev), onPortRev, FALLING);


  
  
  /************TWIfunctions setup********** */
 // twiFunctions::begin ;
  Wire.onRequest(twiFunctions::DataRequest); 
  Wire.onReceive(twiFunctions::DataReceived);

/************Motor controller***********/
motor.MotorSetup();


/************initial values for motor*********** */
StarbDir = HIGH; // High is forward  ++ count
StarbPwr = 200;
ledcWrite(STBDCHANNEL, StarbPwr);
digitalWrite(STBDIODIR, StarbDir) ;

PortDir = LOW; // LOW is forward  
PortPwr = 200;
ledcWrite(PORTCHANNEL, PortPwr);
digitalWrite(PORTIODIR, PortDir) ;
}



void loop(){

    motorRheostat(twiFunctions::rx_data) ;

    unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
         previousMillis = currentMillis;
    
        int32_t starbFrequency = starbEncdCount / 2; 
        int32_t portFrequency = portEncdCount / 2; 

        int32ToBytes(starbFrequency, &twiFunctions::tx_data[0]); 
        int32ToBytes(portFrequency, &twiFunctions::tx_data[4]);
 
        //Serial.printf("Starbd Encoder frequency: %d    port Encoder frequency: %d \n",starbEncdCount, portEncdCount);
       // Serial.printf("Port Encoder frequency: %d \n", portEncdCount);
      //  Serial.printf("tx_data[4]: %x tx_data[5]: %x tx_data[6]: %x tx_data[7]: %x \n",twiFunctions::tx_data[4], twiFunctions::tx_data[5], twiFunctions::tx_data[6], twiFunctions::tx_data[7]);


        starbEncdCount = 0 ;
        portEncdCount = 0 ;
    }


}

void int32ToBytes(int32_t val, uint8_t* bytes_array) {
    memcpy(bytes_array, &val, sizeof(int32_t));
}


/***************Encoder functions************* */

void IRAM_ATTR onStarbPulse() {    
    starbEncdCount++;
}


void IRAM_ATTR onPortPulse() {    
    portEncdCount++;
}

/*
void IRAM_ATTR onStarbRev() {

    starbPulsesPerRev = starbEncdCount ;
    starbEncdCount = 0 ;
    Serial.print("Starb Pulses per Revolution: ");
    Serial.println(starbPulsesPerRev);
}

void IRAM_ATTR onPortRev() {

    portPulsesPerRev = portEncdCount ;
    portEncdCount = 0 ;
    Serial.print("Port Pulses per Revolution: ");
    Serial.println(portPulsesPerRev);

}

*/









  
