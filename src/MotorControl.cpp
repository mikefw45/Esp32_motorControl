#include "MotorControl.hpp"
#include <iostream>
#include "twiFunctions.hpp"

/******No changes have been made to Starboard vs Port in these functions */

void MotorControl::MotorSetup() {
        ledcSetup(STBDCHANNEL, PWMFREQ, PWMRES);
        ledcAttachPin(STBDIOPWM, STBDCHANNEL) ;
        ledcSetup(PORTCHANNEL, PWMFREQ, PWMRES);
        ledcAttachPin(PORTIOPWM, PORTCHANNEL ) ;
        pinMode(STBDIODIR, OUTPUT);
        pinMode(PORTIODIR, OUTPUT) ;
}

void motorRheostat(uint8_t *rx_data){ 
    //strbd motor = 0x0 , port motor = 0x02
    // direction Forward 0x08 =HIGH, Reverse 0x0F 
    // starboard forwards set pin HIGH port forwards set pin LOW
    //power 0x00 to 0xFF
   

    if (rx_data[0] == 0x08) {
      digitalWrite(STBDIODIR, HIGH) ;   // High is forward   
      ledcWrite(STBDCHANNEL, rx_data[1]);
      Serial.printf("Starboard PWM: %d   Direction: %s \n", rx_data[1], "Forward");
    } else if (rx_data[0] == 0x0F) {
      digitalWrite(STBDIODIR, LOW)  ;
      ledcWrite(STBDCHANNEL, rx_data[1]);
      Serial.printf("Starboard PWM: %d   Direction: %s \n", rx_data[1], "Reverse");
    } else 
      Serial.printf("Starboard PWM: %d   Direction: %s \n", rx_data[1], "Direction undefined");
      

    if (rx_data[2] == 0x08) {
      digitalWrite(PORTIODIR, LOW) ;  
      ledcWrite(PORTCHANNEL, rx_data[3]);
      Serial.printf("Port PWM: %d   Direction: %s \n", rx_data[3], "Forward");   
    } else if (rx_data[2] == 0x0F) {
      digitalWrite(PORTIODIR, HIGH)  ;  // LOW is forward 
      ledcWrite(PORTCHANNEL, rx_data[3]);
      Serial.printf("Port PWM: %d   Direction: %s \n", rx_data[3], "Reverse");  
    } else 
      Serial.printf("Portt  PWM: %d   Direction: %s \n", rx_data[1], "Direction undefined");

}

