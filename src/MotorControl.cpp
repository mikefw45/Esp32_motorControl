#include "MotorControl.hpp"

/******No changes have been made to Starboard vs Port in these functions */

void MotorControl::MotorSetup() {
        ledcSetup(STBDCHANNEL, PWMFREQ, PWMRES);
        ledcAttachPin(STBDIOPWM, STBDCHANNEL) ;
        ledcSetup(PORTCHANNEL, PWMFREQ, PWMRES);
        ledcAttachPin(PORTIOPWM, PORTCHANNEL ) ;
        pinMode(STBDIODIR, OUTPUT);
        pinMode(PORTIODIR, OUTPUT) ;
}

void MotorControl::setMotorVelocity(uint8_t motor , int16_t speed ){
        if (motor == STBDIODIR) {
            if (speed > 2500) {
                digitalWrite(STBDIODIR, HIGH) ;
                ledcWrite(STBDCHANNEL, 250);}  // STBD high is forward
            if (speed < 1000) {    
              digitalWrite(STBDIODIR, LOW); 
              ledcWrite(STBDCHANNEL, 250); }   
            else  {
                 ledcWrite(STBDCHANNEL, 0);
            } 
            //speed = abs(speed) ;
            //uint8_t dutyCycle = constrain(speed, 150, 255);  
            
        }
        if (motor == PORTMTR) {
            if (speed > 2500) {
                digitalWrite(PORTIODIR, LOW) ; // PORT high is reverse
                ledcWrite(PORTCHANNEL, 250);}  
            if (speed < 1000) {
                digitalWrite(PORTIODIR, LOW) ; // PORT high is reverse
                ledcWrite(PORTCHANNEL, 250);}     
            else {
                ledcWrite(PORTCHANNEL, 0);   }   
           // speed = abs(speed) ;
           // uint8_t dutyCycle = constrain(speed, 150, 255);  
            
        }  
}


void MotorControl::Joystickvals(int x, int z) {
        if ( x > 2500) {
            StarbDir = HIGH ; 
            StarbPwr = 250 ;
            PortDir = LOW ;
            PortPwr = 250 ;
        }        
       
        if ( x < 1000) {
            StarbDir = LOW ; 
            StarbPwr = 250 ;
            PortDir = HIGH ;
            PortPwr = 250 ;
        }

        if ( z < 1000) {
            StarbDir = HIGH ; 
            StarbPwr = 250 ;
            PortDir = HIGH ;
            PortPwr = 250 ;
        }

        if ( z > 2500) {
            StarbDir = LOW ; 
            StarbPwr = 250 ;
            PortDir = LOW ;
            PortPwr = 250 ; 
        }

       if (z >= 1000 && z <= 2500 && x >= 1000 && x <= 2500){
        StarbPwr = 0 ;
        PortPwr = 0 ;
        }
}
 

/*
    if ( myData.x > 2500) {
      digitalWrite(STBDIODIR, HIGH) ;
      digitalWrite(PORTIODIR, LOW) ;
      ledcWrite(STBDCHANNEL, 250);
      ledcWrite(PORTCHANNEL, 250);
  }
   if ( myData.x < 1000) {
      digitalWrite(STBDIODIR, LOW) ;
      digitalWrite(PORTIODIR, HIGH) ;
      ledcWrite(STBDCHANNEL, 250);
      ledcWrite(PORTCHANNEL, 250);
  }
  if ( myData.z < 1000) {
        digitalWrite(STBDIODIR, LOW) ;
        digitalWrite(PORTIODIR, LOW) ;
        ledcWrite(STBDCHANNEL, 250);
        ledcWrite(PORTCHANNEL, 250);
  }
  if ( myData.z > 2500) {
        digitalWrite(STBDIODIR, HIGH) ;
        digitalWrite(PORTIODIR, HIGH) ;
        ledcWrite(STBDCHANNEL, 250);
        ledcWrite(PORTCHANNEL, 250);
  }

   else {
      ledcWrite(STBDCHANNEL, 0);
      ledcWrite(PORTCHANNEL, 0);

   }
*/       