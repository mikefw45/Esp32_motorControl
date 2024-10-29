#ifndef MOTORCONTROL_HPP
#define MOTORCONTROL_HPP

#include <stdint.h>
#include <Arduino.h>
#define STBDMTR  0   
#define PORTMTR  4  
#define PWMFREQ  500 

#define STBDCHANNEL 1
//#define STBDCHANNEL  0
#define PORTCHANNEL 0
//#define PORTCHANNEL  1
#define PWMRES  8

#define PORTIOPWM 5   //Cytron 5
//#define STBDIOPWM  5  
#define PORTIODIR 17  // Cytron 4
//#define STBDIODIR 17  // Cytron 4
#define STBDIOPWM 19 //Cytron 10
//#define PORTIOPWM 19 //Cytron 10
#define STBDIODIR 18 //Cytron 12
//#define PORTIODIR 18 //Cytron 12

extern bool StarbDir ;
extern uint8_t StarbPwr;
extern bool PortDir ; 
extern uint8_t PortPwr; 


class MotorControl{

  public:
    MotorControl()  { }
     
    void MotorSetup();

    void setMotorVelocity(uint8_t motor , int16_t speed ) ;

    void Joystickvals(int x, int z) ;
  
};








#endif 