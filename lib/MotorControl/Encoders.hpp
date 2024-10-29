#ifndef ENCODERS_HPP
#include <Arduino.h>
#define ENCODERS_HPP


#define Test 16
#include <driver/pcnt.h>  




     #define EncdPortA 32   //Green
     #define EncdPortB 33   //Purple
     #define EncHallPort  14  // Yellow
     
     //#define testPulse  13  //
    
     #define EncdStarbA 25   //Pink
     #define EncdStarbB 26   //White
     #define EncHallStarb   12  // Blue

  extern volatile int32_t portCount ;
  extern volatile int32_t starbCount;
  extern int32_t starbRevs ;

    
  class Encoders{

  public:
    
      Encoders();

      static  void  readStarbEncoder() ;
      static  void  readPortEncoder() ;

      static  void  readHallStarbEncoder() ;
      static  void  readHallPortEncoder() ;
      static void checkandPrintStarbCounts();
      static void checkandPrintPortCounts() ;
     

      void setupEncoders() ;
      void resetStarbEncoder() ;
      void resetPortEncoder() ;
      void resetStarbHall(); 
      static void  resetPortHall() ;

      static void PrintStarbCounts() ;

      static void PrintStarbRevCounts();
};





#endif