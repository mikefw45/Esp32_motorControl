#include "Encoders.hpp"
#include <Arduino.h> 

  volatile int32_t portCount = 0 ;
  volatile int32_t starbCount = 0;

  int32_t starbRevs = 0;
  int32_t portRevs = 0;
  bool newstarbRev = false ;
  bool newportRev = false ;
  volatile unsigned long lastInterruptTime = 0;
  volatile unsigned long currentTime =0 ;
  volatile unsigned long lastUpdateTime = 0;
  Encoders::Encoders(){

  }
  
  void Encoders::setupEncoders(){
            //pinMode(testPulse,  OUTPUT); 
            //digitalWrite(testPulse, LOW); 
            pinMode(EncdStarbB,  INPUT_PULLUP); 
            pinMode(EncdStarbA,  INPUT_PULLUP);
	        attachInterrupt(EncdStarbA, Encoders::readStarbEncoder,  RISING) ;
            pinMode(EncHallStarb , INPUT);
            attachInterrupt(EncHallStarb, Encoders::readHallStarbEncoder,  FALLING);
            
            pinMode(EncdPortB, INPUT_PULLUP); 
            pinMode(EncdPortA, INPUT_PULLUP);
            attachInterrupt(EncdPortB, Encoders::readPortEncoder,  RISING) ;
            pinMode(EncHallPort , INPUT);
            attachInterrupt(EncHallPort, Encoders::readHallPortEncoder,  FALLING);

   
        }

        void IRAM_ATTR Encoders::readStarbEncoder(){  
            if (digitalRead(EncdStarbB)) {
             starbCount++;                                  
             } else {starbCount--;  }    
            
        }  

        void  IRAM_ATTR Encoders::readPortEncoder(){  
            if (digitalRead(EncdPortA)) {            
                portCount-- ;
             } else {portCount++ ; }    
        }

        void  IRAM_ATTR Encoders::readHallStarbEncoder(){
           // unsigned long interruptTime = millis(); 
           // if (interruptTime - lastInterruptTime > 1300) { // debounce time in milliseconds
                starbRevs++;
                newstarbRev = true;
            //    lastInterruptTime = interruptTime;
            //}
        }

        void  IRAM_ATTR Encoders::readHallPortEncoder(){
             portRevs++;
             newportRev = true;
        }

        

        void Encoders::PrintStarbCounts(){


              noInterrupts();
              
            Serial.print("Starbd counts per rev: ");
            Serial.println(starbCount);
                starbCount = 0 ;
               interrupts();

        }

        
        void Encoders::PrintStarbRevCounts(){


              noInterrupts();
              
            Serial.print("Starbd revs per 10 secs: ");
            Serial.println(starbRevs);
                starbRevs = 0 ;
               interrupts();

        }





        void Encoders::checkandPrintStarbCounts(){
            if (newstarbRev) {
               // currentTime = millis() ;
                noInterrupts();
                //float elapsedTime = (currentTime - lastUpdateTime);
                int32_t count = starbCount ;
                starbCount = 0 ;
                newstarbRev = false ;
               /*
                Serial.print("Starbd time per rev: ");
                Serial.print(elapsedTime, 2);
                Serial.print("   ");
                */
                Serial.print("  Starbd counts per rev: ");
                Serial.print(count);

                interrupts();
                // lastUpdateTime = millis() ;
            }
        }

           void Encoders::checkandPrintPortCounts(){
            if (newportRev) {
                noInterrupts();
                int32_t count = portCount ;
                portCount = 0 ;
                newportRev = false ;
                

                Serial.print("   Port counts per rev: ");
                Serial.print(count);

                interrupts();
            }
        }


        void Encoders::resetStarbEncoder(){
            starbCount = 0 ;
        }

          void Encoders::resetPortEncoder(){
             portCount = 0 ;
        } 

         void Encoders::resetStarbHall(){
            starbRevs = 0 ;
        }

          void Encoders::resetPortHall(){
             portRevs = 0 ;
        } 