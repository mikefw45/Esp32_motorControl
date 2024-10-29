#ifndef TWIFUNCTIONS_HPP
#define TWIFUNCTIONS_HPP
#include <Arduino.h>
#include <Wire.h>


class twiFunctions {

public:     
           
            twiFunctions();

            static void begin();


            static void DataRequest();
            static void DataReceived(int byteCount) ;
           

private: 
            
           const int peripheralAddress = 0x49;
           static uint8_t tx_data[8]  ;  // 0-3 Starboard, 4-7 Port  Send to RPI motor Encoder Frequency
           static uint8_t rx_data[4]  ; // 0 strbd dir, 1 strbd pwr,  0 port dir, 1 port pwr,

           static twiFunctions* instance;      // Pointer to hold the single instance of twiFunctions
};

#endif // TWIFUNCTIONS_HPP