#ifndef TWIFUNCTIONS_HPP
#define TWIFUNCTIONS_HPP
#include <Arduino.h>
#include <Wire.h>

#define peripheralAddress = 0x49;


class twiFunctions {

public:     

            static twiFunctions* instance;       // Singleton instance pointer
           // static uint8_t tx_data[8];           // Transmit buffer
           // static uint8_t rx_data[4];           // Receive buffer
           
            twiFunctions();

           // static void begin() {

           
            


            static void DataRequest();
            static void DataReceived(int byteCount) ;

            static uint8_t tx_data[8]  ;  // 0-3 Starboard, 4-7 Port  Send to RPI motor Encoder Frequency
            static uint8_t rx_data[4]  ; // 0 strbd dir, 1 strbd pwr,  0 port dir, 1 port pwr,
           

private: 
            
          // const int peripheralAddress = 0x49;
          

          // static twiFunctions* instance;      // Pointer to hold the single instance of twiFunctions
};

#endif // TWIFUNCTIONS_HPP