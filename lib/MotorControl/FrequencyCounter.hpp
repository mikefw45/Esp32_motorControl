#ifndef FREQUENCYCOUNTER_HPP
#define FREQUENCYCOUNTER_HPP

#include <Arduino.h>

#define EncdStarbA 25   //Pink
#define EncdPortA 32   //Green

class FrequencyCounter {
public:
   
    FrequencyCounter(uint8_t instanceIndex);  //(uint8_t pin, int intervalMs, uint8_t timerID);   // Constructor: initialize with encoder pin, calculation interval in milliseconds
   
    void begin();   // Initializes the counter    
    static void IRAM_ATTR calculateFrequency0();
    static void IRAM_ATTR calculateFrequency1();

     uint32_t getPulseCount() const;
     static FrequencyCounter* instances[2]; // Static array for instances

private:
    uint8_t instanceIndex;
    uint8_t pin;            // Encoder pin
    int intervalMs;         // Frequency calculation interval in milliseconds
    volatile int pulseCount; // Pulse counter (incremented in ISR)
    int frequency;          // Holds calculated frequency in Hz
    hw_timer_t *timer;      // ESP32 hardware timer
    uint8_t timerID;        // Timer ID (0, 1, 2, or 3 for ESP32)

    // Static functions for ISR
    static void IRAM_ATTR onPulse0();          // ISR for Timer 0
    static void IRAM_ATTR onPulse1();          // ISR for Timer 1
    static void IRAM_ATTR calculateFrequency0();
    static void IRAM_ATTR calculateFrequency1();// Calculates frequency every interval

    uint32_t getPulseCount() const;

    
    static FrequencyCounter* instances[2];  // Static array for instances

    //static FrequencyCounter *instances[2]; // Static pointer to class instance for ISRs
   


};

#endif
