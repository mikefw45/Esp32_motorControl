#include "FrequencyCounter.hpp"


FrequencyCounter *FrequencyCounter::instances[2] = {nullptr, nullptr}; // Initialize static instance pointers for each timer

FrequencyCounter::FrequencyCounter(uint8_t instanceIndex) : pulseCount(0), instanceIndex(instanceIndex) {
    if (instanceIndex < 2) {
        instances[instanceIndex] = this;
    }
}

void FrequencyCounter::begin() {
    
    pinMode(pin, INPUT_PULLUP);  //  encoder pin and attach the pulse counter interrupt
        if (timerID == 0) {
        attachInterrupt(digitalPinToInterrupt(EncdStarbA), onPulse0, RISING);
    } else if (timerID == 1) {
        attachInterrupt(digitalPinToInterrupt(EncdPortA), onPulse1, RISING);
    }

    
    timer = timerBegin(0, 80, true); // Timer 0, prescaler 80, count up  Initialize the timer
    timerAttachInterrupt(timer, instances[timerID]->calculateFrequency, true); // Attach ISR to timer
    timerAlarmWrite(timer, intervalMs * 1000, true); // Set timer interval in microseconds
    timerAlarmEnable(timer); // Enable the timer alarm
}

void IRAM_ATTR FrequencyCounter::onPulse0() {
    instances[0]->pulseCount++; // Increment pulse count on each pulse
}

void IRAM_ATTR FrequencyCounter::onPulse1() {
    instances[1]->pulseCount++; // Increment pulse count on each pulse
}

void IRAM_ATTR FrequencyCounter::calculateFrequency() {
    for (int i = 0; i < 4; ++i) {
        if (instances[i]) {
            instances[i]->frequency = instances[i]->pulseCount * (1000 / instances[i]->intervalMs); // Hz
            instances[i]->pulseCount = 0; // Reset pulse count after calculation
        }
    }
}

int FrequencyCounter::getFrequency() const {
    return frequency; // Return the last calculated frequency
}
