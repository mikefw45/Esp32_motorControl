#include "twiFunctions.hpp"
#include <iostream>

twiFunctions* twiFunctions::instance = nullptr;

uint8_t twiFunctions::tx_data[8] = {0}; // Initialize transmit buffer
uint8_t twiFunctions::rx_data[4] = {0}; // Initialize receive buffer

twiFunctions::twiFunctions() {
    instance = this; // Set the singleton instance pointer to this object
}


void twiFunctions::DataRequest() {
    Wire.write(instance->tx_data, sizeof(instance->tx_data)); // Send tx_data
}


void twiFunctions::DataReceived(int byteCount) {
  
    const int expectedByteCount = 4;
    memset(instance->rx_data, 0, sizeof(instance->rx_data));
    
    int index = 0;
    while (Wire.available() && index <  expectedByteCount) {
        instance->rx_data[index++] = Wire.read(); 
    }

    std::cerr << "Data received successfully. rx_data: " ;
    for (int i = 0; i < expectedByteCount; i++) {
        std::cerr << std::hex << static_cast<int>(instance->rx_data[i]) << " ";
    }
    
    std::cerr << std::endl;
}