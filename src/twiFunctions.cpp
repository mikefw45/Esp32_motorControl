#include "twiFunctions.hpp"

twiFunctions* twiFunctions::instance = nullptr;

uint8_t twiFunctions::tx_data[8] = {0}; // Initialize transmit buffer
uint8_t twiFunctions::rx_data[4] = {0}; // Initialize receive buffer

twiFunctions::twiFunctions() {
    instance = this; // Set the singleton instance pointer to this object
}

void twiFunctions::begin() {
    Wire.begin(); // Start I2C communication
    Wire.onRequest(DataRequest); // Register request callback
    Wire.onReceive(DataReceived); // Register receive callback
}

void twiFunctions::DataRequest() {
    Wire.write(instance->tx_data, sizeof(instance->tx_data)); // Send tx_data
}

// I2C receive callback function
void twiFunctions::DataReceived(int byteCount) {
    int index = 0;
    while (Wire.available() && index < sizeof(rx_data)) {
        instance->rx_data[index++] = Wire.read(); // Read received data
    }
}