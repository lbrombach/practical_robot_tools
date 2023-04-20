// a header file for the pins used in the project
// Author: L. Brombach
// Date: 2023-4-15
// Version: 1.0
// License: MIT



#ifndef PIN_H
#define PIN_H

#include <Arduino.h>

class Pin
{
private:
    uint8_t pinNumber;
    uint8_t mode; // INPUT, OUTPUT, INPUT_PULLUP
public:
    // Relays are active low, so the default state is HIGH
    Pin(uint8_t pinNumber, uint8_t mode, uint8_t initialState = 1);
    ~Pin();
    void setPinState(uint8_t state);
    uint8_t getPinState();
};


#endif
