#include "Pin.h"

Pin::Pin(uint8_t pinNumber, uint8_t mode, uint8_t initialState):pinNumber(pinNumber), mode(mode)
{
    pinMode(pinNumber, mode);
    digitalWrite(pinNumber, initialState);
}

Pin::~Pin()
{
}

void Pin::setPinState(uint8_t state)
{
    if(mode == INPUT || mode == INPUT_PULLUP)
    {
        return;
    }
    digitalWrite(pinNumber, state);
}

uint8_t Pin::getPinState()
{
    return digitalRead(pinNumber);
}
