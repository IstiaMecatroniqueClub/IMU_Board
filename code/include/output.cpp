#include "output.h"

Output::Output(volatile uint8_t * pport, uint8_t pinmask) :
    Pin(pport, pinmask)
{
    // Set pin as output
    *(_pddr) |= (1 << _pinmask);
    setHigh();
}

void Output::setHigh() {
    *(_pport) |= (1 << _pinmask);
}

void Output::setLow() {
    *(_pport) &= ~(1 << _pinmask);
}

void Output::toggle() {
    *(_ppin) |= (1 << _pinmask);
}
