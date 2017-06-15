#include "input.h"

Input::Input(volatile uint8_t * pport, uint8_t pinmask) :
    Pin(pport, pinmask)
{
    // Set pin as input
    *(_pddr) &= ~(1 << _pinmask);
}

bool Input::read() {
    return (*_ppin) & (1 << _pinmask);
}
