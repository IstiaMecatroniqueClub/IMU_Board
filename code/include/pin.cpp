#include "pin.h"

Pin::Pin(volatile uint8_t * pport, uint8_t pinmask) :
    _pport(pport),
    _pddr(pport-1),
    _ppin(pport-2),
    _pinmask(pinmask)
{
}
