#include "led.h"

#include <util/delay.h>


Led::Led(volatile uint8_t * pport, uint8_t pinmask, uint8_t polarity) :
    Output(pport, pinmask),
    _polarity(polarity)
{
    off();
}

void Led::on() {
    _polarity ? setHigh() : setLow();
}

void Led::off() {
    _polarity ? setLow() : setHigh();
}

void Led::blink(uint16_t delay) {
    off();
    for(uint16_t i=0; i<delay; i++)
        _delay_ms(1);
    on();
    for(uint16_t i=0; i<delay; i++)
        _delay_ms(1);
    off();
}

void Led::setState(uint8_t state) {
    state ? on() : off();
}
