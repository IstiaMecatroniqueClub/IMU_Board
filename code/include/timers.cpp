#include "timers.h"

#include <avr/io.h>

//---------- 8 bits Timer (Timer0) ----------//

Timer8::Timer8(Timer ps) :
    _prescaler(ps)
{
    // Disable interrupt
    TIMSK0 = 0x00;
    // Normal mode and that's all
    TCCR0A = 0x00;
    // Nothing set, not counting !
    TCCR0B = 0x00;
    reset();
}

void Timer8::start() {
    // Set the prescaler, start counting!
    TCCR0B |= (uint8_t)_prescaler;
}

void Timer8::stop() {
    // Remove prescaler, stopcounting!
    TCCR0B &= 0xF8;
}

void Timer8::reset() {
    stop();
    TCNT0 = 0;
}

uint8_t Timer8::getTicks() {
    return TCNT0;
}

//---------- 16 bits Timer (Timer1) ----------//

Timer16::Timer16(Timer ps) :
    _prescaler(ps)
{
    // Disable interrupt
    TIMSK1 = 0x00;
    // Normal mode and that's all
    TCCR1A = 0x00;
    // Nothing set, not counting !
    TCCR1B = 0x00;
    TCCR1C = 0x00;
    reset();
}

void Timer16::start() {
    // Set the prescaler, start counting!
    TCCR1B |= (uint8_t)_prescaler;
}

void Timer16::stop() {
    // Remove prescaler, stopcounting!
    TCCR1B &= 0xF8;
}

void Timer16::reset() {
    stop();
    TCNT1 = 0;
}

uint16_t Timer16::getTicks() {
    uint8_t low = TCNT1L;
    uint16_t high = TCNT1H;

    return ((high << 8) | low);
}
