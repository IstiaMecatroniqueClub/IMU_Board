#ifndef TIMERS_H
#define TIMERS_H

//! \file timers.h
//! \brief Timer class. 
//! \author Remy Guyonneau, Franck Mercier, Philippe Lucidarme
//! \date 2017 04 24

#include <stdint.h>

//! \enum Timer
//! \brief Timer enumerate class. 
//!
//! Timer enumerate class, to handdle the different timer sizes
enum class Timer : uint8_t{
    TIMER_PRESCALER_1    = 0b00000001, //!< 62.5ns/tick @16MHz (TOP=16us / 4ms)
    TIMER_PRESCALER_8    = 0b00000010, //!< 500ns/tick @16MHz  (TOP=128us / 32.7ms)
    TIMER_PRESCALER_64   = 0b00000011, //!< 4us/tick @16MHz    (TOP=1ms / 262ms)
    TIMER_PRESCALER_256  = 0b00000100, //!< 16us/tick @16MHz   (TOP=4ms / 1.05s)
    TIMER_PRESCALER_1024 = 0b00000101  //!< 64us/tick @16MHz   (TOP=16ms / 4.19s)
};

//! \class Timer8
//! \brief Timer8 class. 
//!
//! The Timer8 class to use a simple 8 bits wide timer
class Timer8 {
public:
    //! \brief Timer8 constructor.
    //! \param[in] ps : The prescaler to use with this timer
    Timer8(Timer ps=Timer::TIMER_PRESCALER_1);

    //! \brief Start the timer
    void start();

    //! \brief Stop the timer
    void stop();

    //! \brief Stop and reset the timer to 0
    void reset();

    //! \brief getTicks Return the number of ticks
    uint8_t getTicks();

private:
    //! \brief The prescaler
    Timer _prescaler;
};


//! \class Timer16
//! \brief Timer16 class. 
//!
//! The Timer16 class to use a simple 16 bits wide timer
class Timer16 {
public:
    //! \brief Timer16 constructor.
    //! \param[in] ps : The prescaler to use with this timer
    Timer16(Timer ps=Timer::TIMER_PRESCALER_1);

    //! \brief Start the timer
    void start();

    //! \brief Stop the timer
    void stop();

    //! \brief Stop and reset the timer to 0
    void reset();

    //! \brief getTicks Return the number of ticks
    uint16_t getTicks();

private:
    //! \brief The prescaler
    Timer _prescaler;
};

#endif // TIMERS_H
