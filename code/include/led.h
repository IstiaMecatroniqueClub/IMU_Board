#ifndef LED_H
#define LED_H

//! \file led.h
//! \brief Led class
//! \author Remy Guyonneau, Philippe Lucidarme
//! \date 2017 04 24

#include <output.h>

//! \class Led
//! \brief Led class. 
//!
//! LED class. This class inherit from Output and implements behavior of an LED.
class Led : public Output
{
public:
    //! \brief Led constructor
    //!
    //! LED constructor
    //!
    //! \param pport : pointer to a volatile int corresponding to the port of the led pin.
    //! \param[in] pinmask : The pinmask for the led pin.
    //! \param[in] polarity : The polarity of the led (1=on with high state)

    Led(volatile uint8_t* pport, uint8_t pinmask, uint8_t polarity);

    //! \brief Switch on the LED
    //!
    //! Switch on the LED
    void on();

    //! \brief Switch off the LED
    //!
    //! Switch off the LED
    void off();

    //! \brief Blink the LED
    //!
    //! Blink the LED
    //!
    //! \param[in] delay The on/off delay (ms) for blinking
    void blink(uint16_t delay=50);

    //! \brief Set LED State
    //!
    //! Set LED State (1: on, 0:off)
    //!
    //! \param[in] state : State to set (1: on, 0:off)
    void setState(uint8_t state);

private:

    uint8_t _polarity; //!< Polarity of the LED. If polarity=1, the LED is ON on a high state
};

#endif // LED_H
