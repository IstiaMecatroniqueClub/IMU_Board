#ifndef PIN_H
#define PIN_H

//! \file pin.h
//! \brief Pin interface. 
//! \author Remy Guyonneau, Philippe Lucidarme
//! \date 2017 04 24

#include <stdint.h>

//! \class Pin
//! \brief Pin class. 
//!
//! IPin class (interface). This class represents a Pin interface and shouldn't be instanciated, only inherited.
class Pin
{
protected:
    //! \brief Pin : Constructor to represent an Pin
    //!
    //! Constructor to represent an Pin
    //!
    //! \param pport : pointer to a volatile int corresponding to the SFR memspace for this port
    //! \param pinmask : Id of the pin to use
    Pin(volatile uint8_t* pport, uint8_t pinmask);

    volatile uint8_t * _pport; //!< Pointer to a volatile int corresponding to the PORTx register for this pin

    volatile uint8_t * _pddr; //!< Pointer to a volatile int corresponding to the DDRx register for this pin

    volatile uint8_t * _ppin; //!< Pointer to a volatile int corresponding to the the PINx register for this pin

    uint8_t _pinmask; //!< Id of the pin to use on this port
};

#endif // PIN_H
