#ifndef INPUT_H
#define INPUT_H

//! \file input.h
//! \brief Input class. 
//! \author Remy Guyonneau, Philippe Lucidarme
//! \date 2017 04 24

#include "pin.h"

//! \class Input
//! \brief Input class. 
//!
//! Input class. This class described the behavior for an input. It inherit from Pin.

class Input : public Pin
{
public:

    //! \brief Constructor for an input
    //!
    //! Constructor for an output
    //!
    //! \param pport : Pointer to a volatile int corresponding to the port of the pin
    //! \param[in] pinmask : The pin id
    Input(volatile uint8_t * pport, uint8_t pinmask);

    //! \brief Get the pin state
    //!
    //! Get the pin state
    bool read();
};

#endif // INPUT_H
