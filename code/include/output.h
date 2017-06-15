#ifndef OUTPUT_H
#define OUTPUT_H

//! \file output.h
//! \brief Output class. 
//! \author Remy Guyonneau, Philippe Lucidarme
//! \date 2017 04 24

#include "pin.h"

//! \class Output
//! \brief Output class. 
//!
//! Output class. This class described the behavior for an output. It inherits from the IPin interface.
class Output : public Pin
{
public:

    //! \brief Constructor for an output
    //!
    //! Constructor for an output
    //!
    //! \param pport : Pointer to a volatile int corresponding to the port of the pin
    //! \param[in] pinmask : The pin id
    Output(volatile uint8_t * pport, uint8_t pinmask);

    //! \brief Switch pin to high state
    //!
    //! Switch pin to high state
    void setHigh();

    //! \brief Switch pin to low state
    //!
    //! Switch pin to low state
    void setLow();

    //! \brief Toggle pin state
    //!
    //! Toggle pin state
    void toggle();
};

#endif // OUPUT_H
