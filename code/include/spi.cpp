//! \file spi.cpp
//! \author Franck Mercier
//! \date 2017 05 12

#include <avr/io.h>
#include "spi.h"

// Constructor
Spi::Spi(){}

// destructeur
Spi::~Spi(){}

// Initialize SPI Master Device
void Spi::spi_init_master (bool redirection)
{
    _spiOut = redirection;
    if (redirection == STD_DIRECTION) {
        DDRB |= 0x82;   // Set MOSI, SCK as Output
        DDRB &= 0xFE;   // Set MISO as Input
        DDRD |= 0x08;   // Set CS as Output
        MCUCR &= 0x7F;  // TODO
        PORTD |= 0x08;  // Set CS at 1
        PORTB |= 0x01;  // Pull up MISO
    }
    else {
        DDRD |= 0x18;   // Set MOSI, SCK as Output
        DDRD &= 0xFB;   // Set MISO as Input
        DDRC |= 0x02;   // Set CS as Output
        MCUCR |= 0x80;  // TODO
        PORTC |= 0x02;  // Set CS at 1
        PORTD |= 0x04;  // Pull up MISO
    }
    SPCR |= 0x5D;   // Enable SPI, Set as Master, Prescaler: Fosc/16
    SPSR &= 0xFE;   // TODO
}

// Initialize SPI Slave Device
void Spi::spi_init_slave (bool redirection)
{
    _spiOut = redirection;
    if (redirection == STD_DIRECTION) {
        DDRB |= 0x01;   // MISO as OUTPUT
        DDRD &= 0xF7;   // Set CS as Input
        MCUCR &= 0x7F;  // TODO
    }
    else {
        DDRD |= 0x04;   // MISO as OUTPUT
        DDRC &= 0xFD;   // Set CS as Input
        MCUCR |= 0x80;  // TODO
    }
    SPCR = (1<<SPE);    // Enable SPI
}

void Spi::spi_begin_transceive()
{
    if (_spiOut == STD_DIRECTION) {
        PORTD &= 0xF7;  //Set CS at 0
    }
    else {
        PORTC &= 0xFD;  //Set CS at 0
    }
}

void Spi::spi_stop_transceive()
{
    if (_spiOut == STD_DIRECTION) {
        PORTD |= 0x08;  //Set CS at 1
    }
    else {
        PORTC |= 0x02;  //Set CS at 1
    }
}
 
//Function to send and receive data
unsigned char Spi::spi_tranceiver (unsigned char data)
{
    SPDR = data;                    // Load data into the buffer
    while(!(SPSR & (1<<SPIF) ));    // Wait until transmission complete
    SPSR &= 0x7F;
    return(SPDR);                   // Return received data
}
