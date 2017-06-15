//! \file main.cpp
//! \brief Main file
//! \author Franck Mercier, Remy Guyonneau
//! \date 2017 05 11
//!
//! Main file for the IMU

#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <string.h> // memcpy


#include "led.h"
#include "spi.h"
#include "fmt1030.h"


// Configuration declaration for the RED LED wiring
#define LED_RED_PORT PORTB //!< The ATMega port where the red LED is wired
#define LED_RED_PIN  6     //!< The ATMega pin where the red LED is wired 
#define LED_RED_POL  0     //!< The polarity of the red LED 

// Configuration declaration for the YELLOW LED wiring
#define LED_YELLOW_PORT PORTB //!< The ATMega port where the red LED is wired
#define LED_YELLOW_PIN  5     //!< The ATMega pin where the red LED is wired 
#define LED_YELLOW_POL  0     //!< The polarity of the red LED 



#define DATA_ID     0X90
#define INPUT_ID    0X77

//! \fn void initCANBus()
//! \brief Intialize the CAN Bus.
//!
//! Function that initializes the CANBus according to the wiring and the needed speed (500kb/s).
//! It also enable the CAN module interruptions
void initCANBus(){
    // Enable of CAN transmit
    DDRC|=0x80; // Configuration of the standby pin as an output
                // (for a transceiver MCP2562 with the stby pin wired to the uC PC7)
    // Enable the MCP2562 (STANDBY to 0)
    PORTC &= 0x7F; // Activation of the MCP2562
                   // (for a transceiver MCP2562 with the stby pin wired to the uC PC7)

    // Initialization of CAN module
    CANGCON  = (1 << SWRES); // Reset CAN module
    CANGCON  = 0x02; // Enable CAN module

    CANBT1 = 0x06; // Speed Config (500kb/s)
    CANBT2 = 0x04;
    CANBT3 = 0x13;

    CANHPMOB = 0x00; // No priority between the CAN MOB

    // Enable CAN interruptions and especially reception interruptions
    CANGIE |= 0xA0;
}

//! \fn void initCANMOB()
//! \brief Intialize the CAN MOB.
//!
//! Intialize the CAN MOB. A MOB can be seen as a "CAN socket".
//! In this case the MOB1 is used to receive remote request
//! and MOB0 is used to send the Ultra sound value
void initCANMOB(){
    CANPAGE = 0x10; // selection of MOB1 (reveice request)

    CANIDT4 = 0x04; // Config as reception remote (rtr = 1)
    CANIDT3 = 0x00;
    CANIDT2 = (uint8_t)( (INPUT_ID & 0x00F)<< 5 ); // using the constant to configure the remote request identifier
    CANIDT1 = (uint8_t)( INPUT_ID >> 3 );

    CANIDM4 = 0x04; // mask over the rtr value
    CANIDM3 = 0xFF; // mask over the identifier (we only want an interruption if the received CAN message is a remote request with the corresponding identifier)
    CANIDM2 = 0xFF;
    CANIDM1 = 0xFF;

    CANCDMOB = 0x80; // Config MOB as reception
    
    CANIE2 = 0x02; // enable the interruptions over the MOB 1

    sei(); // set enable interruption
}


Led redLed(&LED_RED_PORT, LED_RED_PIN, LED_RED_POL); //!< The RED Led
Led yellowLed(&LED_YELLOW_PORT, LED_YELLOW_PIN, LED_YELLOW_POL); //!< The RED Led


void sendCANdata(uint8_t id, uint8_t dlc, uint8_t* data){
CANPAGE  = 0x00; //Selection of MOB 0

CANIDT4 = 0x00; // Config as data (rtr = 0)
CANIDT3 = 0x00;
CANIDT2 = (uint8_t)( (id & 0x00F)<< 5 ); // set the identifier
CANIDT1 = (uint8_t)( id >> 3 );

for(uint8_t i=0; i< dlc; i++){
CANMSG = data[i];
CANPAGE = 0x00 + 1 + i;
}

CANCDMOB = 0x40 + dlc;// send the message using the MOB 0 - DLC = 8

while ( !(CANSTMOB & (1 << TXOK)));
}


//! \fn int main(void)
//! \brief Main function.
//! It makes the led blink when starting, then initialize the CANBus and finally loop forever
int main(void) {

    initCANBus(); // intialization of the CAN Bus
    initCANMOB(); // intialization of the CAN MOB

    DDRC |= 0x40; // Enable SPI level adaptation
    PORTC |= 0x40;

    Fmt1030 imu;
    imu.fmt1030_init();
    MtsspConfiguration notification;
    EulerAngles mesure_angles;

    _delay_ms(200);

    // initialisation de l'IMU
    imu.goToConfig();
    _delay_ms(100);
    imu.setOutputConfiguration(2, XDI_EULERANGLES, 10);
    _delay_ms(100);
    imu.gotoMeasurement();
    _delay_ms(100);

    for(int i=0; i<4; i++){
        redLed.blink(50);
        yellowLed.blink(50);
    }

    //envoi sur bus can des datas
    uint8_t candata[8];

    while(1){
        // lecture du nombre d'octet a lire dans le registre mesure
        notification = imu.read_pipe_status();
        imu.readMesure(notification.measurementMessageSize);

        // lecture du registre mesure
        mesure_angles = imu.getEulerAngles();
 

        candata[0] = 0;
        memcpy(&candata[1], &(mesure_angles.roll), sizeof(float));
        sendCANdata(DATA_ID,5,candata);
        _delay_ms(10);
    
        candata[0] = 1;
        memcpy(&candata[1], &(mesure_angles.pitch), sizeof(float));
        sendCANdata(DATA_ID,5,candata);
        _delay_ms(10);

        candata[0] = 2;
        memcpy(&candata[1], &(mesure_angles.yaw), sizeof(float));
        sendCANdata(DATA_ID,5,candata);

        _delay_ms(100);
    }
}



//! \fn ISR(CAN_INT_vect)
//! \brief CAN interruption.
//! This function is called when an CAN interruption is raised.
//! When it appens, the IMU card blinks...
ISR(CAN_INT_vect){
    cli(); // disable the interruption (no to be disturbed when dealing with one)

    yellowLed.blink(50); // blink the LED
    redLed.blink(50); // blink the LED

    CANPAGE = 0x10; // Selection of MOB 1
    CANSTMOB = 0x00; // Reset the status of the MOB
    CANCDMOB = 0x80; // Config as reception MOB
    CANIE2 = 0x02; // Enable the interuption over MOB 1 (for the next one)

    sei(); // enable the interruptions
}
