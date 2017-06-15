#ifndef FMT1030_H
#define FMT1030_H

//! \file fmt1030.h
//! \brief FMT1030 class. 
//! \author Franck Mercier, Remy Guyonneau
//! \date 2017 05 12

#include <stdint.h>
#include "spi.h"

#define     XMID_WAKEUP             0x3E
#define     XMID_WAKEUPACK          0x3F
#define     XMID_REQDID             0x00
#define     XMID_DEVICEID           0x01
#define     XMID_GOTOCONFIG         0x30
#define     XMID_GOTOCONFIGACK      0x31
#define     XMID_GOTOMEASUREMENT    0x10
#define     XMID_GOTOMEASUREMENTACK 0x11
#define     XMID_MIDATA2            0x36
#define     XMID_REQOUTPUTCONFIG    0xC0
#define     XMID_SETOUTPUTCONFIG    0xC0
#define     XMID_OUTPUTCONFIG       0xC1
#define     XMID_RESET              0x40
#define     XMID_RESETACK           0x41
#define     XMID_ERROR              0x42

#define     XDI_PACKETCOUNTER       0x1020
#define     XDI_SAMPLETIMEFINE      0x1060
#define     XDI_QUATERNION          0x2010
#define     XDI_ROTATIONMATRIX      0x2020
#define     XDI_EULERANGLES         0x2030
#define     XDI_DELTAV              0x4010
#define     XDI_ACCELERATION        0x4020
#define     XDI_RATEOFTURN          0x8020
#define     XDI_DELTAQ              0x8030
#define     XDI_MAGNETICFIELD       0xC020
#define     XDI_STATUSWORD          0xE020

#define     FREQ_100Hz              0x0064
#define     FREQ_50Hz               0x0032
#define     FREQ_25Hz               0x0019
#define     FREQ_20Hz               0x0014
#define     FREQ_10Hz               0x000A
#define     FREQ_5Hz                0x0005
#define     FREQ_4Hz                0x0004
#define     FREQ_2Hz                0x0002
#define     FREQ_1Hz                0x0001
#define     FREQ_NULL               0xFFFF

//! \struct EulerAngles
//! \brief Euler Angles structure 
//!
//! Euler Angles structure to save the roll, pitch and yaw on the same variable.
//! Each angle is coded using a float 32 bits.
typedef struct {
    float roll;  //!< The Roll angle value
    float pitch; //!< The Pitch angle value
    float yaw;   //!< The Yaw angle value
} EulerAngles;

//! \struct MessageXbus
//! \brief Message Xbuss structure 
//!
//! MessageXbus structure to handle message over a XBus communication.
typedef struct {
    uint8_t mid;        //!< The message identifier
    uint8_t dataLenght; //!< The message data length
    uint8_t data[100];  //!< The messsage data
    uint8_t cs;         //!< The message CheckSum
} MessageXbus;

//! \struct MtsspConfiguration
//! \brief Mtssp Configuration structure 
//!
//! MtsspConfiguration structure to handle MTSSP configuration
//! Each angle is coded using a float 32 bits.
typedef struct {
    uint16_t notificationMessageSize;
    uint16_t measurementMessageSize;
} MtsspConfiguration;

//! \class Fmt1030
//! \brief Fmt1030 class
//!
//! Fmt1030 class to use the FMT1030 IMU.
class Fmt1030
{
public:

    //! \brief fmt1030 constructor.
    Fmt1030();

    //! \brief fmt1030 destructor.
    ~Fmt1030();

    //! \brief fmt1030_init Initialisation
    //! \return 1 if the connexion is OK, 0 otherwise
    //! Initialisation, mainly the SPI initialisation
    uint8_t fmt1030_init();

    //! \brief cs_calculator computes the CheckSum
    //! \return the value of the computed CheckSum
    //! Computes the CheckSum of a Xbus message (data field of the _messageXbus variable)
    uint8_t cs_calculator();

    //! \brief sendOpCode Send the SPI OpCode
    //! \param[in] opCode   OpCode to send : 0x01   ProtocolInfo
    //!                                      0x02   ConfigureProtocol
    //!                                      0x03   ControlPipe
    //!                                      0x04   PipeStatus
    //!                                      0x05   NotificationPipe
    //!                                      0x06   MeasurementPipe
    //! Send the OpCode and the 3 filling bytes before sending/receiving SPI data
    void sendOpCode(uint8_t opCode);

    //! \brief sendXbusRawMessage   Send the XBus Message
    //! \param[in] OpCode : SPI OpCode to send first
    //! \return 1 if transmission ok
    uint8_t sendXbusRawMessage(uint8_t OpCode);

    //! \brief getMessage   Accesseur of the _messageXBus variable
    //! \return a const reference over the variable
    const MessageXbus& getMessage();

    //! \brief read_pipe_status Read the content of the Pipe Status register
    //! \return const reference over the content of the Pipe status register
    const MtsspConfiguration& read_pipe_status();

    //! \brief read_notification_pipe Read the content of the notification Pipe register
    //! \return the content of the notification Pipe register
    uint8_t read_notification_pipe();

    //! \brief goToConfig Function that sets the IMU into the Configuration mode
    //! \return Xbus Ack code (0x00 if error)
    uint8_t goToConfig();

    //!\brief GotoMeasurement Function that sets the IMU into the measurement mode
    //! \return Xbus Ack code (0x00 if error)
    uint8_t gotoMeasurement();

    //! \brief readMesure Function that reads the measurements done by the IMU
    //! \param[in] nbOctToRead Number of bytes to reads
    //! \return 1 if everything went well, 0 otherwise
    uint8_t readMesure(uint16_t nbOctToRead);

    //! \brief GetEulerAngles Function that returns the euler angle measurment
    //! \return a const reference over the computed euler angles
    const EulerAngles& getEulerAngles();

    //! \brief SetOutputConfiguration Fonction to define what value the IMU should return
    //! Fonction to define what value the IMU should return. Note that only the Euler Angle are handle for now
    //! \param[in] nb_Arg : number of parameters when calling the function
    //! \param[in] ... : code ID (XDI) of the data to return, following by a 16 bits interger corresponding to the return frequency
    //! \return Xbus Ack cod,  0x00 if error
    uint8_t setOutputConfiguration(uint8_t nb_Arg, ...);

private:

    Spi _spi; //!< To handle the SPI communication with the IMU

    MtsspConfiguration  _notification; //!< To handle MTSSP Configuration
    EulerAngles         _eulerAngles;  //!< To handle the Euler Angles
    MessageXbus         _messageXbus;  //!< To handle the XBus Message
};

#endif // FMT1030_H
