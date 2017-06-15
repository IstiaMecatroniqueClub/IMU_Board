#include "fmt1030.h"
#include <stdint.h>
#include <stdarg.h>
#include <string.h>
#include <util/delay.h>



// Contructor of the class, do nothing
Fmt1030::Fmt1030()
{
}

// Destructor
Fmt1030::~Fmt1030()
{
}

// Initialization of the SPI
uint8_t Fmt1030::fmt1030_init() {
    _spi.spi_init_master(STD_DIRECTION);
    return 1;
}

//---------------------------------------//
//          LOW LEVEL FUNCTIONS          //
//---------------------------------------//

// Compute and return the checksum
uint8_t Fmt1030::cs_calculator() {
    uint8_t tmp = 0xFF + _messageXbus.mid + _messageXbus.dataLenght;
    if (_messageXbus.dataLenght != 0) {
        for (int i=0; i<_messageXbus.dataLenght; i++) {
            tmp += _messageXbus.data[i];
        }
    }
    tmp = 0x100 - tmp;
    return tmp;
}

// Send an OPCode on the SPI
void Fmt1030::sendOpCode(uint8_t opCode) {
    _spi.spi_tranceiver(opCode);
    for (int filler=0; filler<3; filler++) {
        _spi.spi_tranceiver(filler);
    }
}

// Send a Xbus message
uint8_t Fmt1030::sendXbusRawMessage(uint8_t opCode) 
{
    _messageXbus.cs = this->cs_calculator();
    this->sendOpCode(opCode);
    _spi.spi_tranceiver(_messageXbus.mid);
    _spi.spi_tranceiver(_messageXbus.dataLenght);
    if (_messageXbus.dataLenght != 0)
    {
        for (int i=0; i<_messageXbus.dataLenght; i++) {
            _spi.spi_tranceiver(_messageXbus.data[i]);
        }
    }
    _spi.spi_tranceiver(_messageXbus.cs);
    return 1;
}

const MessageXbus& Fmt1030::getMessage()
{
	return _messageXbus;
}

//---------------------------------------//
//          HIGH LEVEL FUNCTIONS         //
//---------------------------------------//

const MtsspConfiguration& Fmt1030::read_pipe_status() {
    uint8_t tabrecep[4];
    _spi.spi_begin_transceive();
    this->sendOpCode(0x04); // PipeStatus OpCode
    for (int i=0; i<4; i++)
    {
        tabrecep[i] = _spi.spi_tranceiver(0);
    }
    _spi.spi_stop_transceive();
    _notification.notificationMessageSize = (tabrecep[1] << 8) + tabrecep[0];
    _notification.measurementMessageSize = (tabrecep[3] << 8) + tabrecep[2];
    return _notification;
}

uint8_t Fmt1030::read_notification_pipe() {
    _spi.spi_begin_transceive();
    this->sendOpCode(0x05); // NotificationPipe OpCode
    uint8_t result = _spi.spi_tranceiver(0);
    _spi.spi_stop_transceive();
    return result;
}


uint8_t Fmt1030::goToConfig()
{
    _messageXbus.mid = XMID_GOTOCONFIG;
    _messageXbus.dataLenght = 0;
    _spi.spi_begin_transceive();
    this->sendXbusRawMessage(0x03);
    _spi.spi_stop_transceive();

    // check the Ack
    uint8_t ack, escape=0;
    do 
    {
        ack = this->read_notification_pipe();
        escape++;
    } while ((ack != XMID_GOTOCONFIGACK) & (escape != 0xFF));
    if (ack == XMID_GOTOCONFIGACK){
        return XMID_GOTOCONFIGACK;
    }
    else
        return 0x00;
}

uint8_t Fmt1030::gotoMeasurement()
{
    _messageXbus.mid = XMID_GOTOMEASUREMENT;
    _messageXbus.dataLenght = 0;
    _spi.spi_begin_transceive();
    this->sendXbusRawMessage(0x03);
    _spi.spi_stop_transceive();

    // check the Ack
    uint8_t ack, escape=0;
    do
    {
        ack = this->read_notification_pipe();
        escape++;
    } while ((ack != XMID_GOTOMEASUREMENTACK) & (escape != 0xFF));
    if (ack == XMID_GOTOMEASUREMENTACK){
        return XMID_GOTOMEASUREMENTACK;
    }
    else
        return 0x00;
}

uint8_t Fmt1030::readMesure(uint16_t nbOctToRead)
{
    uint8_t tabrecep[nbOctToRead];
    _spi.spi_begin_transceive();
    this->sendOpCode(0x06);
    for (uint16_t k=0; k<nbOctToRead; k++) {
        tabrecep[k] = _spi.spi_tranceiver(0);
    }
    _spi.spi_stop_transceive();

    // The received frame for euler data : tabrecep (reduced XBus message)):
    //  1byte : MID = 0x36 => MTData2
    //  1byte : LEN = 0x0F => 15 data bytes
    //  Data :
    //      2 bytes : DID = 0x2030 => identifier of the data (euler angles)
    //      1 byte  : DLEN = 0x0C => 12 data (4 bytes for each angle)
    //      3x4 bytes : 4 Roll, 4 Pitch, 4 Yaw
    //  1 byte : Checksum 
    
    // Note that the value of Roll Pitch and Yaw are IEEE float32,
    // and the bytes are given from the lower weight to the upper weight
    
    // We record those data into float values
    for (uint8_t i=0; i<4; i++) { // memcpy in reverse order!
        memcpy( ((uint8_t*)(&_eulerAngles.roll)) + 3 - i, &tabrecep[i+5], 1);
    }
    for (uint8_t i=0; i<4; i++) { // memcpy in reverse order!
        memcpy( ((uint8_t*)(&_eulerAngles.pitch)) + 3 - i, &tabrecep[i+9], 1);
    }
    for (uint8_t i=0; i<4; i++) { // memcpy in reverse order!
        memcpy( ((uint8_t*)(&_eulerAngles.yaw)) + 3 - i, &tabrecep[i+13], 1);
    }

    return 1; // affectation_done;
}

const EulerAngles& Fmt1030::getEulerAngles() {
    return _eulerAngles;
}

uint8_t Fmt1030::setOutputConfiguration(uint8_t nb_Arg, ...) {
    va_list listImput;
    va_start (listImput, nb_Arg);
    _messageXbus.dataLenght = 0;
    for (int i = 0; i < nb_Arg; i++) {
        int currentImput = va_arg (listImput, int);
        _messageXbus.data[i*2] = ((currentImput >> 8) & 0x00FF);
        _messageXbus.data[(i*2)+1] = (currentImput & 0x00FF);
        _messageXbus.dataLenght += 2;
    }
    va_end (listImput);

    _messageXbus.mid = XMID_SETOUTPUTCONFIG;
    _spi.spi_begin_transceive();
    this->sendXbusRawMessage(0x03);
    _spi.spi_stop_transceive();

    // check the Ack
    uint8_t ack, escape=0;
    do
    {
        ack = this->read_notification_pipe();
        escape++;
    } while ((ack != XMID_OUTPUTCONFIG) & (escape != 0xFF));
    if (ack == XMID_OUTPUTCONFIG){
        return XMID_OUTPUTCONFIG;
    }
    else
        return 0x00;
}
