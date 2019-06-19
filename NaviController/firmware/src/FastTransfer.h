/* 
 * File:   FastTransfer.h
 * Author: John
 *
 * Created on March 1, 2018, 8:27 AM
 */

#ifndef FASTTRANSFER_H
#define	FASTTRANSFER_H

#include "uart_Handler.h"
#include "bufferHandler.h"



#define BUFFER_SIZE UART_BUFFER_SIZE
#define MODULE_ADDRESS 0x03



#define polynomial 0x8C  //polynomial used to calculate crc
#define CRC_COUNT 5 // how many AKNAKs are stored
#define CRC_DEPTH 3  // how many pieces of data are stored with each CRC send event
#define CRC_BUFFER_SIZE (CRC_COUNT * CRC_DEPTH) //crc buffer size 5 deep and 3 bytes an entry

union stuff
{ // this union is used to join and disassemble integers
    unsigned char parts[2];
    uint16_t integer;
};
union stuff group;
typedef struct
{
    int ModuleADDRESS;
    RingBuffer_t *DataBuffer;
    short ReceivedData[20];
    bool ReceivedDataFlags[20];
    bool IsNewData;
    UART_Object_t COMdriver;
    void (*serial_write)( UART_Object_t* , unsigned char);
    unsigned char (*serial_read)(RingBuffer_t* );
    int (*serial_available)(RingBuffer_t*);
    unsigned char (*serial_peek)(RingBuffer_t*);
                                                                                                                
}FastTransfer_t;

//uint8_t GyroFTRXarray[BUFFER_SIZE];
FastTransfer_t GyroFT;
//uint8_t MotorFTRXarray[BUFFER_SIZE];
FastTransfer_t MotorFT;

FastTransfer_t PozyxFT;


//#define the _uartModules so that it will be easier to configure fastTransfer
//names are references to the board file



/*SAMPLE INIT FUNCTION CALL
 * 
 * FastTransfer_t myFastTransfer
 * FastTransferInit(FastTrans_1,UartModule_Sample, MyAddress, Send_put, bufferGet,UART_buff_size,UART_buff_peek);
*/
void InitFastTransferModule(
        FastTransfer_t* _this,
        SYS_MODULE_INDEX _uartModule,
        //int *_receiverAddress,
        int _myAddress, 
        void (*_writeData)(UART_Object_t* _TXobject, unsigned char _data), 
        unsigned char (*_readData)(RingBuffer_t*),
        int (*_isAvailable)(RingBuffer_t*), 
        unsigned char (*_dataPeek)(RingBuffer_t*));

void CloseFastTransferModule(FastTransfer_t* _this);
void ToSend(FastTransfer_t *_this,short _address, int _data);
bool sendData(FastTransfer_t *_FTobject, unsigned char whereToSend);
bool receiveData(FastTransfer_t *_this);
int16_t getFTReceivedData(FastTransfer_t *_this, unsigned char index);
bool isFTFlagSet(FastTransfer_t *_this, unsigned char index);
#endif	/* FASTTRANSFER_H */
