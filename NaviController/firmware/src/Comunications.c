
#include "Comunications.h"
#include "CAN.h"
#include "CANFastTransfer.h"
#include "uart_Handler.h"
#include "FastTransfer.h"
#include "Macros.h"



//- Getting the status of the other controller processes for autonomy
//  so for either the gyro or motor controller, when it is done with a macro
//  it will send a done status through fastTransfer

bool SafeForMacros = false;

void updateFTdata() {
    MotorFT.ReceivedData[1] = 0;
    if (receiveData(&GyroFT)) {
        if (GyroFT.ReceivedData[1] == 1) {
            if (!isMacroRunning()) {
                sendMacroDone();
            }
            setGyroState(DONE);
        } else {
            setGyroState(RUNNING);
        }
        GyroFT.ReceivedData[1] = 0;
    }
    if (receiveData(&MotorFT)) {

        if (MotorFT.ReceivedData[1] == 1 || pinState(&MotorPin1)) {
            LED4 ^= 1;
            if (!isMacroRunning()) {
                sendMacroDone();
            }
            setMotorState(DONE);
        } else {
            setMotorState(RUNNING);
        }
        if (MotorFT.ReceivedData[MACRO_COMMAND_INDEX] == 0) {
            //            if(!isMacroRunning())
            //            {
            //                LED4 ^=1;
            //                sendMacroDone();
            //            }
        }
        MotorFT.ReceivedData[1] = 0;
    }
    receiveData(&PozyxFT);
    //printf("X: %d\rY: %d\rHeading: %d\r",PozyxFT.ReceivedData[1],PozyxFT.ReceivedData[2],PozyxFT.ReceivedData[3]);

}

void sendMacroDone() {
    ToSendCAN(CAN_COMMAND_INDEX, 0);
    sendDataCAN(ROUTER_ADDRESS);
}

void sendMacroClear() {
    ToSend(&GyroFT, MACRO_COMMAND_INDEX, 0);
    ToSend(&MotorFT, MACRO_COMMAND_INDEX, 0);
    sendData(&GyroFT, GYRO_ADDRESS);
    sendData(&MotorFT, MOTOR_ADDRESS);

}

void setMotorMacro(int macroIndex, int data) {
    ToSend(&MotorFT, MACRO_COMMAND_INDEX, macroIndex);

#ifdef REVERSE_DRIVE_DIRECTION 
    ToSend(&MotorFT, UART_COMMAND_DATA_INDEX, -data);
#else
    ToSend(&MotorFT, UART_COMMAND_DATA_INDEX, data);
#endif
    sendData(&MotorFT, MOTOR_ADDRESS);
}

void setMacroSafety(bool state) {
    SafeForMacros = state;
}

bool getMacroSafety() {
    return SafeForMacros;
}

bool updateCanMACROcoms() {
    ReceiveDataCAN();
    if (getNewDataFlagStatus(1 << CAN_COMMAND_INDEX) && getCANFastData(CAN_COMMAND_INDEX) == 0) {
        setMacroSafety(false);
        return false;
    }
    setMacroSafety(true);
    return true;
}

void setGyroMacro(int macroIndex, int data) {
    ToSend(&GyroFT, MACRO_COMMAND_INDEX, macroIndex);
    ToSend(&GyroFT, UART_COMMAND_DATA_INDEX, data);
    sendData(&GyroFT, GYRO_ADDRESS);
}