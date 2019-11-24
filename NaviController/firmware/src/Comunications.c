
#include "Comunications.h"
#include "CAN.h"
#include "CAN_Handler/CANFastTransfer.h"
#include "uart_Handler.h"
#include "FastTransfer.h"
#include "Macros.h"
#include "app.h"
#include "Timers.h"


int lastMacroIndex, lastData;

bool SafeForMacros = false;
timers_t retransmitTimer;
unsigned char gyroState = DONE, motorState = DONE;

//- Getting the status of the other controller processes for autonomy
//  so for either the gyro or motor controller, when it is done with a macro
//  it will send a done status through fastTransfer

void updateFTdata() {
    MotorFT.ReceivedData[1] = 0;
    MotorFT.ReceivedData[MACRO_COMMAND_INDEX] = 0;
    /* We have received info from the gyro */
    if (receiveData(&GyroFT)) {

        if (GyroFT.ReceivedData[1] == 1) {
            /* are we running autonomy or just a macro */
            if (!isMacroRunning()) {
                sendMacroDone();
            }
            setGyroState(DONE);
        } else {
            setGyroState(RUNNING);
        }
        if (GyroFT.ReceivedData[MACRO_COMMAND_INDEX] == lastMacroIndex)
        {
            setGyroState(RUNNING);
            lastMacroIndex = 0;
        }
        GyroFT.ReceivedData[1] = 0;
    }
    if (receiveData(&MotorFT)) {

        if (MotorFT.ReceivedData[1] == 1) {// || pinState(&MotorPin1)) {
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

}

/* Transmits to the Router Card the a Macro Has Completed*/
void sendMacroDone() {
    ToSendCAN(CAN_COMMAND_INDEX, 0);
    sendDataCAN(ROUTER_ADDRESS);
}

/* Clears the macro on the gyro and motor controllers */
void sendMacroClear() {
    ToSend(&GyroFT, MACRO_COMMAND_INDEX, 0);
    ToSend(&MotorFT, MACRO_COMMAND_INDEX, 0);
    sendData(&GyroFT, GYRO_ADDRESS);
    sendData(&MotorFT, MOTOR_ADDRESS);
    setTimerInterval(&retransmitTimer, 500);
}

void setMotorMacro(int macroIndex, int data) {
    ToSend(&MotorFT, MACRO_COMMAND_INDEX, macroIndex);

#ifdef REVERSE_DRIVE_DIRECTION 
    ToSend(&MotorFT, UART_COMMAND_DATA_INDEX, -data);
#else
    ToSend(&MotorFT, UART_COMMAND_DATA_INDEX, data);
#endif
    sendData(&MotorFT, MOTOR_ADDRESS);
    setMotorState(RUNNING);
}

void setMacroSafety(bool state) {
    SafeForMacros = state;
}

bool getMacroSafety() {
    updateCanMACROcoms();
    return SafeForMacros;
}

bool updateCanMACROcoms() {
    ReceiveDataCAN(FT_LOCAL);
    if (getNewDataFlagStatus(FT_LOCAL,1 << CAN_COMMAND_INDEX) && getCANFastData(FT_LOCAL,CAN_COMMAND_INDEX) == 0) {
        setMacroSafety(false);
        handleCANmacro(getCANFastData(FT_LOCAL,CAN_COMMAND_INDEX), getCANFastData(FT_LOCAL,CAN_COMMAND_DATA_INDEX));
        return false;
    }
    setMacroSafety(true);
    return true;
}

void setGyroMacro(int macroIndex, int data) {
    lastMacroIndex = macroIndex;
    lastData = data;
    ToSend(&GyroFT, MACRO_COMMAND_INDEX, macroIndex);
    ToSend(&GyroFT, UART_COMMAND_DATA_INDEX, data);
    sendData(&GyroFT, GYRO_ADDRESS);
    setGyroState(PENDING);
    setTimerInterval(&retransmitTimer,500);
    resetTimer(&retransmitTimer);
}

void setGyroState(unsigned char  state) {
    gyroState = state;
}

void setMotorState(unsigned char state) {
    motorState = state;
}

bool getMotorControllerStatus() {

    return motorState;
}


/* Returns the status of the gyro controller and if we haven't received 
 * confirmation of a macro then a retransmission of the last 
 * macro sent is resulting */
bool getGyroControllerStatus() {
    if (gyroState == PENDING && timerDone(&retransmitTimer)) {
        setGyroMacro(lastMacroIndex, lastData);
    }
    return gyroState;
}
