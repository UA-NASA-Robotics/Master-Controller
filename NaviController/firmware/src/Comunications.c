
#include "Comunications.h"
#include "CAN_Handler/CAN.h"
#include "CAN_Handler/CANFastTransfer.h"
#include "uart_Handler.h"
#include "FastTransfer.h"
#include "Macros.h"
#include "app.h"
#include "Timers.h"
#include "Motor.h"
#include "Definitions.h"


int lastMacroIndex, lastData;

bool SafeForMacros = false;
timers_t retransmitTimer;

//- Getting the status of the other controller processes for autonomy
//  so for either the gyro or motor controller, when it is done with a macro
//  it will send a done status through fastTransfer

void setGyroMacro(int macroIndex, int data) {
    ToSendCAN(CAN_COMMAND_INDEX, macroIndex);
    ToSendCAN(CAN_COMMAND_DATA_INDEX, data);
    sendDataCAN(GYRO_CONTROLLER);
}

void setMotorMacro(int macroIndex, short data) {
    ToSendCAN(CAN_COMMAND_INDEX, macroIndex);
    ToSendCAN(CAN_COMMAND_DATA_INDEX, data);
    sendDataCAN(MOTOR_CONTROLLER);
}



/* Returns the macro status of the motor controller */
bool getMotorControllerStatus() {
    return getCANFastData(FT_GLOBAL, getGBL_MACRO_INDEX(MOTOR_CONTROLLER));
}

/* Returns the status of the gyro controller and if we haven't received 
 * confirmation of a macro then a retransmission of the last 
 * macro sent is resulting */
bool getGyroControllerStatus() {
    
    // returning the macros that the gyro may be running 
    return getCANFastData(FT_GLOBAL, getGBL_MACRO_INDEX(GYRO_CONTROLLER));
}
