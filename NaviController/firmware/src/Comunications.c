
#include "Comunications.h"
#include "CAN_Handler/CAN.h"
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
