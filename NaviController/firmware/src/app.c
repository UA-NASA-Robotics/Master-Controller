// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************

#include "app.h"
#include "Macros.h"
#include "Comunications.h"
#include "FastTransfer.h"
#include "CAN_Handler/CANFastTransfer.h"
#include "Definitions.h"
#include "changeNotification.h"
#include "Timers.h"
#include "uart_Handler.h"
#include "CAN_Handler/CAN.h"
#include "Pozyx.h"
#include "Telemetry.h"
#include "PathFollowing.h"
#include "Macro_Handler/Macro_Mgr.h"
#include "DataPublishing.h"
#include "Map.h"
// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************

APP_DATA appData;


bool isLoaded = false;

timers_t sec, ms100;
timers_t bootTimer, ledTime, watchDog, receiveTimer, BlinkTime;

void APP_Initialize(void) {

    /* Place the App state machine in its initial state. */
    appData.state = APP_STATE_INIT;
    

    LED1 = off;
    LED2 = off;
    LED3 = off;
    LED4 = off;

    /* Initializing Timers */
    setTimerInterval(&bootTimer, 3000);
    setTimerInterval(&watchDog, 100);
    setTimerInterval(&BlinkTime, 700);
    setTimerInterval(&receiveTimer, 20);
    setTimerInterval(&sec, 1000);
    setTimerInterval(&ms100, 100);
    setTimerInterval(&ledTime, 50);


    /* Turning on the Timer the Runs the System's timers_t */
    DRV_TMR0_Start();

    initCANISRs();
    initCANFT();

    //InitFastTransferModule(&MotorFT, Motor_UART, MY_ADDRESS, Send_put, Buffer_Get, Buffer_Size, Buffer_Peek);
    //InitFastTransferModule(&GyroFT, Gyro_UART, MY_ADDRESS, Send_put, Buffer_Get, Buffer_Size, Buffer_Peek);



    DRV_CAN0_Open();

    InitializePathPlanning();

    isLoaded = true;

    /* Publish data on the global bus in a certain period */
    InitDataPublishing();
    //initGlobalData(DEVICE_STATUS, getLoadedState, 500);
    initGlobalData(DEVICE_MACRO, getRunningMacros, 500);

    /* Play pattern on the LEDs */
    while (!timerDone(&bootTimer)) {
        LED1 ^= 1;
        delay(50);
        LED2 ^= 1;
        delay(50);
        LED3 ^= 1;
        delay(50);
        LED4 ^= 1;
        delay(50);
    }
    //    handleCANmacro(1<<2, 0);
//
//    int y;
//    for (y = 0; y < 20; y++) {
//        addObtaclePoint(15, y);
//    }
    //    for(y = 10;y<getWorldWidth();y++){
    //        addObtaclePoint(35, y);
    //    }


    appData.state = APP_STATE_INIT;
}



int NEXT_APP_STATE = 0;

void APP_Tasks(void) {

    /* Check the application's current state. */
    switch (appData.state) {
            /* Application's initial state. */
        case APP_STATE_INIT:
        {
            bool appInitialized = true;

            if (appInitialized) {
                appData.state = APP_STATE_WATCHDOG;
            }
            break;
        }
        case APP_STATE_WATCHDOG:
        {
            //WatchDogToggle();
            appData.state = APP_STATE_COMS_CHECK;
            break;
        }
        case APP_STATE_SERVICE_MACRO:
        {
            if (getRunningMacros() != 0) {
                runMacros();

            }
            if (timerDone(&sec)) {
                LED4 ^= 1;
            }
            appData.state = APP_STATE_AWAITING_RESPONSE;
            break;
        }
        case APP_STATE_COMS_CHECK:
        {
            receivePozyx();
            handleMacroStatus();
            publishData();
            appData.state = APP_STATE_SERVICE_MACRO;
            break;
        }
        case APP_STATE_AWAITING_RESPONSE:
        {
            appData.state = APP_STATE_WATCHDOG;
            break;
        }

        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}

bool getLoadedState() {
    return isLoaded;
}

/*******************************************************************************
 End of File
 */
