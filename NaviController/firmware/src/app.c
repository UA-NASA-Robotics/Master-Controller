// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************

#include "app.h"
#include "Macros.h"
#include "Comunications.h"
#include "FastTransfer.h"
#include "CANFastTransfer.h"
#include "Definitions.h"
#include "changeNotification.h"
#include "Timers.h"
#include "uart_Handler.h"
#include "CAN.h"
#include "Pozyx.h"
#include "Telemetry.h"
// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************

APP_DATA appData;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback functions.
 */

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************


/* TODO:  Add any necessary local functions.
 */


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Remarks:
    See prototype in app.h.
 */
bool isLoaded = false;

timers_t sec, ms100;
timers_t bootTimer, ledTime, watchDog, receiveTimer, BlinkTime;

void APP_Initialize(void) {
    
    /* Place the App state machine in its initial state. */
    appData.state = APP_STATE_INIT;
    initChangeNotification();
    
    LED1 = off;
    LED2 = off;
    LED3 = off;
    LED4 = off;
    
    /* Initializing Timers */
    setTimerInterval(&bootTimer, 8000);
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

    InitFastTransferModule(&MotorFT, Motor_UART, MY_ADDRESS, Send_put, Buffer_Get, Buffer_Size, Buffer_Peek);
    InitFastTransferModule(&GyroFT, Gyro_UART, MY_ADDRESS, Send_put, Buffer_Get, Buffer_Size, Buffer_Peek);
    
    InitPozyx();


    DRV_CAN0_Open();
    
    isLoaded = true;


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

    appData.state = APP_STATE_INIT;

}


/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */



intPin_t* awaitPin;
int NEXT_APP_STATE = 0;
void APP_Tasks(void) {

    /* Check the application's current state. */
    switch (appData.state) {
        /* Application's initial state. */
        case APP_STATE_INIT:
        {
            bool appInitialized = true;

            testPathAlgorithm();
            while(1){
                //runMacro();
            }
            if (appInitialized) {
                appData.state = APP_STATE_WATCHDOG;
                MotorFT.ReceivedData[1] = 0;
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
            if (isMacroRunning()) {
                if (timerDone(&ms100)) {
                    LED2 ^= 1;
                }
            } else {
                if (timerDone(&sec)) {
                    LED1 ^= 1;
                }
            }
            runMacro();
            appData.state = APP_STATE_AWAITING_RESPONSE;
            break;
        }
        case APP_STATE_COMS_CHECK:
        {

            if (timerDone(&receiveTimer)) {
                // CAN FastTransfer Receive
                if (ReceiveDataCAN()) {

                    if (getNewDataFlagStatus(0x02)) {
                        resetTimer(&BlinkTime);
                        while (!timerDone(&BlinkTime)) {
                            while (!timerDone(&ms100));
                            LED1 ^= 1;
                            LED4 ^= 1;
                        }

                    }
                    if (getNewDataFlagStatus(1 << CAN_COMMAND_INDEX)) {
                        handleCANmacro(getCANFastData(CAN_COMMAND_INDEX), getCANFastData(CAN_COMMAND_DATA_INDEX)); 
                    }
                }
                updateFTdata();
            }
            appData.state = APP_STATE_SERVICE_MACRO;
            break;
        }
        case APP_STATE_AWAITING_RESPONSE:
        {
            //            if(pinState(&GyroPin1))
            //            {
            //                printf("Toggle G1\r\n");
            //            }
            //            if(pinState(&GyroPin2))
            //            {
            //                printf("Toggle G2\r\n");
            //            }
            //            if(pinState(&MotorPin1))
            //            {
            //                printf("IO5 From Motor Toggled\r\n");
            //            }
            //            if(pinState(&MotorPin2))
            //            {
            //                printf("IO6 From Motor Toggled\r\n");
            //            }

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

void setAwaitPin(intPin_t* pin, int nextState) {
    appData.state = APP_STATE_AWAITING_RESPONSE;
    awaitPin = pin;
    NEXT_APP_STATE = nextState;
}

/*******************************************************************************
 End of File
 */
