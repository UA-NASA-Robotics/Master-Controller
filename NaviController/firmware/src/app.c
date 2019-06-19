/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It 
    implements the logic of the application's state machine and it may call 
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
// DOM-IGNORE-END


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
#include "Heading.h"
// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.
    
    Application strings and buffers are be defined outside this structure.
 */

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

timers_t sec, ms100, ms10;
timers_t bootTimer, ledTime, watchDog, receiveTimer, BlinkTime;
int lastCommandID = -1, lastCommandData = -1;
    point_t thePoint = {200,200};

void APP_Initialize(void) {
    /* Place the App state machine in its initial state. */
    appData.state = APP_STATE_INIT;
    initChangeNotification();

    setTimerInterval(&bootTimer, 8000);
    setTimerInterval(&watchDog, 100);
    setTimerInterval(&BlinkTime, 700);
    setTimerInterval(&receiveTimer, 20);
    setTimerInterval(&sec, 1000);
    setTimerInterval(&ms100, 100);
    setTimerInterval(&ms10, 10);
    setTimerInterval(&ledTime, 50);

    DRV_TMR0_Start();

    initCANISRs();
    initCANFT();

    InitFastTransferModule(&MotorFT, Motor_UART, MY_ADDRESS, Send_put, Buffer_Get, Buffer_Size, Buffer_Peek);
    InitFastTransferModule(&GyroFT, Gyro_UART, MY_ADDRESS, Send_put, Buffer_Get, Buffer_Size, Buffer_Peek);
    InitPozyx();


    LED1 = off;
    LED2 = off;
    LED3 = off;
    LED4 = off;


    DRV_CAN0_Open();
    isLoaded = true;


    //resetTimer(&bootTimer);
    while (!timerDone(&bootTimer)) {
        LED1 ^= 1;
        while (!timerDone(&ledTime));
        LED2 ^= 1;
        while (!timerDone(&ledTime));
        LED3 ^= 1;
        while (!timerDone(&ledTime));
        LED4 ^= 1;
        while (!timerDone(&ledTime));
    }


    //    setMacro(ROTATION_COMMAND, 90);
    //    
    //    setAwaitPin(&GyroPin1,APP_STATE_SERVICE_MACRO);
    appData.state = APP_STATE_INIT;
    //testStart();
    //testFull();
    //goToLocation(thePoint);
    //setMotorMacro(ENCODER_COMMAND, 500);
    
    //goToLocation((point_t){200,200});
    //goToLocation((point_t) {200, 400});
}


/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */



intPin_t* awaitPin;
int NEXT_APP_STATE = 0;
bool TempFlag = false;
void APP_Tasks(void) {

    /* Check the application's current state. */
    switch (appData.state) {
            /* Application's initial state. */
        case APP_STATE_INIT:
        {
            bool appInitialized = true;
            //continueRunningMacro=autonomousMacro;
            //configureMacro(FULL_AUTONOfMY,0);
            //handleCANmacro(GYRO_ROTATION,90);
            //goToLocation((point_t){200,300});
            if (appInitialized) {
                //printf(">>Starting...\r");
                appData.state = APP_STATE_WATCHDOG;
                MotorFT.ReceivedData[1] = 0;
                //setGyroMacro(ROTATION_COMMAND, 90);
               
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
//            if(!TempFlag && goToLocation((point_t){100,200}))
//            {
//                TempFlag = true;
//            }
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
                        //if(lastCommandID != getCANFastData(CAN_COMMAND_INDEX) )
                        // {

                        lastCommandID = getCANFastData(CAN_COMMAND_INDEX);
                        lastCommandData = getCANFastData(CAN_COMMAND_DATA_INDEX);
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
