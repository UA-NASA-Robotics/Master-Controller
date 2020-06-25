#include "Pozyx.h"
#include "Macros.h"
#include "Definitions.h"
#include "Map.h"
#include "Definitions.h"
#include "CAN_Handler/CANFastTransfer.h"
#include "CompairitorMethods.h"
#include "Timers.h"
#include "Algorithms.h"
#include "LinkedList.h"
#include "Telemetry.h"
#include "RockDetection.h"
#include "app.h"
#include "A_Star.h"
#include "Motor.h"
#include "Macro_Handler/Macro_Mgr.h"
#include <stdint.h>

//*********************************************************
//                      DEFINITIONS
//*********************************************************

#define PointToDig                 (point_t){650,200}



bool autoWaiting = false;


unsigned char MacroState = 0;


point_t pathFrom = {10, 10};
point_t pathTo = {20, 20};
LL_t *RobotPath;


void addStopToPath(point_t _stopPoint);
bool RunPath(int nan);
void setWaiter(bool(*_waitingOn)());
bool(*waitingOn)();
bool waiting = false;
timers_t TimeOut, clearMacroTimer, ClearTimer;
timers_t Retransmit;
bool runningMacroData = 0;
bool MacroRunning = false;

bool isWaiting();
bool getMotorControllerStatus();
bool getGyroControllerStatus();
bool noWaiting();

bool gyroState = DONE, motorState = DONE;


typedef enum {
    Init = 0,
    DRIVE,
            Wait,
    MotorResend
} PathFollowStep_t;
PathFollowStep_t pathSteps = Init;




timers_t voidTime, voidTime2;

bool Dummy(int val) {
    if (voidTime.timerInterval != val) {
        setTimerInterval(&voidTime, val);
    }
    return timerDone(&voidTime);
}

#define TEST_DRIVE (uint16_t)(1<<2)

void handleMacroStatus() {
    ReceiveDataCAN(FT_GLOBAL);
    ReceiveDataCAN(FT_LOCAL);
    /* If a macro is seen on the global bus from the router card*/
    if (getNewDataFlagStatus(FT_GLOBAL, getGBL_MACRO_INDEX(ROUTER_CARD))) {

        int macroID = getCANFastData(FT_GLOBAL, getGBL_MACRO_INDEX(ROUTER_CARD));
        int macroDATA = getCANFastData(FT_GLOBAL, getGBL_MACRO_INDEX(ROUTER_CARD) + 1);
        handleCANmacro(macroID, macroDATA);
    }
    if (getNewDataFlagStatus(FT_LOCAL, CAN_COMMAND_INDEX)) {
        int macroID = getCANFastData(FT_LOCAL, CAN_COMMAND_INDEX);
        int macroDATA = getCANFastData(FT_LOCAL, CAN_COMMAND_DATA_INDEX);
        handleCANmacro(macroID, macroDATA);
    }

}

void handleCANmacro(short _macroID, short _macroDATA) {
    if (_macroID == 0) {
        clearMacros();
        if (RobotPath != NULL)
            LL_clear(RobotPath);
    } else {
        /* Add the macro to the queue*/
        switch (_macroID) {

            case TEST_DRIVE:
                setMacroCallback(RunPath, _macroDATA, TEST_DRIVE);
              
                break;
            default:
                break;
        }
    }
}

void testPathAlgorithm() {

    RobotPath = LL_init();

    receivePozyx();
    pathFrom = getLocation();
    getPolarPath(RobotPath, pathFrom, pathTo);
    //    addStopToPath((point_t) {
    //        100, 100
    //    });
    //    addStopToPath((point_t) {
    //        100, 0
    //    });
    //
    //    addStopToPath((point_t) {
    //        0, 0
    //    });
    //    addStopToPath((point_t){0, 0});
}


double gyroHeading, motorDist;
double myHeading = 0;
int data;
point_t testPoint;

bool RunPath(int nan) {
    if (getMotorControllerStatus() == DONE ) {
        if (pathSteps == Init) {
            /* Build the new path */
            testPathAlgorithm();
            /* Set the first stage if the state machine */
            pathSteps = DRIVE;
        } else if(pathSteps == Wait) {
            if (RobotPath != 0 || RobotPath->size < 1 || pathSteps != MotorResend) {
                LL_destroy(RobotPath);
                pathSteps = Init;
                return true;
            }else{
                pathSteps = DRIVE;
            }
        }
        receivePozyx();
        myHeading = getPozyxHeading();
        switch (pathSteps) {
            case DRIVE:
                /* Getting the next point in the path to go to */
                testPoint = (point_t) ((waypoint_t*) (RobotPath->first->data))->Endpoint;
                //data = (((waypoint_t*)(RobotPath->first->data))->Endpoint.x << 8) |(((waypoint_t*)(RobotPath->first->data))->Endpoint.y & 0xff);
                /* Packaging the location into a single byte to send to the motor processor */
                data = (uint16_t) ((testPoint.y & 0xFF) | ((testPoint.x & 0xFF) << 8));
                /* Sending the  X & Y location data in one 16 bit word to the motor processor for Macro Processing */
                setMotorMacro(AUTO_DRIVE_MACRO, (uint16_t) ((testPoint.y & 0xFF) | ((testPoint.x & 0xFF) << 8)));
                /* Removing the point from the path list */
                LL_pop(RobotPath);
                /* Making the next stage of the state machine to run */
                pathSteps = MotorResend;
                /* Setting the timer that will trigger a macro retransmit if the macro hasn't been set */
                setTimerInterval(&Retransmit, 1000);
                resetTimer(&Retransmit);
                break;
            case MotorResend:
                delay(1500);
                ReceiveDataCAN(FT_GLOBAL);
                /* if the Motor Controller hasn't set its macro with in this timer we will resend the macro */
                if (((getMotorControllerStatus() & AUTO_DRIVE_MACRO) == 0) && timerDone(&Retransmit)) {
                    /* Sending the  X & Y location data in one 16 bit word to the motor processor for Macro Processing */
                    setMotorMacro(AUTO_DRIVE_MACRO, (uint16_t) ((testPoint.y & 0xFF) | ((testPoint.x & 0xFF) << 8)));
                } else {
                    /* the macro is started so set the next stage of the state machine to get the next point in the path */
                    pathSteps = DRIVE;
                }
                break;
        }
    } else if(pathSteps == MotorResend){
        pathSteps = Wait;
    }
    return false;
}

typedef enum {
    Start = 0,
    PreDriveDig,
    Drive,
    RobotMesh,
    Dump
} FullAuto_t;
FullAuto_t FA_step = Start;

bool FullAuto(int nan) {
    /* Is there a macro running on the Motor Controller */
    if (getMotorControllerStatus() == DONE && getMotorControllerStatus() == DONE) {
        return false;
    }
    switch (FA_step) {
        case Start:
            /* Set the Pozyx to initialization state */
            /* Rotate robot 50 degrees to get the Gyros Initialized */
            /* Is the Secondary Robot connected to the system? */
            
            break;
        case Drive:
            /* When we have successfully navigated to our destination we will move to the next state */
            if(RunPath(0) == true)
                
            break;
        case PreDriveDig:
            RobotPath = LL_init();
            receivePozyx();
            pathFrom = getLocation();
            
            getPolarPath(RobotPath, pathFrom, pathTo);
            break;
    }
}

void addStopToPath(point_t _stopPoint) {
    point_t endpoint = ((point_t) ((waypoint_t*) RobotPath->last->data)->Endpoint);
    getPolarPath(RobotPath, endpoint, _stopPoint);
}


