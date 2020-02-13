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

#define CENTER_DRIVE_DISTANCE       50   //DISTANCE FROM STARTING TO CENTER OF ARENA
#define MIGRATION_DISTANCE_DIGGING  350
#define MIGRATION_DISTANCE_DUMPING  350
#define PLOWING_DISTANCE            100
#define FORWARD                     true
#define BACKWARD                    false
#define StartDriveDist              20
#define PointToDig                 (point_t){650,200}



#define START_INFO_DIST_MASK  0x7FFF
#define StartInfo_Side_Mask  0x8000     // MSB = 1 (robot is in front of the master lider): MAB = 0 (Robot is in front of the slave Lidar)




bool autoWaiting = false;


unsigned char MacroState = 0;


point_t pathFrom = {50, 50};
point_t pathTo = {25, 10  };
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

bool isWaiting() {
    if (waitingOn == NULL) {
        return DONE;
    }
    return waitingOn();
}

void setWaiter(bool(*_waitingOn)()) {
    waitingOn = _waitingOn;
}

bool noWaiting() {
    return DONE;
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

typedef enum {
    Init = 0,
    ROTATION,
    DRIVE,
    GyroWait,
    MotorResend
} PathFollowStep_t;
PathFollowStep_t pathSteps = Init;
double gyroHeading, motorDist;
double myHeading = 0;
int data;
point_t testPoint;

bool RunPath(int nan) {
    if (getMotorControllerStatus() == DONE ) {
        if (pathSteps == Init) {
            testPathAlgorithm();
            pathSteps = DRIVE;
        } else {
            if (RobotPath != 0 && RobotPath->size < 1 && pathSteps != MotorResend) {
                LL_destroy(RobotPath);
                return true;
            }
        }

        receivePozyx();
        myHeading = getPozyxHeading();
        switch (pathSteps) {

            case DRIVE:
                motorDist = ((waypoint_t*) (RobotPath->first->data))->Distance;
                testPoint = (point_t) ((waypoint_t*) (RobotPath->first->data))->Endpoint;
                //data = (((waypoint_t*)(RobotPath->first->data))->Endpoint.x << 8) |(((waypoint_t*)(RobotPath->first->data))->Endpoint.y & 0xff); 
                data = (uint16_t) ((testPoint.y & 0xFF) | ((testPoint.x & 0xFF) << 8));


                setMotorMacro(AUTO_DRIVE_MACRO, (uint16_t) ((testPoint.y & 0xFF) | ((testPoint.x & 0xFF) << 8))); // motorDist*10);
                LL_pop(RobotPath);
                pathSteps = MotorResend;
                setTimerInterval(&Retransmit, 1000);
                resetTimer(&Retransmit);
                break;
            case MotorResend:
                delay(1500);
                ReceiveDataCAN(FT_GLOBAL);
                if (((getMotorControllerStatus() & AUTO_DRIVE_MACRO) == 0) && timerDone(&Retransmit)) {
                    setMotorMacro(AUTO_DRIVE_MACRO, (uint16_t) ((testPoint.y & 0xFF) | ((testPoint.x & 0xFF) << 8)));
                }else{
                    pathSteps = DRIVE;
                }
                break;
        }
    } else {
        pathSteps = DRIVE;
    }
    return false;
}

void addStopToPath(point_t _stopPoint) {
    point_t endpoint = ((point_t) ((waypoint_t*) RobotPath->last->data)->Endpoint);
    getPolarPath(RobotPath, endpoint, _stopPoint);
}


