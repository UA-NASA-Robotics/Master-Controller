#include "Pozyx.h"
#include "Macros.h"

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




void setWaiter(bool(*_waitingOn)());
bool(*waitingOn)();
bool waiting = false;
timers_t TimeOut, clearMacroTimer, ClearTimer;
bool runningMacroData = 0;
bool MacroRunning = false;

bool isWaiting();
bool getMotorControllerStatus();
bool getGyroControllerStatus();
bool noWaiting();

bool gyroState = DONE, motorState = DONE;

void handleCANmacro(short macroIndex, short macroData) {
    setTimerInterval(&clearMacroTimer, 700);
    setTimerInterval(&ClearTimer, 100);
    switch (macroIndex) {
        case MACRO_CLEAR:
            clearMacros();
            while (!timerDone(&clearMacroTimer)) {
                while (!timerDone(&ClearTimer));
                LED2 ^= 1;
                LED3 ^= 1;
            }
            LED2 = off;
            break;

            /*********************** E X T E R N A L   M A C R O S*************************/
      
            /*********************** I N T E R N A L   M A C R O S*************************/
        case FULL_AUTO:

            //resetAutonomousSystem();
            break;
        case START_CENTER:
            break;
        case FULL_DIGGING:
            unning = true;

            break;
        case FULL_AUTO_2:

            break;
        case EMPTY_7:

            break;
        case EMPTY_6:

            break;
        case EMPTY_5:

            break;
        case EMPTY_4:


            break;
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

point_t pathFrom = {0, 0};
point_t pathTo = {0, 100};
LL_t *RobotPath;
void InitPathAlgorithm();
void addStopToPath(point_t _stopPoint);
bool RunPath(int nan);

void testPathAlgorithm() {
    //InitPathAlgorithm();
    RobotPath = LL_init();
    getPolarPath(RobotPath, pathFrom, pathTo);
    setMacroCallback(RunPath, 0, 3);
    MacroRunning = true;

    addStopToPath((point_t) {
        100, 100});

    addStopToPath((point_t) {
        100, 0});
    addStopToPath((point_t) {
        0, 0});
    //addStopToPath((point_t){0, 0});
}

typedef enum {
    ROTATION = 0,
    DRIVE
} PathFollowStep_t;
PathFollowStep_t pathSteps = ROTATION;
double gyroHeading, motorDist;
double myHeading = 0;

bool RunPath(int nan) {
    if (getGyroControllerStatus() == DONE && getMotorControllerStatus() == DONE) {
        if (RobotPath->size < 1) {
            macroComplete(0);
            return true;
        }
        switch (pathSteps) {
            case ROTATION:
                gyroHeading = ((waypoint_t*) (RobotPath->first->data))->heading;
                if (gyroHeading > myHeading) {
                    gyroHeading = gyroHeading - myHeading;
                } else {
                    gyroHeading = myHeading - gyroHeading;
                }
                if(gyroHeading > 180)
                    gyroHeading -= 180;
                setGyroMacro(ROTATION_COMMAND,  gyroHeading );
                myHeading += gyroHeading;
                pathSteps = DRIVE;
                break;
            case DRIVE:
                motorDist = ((waypoint_t*) (RobotPath->first->data))->Distance;
                setMotorMacro(ENCODER_COMMAND, motorDist);
                LL_pop(RobotPath);
                pathSteps = ROTATION;
                break;
        }
    }
    return false;
}



void addStopToPath(point_t _stopPoint) {
    point_t endpoint = ((point_t) ((waypoint_t*) RobotPath->last->data)->Endpoint);
    getPolarPath(RobotPath, endpoint, _stopPoint);
}


