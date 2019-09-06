#include "Pozyx.h"
#include "Macros.h"
#include <stdlib.h>
#include "Definitions.h"
#include "CANFastTransfer.h"
#include "CompairitorMethods.h"
#include "Timers.h"
#include "Algorithms.h"
#include "LinkedList.h"
#include "Telemetry.h"
#include <math.h>
#include <stdlib.h>
#include "RockDetection.h"
#include "app.h"
#include "A_Star.h"
#include "Motor.h"

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

bool centerNavigation = FORWARD;

unsigned char MacroState = 0;

LL_t *robotDistinationPath;
point_t DIGGING_POINT = PointToDig;
point_t DestinationPoint = PointToDig;
const point_t COLLECTION_BIN_POINT = {50, 50};

void setWaiter(bool(*_waitingOn)());
bool(*waitingOn)();
bool waiting = false;
timers_t TimeOut, clearMacroTimer, ClearTimer;
bool(*ConfiguredMacro)();
bool runningMacroData = 0;
bool MacroRunning = false;
void(*subMacro)();

bool isWaiting();
bool motorsDone();
bool gyrosDone();
bool noWaiting();

void getRobotPosition();

void initTestFollowPath();
bool testFollowPathMacro();
bool goToDestination(LL_t *pathList);
bool followPathMacro(point_t _destination);
bool goToSpot(double heading, double distance);

bool gyroState = true, motorState = true;

LL_t *RobotPath;
point_t startPoint;
point_t endPoint;
waypoint_t *currentWaypoint;
bool homewardBound = false;

void handleCANmacro(short macroIndex, short macroData) {
   
    
    /* Give RouterCard a confirmation that the packet was received*/
    ToSendCAN(CAN_COMMAND_INDEX, macroIndex);
    sendDataCAN(ROUTER_ADDRESS);
    setMacroSafety(true); 
    setTimerInterval(&clearMacroTimer, 700);
    setTimerInterval(&ClearTimer, 100);
    switch (macroIndex) {
        case MACRO_CLEAR:
            sendMacroClear();
            stopMacro();
            while (!timerDone(&clearMacroTimer)) {
                while (!timerDone(&ClearTimer));
                LED2 ^= 1;
                LED3 ^= 1;
            }
            LED2 = off;
            break;

            /*********************** E X T E R N A L   M A C R O S*************************/
        case GYRO_ROTATION:
            setGyroMacro(ROTATION_COMMAND, (abs(macroData) > 0 ? macroData : -90));
            break;
        case GYRO_CORRECTION_MONITOR:
            setGyroMacro(ROTATION_MONITORING, 0);
            break;
        case DRIVE_ENCODER:
            setMotorMacro(ENCODER_COMMAND, (abs(macroData) > 0 ? macroData : 100));
            break;
        case DIGGING_DUMP:
            setMotorMacro(DUMP_COMMAND, 0);
            break;
        case DIGGING_DIG:
            setMotorMacro(DIG_COMMAND, (isMacroRunning() ? 0 : 1));
            break;
        case ARC_DRIVE_MACRO:
            setMotorMacro(ARC_DRIVE, (abs(macroData) > 0 ? macroData : 75));
            break;
        case ZERO_BUCKET:
            setMotorMacro(ZERO_MACRO, 0);
            break;
        case EMPTY_1:

            break;
            /*********************** I N T E R N A L   M A C R O S*************************/
        case FULL_AUTO:
            ConfiguredMacro = autonomousMacro;
            MacroRunning = true;
            //resetAutonomousSystem();
            break;
        case START_CENTER:
            break;
        case FULL_DIGGING:
            MacroRunning = true;
            int i;
            
            for(i=0;i<2;i++){
                if(PozyxFT.ReceivedDataFlags[1] != true) {
                    receiveData(&PozyxFT);
                    delay(1000);
                }else{i=2;}
            }
            /* 
             * Assume we are in one of four positions here when Starting this macro
             * 0, 90, 180, 270 */
             
            CalcInicialHeading();
            MacroRunning = true;

            break;
        case FULL_AUTO_2:
            MacroRunning = true;
            while (PozyxFT.ReceivedDataFlags[1] != true) {
                receiveData(&PozyxFT);
                //delay(1000);
            }
            CalcInicialHeading();
            MacroRunning = true;
            initTestFollowPath();
            ConfiguredMacro = testFollowPathMacro;

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

bool isMacroRunning() {
    return MacroRunning;
}

void runMacro() {
    if (ConfiguredMacro != NULL) {
        if (ConfiguredMacro()) {
            ConfiguredMacro = NULL;
        }
    }
}

void macroComplete(int macroID) {
    ConfiguredMacro = NULL;
    MacroRunning = false;
}

void stopMacro() {
    setMacroSafety(false);
    macroComplete(0);
    waiting = false;
    setWaiter(NULL);
    setGyroState(DONE);
    setMotorState(DONE);
}
MacroStateAutonomous_s autoState = AUTO_START;
bool EXTRA_STEP_INSERTED = false;



void resetAutonomousSystem(void) {
    autoState = AUTO_START;
}




bool autonomousMacro() {
    switch (autoState) {
        case AUTO_START:
            robotDistinationPath = LL_init();
            /*
             * Where Am I
             * Starting Position?
             * 
             * if(we need to return home(hopper full)
             *      returnHomeMacro();
             * else
             *      let's go digging
             */

            CalcInicialHeading(); 
            autoState = TRAVELING;

            break;
        case TRAVELING:
            /*
             * check to see if there is status from cameras
             * 
             * if(we need to return home(hopper full)) then
             *    returnHomeMacro();
             * else
             *    NavigateToDiggingMacro();
             * end if
             * 
             */
            LL_push(robotDistinationPath, &DestinationPoint);
            goToDestination(robotDistinationPath);
            break;
        case DIG_MATERIAL:
            /*
             * dig down then once reached depth move backwards (forwards)
             * lift drive forward clear sand. return and dig again monitoring
             * weight of the drum.
             */
            //if(!waitingOn())
            autoState = DEPOSIT_MATERIAL;
            setWaiter(motorsDone);
            setMotorState(false);
            setMotorMacro(DIG_COMMAND, PROFILE_1);

            //            setWaiter(motorsDone);
            //            setMotorState(false);
            //            if(!waitingOn())
            //                autoState = DEPOSIT_MATERIAL;
            break;
        case DEPOSIT_MATERIAL:
            /*
             * if (not in front of the collection bin) then
             *     travel home
             * else
             *    approach collection bin and once over reverse spin the drum
             */
            /* If we are within the bounds of the collection bin */
            if (!isWaiting()) {
                if (abs(COLLECTION_BIN_POINT.x - PozyxFT.ReceivedData[X]) < 50 && abs(COLLECTION_BIN_POINT.x - PozyxFT.ReceivedData[Y]) < 10) {
                    if (compairHeading(PozyxFT.ReceivedData[Heading], -90)) {
                        /* lift Arm to max height */
                        /* Rotate drum */
                    } else {
                        int angle = -90 - PozyxFT.ReceivedData[Heading];
                        if (angle < 180) angle = 180 - abs(angle - 180);
                        setGyroMacro(ROTATION_COMMAND, angle);
                    }
                }
            }
            break;
        case AUTO_WAITING:
            break;
    }
    return false;
}
LL_t *SquarePath;
PathNode p1, p2, p3, p4;
void initTestFollowPath()
{
    if(SquarePath != NULL)
    LL_clear(SquarePath);
}
bool testFollowPathMacro() {
    if (SquarePath != NULL && SquarePath->size > 0) {
        if (goToDestination(SquarePath))
            return true;
    } else {
        SquarePath = LL_init();

        p1.x = 150;
        p1.y = 150;
        LL_push(SquarePath, &p1);
        p2.x = 150;
        p2.y = 200;
        LL_push(SquarePath, &p2);
        p3.x = 200;
        p3.y = 200;
        LL_push(SquarePath, &p3);
        p4.x = 150;
        p4.y = 200;
        LL_push(SquarePath, &p4);
    }
    return false;
}

typedef enum {
    LegDone = 0,
    Driving

} TravelState_t;
TravelState_t TravelState = LegDone;
point_t *nextPoint;
#define WITH_POZYX 1

/*
 * Compiles a list of points to land at. these points are stored in
 * the robotDistinationPath. this will update this path when the followPathMacro is completed
 * The macro run pointer will use this followPathMacro as a destination point.
 */
bool goToDestination(LL_t *pathList) {
    if (pathList == NULL)
        return true;
    if (followPathMacro(*(point_t*) LL_last(pathList)))
        LL_popBack(pathList);
    if (pathList != NULL && pathList->size == 0)
        return true;
    return false;

}

bool followPathMacro(point_t _destination) {
    if (RobotPath == NULL) {
        RobotPath = LL_init();
    }
    /* Is there an external Macro Running */
    if (!isWaiting()) {
        /* Is there node in the path*/
        if (TravelState == LegDone && RobotPath->size == 0) {
            //if(RobotPath->size == 0 ) {
#if WITH_POZYX
            startPoint = getLocation(); /*  Get robot location */
#else
            startPoint = (point_t){0, 0}; /*  Get robot location */
#endif
            endPoint = (point_t) _destination;
            /* Generate the path from the start and end points */
            getPolarPath(RobotPath, startPoint, endPoint);
            /* Load the first element of the path */
            currentWaypoint = (waypoint_t*) LL_first(RobotPath);
            TravelState = Driving;
            //}

        }
#if WITH_POZYX
        /* Verify we moved to the correct location */
        if (goToLocation(currentWaypoint->Endpoint)) {//compairPoint(currentWaypoint->Endpoint, getLocation())){
            if (RobotPath->size > 1) {
                /* Remove old way-point (that we just moved to) */
                RobotPath = LL_pop(RobotPath);
                currentWaypoint = (waypoint_t*) LL_first(RobotPath);
                setHeadingWaypoint(getLocation());
            } else {
                LL_destroy(RobotPath);
                /* Have we reach our destination */
                //if (!compairPoint(DestinationPoint, COLLECTION_BIN_POINT)) {
               //while (goToLocation(_destination) && updateCanMACROcoms());
                    //return true;
                    //                } else {
                    TravelState = LegDone;
                return true;
                //}
            }
        }
#else
        if (TravelState == LegDone) {
            /* Rotate the to Heading first */
            setGyroMacro(ROTATION_COMMAND, currentWaypoint->heading);
            TravelState = Rotating;
        } else if (TravelState == Rotatting) {
            /* Drive distance */
            setMotorMacro(ENCODER_COMMAND, currentWaypoint->Distance);
            RobotPath = LL_pop(RobotPath);
            TravelState = LegDone;
        }
#endif
    } else {
        /* verify we are going in the right direction */
        //        if (compairHeading(CalculateHeading(getHeadingWaypoint(), getLocation()), CalculateHeading(getLocation(), currentWaypoint->Endpoint))) {
        //
        //        }
        //        /* Check Camera status to see if we encountered object */
        //        /* if we did ....*/
        //        if (PozyxFT.ReceivedDataFlags[4] && PozyxFT.ReceivedData[4] > 0) {
        //            /* Insert the Rock Locations into the map */
        //            getRockLocations();
        //            LL_clear(RobotPath);
        //        }
        //        if (0) {
        //            LL_clear(RobotPath);
        //            waitingOn = noWaiting;
        //            /* Stop motor and gyro processors */
        //            sendMacroClear();
        //        }
    }



    return false;
}
double head;
point_t loc;
double myHeadingDifferenc;
int lastPos = 0;
timers_t DataAge;
point_t krazPoint;
point_t tempPoint;
bool goToLocation(point_t _thatSpot) {

    if( !getMacroSafety())
        return false;
    updateFTdata();



    tempPoint = getLocation();
    if (compairPoint(_thatSpot, tempPoint)) {
        setWaiter(NULL);
        return true;

    }
    // updateFTdata();
    krazPoint = getLocation();
    if(krazPoint.y < 0){
        setWaiter(NULL);
        setMotorState(DONE);
        setGyroState(DONE);
        sendMacroClear();
    }
    if (!isWaiting() ) {
        if (!compairHeading(getHeading(), CalculateHeading(getLocation(), _thatSpot)) && !compairPoint(getLocation(), _thatSpot)) {
            loc = getLocation();

            head = (double) getHeading();
            myHeadingDifferenc = CalculateHeading(loc, _thatSpot) - head;
            if (myHeadingDifferenc > 0)
                myHeadingDifferenc -= (myHeadingDifferenc*0.8 )/ 3;
            else
                myHeadingDifferenc += (myHeadingDifferenc*0.8) / 3;
            if(myHeadingDifferenc > 180)
                myHeadingDifferenc -= 360;
            else if(myHeadingDifferenc < -180)
                myHeadingDifferenc += 360;
                    
            /* Rotate the to Heading first */
            setGyroMacro(ROTATION_COMMAND, myHeadingDifferenc);
            setWaiter(gyrosDone);
            setGyroState(RUNNING);
#ifdef POZYX_FakeIT
            updateHeading(myHeadingDifferenc);
#endif
        } else if (!compairPoint(getLocation(), _thatSpot)) {
            //            /* Drive distance */
            delay(1000);
            setMotorMacro(ENCODER_COMMAND, -pointDistance(getLocation(), _thatSpot)*0.9);
            setWaiter(motorsDone); //Set the waiter function to wait on the motors to finish their macro to proceed 
            /* Get waypoint so that we can check the heading later*/
            getHeadingWaypoint(getLocation());
            setMotorState(RUNNING);
        } else {
            return true;
        }
    } else {
        if (timerDone(&DataAge)) {
            requestMotorData(RightMotor.ID, ENCODER_POSITION_REQUESTED);
            requestMotorData(LeftMotor.ID, ENCODER_POSITION_REQUESTED);
        }
        if ((RightMotor.Position + LeftMotor.Position) / 2 != lastPos && (RightMotor.Position + LeftMotor.Position) / 2 > 1) 
        {
            lastPos = (RightMotor.Position + LeftMotor.Position) / 2;
        }

    }
    //}
    return false;
}

bool goToSpot(double heading, double distance) {
    if (!isWaiting()) {
        if (compairHeading(getHeading(), heading)) {
            /* Rotate the to Heading first */
            setGyroMacro(ROTATION_COMMAND, heading);
        } else {
            /* Drive distance */
            setMotorMacro(ENCODER_COMMAND, distance);
        }
    } else {
        return true;
    }
    return false;
}

bool goToDumLocation() {
    point_t myLocation = getLocation();
    if (!compairPoint_TalVal(myLocation, COLLECTION_BIN_POINT, 5)) {
        goToLocation(COLLECTION_BIN_POINT);
    }
    /* Set the Heading be facing the bucket toward the collection bin */
    int theHeading = getPozyxHeading();
    int newHeading;
    if (!compairHeading(theHeading, 270)) {

        if (theHeading > 90 && theHeading < 270) {
            newHeading = 180 - (theHeading - 90);
        } else if (theHeading < 90) {
            newHeading = (-1)*(180 - theHeading);
        } else if (theHeading > 270) {
            newHeading = (-1)*(theHeading - 270);
        }
        setGyroMacro(ROTATION_COMMAND, newHeading);
    } else {
        point_t loc = getLocation();
        setMotorMacro(ENCODER_COMMAND, loc.y - 250);

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

void setGyroState(bool state) {
    gyroState = state;
}

void setMotorState(bool state) {
    motorState = state;
}

bool state;

bool motorsDone() {
    state = motorState;
    //motorState = DONE;
    return state;
}

bool gyrosDone() {
    state = gyroState;
    //gyroState = DONE;
    return state;
}
bool noWaiting() {
    return DONE;
}


