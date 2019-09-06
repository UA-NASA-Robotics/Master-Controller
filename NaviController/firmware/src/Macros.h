/* 
 * File:   Macros.h
 * Author: John
 *
 * Created on May 5, 2018, 9:30 PM
 */

#ifndef MACROS_H
#define	MACROS_H

#include "Definitions.h"
#include "FastTransfer.h"
#include "Comunications.h"
#include "changeNotification.h"

typedef enum {
    MACRO_CLEAR = 0,
    //first eight will be from the left two columns of the control box
    FULL_AUTO,
    START_CENTER,
    FULL_DIGGING,
    FULL_AUTO_2,
    EMPTY_7,
    EMPTY_6,
    EMPTY_5,
    EMPTY_4,
    // Second eight will be from the Right two columns of the control box
    GYRO_ROTATION,
    GYRO_CORRECTION_MONITOR,
    DRIVE_ENCODER,
    DIGGING_DUMP,
    DIGGING_DIG,
    ARC_DRIVE_MACRO,
    ZERO_BUCKET,
    EMPTY_1,


} MacroList_t;

typedef enum {
    isSTART = 0,
    START,
    STEP1,
    STEP2,
    STEP3,
    STEP4,
    STEP5,
    END,
    WAITING
} MacroState_s;

typedef enum {
    AUTO_START = 0,
    TRAVELING,
    DIG_MATERIAL,
    DEPOSIT_MATERIAL,
    AUTO_WAITING
} MacroStateAutonomous_s;




void macroComplete(int macroID);
void configureMacro(int macroID, int macroData);
void runMacro();
void stopMacro();
void handleCANmacro(short macroIndex, short macroData);

bool isMacroRunning();

bool autonomousMacroTwo();
bool autonomousMacro();
void setExternalMacro(int macroIndex, int commandData);

void setGyroState(bool state);
void setMotorState(bool state);

void resetAutonomousSystem(void);



bool goToLocation(point_t _thatSpot);

#endif	/* MACROS_H */

