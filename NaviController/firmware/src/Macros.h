/* 
 * File:   Macros.h
 * Author: John
 *
 * Created on May 5, 2018, 9:30 PM
 */

#ifndef MACROS_H
#define	MACROS_H
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>
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




void macroComplete(int macroID);
void configureMacro(int macroID, int macroData);
void runMacro();
void stopMacro();
void handleCANmacro(short macroIndex, short macroData);

bool isMacroRunning();

bool autonomousMacroTwo();
bool autonomousMacro();
void setExternalMacro(int macroIndex, int commandData);



void testPathAlgorithm();

#endif	/* MACROS_H */

