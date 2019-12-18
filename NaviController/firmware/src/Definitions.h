/* 
 * File:   Definitions.h
 * Author: John
 *
 * Created on May 5, 2018, 1:16 PM
 */
//**********************************Master Controller*********************8
#ifndef DEFINITIONS_H
#define	DEFINITIONS_H

#include "CAN_Handler/GlobalCAN_IDs.h"


#define RAD_TO_DEGREE   57.2957795
#define DEGREE_TO_RAD   0.01745329
#define mm_to_cm        0.1

#define LED2    LATEbits.LATE6
#define LED3    LATEbits.LATE5
#define LED1    LATEbits.LATE7
#define LED4    LATEbits.LATE4

#define off     1
#define on      0




#define MY_ADDRESS   MASTER_CONTROLLER


#define MACRO_COMMAND_INDEX      8
#define UART_COMMAND_DATA_INDEX  9

#define CAN_ADDRESS_INDEX       0

#define CAN_COMMAND_INDEX       8
#define CAN_COMMAND_DATA_INDEX  9

#define REVERSE_DRIVE_DIRECTION 

#define POZYX_FakeIT


#define ROBOT_LENGTH    10
#define ROBOT_WIDTH     5
#define ARENA_LENGTH    550
#define ARENA_WIDTH     350

//******************************************************
//                  MACRO INDEXES
//******************************************************
//**********MASTER Macros***************

    #define STARTING_CENTER         2
    #define FULL_AUTONOMY           1
//**********Motor Macros****************
   
    #define ENCODER_COMMAND         3    //drive a distance
    #define ARC_DRIVE               4
#define dumbMac1 7
#define dumbMac2 8
#define dumbMac3 9
#define dumbMac4 10

    #define DIG_COMMAND             5
        #define PROFILE_1           0

    #define DUMP_COMMAND            6
    #define ZERO_MACRO              100

//**********Gyro Macros*****************
    #define ROTATION_COMMAND        1  //rotate
    #define ROTATION_MONITORING     2   

 //**************TO ALL**********************
    #define PAUSE_COMMAND           4     //Pause the cammand that is running
    #define STOP_COMMMAND           0

typedef struct{
    int x;
    int y;
}point_t;

typedef struct{
    int x;
    int y;
    point_t *nextPoint;
}pointList_t;

#define NULL_POINT 
typedef struct{
    int angle;
    int mag;
}polar_t;

typedef struct{
    point_t Endpoint;
    double heading;
    double Distance;
}waypoint_t;

#endif	/* DEFINITIONS_H */

