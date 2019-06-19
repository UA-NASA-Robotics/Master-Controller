/* 
 * File:   Motors.h
 * Author: John
 *
 * Created on May 1, 2018, 4:00 PM
 */

#ifndef MOTOR_H
#define	MOTOR_H

#include "motorHandler.h"
#include "CAN.h"

void initMotors();
void MotorsAllStop();
void setMotor_Vel(int leftSpeed,int rightSpeed);
void requestMotorData(uint16_t motorAddress, int dataRequested);
void sendDriveCommand(int distance);

#endif	/* MOTOR_H */

