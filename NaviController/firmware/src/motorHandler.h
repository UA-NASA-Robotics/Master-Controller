/* 
 * File:   motorHandler.h
 * Author: John
 *
 * Created on May 1, 2018, 3:11 PM
 */

#ifndef MOTORHANDLER_H
#define	MOTORHANDLER_H

#include "MotorDefinitions.h"
#include "CAN.h"

void InitMotor_BG75(Motor_t* motor, char address, char statusBit,char mob, int maxRPM, int maxCurrent, LimitSwitch_t limitSwitch);
void InitMotor(Motor_t* motor, char address, char statusBit,char mob, int maxRPM, int maxCurrent, LimitSwitch_t limitSwitch, bool brushless);

void clearMotorErrorStatusReg(Motor_t* motor);

void ReEstablishComms(Motor_t* motor);
//*******************************************************
//------------------------Setters------------------------
//*******************************************************

void setMotorControlMode(Motor_t *motor, unsigned char mode);
void SetMotorLimit(Motor_t *motor);

void setMotorPos(Motor_t *motor, int pos);
void setMotorPosNoSafetyComms(Motor_t *motor, int pos);

void setMotorVel(Motor_t *motor, int Vel);

void setMotorCounts(Motor_t* motor, long counts);
void setMotorCountsNoSafetyComms(Motor_t *motor, long counts);

void storeMotorPosition(Motor_t * motor, long pos);

//*******************************************************
//------------------------Getters------------------------
//*******************************************************

long getMotorPos(Motor_t *motor);
char getMotorVoltage(Motor_t *motor);
char getMotorTemp(Motor_t *motor);
bool getMotorPosReached(Motor_t *motor);
//bool getMotorStatus(Motor_t *motor);
char getMotorDigital(Motor_t *motor);
int getMotorAnalog(Motor_t *motor);

#endif	/* MOTORHANDLER_H */

