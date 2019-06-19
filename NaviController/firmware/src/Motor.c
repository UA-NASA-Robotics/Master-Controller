#include "Motor.h"
#include "MotorDefinitions.h"
#include "CAN.h"
void initMotors()
{
	//WARNING!!! - Why is limit switch have 3 ints and we are passing only 2?
	//WARNING!!! - Why is limit switch have 3 ints and we are passing only 2?
	//WARNING!!! - Why is limit switch have 3 ints and we are passing only 2?
	#ifndef DISABLE_LEFT_MOTOR

	//LeftMotor = (Motor_t){LEFTMOTORID,LEFTMOTOR_STATUS,LEFTMOTOR_MOB,MAXRPM,MAXCURRENTBG65,ACCEL_CONST,TRUE,(LimitSwitch_t){0,0},LeftMotor.Motor_Buffer};
	InitMotor(&LeftMotor, LEFTMOTORID, LEFTMOTOR_STATUS, LEFTMOTOR_MOB, MAXRPM, MAXCURRENTBG65,(LimitSwitch_t){0,0}, true);
	setMotorControlMode(&LeftMotor, Velocity);
	setMotorVel(&LeftMotor, 0);
    
	#endif /*DISABLE_LEFT_MOTOR*/
	
	#ifndef DISABLE_RIGHT_MOTOR
	InitMotor(&RightMotor, RIGHTMOTORID, RIGHTMOTOR_STATUS, RIGHTMOTOR_MOB, MAXRPM, MAXCURRENTBG65,(LimitSwitch_t){0,0},true);
	setMotorControlMode(&RightMotor, Velocity);
	setMotorVel(&RightMotor, 0);
	#endif /*DISABLE_ARM_MOTOR*/
	
	#ifndef DISABLE_ARM_MOTOR
	InitMotor_BG75(&ArmMotor, ARMMOTORID, ARMMOTOR_STATUS, ARMMOTOR_MOB, MAXRPM, MAXCURRENTARM, (LimitSwitch_t){LIM_D0L,LIM_D2L});
	setMotorControlMode(&ArmMotor, Velocity);
	setMotorVel(&ArmMotor, 0);
	#endif /*DISABLE_LEFT_MOTOR*/
	
	#ifndef DISABLE_BUCKET_MOTOR
	InitMotor(&BucketMotor, BUCKETMOTORID, BUCKETMOTOR_STATUS, BUCKETMOTOR_MOB, MAXRPM, MAXCURRENTBUCKET,(LimitSwitch_t){LIM_D0L,LIM_D1L},false);
	setMotorControlMode(&BucketMotor, Velocity);
	setMotorVel(&BucketMotor, 0);
	#endif /*DISABLE_LEFT_MOTOR*/
	
	#ifndef DISABLE_PLOW_MOTOR
	InitMotor(&PlowMotor, PLOWMOTORID, PLOWMOTOR_STATUS, PLOWMOTOR_MOB, MAXRPM, MAXCURRENTPLOW,(LimitSwitch_t){0,0},false);
	setMotorControlMode(&PlowMotor, Velocity);
	setMotorVel(&PlowMotor, 0);
	#endif /*DISABLE_LEFT_MOTOR*/
	
}
void MotorsAllStop()
{
	
	#ifndef DISABLE_MOTOR_SYSTEMS
	
	setMotorVel(&RightMotor, 0);
	setMotorVel(&LeftMotor, 0);

	#ifndef DISABLE_BUCKET_MOTOR
	setMotorVel(&BucketMotor,0);
	#endif
	
	#ifndef DISABLE_ARM_MOTOR
	setMotorVel(&ArmMotor, 0);
	#endif
	
	#endif
}
void setMotor_Vel(int leftSpeed,int rightSpeed)
{
    setMotorVel(&RightMotor, rightSpeed);

	setMotorVel(&LeftMotor, leftSpeed);
    
}

void requestMotorData(uint16_t motorAddress, int dataRequested)
{
    switch(dataRequested)
    {
        case ENCODER_POSITION_REQUESTED:
            requestMotorPacketWithResponse(motorAddress,MOTOR_ENCODER_POSITION_REQUEST,HALL_POSITION_REQUESTED);             
            break;
        case HALL_POSITION_REQUESTED:        
            requestMotorPacketWithResponse(motorAddress,MOTOR_HALL_POSITION_REQUEST,SSI_ENCODER_POSITION_REQUESTED);   
            break;
        case SSI_ENCODER_POSITION_REQUESTED:  
            requestMotorPacketWithResponse(motorAddress,SSI_ENCODER_POSITION_REQUEST,ENCODER_POSITION_REQUESTED);          
            break;           
    }
}
void sendDriveCommand(int distance)
{
    
}

