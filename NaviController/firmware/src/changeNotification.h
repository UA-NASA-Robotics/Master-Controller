/* 
 * File:   changeNotification.h
 * Author: Seth Carpenter
 *
 * Created on April 21, 2018, 8:01 PM
 */
//**********************MASTER CONTROLLER********************
#ifndef CHANGENOTIFICATION_H
#define	CHANGENOTIFICATION_H
//#include "boolean.h"
#include "stdbool.h"
#include <xc.h>
#define CN_PIN_COUNT 4      //the number of Change notification input pins


#define INPUT_INTURRUPT_PIN_0    PORTBbits.RB8         // pin21 - IO9   - From pin 62 on Gyro
#define INPUT_INTURRUPT_PIN_1    PORTBbits.RB9         // pin22 - IO10  - From Pin 14 on Gyro
#define INPUT_INTURRUPT_PIN_2    PORTBbits.RB12        // pin27 - IO5   - From Pin 63 on Motor
#define INPUT_INTURRUPT_PIN_3    PORTBbits.RB13        // pin28 - IO6   - From Pin 62 on Motor


#define OUTPUT_INTURRUPT_PIN_0   PORTBbits.RB10        // pin23 - IO11  - To Pin 24 on Gyro
#define OUTPUT_INTURRUPT_PIN_1   PORTBbits.RB11        // pin24 - IO12  - To Pin 32 on Gyro 
#define OUTPUT_INTURRUPT_PIN_2   PORTBbits.RB14        // pin29 - IO7   - To Pin 58 on Motor
#define OUTPUT_INTURRUPT_PIN_3   PORTBbits.RB15        // pin30 - IO8   - To Pin 57 on Motor

typedef struct {
    unsigned char pinId; //this should be a number between 0 and 3 that corresponds to the input pin
    unsigned char prevState;
    bool changed;
} intPin_t;

intPin_t MotorPin1;
intPin_t MotorPin2;
intPin_t GyroPin1;
intPin_t GyroPin2;

void initChangeNotification();
bool pinState(intPin_t *pin);
void pinChangeNotified();
void setPinState(intPin_t *pin, unsigned char state);
void togglePinState(intPin_t *pin);
#endif	/* CHANGENOTIFICATION_H */

