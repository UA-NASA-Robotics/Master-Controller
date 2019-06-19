#include "changeNotification.h"

intPin_t *pinArray[CN_PIN_COUNT];
unsigned char getPinState(unsigned char ID);
void initChangeNotification()
{
    pinArray[0] = &GyroPin1;
    pinArray[1] = &GyroPin2;
    pinArray[2] = &MotorPin1;
    pinArray[3] = &MotorPin2;
    
    int i;
    for(i = 0; i < CN_PIN_COUNT; i++)
    {
        pinArray[i]->pinId = i;
        pinArray[i]->changed = false;
        pinArray[i]->prevState = getPinState(pinArray[i]->pinId);
    }
}


bool pinState(intPin_t *pin)
{
    if(pin->changed)
    {
        pin->changed = false;
        pin->prevState = getPinState(pin->pinId);
        return true;
    }
    return false;
}
// put in system interrupt 
void pinChangeNotified()
{
    int pinIndex;
    for(pinIndex = 0; pinIndex < CN_PIN_COUNT; pinIndex++)
    {
        if(pinArray[pinIndex]->prevState != getPinState(pinIndex) && pinArray[pinIndex]->changed == false)
        {
            pinArray[pinIndex]->changed = true;
        }
    }
}
unsigned char getPinState(unsigned char ID)
{
    switch(ID)
    {
        case 0:
        {
            return INPUT_INTURRUPT_PIN_0;
            break;
        }
        case 1:
        {
            return INPUT_INTURRUPT_PIN_1;
            break;
        }
        case 2:
        {
            return INPUT_INTURRUPT_PIN_2;
            break;
        }
        case 3:
        {
            return INPUT_INTURRUPT_PIN_3;
            break;
        }
    }
    return 0;
}
void togglePinState(intPin_t *pin)
{
    
    switch(pin->pinId)
    {
        case 0:
        {
            OUTPUT_INTURRUPT_PIN_0 ^= 1;
            break;
        }
        case 1:
        {
            OUTPUT_INTURRUPT_PIN_1 ^= 1;
            break;
        }
        case 2:
        {
            OUTPUT_INTURRUPT_PIN_2 ^= 1;
            break;
        }
        case 3:
        {
            OUTPUT_INTURRUPT_PIN_3 ^= 1;
            break;
        }
    }
}
void setPinState(intPin_t *pin, unsigned char state)
{
    
    switch(pin->pinId)
    {
        case 0:
        {
            OUTPUT_INTURRUPT_PIN_0 = (state & 0x01);
            break;
        }
        case 1:
        {
            OUTPUT_INTURRUPT_PIN_1 = (state & 0x01);
            break;
        }
        case 2:
        {
            OUTPUT_INTURRUPT_PIN_2 = (state & 0x01);
            break;
        }
        case 3:
        {
            OUTPUT_INTURRUPT_PIN_3 = (state & 0x01);
            break;
        }
    }
}
