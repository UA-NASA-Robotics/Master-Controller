#include "Pozyx.h"
#include "FastTransfer.h"

#include <math.h>
#include <stdint.h>

point_t WaypointStart = {0, 0};
RingBuffer_t DataBuffer;
void InitPozyx() {
    InitFastTransferModule(&PozyxFT, External_UART, MY_ADDRESS, Send_put, Buffer_Get, Buffer_Size, Buffer_Peek);
}
void receivePozyx()
{
    receiveData(&PozyxFT);
}
point_t getHeadingWaypoint() {
    return WaypointStart;
}

point_t getLocation() {
     receiveData(&PozyxFT);
     // returning the pozyx data but first converting the data to cm scale
    return (point_t) {(int16_t) (PozyxFT.ReceivedData[X] / 100), (int16_t) (PozyxFT.ReceivedData[Y] / 100)};
}
int getXpos()
{
    receiveData(&PozyxFT);
    return (int16_t)(PozyxFT.ReceivedData[X]);
}
int getYpos()
{
    receiveData(&PozyxFT);
    return (int16_t)(PozyxFT.ReceivedData[Y]);
}
double headingAvg[10];
int getPozyxHeading() {
#ifdef USE_POZYX_HEADING 
//    if (PozyxFT.ReceivedData[Heading] > 180)
//        return ((double) PozyxFT.ReceivedData[Heading]) - 360;
//    else
//    double sum = 0;
//    int i;
//    for(i=0;i<9;i++){
//        sum +=headingAvg[i];
//        headingAvg[i] = headingAvg[i+1];
//    }
//    headingAvg[9] = PozyxFT.ReceivedData[Heading];
//    sum += headingAvg[9];

        return (int)PozyxFT.ReceivedData[Heading];// sum / 10;
#else
    CalculateHeading(WaypointStart, getLocation());
#endif
    }
