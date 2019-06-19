#include "Pozyx.h"
#include "FastTransfer.h"
#include <math.h>

point_t WaypointStart = {0, 0};

void InitPozyx() {
    InitFastTransferModule(&PozyxFT, External_UART, MY_ADDRESS, Send_put, Buffer_Get, Buffer_Size, Buffer_Peek);
}

point_t getHeadingWaypoint() {
    return WaypointStart;
}

point_t getLocation() {

    return (point_t) {(int16_t) (PozyxFT.ReceivedData[X] / 10), (int16_t) (PozyxFT.ReceivedData[Y] / 10)};
}
double getPozyxHeading() {
#ifdef USE_POZYX_HEADING
    if (PozyxFT.ReceivedData[Heading] > 180)
        return ((double) PozyxFT.ReceivedData[Heading]) - 360;
    else
        return (double) PozyxFT.ReceivedData[Heading];
#else
    CalculateHeading(WaypointStart, getLocation());
#endif
    }
void setHeadingWaypoint(point_t _point) {
    WaypointStart = _point;
}