#include "Heading.h"
#include "Definitions.h"
#include "Pozyx.h"
#include "FastTransfer.h"
#include <math.h>
#include "Timers.h"

#define InicialHeading  0
int HeadingVal = InicialHeading;
point_t HeadingStartWayPoint;

double thisheading;

double CalculateHeading(point_t startPoint, point_t endPoint) {
    thisheading = atan2((double) ((double) endPoint.y - (double) startPoint.y), (double) ((double) endPoint.x - (double) startPoint.x)) * RAD_TO_DEGREE;
//    if ((endPoint.y - startPoint.y) < 0 && (endPoint.x - startPoint.x) < 0) //Quad 3
//        thisheading = -thisheading;
//    else if ((endPoint.y - startPoint.y) < 0 && (endPoint.x - startPoint.x) > 0) //quad 4
//        thisheading = -180 + thisheading;
//    else if ((endPoint.y - startPoint.y) > 0 && (endPoint.x - startPoint.x) > 0) //quad 1
//        thisheading = thisheading;
//    else if ((endPoint.y - startPoint.y) > 0 && (endPoint.x - startPoint.x) < 0) //quad 2
//        thisheading = 180 - thisheading;
    return thisheading;

}

double pointDistance(point_t pointA, point_t pointB) {
    return sqrt((pointB.y - pointA.y)*(pointB.y - pointA.y) + (pointB.x - pointA.x)*(pointB.x - pointA.x));
}

void updateHeading(int _heading) {
    HeadingVal += _heading;
}

int getHeading() {
    return HeadingVal;
}

int getInicialHeading() {
    return InicialHeading;
}

void CalcInicialHeading() {
    int16_t i, pozyxHeading = 0;
    //for (i = 0; i < 5; i++) {
        //while (PozyxFT.ReceivedDataFlags[3] != true) {
delay(1000);
            receiveData(&PozyxFT);
            
        //}
        pozyxHeading = (int16_t)getPozyxHeading();
        
        //PozyxFT.ReceivedDataFlags[3] = false;
    //}
    //pozyxHeading /= 5;

    if (pozyxHeading < 45 && pozyxHeading > -45)
        HeadingVal = 0;
    else if (pozyxHeading >= 45 && pozyxHeading < 135)
        HeadingVal = 90;
    else if (abs(pozyxHeading) >= 135)
        HeadingVal = 180;
    else if (pozyxHeading <= -45 && pozyxHeading > -135)
        HeadingVal = -90;
}

void setInicialHeading(int _heading) {
    HeadingVal = _heading;
}

void captureHeadingWaypoint(point_t _Waypoint) {
    HeadingStartWayPoint = _Waypoint;
}

double calcHeadingFromWaypoint(point_t secondWaypoint) {
    return CalculateHeading(HeadingStartWayPoint, secondWaypoint);

}