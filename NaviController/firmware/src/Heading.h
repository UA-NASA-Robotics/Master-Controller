/* 
 * File:   Heading.h
 * Author: John
 *
 * Created on March 26, 2019, 11:26 AM
 */

#ifndef HEADING_H
#define	HEADING_H

#ifdef	__cplusplus
extern "C" {
#endif
#include "Definitions.h"
double CalculateHeading(point_t startPoint, point_t endPoint);
double pointDistance (point_t pointA, point_t pointB);
void setInicialHeading(int _heading);
int getInicialHeading();
int getHeading();
void updateHeading(int _heading);
void captureHeadingWaypoint(point_t _Waypoint);
double calcHeadingFromWaypoint(point_t secondWaypoint);
void CalcInicialHeading();


#ifdef	__cplusplus
}
#endif

#endif	/* HEADING_H */

