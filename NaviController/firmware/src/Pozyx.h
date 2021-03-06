/* 
 * File:   Pozyx.h
 * Author: John
 *
 * Created on March 26, 2019, 11:40 AM
 */

#ifndef POZYX_H
#define	POZYX_H

#ifdef	__cplusplus
extern "C" {
#endif
#include "Definitions.h"
#include <stdbool.h>
    
/* POZYX FASTTRANSFER */
#define X        1
#define Y        2
#define Heading  3
    
#define USE_POZYX_HEADING
    
void InitPozyx();
void receivePozyx();
int getXpos();
int getYpos();
point_t getHeadingWaypoint();
int getPozyxHeading();
point_t getLocation();



#ifdef	__cplusplus
}
#endif

#endif	/* POZYX_H */

