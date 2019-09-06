/* 
 * File:   CompairisonMethods.h
 * Author: John
 *
 * Created on June 12, 2019, 3:24 PM
 */

#ifndef COMPAIRISONMETHODS_H
#define	COMPAIRISONMETHODS_H

#ifdef	__cplusplus
extern "C" {
#endif
#include "Definitions.h"
#include <stdbool.h>
bool compairPoint_TalVal(point_t _pointA, point_t _pointB, int _tolVal);
bool compairPoint(point_t _pointA, point_t _pointB);
bool compairHeading(int headingA, int headingB);


#ifdef	__cplusplus
}
#endif

#endif	/* COMPAIRISONMETHODS_H */

