/* 
 * File:   CompairitorMethods.c
 * Author: Seth Carpenter
 *
 * Created on June 12, 2019, 3:24 PM
 */
#include "CompairitorMethods.h"
#include "Definitions.h"
#include <stdlib.h>
#define HeadingToleranceVal    9
#define ToleranceVal           15


bool compairHeading(int headingA, int headingB) {
    return (abs(headingA - headingB) < HeadingToleranceVal);
}

bool compairPoint(point_t _pointA, point_t _pointB) {
    return (abs(_pointA.x - _pointB.x) < ToleranceVal && abs(_pointA.y - _pointB.y) < ToleranceVal);
}

bool compairPoint_TalVal(point_t _pointA, point_t _pointB, int _tolVal) {
    return (abs(_pointA.x - _pointB.x) < _tolVal && abs(_pointA.y - _pointB.y) < _tolVal);

}