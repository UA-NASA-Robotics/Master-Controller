/* 
 * File:   Comunications.h
 * Author: Seth Carpenter
 *
 * Created on May 8, 2018, 11:03 AM
 */

#ifndef COMUNICATIONS_H
#define	COMUNICATIONS_H

#include <stdbool.h>

#define RUNNING true
#define DONE    false


void setMacroSafety(bool state);
bool getMacroSafety();
bool updateCanMACROcoms();
void setPositionSnapshot();
void setMotorMacro(int macroIndex, int data);
void setGyroMacro(int macroIndex, int data);
void updateFTdata();

void sendMacroDone();   
void sendMacroClear();


#endif	/* COMUNICATIONS_H */

