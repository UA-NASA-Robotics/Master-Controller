#include "map.h"
#include "Definitions.h"
#include <stdlib.h>
#include <stdio.h>


void writeBitVal(int location, int _val);
int getBitVal(int location);

#define WorldArraySize 61
#define SCALE_FACTOR 10
// assuming a 32 bit architecture (ARENA_LENGTH * ARENA_WIDTH) / 32
unsigned int world[WorldArraySize]; // =
int getObstaclePoint(int x, int y);
void writeBitVal(int location, int _val);
int getBitVal(int location);
void addObtaclePoint(int x, int y);

void ClearWorld() {
    int i;
    for (i = 0; i < WorldArraySize; i++) {
        world[i] = 0;
    }
}

/** \brief: This function given and X & Y value will provide a 1 for
 *          the presence of an obstacle at that location or 0 of none.
 *          Additionally, -1 will be returned if x or y value lay
 *          outside the current map.
 *
 *
 * \param:  The X coordinate that you wish to evaluate the map at
 * \param:  The Y coordinate that you wish to evaluate the map at
 * \return: will return the value of the map at the provided location
 *
 */
int h, t;

int WorldAt(int x, int y) {
    //    h = x;
    //    t = y;
    if (x >= 0 && x <= (getWorldHight()) && y >= 0 && y <= (getWorldWidth())) {
        return getObstaclePoint((x),(y));
    } else {
        return -1;
    }
}


/** \brief
 *
 * \param
 * \param
 * \return
 *
 */
#define COLLECTION_WIDTH 16

void generateObstacleBoarder(int _boarderWidth) {
    unsigned long i;
    int j;
    // Top & Bottom Of Map boarder
    for(i = 0; i <= (getWorldWidth()); i++)
    {
        for(j = 0; j < _boarderWidth; j++){
            if((i) > COLLECTION_WIDTH)
                addObtaclePoint(i, j);
            addObtaclePoint(i, getWorldHight() - j);
        }
    }

    for(i = 0; i <= getWorldHight(); i++)
    {
        for(j = 0; j < _boarderWidth; j++)
        {
            addObtaclePoint(j , i);
            addObtaclePoint(getWorldWidth() - (j) , i);
        }

    }
}


void writeBitVal(int location, int _val) {
    int wordToAccess = location / 32;
    int bitToAccess = location % 32;

    if (_val > 0) {
        world[wordToAccess] = world[wordToAccess] | (1 << bitToAccess);
    } else {
        world[wordToAccess] = world[wordToAccess] & (!(1 << bitToAccess));
    }
}

int getBitVal(int location) {
    int wordToAccess = location / 32;
    int bitToAccess = location % 32;
    if (wordToAccess < WorldArraySize) {
        if (world[wordToAccess] & ((1 << bitToAccess)))
            return 1;
        else
            return 0;
    }
    return 1;

}
int getObstaclePoint(int x, int y)
{
    return getBitVal(y*getWorldWidth() + x);
}
void addObtaclePoint(int x, int y) {
    int subX,subY;
    for(subY = y -1;subY <= y+1;subY++)
    {
        for(subX = x -1;subX <= x+1;subX++){
            writeBitVal((subY)*(getWorldWidth())+(subX), 1);
        }
    }
    
}

int getWorldHight() {
    return ARENA_LENGTH;
}

int getWorldWidth() {
    return ARENA_WIDTH;
}
