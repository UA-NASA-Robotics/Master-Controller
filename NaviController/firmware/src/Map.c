#include "map.h"
#include "Definitions.h"
#include <stdlib.h>
#include <stdio.h>


void writeBitVal(int location, int _val);
int getBitVal(int location);
// assuming a 32 bit architecture (ARENA_LENGTH * ARENA_WIDTH) >> 8
unsigned char world[6016];


void addObtaclePoint(int x, int y);
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

int WorldAt(int x, int y)
{
    if (x >= 0 && x < getWorldWidth() && y >= 0 && y < getWorldHight())
    {
        return getBitVal(y*getWorldWidth()+x);//world[y*getWorldWidth()+x];
    }
    else
    {
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

void generateObstacleBoarder(int _boarderWidth)
{
    unsigned long i;
    unsigned long worldArea = (ARENA_LENGTH*ARENA_WIDTH);
    // Top & Bottom Of Map boarder
    for(i = 0; i < ARENA_WIDTH * _boarderWidth; i++)
    {
        writeBitVal(i,1);
        writeBitVal(worldArea - i -1,1);
    }
    int j;
    for(i = 0; i < worldArea; i+=ARENA_WIDTH)
    {
        for(j = 0; j < _boarderWidth; j++)
        {
            writeBitVal(i + j, 1);
            writeBitVal(i - (j+1),  1);
        }

    }
}

void addobstacle(int _x, int _y)
{
     int x,y;
    // half of the robots width
    int objectHalf = (ROBOT_WIDTH >> 1);
    /* Should make a square rock that is the size of the robot in the Map */
    for(y = _y - objectHalf; y < objectHalf + _y; y++)
    {
        for(x = _x - objectHalf; x < objectHalf +_x; x++)
        {
            writeBitVal(y*getWorldWidth()+x, 1);
        }
    }

}
void writeBitVal(int location, int _val)
{
    int wordToAccess = location / 32;
    int bitToAccess = location % 32;

    if(_val > 0)
    {
        world[wordToAccess] = world[wordToAccess] | (1 << bitToAccess);
    }else{
          world[wordToAccess] = world[wordToAccess] & (!(1 << bitToAccess));
    }
}
int getBitVal(int location)
{
    int wordToAccess = location / 32;
    int bitToAccess = location % 32;
    if( world[wordToAccess] & ((1 << bitToAccess)))
        return 1;
    else
        return 0;


}
void addObtaclePoint(int x, int y)
{
    writeBitVal(y*getWorldWidth()+x, 1);
}
int getWorldHight()
{
    return ARENA_LENGTH;
}

int getWorldWidth()
{
    return ARENA_WIDTH;
}

