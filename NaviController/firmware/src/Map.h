#ifndef MAP_H_INCLUDED
#define MAP_H_INCLUDED


#define STARTING_AREA_HEIGHT 190
#define OBSTACLE_AREA_DEPTH 274
#define DIGGING_AREA_DEPTH 274

#define worldHeight  738
#define worldWidth   378

int WorldAt(int x, int y);
void gentateobstacleBoarder(int boarderWidth);
void addRock(int _x, int _y);


#endif // MAP_H_INCLUDED
