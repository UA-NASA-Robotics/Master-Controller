
#include "map.h"
#include "Algorithms.h"

#include "A_Star.h"
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>


LL_t *start = NULL;


ASPathNodeSource NodeSource;

PathNode pathFrom = {5,3};
PathNode pathTo = {17,20};


bool IsPathThroughOpsticle();
static void PathNodeNeighbors(ASNeighborList neighbors, void *node, void *context);
static float PathNodeHeuristic(void *fromNode, void *toNode, void *context);
bool isPathPoint(ASPath _path, int x, int y);
void POINT(int y, int x);
void useVisionLine (int y1, int x1, int y2, int x2);


static const ASPathNodeSource PathNodeSource =
{
    sizeof(PathNode),
    &PathNodeNeighbors,
    &PathNodeHeuristic,
    NULL,
    NULL
};

void printListdata(void* nodePtr)
{
    printf("X: %d, Y: %d \r\n",(int)*((int*)nodePtr), (int)*((int*)nodePtr + sizeof(int)));
}
/*
*   Looks to see if the node has any neighbors that are an obstacle
*/
static void PathNodeNeighbors(ASNeighborList neighbors, void *node, void *context)
{
    PathNode *pathNode = (PathNode *)node;

    if (WorldAt(pathNode->x+1, pathNode->y) == 0)
    {
        ASNeighborListAdd(neighbors, &(PathNode)
        {
            pathNode->x+1, pathNode->y
        }, 1);
    }
    if (WorldAt(pathNode->x-1, pathNode->y) == 0)
    {
        ASNeighborListAdd(neighbors, &(PathNode)
        {
            pathNode->x-1, pathNode->y
        }, 1);
    }
    if (WorldAt(pathNode->x, pathNode->y+1) == 0)
    {
        ASNeighborListAdd(neighbors, &(PathNode)
        {
            pathNode->x, pathNode->y+1
        }, 1);
    }
    if (WorldAt(pathNode->x, pathNode->y-1) == 0)
    {
        ASNeighborListAdd(neighbors, &(PathNode)
        {
            pathNode->x, pathNode->y-1
        }, 1);
    }
}

static float PathNodeHeuristic(void *fromNode, void *toNode, void *context)
{
    PathNode *from = (PathNode *)fromNode;
    PathNode *to = (PathNode *)toNode;

    static int xd, yd;
    double d;
    xd = to->x - from->x;
    yd = to->y - from->y;

    // Euclidian Distance
    //d=(double)(sqrt((double)(xd*xd)+(double)(yd*yd)));

    // Manhattan distance
    d=fabs(from->x - to->x) + fabs(from->y - to->y);

    // Chebyshev distance
    //d=max(abs(xd), abs(yd));
    // using the manhatten distance since this is a simple grid and you can only move in 4 directions
    return d;
}

bool isPathPoint(ASPath _path, int x, int y)
{
    int i;
    for (i=0; i<ASPathGetCount(_path); i++)
    {
        PathNode *pathNode = ASPathGetNode(_path, i);
        if(pathNode->x == x && pathNode->y == y)
            return true;

    }
    return false;
}
// This can be optimized so that we step throught the path points from starting point to current point and
// when there is a valid path that is where we will put our waypoint on the map. this is contrary to how it
// is currently done where we start with a current point and loop backwards towards the start point and set
// the way point on the first collision with a wall or obstical
void printListH(void* nodePtr)
{
    printf("Heading: %d, Dist: %d \r\n",((PathNode*)nodePtr)->x,((PathNode*)nodePtr )->y);
}

LL_t *RobotPathPoints;
void getPolarPath(LL_t* finalPath, point_t _startPoint, point_t _endPoint)
{
    int pathSize;
    pathFrom = _startPoint;
    pathTo = _endPoint;
    
    /* Initializing the empty pointer */
    RobotPathPoints = LL_init();
    /* Getting the path from the A* algorithm */
    ASPath path = ASPathCreate(&PathNodeSource, NULL, &pathFrom, &pathTo);
    pathSize = ASPathGetCount(path);
    
    PathNode *endNode = ASPathGetNode(path, pathSize - 1);
    PathNode lastNode;
    PathNode lastWaypoint;
    if (pathSize > 1)
    {
        
        lastWaypoint = (PathNode)
        {
            endNode->x,endNode->y
        };
        int i;

        LL_push(RobotPathPoints, &pathTo);
        for (i=pathSize -1; i >=0; i--)
        {
            PathNode *pathNode = ASPathGetNode(path, i);

            // find the nodes intersecting the tangent path between the two nodes
            useVisionLine (pathNode->y, pathNode->x, endNode->y, endNode->x);
            if(IsPathThroughOpsticle() || (pathNode->x == pathFrom.x && pathNode->y == pathFrom.y))
            {

                double tanFrac =(double)((double)(lastWaypoint.y - lastNode.y)/(double)(lastWaypoint.x - lastNode.x)) ;

                double heading;
                if(tanFrac < 0)
                    heading = 180 + atan((tanFrac)) * (180.0/3.14159);
                else
                    heading = atan((tanFrac)) * (180.0/3.14159);

                waypoint_t *polarTmp = malloc(sizeof(waypoint_t));
                polarTmp->heading = (int)heading;
                polarTmp->Distance = (int)sqrt(pow(lastWaypoint.x - lastNode.x,2) + pow(lastWaypoint.y - lastNode.y,2));
                polarTmp->Endpoint = (point_t)lastNode;
                //Add polar way-point to list
                LL_push(finalPath, polarTmp);
                lastWaypoint.x = lastNode.x;
                lastWaypoint.y = lastNode.y;
                *endNode = lastWaypoint;

                //printf("Waypoint: X: %d, Y: %d\r\n",lastNode.x,lastNode.y);

                PathNode *tmp = (PathNode*)malloc(sizeof(PathNode));
                tmp->x = lastWaypoint.x;
                tmp->y = lastWaypoint.y;
                LL_push(RobotPathPoints, tmp);
            }
            lastNode = (PathNode)
            {
                pathNode->x,pathNode->y
            };

        }
         PathNode *tmp = malloc(sizeof(PathNode));
         *tmp = (PathNode)lastWaypoint;
        LL_push(RobotPathPoints, tmp);


        LL_push(RobotPathPoints, &pathFrom);

//        int x,y;
//        for(y = 0; y < worldHeight; y++)
//        {
//            printf("%2d: ",y);
//            for(x =0; x< worldWidth; x++)
//            {
//                if(isPathPoint(path,  x,  y))
//                {
//                    printf("+ ");
//                }
//                else if(WorldAt(x,y)==1)
//                {
//                    printf("O ");
//                }
//                else
//                {
//                    printf("  ");
//                }
//
//            }
//            printf("|\r\n");
//        }
//        printf("    ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^\r\n");
    }
    LL_each(RobotPathPoints, printListH);
    ASPathDestroy(path);
    LL_destroy(RobotPathPoints);
}

void POINT(int y, int x)
{
    PathNode *tmp;
    tmp = malloc(sizeof(PathNode));
    tmp->x = x;
    tmp->y = y;
    //printf("X: %d, Y: %d \r\n",x, y);
    LL_push(start, tmp);
}
bool IsPathThroughOpsticle()
{
    int i;
    for(i = 0;i<start->size - 1;i++)
    {
        LL_node_t *tmpNode = LL_get(start, i);
        PathNode *ptr = (PathNode*)tmpNode;
        //printf("val: x %d\r\n", ptr->x);
        if(WorldAt(ptr->x,ptr->y)==1)
        {
            return true;
        }
    }
    return false;
}

void useVisionLine (int y1, int x1, int y2, int x2)
{
    int i;               // loop counter
    int ystep, xstep;    // the step on y and x axis
    int error;           // the error accumulated during the increment
    int errorprev;       // *vision the previous value of the error variable
    int y = y1, x = x1;  // the line points
    int ddy, ddx;        // compulsory variables: the double values of dy and dx
    int dx = x2 - x1;
    int dy = y2 - y1;
    if(start != NULL)
        LL_destroy(start);
    start = LL_init();
    POINT (y1, x1);  // first point
    // NB the last point can't be here, because of its previous point (which has to be verified)
    if (dy < 0)
    {
        ystep = -1;
        dy = -dy;
    }
    else
        ystep = 1;
    if (dx < 0)
    {
        xstep = -1;
        dx = -dx;
    }
    else
        xstep = 1;
    ddy = 2 * dy;  // work with double values for full precision
    ddx = 2 * dx;
    if (ddx >= ddy)   // first octant (0 <= slope <= 1)
    {
        // compulsory initialization (even for errorprev, needed when dx==dy)
        errorprev = error = dx;  // start in the middle of the square
        for (i=0 ; i < dx ; i++)    // do not use the first point (already done)
        {
            x += xstep;
            error += ddy;
            if (error > ddx)   // increment y if AFTER the middle ( > )
            {
                y += ystep;
                error -= ddx;
                // three cases (octant == right->right-top for directions below):
                if (error + errorprev < ddx)  // bottom square also
                    POINT (y-ystep, x);
                else if (error + errorprev > ddx)  // left square also
                    POINT (y, x-xstep);
                else   // corner: bottom and left squares also
                {
                    POINT (y-ystep, x);
                    POINT (y, x-xstep);
                }
            }
            POINT (y, x);
            errorprev = error;
        }
    }
    else   // the same as above
    {
        errorprev = error = dy;
        for (i=0 ; i < dy ; i++)
        {
            y += ystep;
            error += ddx;
            if (error > ddy)
            {
                x += xstep;
                error -= ddy;
                if (error + errorprev < ddy)
                    POINT (y, x-xstep);
                else if (error + errorprev > ddy)
                    POINT (y-ystep, x);
                else
                {
                    POINT (y, x-xstep);
                    POINT (y-ystep, x);
                }
            }
            POINT (y, x);
            errorprev = error;
        }
    }
// assert ((y == y2) && (x == x2));  // the last point (y2,x2) has to be the same with the last point of the algorithm
}
