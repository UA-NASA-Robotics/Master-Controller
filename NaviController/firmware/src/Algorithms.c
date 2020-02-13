
#include "map.h"
#include "Algorithms.h"

#include "A_Star.h"
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>


LL_t *start = NULL;

ASPathNodeSource NodeSource;


bool isPointThroughObstacle(int _x, int _y);
static void PathNodeNeighbors(ASNeighborList neighbors, void *node, void *context);
static float PathNodeHeuristic(void *fromNode, void *toNode, void *context);
bool isPathPoint(ASPath _path, int x, int y);
bool isTangentLineIntersecting(int y1, int x1, int y2, int x2);


static const ASPathNodeSource PathNodeSource = {
    sizeof (PathNode),
    &PathNodeNeighbors,
    &PathNodeHeuristic,
    NULL,
    NULL
};

void printListdata(void* nodePtr) {
    printf("X: %d, Y: %d \r\n", (int) *((int*) nodePtr), (int) *((int*) nodePtr + sizeof (int)));
}

/*
 *   Looks to see if the node has any neighbors that are an obstacle
 */
static void PathNodeNeighbors(ASNeighborList neighbors, void *node, void *context) {
    PathNode *pathNode = (PathNode *) node;

    if (WorldAt(pathNode->x + 1, pathNode->y) == 0) {

        ASNeighborListAdd(neighbors, &(PathNode) {
            pathNode->x + 1, pathNode->y
        }, 1);
    }
    if (WorldAt(pathNode->x - 1, pathNode->y) == 0) {

        ASNeighborListAdd(neighbors, &(PathNode) {
            pathNode->x - 1, pathNode->y
        }, 1);
    }
    if (WorldAt(pathNode->x, pathNode->y + 1) == 0) {

        ASNeighborListAdd(neighbors, &(PathNode) {
            pathNode->x, pathNode->y + 1
        }, 1);
    }
    if (WorldAt(pathNode->x, pathNode->y - 1) == 0) {

        ASNeighborListAdd(neighbors, &(PathNode) {
            pathNode->x, pathNode->y - 1
        }, 1);
    }
}

static float PathNodeHeuristic(void *fromNode, void *toNode, void *context) {
    PathNode *from = (PathNode *) fromNode;
    PathNode *to = (PathNode *) toNode;

    static int xd, yd;
    double d;
    xd = to->x - from->x;
    yd = to->y - from->y;

    // Euclidian Distance
    d = (double) (sqrt((double) (xd * xd)+(double) (yd * yd)));

    // Manhattan distance
    //d=fabs(from->x - to->x) + fabs(from->y - to->y);

    // Chebyshev distance
    //d=max(abs(xd), abs(yd));
    // using the manhatten distance since this is a simple grid and you can only move in 4 directions
    return d;
}

bool isPathPoint(ASPath _path, int x, int y) {
    int i;
    for (i = 0; i < ASPathGetCount(_path); i++) {
        PathNode *pathNode = ASPathGetNode(_path, i);
        if (pathNode->x == x && pathNode->y == y)
            return true;

    }
    return false;
}
// This can be optimized so that we step throught the path points from starting point to current point and
// when there is a valid path that is where we will put our waypoint on the map. this is contrary to how it
// is currently done where we start with a current point and loop backwards towards the start point and set
// the way point on the first collision with a wall or obstical

void printListH(void* nodePtr) {
    printf("Heading: %d, Dist: %d \r\n", ((PathNode*) nodePtr)->x, ((PathNode*) nodePtr)->y);
}

//LL_t *RobotPathPoints;

void getPolarPath(LL_t* finalPath, point_t _pathFrom, point_t _pathTo) {
    int pathSize;

    //    RobotPathPoints = LL_init();
    // Not that the start point CAN NOT be 0, 0
    const ASPath path = ASPathCreate(&PathNodeSource, NULL, &_pathFrom, &_pathTo);

    pathSize = ASPathGetCount(path);
    // Make sure there is a path to examine
    if (pathSize > 1) {
        point_t segmentEndNode = *((point_t*) ASPathGetNode(path, 0)); //pathSize - 1));
        point_t lastNode;
        lastNode = segmentEndNode;
        int i;

        // Pushing the Ending point onto the robot's new Path
        //        LL_push(RobotPathPoints, &_pathFrom);

        point_t pathNode;

        // This starts at the destination node and looks for viable paths in reverse
        for (i = 0; i < pathSize; i++)//(i=pathSize -1; i >=0; i--)
        {
            // Getting a node from the A* path
            pathNode = *((point_t*) ASPathGetNode(path, i));
            // find the nodes intersecting the tangent path between the two nodes
            // is the tangent line going through the obstacle have we reached the final node of comparison
            // Or if we have reached the final node in the list
            if (isTangentLineIntersecting(pathNode.y, pathNode.x, segmentEndNode.y, segmentEndNode.x) || (pathNode.x == _pathTo.x && pathNode.y == _pathTo.y)) {

                if ((pathNode.x == _pathTo.x && pathNode.y == _pathTo.y))//||(pathNode.x == _pathTo.x && pathNode.y == _pathTo.y))
                {
                    lastNode = pathNode;
                }
                if(segmentEndNode.x == lastNode.x && segmentEndNode.y == lastNode.y)
                    lastNode = pathNode;
                double tanFrac = (double) ((double) (lastNode.y - segmentEndNode.y) / (double) (lastNode.x - segmentEndNode.x));

                double heading;
                heading = 360 - ((atan2((double)(lastNode.x-segmentEndNode.x),(double)(lastNode.y - segmentEndNode.y)) * (180.0/M_PI)) + 270.0) ;
                if(heading <0) heading += 360;

                waypoint_t *polarTmp = malloc(sizeof (waypoint_t));
                polarTmp->heading = (double) heading;
                polarTmp->Distance = (double) sqrt(pow(lastNode.x - segmentEndNode.x, 2) + pow(lastNode.y - segmentEndNode.y, 2));
                polarTmp->Endpoint = (point_t) lastNode;

                // Add polar way-point to list
                LL_pushBack(finalPath, polarTmp);

                segmentEndNode = lastNode;

                //                point_t *tmp = (point_t*) malloc(sizeof (point_t));
                //                tmp->x = lastNode.x;
                //                tmp->y = lastNode.y;
                //                LL_push(RobotPathPoints, tmp);

            }
            lastNode = pathNode;
        }
    }

    //LL_each(RobotPathPoints, printListH);

    ASPathDestroy(path);
    //    LL_destroy(RobotPathPoints);
}

bool isPointThroughObstacle(int _x, int _y) {

    if (WorldAt(_x, _y) == 1) {
        return true;
    }
    return false;
}


bool isTangentLineIntersecting(int y1, int x1, int y2, int x2) {
    int i; // loop counter
    int ystep, xstep; // the step on y and x axis
    int error; // the error accumulated during the increment
    int errorprev; // *vision the previous value of the error variable
    int y = y1, x = x1; // the line points
    int ddy, ddx; // compulsory variables: the double values of dy and dx
    int dx = x2 - x1;
    int dy = y2 - y1;
    bool intersectingObj = false;
    if (start != NULL)
        LL_destroy(start);
    start = LL_init();
    /* For every instance that intersectingObj is 'or'ed ('|') with its self so that if it found true it will remain true*/
    intersectingObj = intersectingObj | isPointThroughObstacle(x1, y1); // first point
    // NB the last point can't be here, because of its previous point (which has to be verified)
    if (dy < 0) {
        ystep = -1;
        dy = -dy;
    } else
        ystep = 1;
    if (dx < 0) {
        xstep = -1;
        dx = -dx;
    } else
        xstep = 1;
    ddy = 2 * dy; // work with double values for full precision
    ddx = 2 * dx;
    if (ddx >= ddy) // first octant (0 <= slope <= 1)
    {
        // compulsory initialization (even for errorprev, needed when dx==dy)
        errorprev = error = dx; // start in the middle of the square
        for (i = 0; i < dx; i++) // do not use the first point (already done)
        {
            x += xstep;
            error += ddy;
            if (error > ddx) // increment y if AFTER the middle ( > )
            {
                y += ystep;
                error -= ddx;
                // three cases (octant == right->right-top for directions below):
                if (error + errorprev < ddx) // bottom square also
                    intersectingObj = intersectingObj | isPointThroughObstacle(x, y - ystep);
                else if (error + errorprev > ddx) // left square also
                    intersectingObj = intersectingObj |isPointThroughObstacle(x - xstep, y);
                else // corner: bottom and left squares also
                {
                    intersectingObj = intersectingObj | isPointThroughObstacle(x, y - ystep);
                    intersectingObj = intersectingObj | isPointThroughObstacle(x - xstep, y);
                }
            }
            intersectingObj = intersectingObj | isPointThroughObstacle(x,y);
            errorprev = error;
        }
    } else // the same as above
    {
        errorprev = error = dy;
        for (i = 0; i < dy; i++) {
            y += ystep;
            error += ddx;
            if (error > ddy) {
                x += xstep;
                error -= ddy;
                if (error + errorprev < ddy)
                    intersectingObj = intersectingObj | isPointThroughObstacle(x - xstep, y);
                else if (error + errorprev > ddy)
                    intersectingObj = intersectingObj |isPointThroughObstacle(x, y - ystep);
                else {
                    intersectingObj = intersectingObj | isPointThroughObstacle(x - xstep, y);
                    intersectingObj = intersectingObj |isPointThroughObstacle(x, y - ystep);
                }
            }
            intersectingObj = intersectingObj | isPointThroughObstacle(x,y);
            errorprev = error;
        }
    }


    return intersectingObj;
    // assert ((y == y2) && (x == x2));  // the last point (y2,x2) has to be the same with the last point of the algorithm
}
