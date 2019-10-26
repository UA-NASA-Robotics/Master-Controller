#ifndef MAP_H_INCLUDED
#define MAP_H_INCLUDED




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
int WorldAt(int x, int y);



/** \brief: Generates a boarder around the map who's size is defined
            by functions getWorldWidth() & getWorldHight()
 *
 * \param:  The width of the boarder to be made where 1 = 10cm
 * \return: N/A
 *
 */
void generateObstacleBoarder(int _boarderWidth);


/** \brief: Gives the ability to add objects to the map at specific
 *          locations of a specific size
 *
 * \param X: Location in the x direction you wish to place the object
 * \param Y: Location in the Y direction you wish to place the object
 * \return
 *
 */
void addObstacle(int _x, int _y);


/** \brief
 *
 * \param
 * \param
 * \return
 *
 */
int getWorldHight();


/** \brief
 *
 * \param
 * \param
 * \return
 *
 */
int getWorldWidth();


#endif // MAP_H_INCLUDED
