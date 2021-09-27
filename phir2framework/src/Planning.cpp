#include "Planning.h"

#include <queue>
#include <float.h> //DBL_MAX

////////////////////////
///                  ///
/// Métodos Públicos ///
///                  ///
////////////////////////

Planning::Planning()
{
    newRobotPosition.x = 0;
    newRobotPosition.y = 0;

    newGridLimits.minX = newGridLimits.minY = 1000;
    newGridLimits.maxX = newGridLimits.maxY = -1000;
}

Planning::~Planning()
{}

void Planning::setGrid(Grid *g)
{
    grid = g;
}

void Planning::setMaxUpdateRange(int r)
{
    maxUpdateRange = 1.2*r*grid->getMapScale();
}

void Planning::setNewRobotPose(Pose p)
{
    newRobotPosition.x = (int)(p.x*grid->getMapScale());
    newRobotPosition.y = (int)(p.y*grid->getMapScale());

    newGridLimits.minX = std::min(newGridLimits.minX,newRobotPosition.x-maxUpdateRange);
    newGridLimits.maxX = std::max(newGridLimits.maxX,newRobotPosition.x+maxUpdateRange);
    newGridLimits.minY = std::min(newGridLimits.minY,newRobotPosition.y-maxUpdateRange);
    newGridLimits.maxY = std::max(newGridLimits.maxY,newRobotPosition.y+maxUpdateRange);
}

void Planning::run()
{
    pthread_mutex_lock(grid->mutex);

    // resetCellsTypes();

    // update robot position and grid limits using last position informed by the robot
    robotPosition = newRobotPosition;
    gridLimits = newGridLimits;

    updateCellsTypes();

    pthread_mutex_unlock(grid->mutex);
}

/////////////////////////////////////////////
///                                       ///
/// Métodos para classificacao de celulas ///
///                                       ///
/////////////////////////////////////////////

void Planning::resetCellsTypes()
{
    for(int i=gridLimits.minX;i<=gridLimits.maxX;i++){
        for(int j=gridLimits.minY;j<=gridLimits.maxY;j++){

            Cell* c = grid->getCell(i,j);

            c->occType = UNEXPLORED;
            c->planType = REGULAR;
        }
    }
}

void Planning::updateCellsTypes()
{
    int maxRangeInt = maxUpdateRange;

    int robotX = robotPosition.x;
    int robotY = robotPosition.y;
    int maxX, maxY, minX, minY = 0;

    int DANGER_WIDTH = 3;
    int FRONTIER_WIDTH = 1;

    // If you want to access the current cells surrounding the robot, use this range
    //
    //  (robotPosition.x-maxUpdateRange, robotPosition.y+maxUpdateRange)  -------  (robotPosition.x+maxUpdateRange, robotPosition.y+maxUpdateRange)
    //                                 |                                    \                                     |
    //                                 |                                     \                                    |
    //                                 |                                      \                                   |
    //  (robotPosition.x-maxUpdateRange, robotPosition.y-maxUpdateRange)  -------  (robotPosition.y+maxUpdateRange, robotPosition.y-maxUpdateRange)
    minX = robotX - maxRangeInt;
    minY = robotY - maxRangeInt;
    maxX = robotX + maxRangeInt;
    maxY = robotY + maxRangeInt;

    for(int i = minX; i <= maxX; i++)
    {
        for(int j = minY; j <= maxY; j++)
        {
            // Cell to be analyzed
            Cell* c = grid->getCell(i,j);
            
            if(c->occType == UNEXPLORED)
            {
                if(c->occupancy > 0.6)
                {
                    c->occType = OCCUPIED;
                }
                else if(c->occupancy <= 0.3)
                {
                    c->occType = FREE;
                }
            }
            else
            {
                if(c->occupancy > 0.6)
                {
                    c->occType = OCCUPIED;
                }
                else if(c->occupancy <= 0.3)
                {
                    c->occType = FREE;
                }
            }

            // Classifies based on the planning type
            
            // Frontier cells
            if( c->occType == FREE )
            {
                for(int x = i - FRONTIER_WIDTH; x <= i + FRONTIER_WIDTH; x++)
                {
                    for(int y = j - FRONTIER_WIDTH; y <= j + FRONTIER_WIDTH; y++)
                    {
                        Cell* n = grid->getCell(x, y);

                        if( n->occType == UNEXPLORED )
                        {
                            n->planType = FRONTIER;
                        }
                    }
                }
            }

            // Danger cells
            if( c->occType == OCCUPIED )
            {
                for(int x = i - DANGER_WIDTH; x <= i + DANGER_WIDTH; x++)
                {
                    for(int y = j - DANGER_WIDTH; y <= j + DANGER_WIDTH; y++)
                    {
                        Cell* n = grid->getCell(x, y);

                        if( n->occType == FREE )
                        {
                            n->planType = DANGER;
                        }
                    }
                }
            }
        }
    }
}

