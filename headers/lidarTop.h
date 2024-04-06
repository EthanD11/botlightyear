#ifndef BOTLIGHTYEAR_LIDARTOP_H
#define BOTLIGHTYEAR_LIDARTOP_H

#include "lidar.h"



/**
 * function that can be called by other files to retrieve robot position from a reference frame
 *   ________________1______________
 *  |                              |
 *  |                              |
 *  |                              |
 *  |                              |
 *  |                              |
 *  |______________________________|
 * 3                               2
 * Beacon 3 at (0,0)
 *
 * @param lidarData : structure with previous data and where we save new data
 * @param i : int to find out the number of times we call the function
 */
void lidarGetRobotPosition(LidarData *lidarData, int i, bool fullScan = false);


/**
 *
 * @param lidarData : structure memory already allocated
 */
void init_lidar(LidarData *lidarData);

/**
 *
 * @param lidarData : structure to delete
 */
void clear_lidar(LidarData *lidarData);

#endif //BOTLIGHTYEAR_LIDARTOP_H
