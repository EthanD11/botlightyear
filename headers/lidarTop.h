#ifndef BLY_LIDARTOP_H
#define BLY_LIDARTOP_H

#include "lidar.h"

typedef struct LidarData{
    //data to read
    double readLidar_x_robot;
    double readLidar_y_robot;
    double readLidar_theta_robot;
    double readLidar_x_opponent;
    double readLidar_y_opponent;
    double readLidar_d_opponent;
    double readLidar_a_opponent;
    bool readLidar_lost;

    ///toutes les données dans les coordonées tq balise en 0,0
    double x_robot;
    double y_robot;
    double orientation_robot;

    double transfo_x;
    double transfo_y;
    double transfo_a;
    double old_transfo_x;
    double old_transfo_y;
    double old_transfo_a;

    double x_adv;
    double y_adv;
    double d_adv;
    double a_adv;

    // distance and angle of beacon and opponent from the previous data
    double * beaconAdv;
    int countObj_adv;


    //to find beacon from the position of the robot if we are lost
    double x_odo;
    double y_odo;
    double theta_odo;

}LidarData;

/**
 * function that can be called by other files to retrieve robot position from a reference frame
 *   ______1______
 *  |             |
 *  |             |
 *  |             |
 *  |             |
 *  |             |
 *  |_____________|
 * 3               2
 * Beacon 3 at (0,0)
 *
 * @param lidarData : structure with previous data and where we save new data
 * @param i : int to find out the number of times we call the function
 * @param fullScan : does not take previous data into account 
 * @param fromOdo : it is based on the position of the robot according to the odometers to find the beacons, lidraData->x_odo, ->y_odo, ->theta_odo must be up to date
 */
void lidarGetRobotPosition(LidarData *lidarData, int i, bool fullScan = false,  bool fromOdo =false);


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
