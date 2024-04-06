//
// Created by pauline on 20/03/24.
//

#ifndef BOTLIGHTYEAR_LIDARTOP_H
#define BOTLIGHTYEAR_LIDARTOP_H

#include "lidar.h"
#define ARRAYSIZELIDAR 1000;
//TODO CHANGE COMMENTAIRE
/**
 * from the polar coordinates of the 3 beacons and knowing that the robot is at the origin of the reference frame,
 * we recover the coordinates of the robot in (x,y) according to the reference frame determined with beacon 3 in (0,0).
 *   ________________1______________
 *  |                              |
 *  |                              |
 *  |                              |
 *  |                              |
 *  |                              |
 *  |______________________________|
 * 3                               2
 *
 * input array pointer of size 3
 * @param db : distance between beacons 2 by 2
 * @param x , y : coordinates of 3 beacons
 * @param robot : array of size 2 (x,y) coordinates of the robot -> output
 * @param transfoTODOCHANGE : input (0,0,0,0); output : transformations to be made to obtain the x,y coordinates;
 *                  transfo[0] : deltax; transfo[1] : deltay; transfo[2] : angle of rotation;
 *                  transfo[3] : number of elements to be analysed
 * @param anglesBeacons : input : orientation of the 3 beacons to determine the orientation of the robot on the table
 */
void rotationPosition(double *db, double *x, double *y, LidarData *lidarData, double *anglesBeacons)  ;

/**
 * Find the avdersaire and save its position relative to the map and its position relative to the robot in adversaryCoordinates
 * @param anglesAdv coordinates relative to the robot of possible opponents
 * @param distancesAdv coordinates relative to the robot of possible opponents;
 * @param transfo transformations to be made to obtain the x,y coordinates; transfo[0] : deltax; transfo[1] : deltay;
 *                  transfo[2] : angle of rotation; transfo[3] : number of elements to be analysed
 * @param adversaryCoordinates of size 4, input all =0; output: opponent's coordinates
 *          0: x relative to beacon3); 1: y relative to beacon3; 2: distance to our robot; 3: angle relative to our robot
 */
int Adversary(double *anglesAdv, double *distancesAdv, LidarData *lidarData);

/**
 * From the raw lidar data, return (in the robot table) the coordinates of the robot according to the defined plane.
 * @param angles : array of size 8192 with angle in degree
 * @param distances : array of size 8192 with distances in m
 * @param quality : array of size 8192 with quality
 * @param robot : array of size 2 to be filled with robot coordinates, if position not found, not change ((0,0)-->input)
 * @param adversaryCoordinates : array of size 4 to be filled with opponent coordinates, if position not found, not change ((0,0,0,0)--> input)
 * @param fullScan : if true: performs a full scan with no position prediction,
 *                  if false: performs a more accurate scan, mimicked by an estimate of the position of the beacons and the opponent
 * @param previousBeaconAdv : contains the distances and angles of the 3 beacons and the opponent in the previous iteration (useful if fullscan=false)
 *   ________________1______________
 *  |                              |
 *  |                              |
 *  |                              |
 *  |                              |
 *  |                              |
 *  |______________________________|
 * 3                               2
 *
 * Beacon 3 at (0,0)
 */
void checkBeacon(double *angles, double *distances, double *quality, LidarData *lidarData, bool fullScan);


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
 * @param robot : array of size 2 : writes in the robot's x,y coordinates --> relative to beacon3
 * @param adv : array of size 4 : writes in the opponent's x,y,d,a coordinates --> relative to beacon3 and to our robot
 */
void lidarGetRobotPosition(LidarData *lidarData, int i);

void init_lidar(LidarData *lidarData);
void clear_lidar(LidarData *lidarData);

#endif //BOTLIGHTYEAR_LIDARTOP_H
