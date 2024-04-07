#ifndef BLY_LIDAR_H
#define BLY_LIDAR_H

#include "rplidar_sdk/rplidar.h"
#include <cstdio>
#include <cstdlib>
#include <cmath>

//pour plot
#include <iostream>
#include <vector>
using std::string;

//For sleep
#include <unistd.h>

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
 *  start lidar to retrieve data for other functions
 */
void StartLidar();


/**
 * close Lidar
 */
void StopLidar();


/**
 * input : 3 empty tables to be filled with lidar data
 * @param angles in radian
 * @param distances in m
 * @param quality
 * @param arraySize : maximum array size
 */
void updateData(double* angles, double* distances, double* quality,  size_t * arraySize);


/**
 * input : 3 empty tables to be filled with lidar, the data being saved in the filename file
 * @param angles input : void; output : angles from lidar in radian
 * @param distances input : void; output : distances from lidar in meters
 * @param quality input : void; output : void (not used)
 * @param filename input : name of the file containing all the lidar data to be analysed
 * @param arraySize output : updates the number of elements
 */
void updateDataFile(double* angles, double* distances, double* quality, string filename,  size_t * arraySize);

/**
 * Saves lidar data for local testing afterwards
 * @param filename : name of the file in which the data will be saved
 */
void DataToFile(string filename);

void init_lidar(LidarData *lidarData);




#endif //MECATROMINIBOT_LIDAR_H