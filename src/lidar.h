#ifndef MECATROMINIBOT_LIDAR_H
#define MECATROMINIBOT_LIDAR_H

#include <rplidar.h>
#include <cstdio>
#include <cstdlib>
#include <cmath>

//pour plot
#include <iostream>
#include <vector>
using std::string;

//For sleep
#include <unistd.h>



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
void updateData(double* angles, double* distances, double* quality, int arraySize);


/**
 * input : 3 empty tables to be filled with lidar, the data being saved in the filename file
 * @param angles input : void; output : angles from lidar in radian
 * @param distances input : void; output : distances from lidar in meters
 * @param quality input : void; output : void (not used)
 * @param filename input : name of the file containing all the lidar data to be analysed
 * @param arraySize output : updates the number of elements
 */
void updateDataFile(double* angles, double* distances, double* quality, string filename, size_t *arraySize);

/**
 * Saves lidar data for local testing afterwards
 * @param filename : name of the file in which the data will be saved
 */
void DataToFile(string filename);


#endif //MECATROMINIBOT_LIDAR_H