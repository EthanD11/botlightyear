#ifndef BLY_LIDARBOTTOM_H
#define BLY_LIDARBOTTOM_H

#include "lidar.h"
#include <chrono>

typedef struct Point {
    double x;
    double y;
    double theta;
} Point;

typedef struct PlantZone {
    double distance;
    double angle; // en radians (angle au centre)
    double startAngle;
    double endAngle;
    bool isAccessible;
    bool empty;
    int numberPlant;
    double *aPlant;
    double *dPlant;
    double *xPlant;
    double *yPlant;
    double xClosestPlant;
    double yClosestPlant;

} PlantZone;


int getNumberOfPlantInAllZone(double x_robot, double y_robot, double theta_robot, int *zone, PlantZone **plantZonePolar);

void initBottomLidar(PlantZone **polarCoord);

void positionBottomLidarLeftFront(double* return_x, double* return_y, double* return_theta);

void calibrationBottom(double* return_x, double* return_y, double* return_theta);
#endif //MECATROMINIBOT_LIDARBOTTOM_H
