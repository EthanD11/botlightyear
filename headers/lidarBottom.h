
#ifndef MECATROMINIBOT_LIDARBOTTOM_H
#define MECATROMINIBOT_LIDARBOTTOM_H
#include "lidar.h"
typedef struct Point {
    double x;
    double y;
    double theta;
}Point;

typedef struct PlantZone {
    double distance;
    double angle; // en radians (angle au centre)
    double startAngle;
    double endAngle;
    bool isAccessible;
    bool empty;
    int numberPlant = 0;
    double* aPlant = new double[6];
    double* dPlant = new double[6];
}PlantZone;

//void obstaclesPosition(double* dCoin, double* aCoin, double* angles, double* distances, double* dObstacles, double* aObstacles);

#endif //MECATROMINIBOT_LIDARBOTTOM_H
