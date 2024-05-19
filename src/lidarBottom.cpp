#include "lidarBottom.h"
#include "lidar.h"
#include <cmath>
static double sizePlant = 0.051;
int arraysize = 8000;
bool printplotpy = false; //useful for graphing in Python later on

bool debug = false;

/// radius of a plant zone with a safety factor
double rayonZone = 0.17; 
/// radius of a plant with a safety factor
double rayonPlante = 0.018;

/// distance between the ref and the bottomLidar
double dLidarCentre = 0.055;

///coordonnées en x,y selon le repère balise b3=(0,0)
Point **zoneP;

/**
 * calculate the distance between 2 points
 * @param a : first point
 * @param b : second point
 * @return : distance between a and b
 */
double calculateDistance(Point *a, Point *b) {
    return std::sqrt(std::pow(b->x - a->x, 2) + std::pow(b->y - a->y, 2));
}

/**
 * @param a : point
 * @param b : point
 * @return : angle between the 2 point in radian
 */
double calculateAngle(Point *a, Point *b) {
    return std::atan2(b->y - a->y, b->x - a->x);
}

/**
 * estimating the position of the centres of the plant zones according to the lidar polar centre marker
 * @param robot
 * @param polarCoord
 */
void zoneInPolar(Point *robot, PlantZone **polarCoord) {
    for (int i = 0; i < 6; ++i) {
        polarCoord[i]->distance = calculateDistance(robot, zoneP[i]);
        polarCoord[i]->angle = moduloLidarMPIPI(calculateAngle(robot, zoneP[i]) - robot->theta);
    }

    if (printplotpy) {
        printf("[%f, %f, %f, %f, %f, %f], ", polarCoord[0]->angle, polarCoord[1]->angle, polarCoord[2]->angle,
               polarCoord[3]->angle, polarCoord[4]->angle, polarCoord[5]->angle);
        printf("[%f, %f, %f, %f, %f, %f]\n\n", polarCoord[0]->distance, polarCoord[1]->distance,
               polarCoord[2]->distance, polarCoord[3]->distance, polarCoord[4]->distance, polarCoord[5]->distance);
    }

    return;
}

/**
 * fills the structure with data from plants in the zones
 * @param robot : position of the lidar on the table
 * @param angles : liste of angles detect by the lidar
 * @param distances : liste of distances detect by the lidar
 * @param obstacle : not useful
 * @param arraysize : number of element in angles (same as in distances)
 * @param plantZone : liste of structure with the data of all plantzone
 */
void lidarGetPlantPosition(Point *robot, double *angles, double *distances, double *obstacle, int arraysize,
                           PlantZone **plantZone) {
    /*
     * ______________________
     * |                     |
     * |                     |
     * |     1       2       |
     * |                     |
     * |  0             3    |
     * |                     |
     * |      5       4      |
     * |                     |
     * |_____________________|
     */
    //Step 1: zone coordinates
    zoneInPolar(robot, plantZone);
    //at this stage, plantzone->distance,->angle are filled in

    //Step 2: Is it right in front of us?
    double deltaAngle;
    for (int i = 0; i < 6; ++i) {
        deltaAngle = atan(rayonZone /plantZone[i]->distance);
        plantZone[i]->startAngle = plantZone[i]->angle - deltaAngle;
        plantZone[i]->endAngle = plantZone[i]->angle + deltaAngle;

        /// Modulo
        plantZone[i]->startAngle = moduloLidarZero2PI(plantZone[i]->startAngle);
        plantZone[i]->endAngle = moduloLidarZero2PI(plantZone[i]->endAngle);

        // an area is considered accessible if it faces us
        if (plantZone[i]->angle < M_PI / 4.0 || plantZone[i]->angle > 7.0 * M_PI / 4.0) {
            plantZone[i]->isVisible = true;
        } else {
            plantZone[i]->isVisible = false;
        }
    }
    /// at this stage, we know the start and end angles of the zones and whether the zone is accessible

    bool objet = false;
    int start, stop, i;
    int countGap = 0;
    double size, a1, a2, d1, d2;
    d2 = 5.0;
    for (int zp = 0; zp < 6; ++zp) {
        printf("\n");
        if (plantZone[zp]->isVisible) {
            start = (int) (plantZone[zp]->startAngle / (2.0 * M_PI) * arraysize);
            stop = (int) (plantZone[zp]->endAngle / (2.0 * M_PI) * arraysize);
            objet = false;
            if (start > stop) {
                //useful if plant zone between 355 and 5° for example
                stop += arraysize;
            }

            for (int j = start; j < stop; ++j) {
                i = j % arraysize; // useful if plant zone between 355 and 5° for example
                
                /// check if the object is potentially on the zone
                if (std::abs(distances[i] - plantZone[zp]->distance) < rayonZone) {
                    if (!objet) {
                        /// no previous object: a new object to be initialized
                        objet = true;
                        a1 = angles[i];
                        d1 = distances[i];
                        d2 = 5.0;//to initiate the comparison
                    } else {
                        // object present before: if distance small enough it's the same (delta<3cm) -> nothing to do
                        // delta >3cm : new object
                        if (std::abs(d2 - distances[i]) > 0.03  ) {
                            //what we detect is a new object
                            plantZone[zp]->aPlant[plantZone[zp]->numberPlant] = a2;
                            plantZone[zp]->dPlant[plantZone[zp]->numberPlant] = d2+rayonPlante;
                            printf("debug d et a : %f %f\n", d2,a2*180.0/M_PI);
                            double gamma = robot->theta + plantZone[zp]->aPlant[plantZone[zp]->numberPlant] +1.8/180.0*M_PI;
                            double dist = plantZone[zp]->dPlant[plantZone[zp]->numberPlant];
                            plantZone[zp]->xPlant[plantZone[zp]->numberPlant] = robot->x + dist*std::cos(gamma);
                            plantZone[zp]->yPlant[plantZone[zp]->numberPlant] = robot->y + dist*std::sin(gamma);
                            if (debug){
                                printf("plant x = %f y = %f \n",plantZone[zp]->xPlant[plantZone[zp]->numberPlant], plantZone[zp]->yPlant[plantZone[zp]->numberPlant]);
                                printf("detect d= %f a=%f\n",plantZone[zp]->dPlant[plantZone[zp]->numberPlant],plantZone[zp]->aPlant[plantZone[zp]->numberPlant] );
                                printf("calcul gamma %f \n", gamma);
                            }


                            plantZone[zp]->numberPlant++;
                            /// new object : initial values are stored
                            d1 = distances[i];
                            a1 = angles[i];
                            d2 = 5.0;//to initiate the comparison

                        }
                    }
                    /// distance ok -> next object: update current end values
                    if (d2>distances[i]){
                        d2 = distances[i];
                        a2 = angles[i];
                    }
                    

                    /// we have an element so there are no data gaps
                    countGap = 1;

                } else {
                    countGap++;
                    if (objet && countGap / d2 > 10) {
                        objet = false;
                        plantZone[zp]->aPlant[plantZone[zp]->numberPlant] = a2;
                        plantZone[zp]->dPlant[plantZone[zp]->numberPlant] = d2+rayonPlante;
                        printf("debug d et a : %f %f\n", d2,a2*180.0/M_PI);

                        double gamma = robot->theta + plantZone[zp]->aPlant[plantZone[zp]->numberPlant]+1.8/180.0*M_PI;
                        double dist = plantZone[zp]->dPlant[plantZone[zp]->numberPlant];
                        plantZone[zp]->xPlant[plantZone[zp]->numberPlant] = robot->x + dist*std::cos(gamma);
                        plantZone[zp]->yPlant[plantZone[zp]->numberPlant] = robot->y + dist*std::sin(gamma);
                        if (debug){
                        printf("plant x = %f y = %f \n",plantZone[zp]->xPlant[plantZone[zp]->numberPlant], plantZone[zp]->yPlant[plantZone[zp]->numberPlant]);
                        printf("detect d= %f a=%f\n",plantZone[zp]->dPlant[plantZone[zp]->numberPlant],plantZone[zp]->aPlant[plantZone[zp]->numberPlant] );
                        printf("calcul gamma %f \n", gamma);}
                        plantZone[zp]->numberPlant++;
                        countGap = 1;
                    }
                }
            }
            if (objet) {
                plantZone[zp]->aPlant[plantZone[zp]->numberPlant] = a2;
                plantZone[zp]->dPlant[plantZone[zp]->numberPlant] = d2+rayonPlante;
                printf("debug d et a : %f %f\n", d2,a2*180.0/M_PI);

                double gamma = robot->theta + plantZone[zp]->aPlant[plantZone[zp]->numberPlant]+1.8/180.0*M_PI;
                double dist = plantZone[zp]->dPlant[plantZone[zp]->numberPlant];
                plantZone[zp]->xPlant[plantZone[zp]->numberPlant] = robot->x + dist*std::cos(gamma);
                plantZone[zp]->yPlant[plantZone[zp]->numberPlant] = robot->y + dist*std::sin(gamma);
                if (debug){
                printf("plant x = %f y = %f \n",plantZone[zp]->xPlant[plantZone[zp]->numberPlant], plantZone[zp]->yPlant[plantZone[zp]->numberPlant]);
                printf("detect d= %f a=%f\n",plantZone[zp]->dPlant[plantZone[zp]->numberPlant],plantZone[zp]->aPlant[plantZone[zp]->numberPlant] );
                printf("calcul gamma %f \n", gamma);}
                plantZone[zp]->numberPlant++;
            }
        }
    }
    if (printplotpy) {
        for (int j = 0; j < 6; ++j) {
            for (int k = 0; k < plantZone[j]->numberPlant; ++k) {
                printf("%f, ", plantZone[j]->dPlant[k]);
            }
        }
        printf("\n");
        for (int j = 0; j < 6; ++j) {
            for (int k = 0; k < plantZone[j]->numberPlant; ++k) {
                printf("%f, ", plantZone[j]->aPlant[k]);
            }

        }
        printf("\n");
    }


}


/**
 *
 * @param x_robot : position of the robot
 * @param y_robot : position of the robot
 * @param theta_robot : orientation of the robot
 * @param zone : array with 6 int : 1 if we will check the zone, 0 however
 * @param plantZonePolar : structure with all data need (init by initBottomLidar)
 * @return
 */
int getNumberOfPlantInAllZone(double x_robot, double y_robot, double theta_robot, int *zone, PlantZone **plantZonePolar) {
    double *angles = new double[8000];
    double *distances = new double[8000];
    double *quality = new double[8000];
    double *obj = new double[8000];
    Point *robot = new Point[sizeof(Point)];
    robot->x = x_robot + dLidarCentre * cos(theta_robot);
    robot->y = y_robot + dLidarCentre * sin(theta_robot);
    robot->theta = theta_robot;
    printf("position odo %f %f %f\nposition lid %f %f\n", x_robot, y_robot,theta_robot*180.0/M_PI, robot->x, robot->y);
    
    size_t *asize = new size_t[2]{8000, 8000};
    updateDataBottom(angles, distances, quality, asize);
    arraysize = asize[0];
    lidarGetPlantPosition(robot, angles, distances, obj, asize[0], plantZonePolar);
    for (int i = 0; i < 6; ++i) {
        double dmin = 10;
        for (int j = 0; j < plantZonePolar[i]->numberPlant; ++j) {
            if (plantZonePolar[i]->dPlant[j]<dmin){
                plantZonePolar[i]->xClosestPlant = plantZonePolar[i]->xPlant[j];
                plantZonePolar[i]->yClosestPlant = plantZonePolar[i]->yPlant[j];
                dmin = plantZonePolar[i]->dPlant[j];
            }
        }
    }
    delete[] (angles);
    delete[] (distances);
    delete[] (quality);
    delete[] (obj);
    delete[] (robot);
    delete[] (asize);
    return 1;
}

/**
 *
 * @param polarCoord structure for init (already allocate)
 */
void initBottomLidar(PlantZone **polarCoord) {
    zoneP = new Point *[6 * sizeof(Point)];
    zoneP[0] = new Point[sizeof(Point)];
    zoneP[0]->x = -0.6+1;
    zoneP[0]->y = 0.0+1.5;
    zoneP[1] = new Point[sizeof(Point)];
    zoneP[1]->x = -0.3+1;
    zoneP[1]->y = 0.5+1.5;
    zoneP[2] = new Point[sizeof(Point)];
    zoneP[2]->x = 0.3+1;
    zoneP[2]->y = 0.5+1.5;
    zoneP[3] = new Point[sizeof(Point)];
    zoneP[3]->x = 0.6+1;
    zoneP[3]->y = 0.0+1.5;
    zoneP[4] = new Point[sizeof(Point)];
    zoneP[4]->x = 0.3+1;
    zoneP[4]->y = -0.5+1.5;
    zoneP[5] = new Point[sizeof(Point)];
    zoneP[5]->x = -0.3+1;
    zoneP[5]->y = -0.5+1.5;

    for (int i = 0; i < 6; i++) {
        polarCoord[i] = new PlantZone[sizeof(PlantZone)];
        polarCoord[i]->distance = 0.0;
        polarCoord[i]->angle = 0.0; // en radians (angle au centre)
        polarCoord[i]->startAngle = 0.0;
        polarCoord[i]->endAngle = 0.0;
        polarCoord[i]->isVisible = true;
        polarCoord[i]->empty = false;
        polarCoord[i]->numberPlant = 0;
        polarCoord[i]->aPlant = new double[20];
        polarCoord[i]->dPlant = new double[20];
        polarCoord[i]->xPlant = new double[20];
        polarCoord[i]->yPlant = new double[20];
        polarCoord[i]->xClosestPlant = 10;
        polarCoord[i]->yClosestPlant = 10;
    }
    
    return;
}


/**
 *
 * @param polarCoord : structure for delete
 */
void deleteBottomLidar(PlantZone **polarCoord) {
    for (size_t i = 0; i < 6; i++) {
        delete (polarCoord[i]);
        delete (zoneP[i]);
    }
    delete (polarCoord);
    delete (zoneP);
    return;
}
