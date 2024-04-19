#include "lidarTop.h"
#include <chrono>
#include "shared_variables.h"

///facteur si mouvent brusque
int facteurLost = 1;

///pour dif un full scan d'un scan full en connaissant sa precedence position
bool fullScanPcqLost = false;
bool premierScan = true;

///global variable to find out the size of the file
size_t arraySize = 8000;

/// distance between 2 beacons on the same side
double dref1 = 2 * 0.950;

/// distance between 2 beacons not on the same side
double dref2 = sqrt((0.95) * (0.95) + (1.594 * 2) * (2 * 1.594));


///taille balise
double largeurMatBalise = 0.12;//TODO modif remettre 0.065
double largeurMatAdvers = 0.22;

///utile pour des prints
bool analyseDetail = false;
bool analyseDetail_objet = false;
bool analyseRotationBalise = false;

///précision pour le déplacement effectué-> permet d'etre augmenté si on est perdu
double precisionPredef = 0.25;
double precisionAngle = 90.0/180.0*M_PI;

///décalage balise et coin de la table (x_reel = x_B3 + delta)
double deltaXB3 = 0.05;
double deltaYB3 = -0.09;

///dimension de la table
double tableX = 2.1;
double tableY = 3.1;
double minTableX = -0.1;
double minTableY = 0.01;


/**
 * We calculate the position of the robot in beacon reference
 * from the polar coordinates of the 3 beacons and knowing that the robot is at the origin of the reference frame,
 * we recover the coordinates of the robot in (x,y) according to the reference frame determined with beacon 3 in (0,0).
 *
 * @param db : distance between beacons 2 by 2
 * @param x : positions of the beacons in x (reference robot (0;0)
 * @param y : positions of the beacons in y (reference robot (0;0)
 * @param lidarData : structure where we can save the position
 * @param anglesBeacons : angle between the robot and the beacons
 * Save beacon such that :
 * ---------D1---------
 * |                  |
 * |                  |
 * |                  |
 * |                  |
 * |                  |
 * D3----------------D2
 * (0,0)
 */
void rotationPosition(double *db, double *x, double *y, LidarData *lidarData, double *anglesBeacons) {
    double *beacon1;
    double *beacon2;
    double *beacon3;

    ///in the current reference of the beacons, the robot is in (0,0)
    lidarData->x_robot = 0.0;
    lidarData->y_robot = 0.0;
    double orientation;

    for (int i = 0; i < 3; ++i) {
        if (std::abs(db[(i + 2) % 3] - db[(i + 1) % 3]) < 0.36) {
            //we will always have this condition met 1 time, otherwise we would not have used the function
            beacon1 = new double[2]{x[i], y[i]};
            beacon2 = new double[2]{x[(i + 1) % 3], y[(i + 1) % 3]};
            beacon3 = new double[2]{x[(i + 2) % 3], y[(i + 2) % 3]};

            ///to determine the orientation of the robot on the table relative to beacon3
            orientation = anglesBeacons[(i + 2) % 3];
        }
    }

    ///Transformation so that D3 is in 0.0
    ///save transfo
    lidarData->transfo_x = beacon3[0];
    lidarData->transfo_y = beacon3[1];

    beacon2[1] -= beacon3[1];
    lidarData->y_robot -= beacon3[1];

    beacon2[0] -= beacon3[0];
    lidarData->x_robot -= beacon3[0];
    if (analyseRotationBalise) {
        beacon1[1] -= beacon3[1];
        beacon3[1] -= beacon3[1];

        beacon1[0] -= beacon3[0];
        beacon3[0] -= beacon3[0];
    }

    ///calculating the angle of rotation for the second transformation
    double alpha = -atan2(beacon2[1], beacon2[0]);

    ///save transfo
    lidarData->transfo_a = alpha;

    ///Rotation so that the table is in positive x, y
    double xtemp;
    double ytemp;

    if (analyseRotationBalise) {
        xtemp = beacon2[0];
        ytemp = beacon2[1];
        beacon2[0] = cos(alpha) * xtemp - sin(alpha) * ytemp;
        beacon2[1] = sin(alpha) * xtemp + cos(alpha) * ytemp;

        xtemp = beacon1[0];
        ytemp = beacon1[1];
        beacon1[0] = cos(alpha) * xtemp - sin(alpha) * ytemp;
        beacon1[1] = sin(alpha) * xtemp + cos(alpha) * ytemp;
    }

    xtemp = lidarData->x_robot;
    ytemp = lidarData->y_robot;
    lidarData->x_robot = cos(alpha) * xtemp - sin(alpha) * ytemp;
    lidarData->y_robot = sin(alpha) * xtemp + cos(alpha) * ytemp;

    ///to determine the orientation of the robot on the table
    //lidarData->orientation_robot = M_PI - orientation + atan2(lidarData->x_robot, lidarData->y_robot);

    //lidarData->orientation_robot = moduloLidarMPIPI(M_PI/2-lidarData->orientation_robot);
    lidarData->orientation_robot = moduloLidarMPIPI(-M_PI/2.0+orientation-atan2(lidarData->x_robot, lidarData->y_robot));


    delete[] (beacon1);
    delete[] (beacon2);
    delete[] (beacon3);
    return;

}

/**
 * Find the adversaire and save its position relative to the map and its position relative to the robot in lidarData
 * @param anglesAdv coordinates relative to the robot of possible opponents
 * @param distancesAdv coordinates relative to the robot of possible opponents;
 * @param lidarData structure with useful data
 *
 */
int Adversary(double *anglesAdv, double *distancesAdv, LidarData *lidarData) {
    ///transfo contains 4 elem : deltaX, deltaY, angle of rotation, the number of elements in possible opponents (number of elements in *anglesAdv)
    int size = lidarData->countObj_adv;


    ///maximum table dimensions
    double xmax = tableX;
    double ymax = tableY;

    ///coordinates in xy after one and two transformations (translations then rotation)
    double xtemp;
    double ytemp;
    double xobj;
    double yobj;
    double yold = 6;

    for (int i = 0; i < size; ++i) {
        /// transformation identical to that of beacons and robots
        xtemp = (distancesAdv[i] * std::cos(-anglesAdv[i])) - lidarData->transfo_x;
        ytemp = (distancesAdv[i] * std::sin(-anglesAdv[i])) - lidarData->transfo_y;
        xobj = (cos(lidarData->transfo_a) * xtemp - sin(lidarData->transfo_a) * ytemp);
        /// check whether the x coordinate is valid (on the table)
        if (xobj > 0.001 && xobj < xmax) {
            yobj = (sin(lidarData->transfo_a) * xtemp + cos(lidarData->transfo_a) * ytemp);

            ///check whether the y coordinate is valid (on the table)
            if (yobj > 0.001 && yobj < ymax && abs(yobj-1.5)<abs(yold-1.5)) {
                /// if the object is on the table, it's our opponent,
                /// we save its coordinates in the new base (xy based on beacon3)
                /// and the original coordinates (relative to the robot, distance and angle)
                if(!((yobj<0.15) && ((abs(xobj)<0.15)||(abs(xobj-1.0)<0.15)||(abs(xobj-1.9)<0.15)))||((abs(yobj-3)<0.2) && ((abs(xobj)<0.15)||(abs(xobj-1.0)<0.15)||(abs(xobj-1.9)<0.15)))){
                lidarData->x_adv = xobj;
                lidarData->y_adv = yobj;
                lidarData->d_adv = distancesAdv[i];
                lidarData->a_adv = anglesAdv[i];
                yold = yobj;
                }
                /// we assume that the lidar can only see one object on the table
                /// and that it is therefore automatically the robot
                //return 0;
            }
        }
    }
    /// We haven't found the opponent, by default the coordinates remain in 0
    return (yold==6);
}

/**
 * if we don't found adversary from previous data we check everywhere
 * @param angles array with angle in radian from lidar
 * @param distances array with distances in m from lidar
 * @param lidarData structure with previous data and where we save current data
 */
void lidarPerduAdv(double *angles, double *distances, LidarData *lidarData) {
    bool objet;
    double distMax = 3.55;
    ///distance and angle of the first and last points of an object
    double d1 = 0;
    double d2 = 0;
    double a1 = 0;
    double a2 = 0;

    /// size of detected object
    double size;

    /// list of objects that are possible beacon
    double *dObj_adv, *aObj_adv;
    dObj_adv = new double[arraySize];
    aObj_adv = new double[arraySize];

    /// number of items in list
    int countObj_adv = 0;

    /// count of missing data+1
    /// useful for knowing the difference in maximum distance when a lot of data is lost between 2 elements
    int countGap = 1;
    objet = false;
    for (int i = 0; i < arraySize; ++i) {

        /// check if the object is potentially on the table
        if (0.2 < distances[i] && distances[i] < distMax - 0.1) {

            /// no previous object: a new object to be initialized
            if (!objet) {
                objet = true;
                a1 = angles[i];
                d1 = distances[i];
            } else {
                // object present before: if distance small enough it's the same (delta<2cm) -> nothing to do
                // delta >2cm : new object -> check if the previous one is a beacon

                //TODO check le count gap max possible : prendre donnée d'un poteau le plus porche possible et voir combien d'éléments ca prend
                // lien avec d2 ok ??

                if (std::abs(d2 - distances[i]) > 0.1) {//TODO CHECK CE PROB :  && countGap/d2<10) {
                    ///size of object previously detected
                    size = std::sqrt(d2 * d2 + d1 * d1 - 2 * d2 * d1 * std::cos(a2 - a1));

                    if (size < 0.11) {
                        aObj_adv[countObj_adv] = (a1 + a2) / 2;
                        dObj_adv[countObj_adv] = (d1 + d2) / 2;
                        countObj_adv++;
                    }
                    /// new object : initial values are stored
                    d1 = distances[i];
                    a1 = angles[i];
                }
            }
            /// distance ok -> next object: update current end values
            d2 = distances[i];
            a2 = angles[i];

            /// we have an element so there are no data gaps
            countGap = 1;

        } else if ((distances[i] != 0) && objet) {
            size = std::sqrt(d2 * d2 + d1 * d1 - 2 * d2 * d1 * std::cos(a2 - a1));
            if (size < 0.15 && d1 > 0) {
                aObj_adv[countObj_adv] = (a1 + a2) / 2;
                dObj_adv[countObj_adv] = (d1 + d2) / 2;
                countObj_adv++;
            }
            d1 = 0;
            d2 = 0;
            a1 = 0;
            a2 = 0;
            objet = false;
            countGap = 1;
        } else if (distances[i] == 0) {
            countGap += 1;
            if (countGap>5){
                if (objet) {
                    size = std::sqrt(d2 * d2 + d1 * d1 - 2 * d2 * d1 * std::cos(a2 - a1));
                        if (size < 0.15 && d1 > 0) {
                            aObj_adv[countObj_adv] = (a1 + a2) / 2;
                            dObj_adv[countObj_adv] = (d1 + d2) / 2;
                            countObj_adv++;
                        }
                }
            d1 = 0;
            d2 = 0;
            a1 = 0;
            a2 = 0;
            objet = false;
            countGap = 1;
            }
        }
    }
    /// we save the number of elements that could possibly be beacon (opponent)
    Adversary(aObj_adv, dObj_adv, lidarData);
    ///we assume that we can only find 3 points corresponding to our beacons once
    delete (dObj_adv);
    delete (aObj_adv);
    return;
}

int foundAdvWithOdo(double *anglesAdv, double *distancesAdv, LidarData *lidarData){
    //recalcule des transformations pour trouver l'adversaire
    ///transfo contains 4 elem : deltaX, deltaY, angle of rotation, the number of elements in possible opponents (number of elements in *anglesAdv)
    int size = lidarData->countObj_adv;
    ///maximum table dimensions
    double xmax = tableX;
    double ymax = tableY;

    ///coordinates in xy after one and two transformations (translations then rotation)
    double xobj;
    double yobj;
    double yold = 6;

    for (int i = 0; i < size; ++i) {
        //printf("robot : %f %f %f\n", lidarData->x_robot, lidarData->y_robot, lidarData->orientation_robot );
        /// transformation identical to that of beacons and robots
        double gamma =  lidarData->orientation_robot - anglesAdv[i];
        xobj = lidarData->x_robot + distancesAdv[i] * std::cos(gamma);
        yobj = lidarData->y_robot + distancesAdv[i] * std::sin(gamma);
        //printf("tttttttt %f %f %f %f %f\n", distancesAdv[i],anglesAdv[i]*180.0/M_PI, xobj,yobj, gamma);

        if (xobj > 0.03 && xobj < xmax-0.03) {

            ///check whether the y coordinate is valid (on the table)
            ///deplacement de l'adversaire
            double deltX = lidarData->old_x_adv-xobj;
            double deltY = lidarData->old_y_adv-yobj;
            
            double depl = sqrt(deltX*deltX+deltY*deltY);
            if (yobj > 0.03 && yobj < ymax-0.03) {
                //&& (yobj-1.5)<abs(yold-1.5)
                /// if the object is on the table, it's our opponent,
                /// we save its coordinates in the new base (xy based on beacon3)
                /// and the original coordinates (relative to the robot, distance and angle)
                lidarData->x_adv = xobj;
                lidarData->y_adv = yobj;
                lidarData->d_adv = distancesAdv[i];
                lidarData->a_adv = anglesAdv[i];
                yold = yobj;
                /// we assume that the lidar can only see one object on the table
                /// and that it is therefore automatically the robot
            }
        }
    }
    /// We haven't found the opponent, by default the coordinates remain in 0
    return (yold==6);
}

/**
 * From the raw lidar data, save in lidarData the coordinates of the robot according to the defined plane and the coordinates of the opponent
 * @param angles : array of size max 8192 with angle in degree
 * @param distances : array of size max 8192 with distances in m
 * @param quality : array of size max 8192 with quality
 * @param lidarData : structure with useful data (previous data) and where we save the new data
 * @param fullScan : if true: performs a full scan with no position prediction,
 *                  if false: performs a more accurate scan, mimicked by an estimate of the position of the beacons and the opponent
 *
 * if (fullScan) we need data in lidarData->beaconAdv : table of 8 with the angles and distances of the 3 beacons and the opponent (a1, d1, a2, d2, a3, d3, a, d)
 *   __________1_________
 *  |                   |
 *  |                   |
 *  |                   |
 *  |                   |
 *  |                   |
 *  |___________________|
 * 3                    2
 *
 * Beacon 3 at (0,0)
 */
void checkBeacon(double *angles, double *distances, double *quality, LidarData *lidarData, bool fullScan) {

    double oldXRobot = lidarData->x_robot;
    double oldYRobot = lidarData->y_robot;
    double oldOrientationRobot = lidarData->orientation_robot;
    double olddistB1 = lidarData->beaconAdv[1];
    double olddistB2 = lidarData->beaconAdv[3];
    double olddistB3 = lidarData->beaconAdv[5];
    double oldAngB1 = lidarData->beaconAdv[0];
    double oldAngB2 = lidarData->beaconAdv[2];
    double oldAngB3 = lidarData->beaconAdv[4];

    ///par défaut a 3.55 mais peut être diminuer en fonction des distances des balises précédentes
    double distMax = 3.55;

    ///objet==true : object detected at probable distance
    bool objet = false;

    ///distance and angle of the first and last points of an object
    double d1 = 0;
    double d2 = 0;
    double a1 = 0;
    double a2 = 0;

    /// size of detected object
    double size;

    /// list of objects that are possible beacon
    double *dObj = new double[arraySize];
    double *aObj = new double[arraySize];
    double *dObj_adv = new double[arraySize];
    double *aObj_adv = new double[arraySize];

    /// number of items in list
    int countObj = 0;
    int countObj_adv = 0;

    /// count of missing data+1
    /// useful for knowing the difference in maximum distance when a lot of data is lost between 2 elements
    int countGap = 1;
    for (int i = 0; i < arraySize; ++i) {
        /// check if the object is potentially on the table
        if (0.1 < distances[i] && distances[i] < distMax) {
            /// no previous object: a new object to be initialized
            if (!objet) {
                objet = true;
                a1 = angles[i];
                d1 = distances[i];

            } else {
                // object present before: if distance small enough it's the same (delta<2cm) -> nothing to do
                // delta >2cm : new object -> check if the previous one is a beacon
                //TODO check le count gap max possible : prendre donnée d'un poteau le plus porche possible et voir combien d'éléments ca prend
                // lien avec d2 ok ??

                if (std::abs(d2 - distances[i]) > 0.1) {//TODO CHECK CE PROB :  && countGap/d2<10) {
                    //what we detect is a new object

                    ///size of object previously detected
                    size = std::sqrt(d2 * d2 + d1 * d1 - 2 * d2 * d1 * std::cos(a2 - a1));
                    if ((size < largeurMatBalise) && (fullScan || premierScan || (((std::abs(d1 - olddistB1) < 0.2)) ||
                                                      ((std::abs(d1 - olddistB2) < 0.2)) ||
                                                      ((std::abs(d1 - olddistB3) < 0.2))))) {
                        ///it may be a beacon, its data is saved
                        aObj[countObj] = (a1 + a2) / 2;
                        dObj[countObj] = (d1 + d2) / 2;
                        countObj++;
                    }
                    ///a beacon on an opponent can be larger
                    if (size < largeurMatAdvers) {
                        aObj_adv[countObj_adv] = (a1 + a2) / 2;
                        dObj_adv[countObj_adv] = (d1 + d2) / 2;
                        countObj_adv++;
                    }
                    /// new object : initial values are stored
                    d1 = distances[i];
                    a1 = angles[i];
                }
            }
            /// distance ok -> next object: update current end values
            d2 = distances[i];
            a2 = angles[i];

            /// we have an element so there are no data gaps
            countGap = 1;

        } else if ((distances[i] != 0) && objet) {
            // if outside desired distance :
            //  - either 0 (not taken into account: possible lack of values in data capture)
            //   -object too far away
            //      -> end of object (objet==true : same as above)
            //      -> no previous object : nothing to do
            size = std::sqrt(d2 * d2 + d1 * d1 - 2 * d2 * d1 * std::cos(a2 - a1));

            if (size < largeurMatBalise && d1 > 0) {
                aObj[countObj] = (a1 + a2) / 2;
                dObj[countObj] = (d1 + d2) / 2;
                countObj++;
            }
            if (size < largeurMatAdvers && d1 > 0) {
                aObj_adv[countObj_adv] = (a1 + a2) / 2;
                dObj_adv[countObj_adv] = (d1 + d2) / 2;
                countObj_adv++;
            }
            d1 = 0;
            d2 = 0;
            a1 = 0;
            a2 = 0;
            objet = false;
            countGap = 1;
        } else if (distances[i] == 0) {
            countGap += 1;
            if (countGap>5){
                if(objet){
                    size = std::sqrt(d2 * d2 + d1 * d1 - 2 * d2 * d1 * std::cos(a2 - a1));
                    if (size < largeurMatBalise && d1 > 0) {
                        aObj[countObj] = (a1 + a2) / 2;
                        dObj[countObj] = (d1 + d2) / 2;
                        countObj++;
                    }
                    if (size < largeurMatAdvers && d1 > 0) {
                        aObj_adv[countObj_adv] = (a1 + a2) / 2;
                        dObj_adv[countObj_adv] = (d1 + d2) / 2;
                        countObj_adv++;
                    }
                    d1 = 0;
                    d2 = 0;
                    a1 = 0;
                    a2 = 0;
                    objet = false;
                    countGap = 1;
                }
            }
        }
    }
    if (analyseDetail) {
        printf("count : %d %d\n", countObj, countObj_adv);
    }

    lidarData->countObj_adv = countObj_adv;
    // We now have a list of objects whose size could match that of a beacon,
    // and based on their positions we'll determine which 3 objects are a beacon

    ///coordinates and distances of the 3 beacons
    double x1, x2, x3, y1, y2, y3, db1, db2, db3;

    if (analyseDetail_objet) {
        printf("result\n");
        for (int i = 0; i < countObj; i++) {
            printf("%f %f \n", aObj[i] * 180 / M_PI, dObj[i]);
        }
    }

    ///récup balise perdue
    int countObjPlusOld = countObj;
    bool B1 = false;
    bool B2 = false;
    bool B3 = false;
    //TODO delete ?
    for (int i = 0; i < countObj; ++i) {
        if ((std::abs(dObj[i] - olddistB1) < 0.2) &&
            ((std::abs(aObj[i] - oldAngB1) < 0.5) || std::abs(2 * M_PI - std::abs(aObj[i] - oldAngB1)) < 0.5)) {
            B1 = true;
            if (analyseDetail) {
                printf("B1 ok %f\n", aObj[i] * 180.0 / M_PI);
            }
        }
        if ((std::abs(dObj[i] - olddistB2) < 0.2) &&
            ((std::abs(aObj[i] - oldAngB2) < 0.5) || std::abs(2 * M_PI - std::abs(aObj[i] - oldAngB2)) < 0.5)) {
            B2 = true;
            if (analyseDetail) {
                printf("B2 ok %f\n", aObj[i] * 180.0 / M_PI);
            }

        }
        if ((std::abs(dObj[i] - olddistB3) < 0.2) &&
            ((std::abs(aObj[i] - oldAngB3) < 0.5) || std::abs(2 * M_PI - std::abs(aObj[i] - oldAngB3)) < 0.5)) {
            B3 = true;
            if (analyseDetail) {
                printf("B3 ok %f\n", aObj[i] * 180.0 / M_PI);
            }

        }
    }

    dObj[countObjPlusOld] = olddistB1 + 0.05;
    aObj[countObjPlusOld] = oldAngB1 + 0.05;
    countObjPlusOld++;

    dObj[countObjPlusOld] = olddistB1 - 0.05;
    aObj[countObjPlusOld] = oldAngB1 + 0.05;
    countObjPlusOld++;

    dObj[countObjPlusOld] = olddistB1 - 0.05;
    aObj[countObjPlusOld] = oldAngB1 - 0.05;
    countObjPlusOld++;

    dObj[countObjPlusOld] = olddistB1 + 0.05;
    aObj[countObjPlusOld] = oldAngB1 - 0.05;
    countObjPlusOld++;


    dObj[countObjPlusOld] = olddistB1;
    aObj[countObjPlusOld] = oldAngB1;
    countObjPlusOld++;

    dObj[countObjPlusOld] = olddistB2 + 0.05;
    aObj[countObjPlusOld] = oldAngB2 + 0.05;
    countObjPlusOld++;

    dObj[countObjPlusOld] = olddistB2 - 0.05;
    aObj[countObjPlusOld] = oldAngB2 + 0.05;
    countObjPlusOld++;

    dObj[countObjPlusOld] = olddistB2 - 0.05;
    aObj[countObjPlusOld] = oldAngB2 - 0.05;
    countObjPlusOld++;

    dObj[countObjPlusOld] = olddistB2 + 0.05;
    aObj[countObjPlusOld] = oldAngB2 - 0.05;
    countObjPlusOld++;

    dObj[countObjPlusOld] = olddistB2;
    aObj[countObjPlusOld] = oldAngB2;
    countObjPlusOld++;


    dObj[countObjPlusOld] = olddistB3 + 0.05;
    aObj[countObjPlusOld] = oldAngB3 + 0.05;
    countObjPlusOld++;

    dObj[countObjPlusOld] = olddistB3 - 0.05;
    aObj[countObjPlusOld] = oldAngB3 + 0.05;
    countObjPlusOld++;

    dObj[countObjPlusOld] = olddistB3 - 0.05;
    aObj[countObjPlusOld] = oldAngB3 - 0.05;
    countObjPlusOld++;

    dObj[countObjPlusOld] = olddistB3 + 0.05;
    aObj[countObjPlusOld] = oldAngB3 - 0.05;
    countObjPlusOld++;

    dObj[countObjPlusOld] = olddistB3;
    aObj[countObjPlusOld] = oldAngB3;
    countObjPlusOld++;
    ///récup balise perdue fin

    int startb3;
    int stopb3;

    int startb2;
    int stopb2;
    /// if i==0 : look for the 3 beacons among the found objects
    /// if i==1 : we assume that 1 beacon has been lost
    /// if i==1 : we assume that 2 beacons have been lost
    for (int i = 0; i < 3; ++i) {
        if (i == 0) {
            stopb3 = countObj;
        } else {
            stopb3 = countObjPlusOld;
        }
        if (i < 2) {
            stopb2 = countObj;
        } else {
            stopb2 = countObjPlusOld;
        }


        ///b1, b2, b3  : 3 beacons
        for (int ba1 = 0; ba1 < countObj; ++ba1) {
            if (i < 2) {
                startb2 = ba1 + 1;
            } else {
                startb2 = countObj;
            }
            for (int ba2 = startb2; ba2 < stopb2; ++ba2) {
                if (i == 0) {
                    startb3 = ba2 + 1;
                } else {
                    startb3 = countObj;
                }
                for (int ba3 = startb3; ba3 < stopb3; ++ba3) {
                    ///utile dans le cas ou on recrée des balises
                    int b1 = ba1;
                    int b2 = ba2;
                    int b3 = ba3;
                    int temp;
                    if (aObj[b1] > aObj[b2]) {
                        temp = b1;
                        b1 = b2;
                        b2 = temp;
                    }
                    if (aObj[b2] > aObj[b3]) {
                        temp = b2;
                        b2 = b3;
                        b3 = temp;
                    }
                    if (aObj[b1] > aObj[b2]) {
                        temp = b1;
                        b1 = b2;
                        b2 = temp;
                    }

                    ///position in (x,y) of the 3 beacons with the robot in (0,0)
                    x1 = dObj[b1] * std::cos(-aObj[b1]);
                    x2 = dObj[b2] * std::cos(-aObj[b2]);
                    x3 = dObj[b3] * std::cos(-aObj[b3]);

                    y1 = dObj[b1] * std::sin(-aObj[b1]);
                    y2 = dObj[b2] * std::sin(-aObj[b2]);
                    y3 = dObj[b3] * std::sin(-aObj[b3]);

                    db3 = std::sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
                    db1 = std::sqrt((x2 - x3) * (x2 - x3) + (y2 - y3) * (y2 - y3));
                    db2 = std::sqrt((x3 - x1) * (x3 - x1) + (y3 - y1) * (y3 - y1));

                    /// sum of the 3 sides close to the expected value
                    
                    if (std::abs((db1 + db2 + db3) - dref1 - 2 * dref2) < 0.5) {//TODO check precision
                        /// 2 size sides ok (3rd also ok because sum ok)
                        
                        if (((std::abs(db1 - dref1) < 0.15) || (std::abs(db2 - dref1) < 0.15) ||
                             (std::abs(db3 - dref1) < 0.15)) &&
                            ((std::abs(db1 - dref2) < 0.15) || (std::abs(db2 - dref2) < 0.15) ||
                             (std::abs(db3 - dref2) < 0.15)) &&
                            ((std::abs(db1 - db2) < 0.35) || (std::abs(db3 - db2) < 0.35) ||
                             (std::abs(db1 - db3) < 0.35))) {
                            double *db = new double[3]{db1, db2, db3};
                            double *yrp = new double[3]{y1, y2, y3};
                            double *xrp = new double[3]{x1, x2, x3};
                            double *abeac = new double[3]{aObj[b1], aObj[b2], aObj[b3]};
                            rotationPosition(db, xrp, yrp, lidarData, abeac);
                            delete[] (db);
                            delete[] (yrp);
                            delete[] (xrp);
                            delete[] (abeac);
                            if (i == 0) {
                                if (analyseDetail) {
                                    printf("angle : %f %f %f\n", 180.0 / M_PI * aObj[b1], 180.0 / M_PI * aObj[b2],
                                           180.0 / M_PI * aObj[b3]);
                                    printf("dist : %f %f %f \n", dObj[b1], dObj[b2], dObj[b3]);
                                    printf("rob %f %f\n", lidarData->x_robot, lidarData->y_robot);
                                }
                            }
                            //TODO check ces conditions
                            double precision = precisionPredef;
                            if (B1 && B2 && B3 && (i == 0)) {
                                precision *= 2;
                            }
                            if (lidarData->readLidar_lost) {
                                precision *= 2;
                            }
                            //TODO CHECK tableX tableY
                            /*printf("precision : %f\n",precision);
                            printf(" old : %f %f \n", oldXRobot, oldYRobot);
                            printf("table x : %f %f\n", minTableX, tableX);
                            printf("table Y : %f %f \n", minTableY, tableY);
                            printf("estimation : %f %f\n\n", lidarData->x_robot, lidarData->y_robot);
                            printf("%d fullscan\n", fullScan);
                            printf("%d %d %d %d %d %d\n", (lidarData->x_robot > minTableX) , (lidarData->x_robot < tableX), (lidarData->y_robot > minTableY) ,(lidarData->y_robot < tableY),(std::abs(lidarData->x_robot - oldXRobot) < precision), std::abs(lidarData->y_robot - oldYRobot) < precision);
                            printf("%d \n\n\n", std::abs(lidarData->orientation_robot - oldOrientationRobot) < 60.0/180.0*M_PI);

                            printf("lidarData->x_robot > minTableX = %d\n", lidarData->x_robot > minTableX); 
                            printf("lidarData->x_robot < tableX = %d\n", lidarData->x_robot < tableX); 
                            printf("lidarData->y_robot > minTableY = %d\n", lidarData->y_robot > minTableY);
                            printf("lidarData->y_robot < tableY = %d\n", lidarData->y_robot < tableY); 
                            printf("fullScan = %d\n", fullScan);
                            
                            printf("std::abs(lidarData->x_robot - oldXRobot) < precision = %d\n", std::abs(lidarData->x_robot - oldXRobot) < precision);
                            printf("std::abs(lidarData->y_robot - oldYRobot) < precision = %d\n", std::abs(lidarData->y_robot - oldYRobot) < precision);
                            printf("lidarData-orientation_robot %f\n", lidarData->orientation_robot);
                            printf("OldOrientationRobot %f\n", oldOrientationRobot);
                            printf("std::abs(lidarData->orientation_robot - oldOrientationRobot) < 90.0/180.0*M_PI) = %d\n", fabs(moduloLidarMPIPI(lidarData->orientation_robot - oldOrientationRobot)) < 60.0/180.0*M_PI);
                            printf("balises d= %f a=%f\nd= %f a= %f \n d= %f a= %f", dObj[b1], aObj[b1]*180.0/M_PI,dObj[b2], aObj[b2]*180.0/M_PI,dObj[b3], aObj[b3]*180.0/M_PI);
                            
                            printf("\n");*/
                            
                            if (
                                (lidarData->x_robot > minTableX) && 
                                (lidarData->x_robot < tableX) && 
                                (lidarData->y_robot > minTableY) &&
                                (lidarData->y_robot < tableY) && 
                                (fullScan || (
                                    (std::abs(lidarData->x_robot - oldXRobot) < precision) &&
                                    (std::abs(lidarData->y_robot - oldYRobot) < precision) &&
                                    (std::abs(moduloLidarMPIPI(lidarData->orientation_robot )- oldOrientationRobot) < precisionAngle))
                                )) 
                                {
                                /// we save the number of elements that could possibly be beacon (opponent)
                                lidarData->countObj_adv = countObj_adv;
                                Adversary(aObj_adv, dObj_adv, lidarData);


                                lidarData->beaconAdv[0] = aObj[b1];
                                lidarData->beaconAdv[1] = dObj[b1];
                                lidarData->beaconAdv[2] = aObj[b2];
                                lidarData->beaconAdv[3] = dObj[b2];
                                lidarData->beaconAdv[4] = aObj[b3];
                                lidarData->beaconAdv[5] = dObj[b3];

                                ///we assume that we can only find 3 points corresponding to our beacons once
                                delete[] (dObj);
                                delete[] (aObj);
                                delete[] (dObj_adv);
                                delete[] (aObj_adv);
                                if (analyseDetail) {
                                    printf("found :)\n");
                                }
                                lidarData->readLidar_lost = false;
                                return;
                            }
                            lidarData->x_robot = oldXRobot;
                            lidarData->y_robot = oldYRobot;
                            lidarData->orientation_robot = oldOrientationRobot;
                                                        
                        }
                    }
                }
            }
        }
    }

    //We haven't found our position, by default the coordinates remain in 0
    //we check the adv with localisation of precedent 
    lidarData->transfo_a = lidarData->old_transfo_a;
    lidarData->transfo_x = lidarData->old_transfo_x;
    lidarData->transfo_y = lidarData->old_transfo_y;

    foundAdvWithOdo(aObj_adv, dObj_adv, lidarData);
    delete[] (dObj);
    delete[] (aObj);
    delete[] (dObj_adv);
    delete[] (aObj_adv);
    if (analyseDetail) {
        printf("not found :( %f %f\n", lidarData->x_robot, lidarData->y_robot);
    }
    lidarData->x_robot = oldXRobot;
    lidarData->y_robot = oldYRobot;
    lidarData->orientation_robot = oldOrientationRobot;
    lidarData->readLidar_lost = true;
    return;
}

void lidarGetRobotPosition(LidarData *lidarData, int i, bool fullScan, bool fromOdo) {
        if(premierScan){
        lidarData->x_odo = 2.0-0.9;
        lidarData->y_odo = 3.0-2.9;
        lidarData->theta_odo = moduloLidarMPIPI(-120.0/180.0*M_PI-M_PI);
    }//DOELELELlelellelelel    lidarData->readLidar_lost = false;
    lidarData->old_transfo_x = lidarData->x_adv;
    lidarData->old_transfo_y = lidarData->y_adv;
    facteurLost = 1;
    double *angles = new double[8000];
    double *distances = new double[8000];
    double *quality = new double[8000];
    size_t *as = new size_t[2]{8000, 8000};
    //fullScan=true;
    //TODO TEEEEEEEEEEEEEEEEEEEEEEEST AAAAAAAAAAAAAAH   
    //fullScanPcqLost = true;
    //if (shared.color == TeamBlue) {
    if (false){
        lidarData->orientation_robot = lidarData->theta_odo;
        lidarData->x_robot = lidarData->x_odo + 0.1 * cos(lidarData->orientation_robot) + deltaXB3;
        lidarData->y_robot = lidarData->y_odo + 0.1 * sin(lidarData->orientation_robot) + deltaYB3+0.2;//TODO PAULINE
    } else {
        // printf("set pos %f\n", 2-lidarData->x_robot);
        lidarData->orientation_robot = -M_PI+lidarData->theta_odo;
        lidarData->x_robot = 2 - (lidarData->x_odo + 0.1 * cos(lidarData->orientation_robot) + deltaXB3);
        lidarData->y_robot = 3 - (lidarData->y_odo + 0.1 * sin(lidarData->orientation_robot) + deltaYB3);

    }

    //TODO TEEEEEEEEEEEEEEEEEEEEEEEST AAAAAAAAAAAAAAH   

    updateDataTop(angles, distances, quality, as);
    //updateDataFile(angles, distances, quality, "DataTest/DataP/testLidarMobile/" + std::to_string(i), as);
    arraySize = as[0];
    if (fromOdo) {
        facteurLost = 10;
        //xyToBeacon(lidarData);
        fullScanPcqLost = false;
        //fullScan = true;
        /*printf("beacon : ");
        for (int j = 0; j < 8; j += 2) {
            printf("%f %f ", lidarData->beaconAdv[j] * 180.0 / M_PI, lidarData->beaconAdv[j + 1]);
        }
        printf("\n");*/
    }
    if (analyseDetail) {
        printf("size : %ld\n", as[0]);
    }
    if (i == 0) {
        fullScan = true;
    }
    // printf("%f %f %f %f %d", angles[0],);
    checkBeacon(angles, distances, quality, lidarData, fullScan);


    delete[] (angles);
    delete[] (distances);
    delete[] (quality);
    delete[] (as);

    if (!lidarData->readLidar_lost) {
        //2 des positions pour être centré au niveau des roues sauf la distance et l'angle de l'adversaire
        //if (shared.color == TeamBlue) {
        if (false){
            lidarData->readLidar_x_robot = lidarData->x_robot - 0.1 * cos(lidarData->orientation_robot) + deltaXB3;
            lidarData->readLidar_y_robot = lidarData->y_robot - 0.1 * sin(lidarData->orientation_robot) + deltaYB3;
            lidarData->readLidar_theta_robot = moduloLidarMPIPI(lidarData->orientation_robot);
            lidarData->readLidar_x_opponent = lidarData->x_adv + deltaXB3;
            lidarData->readLidar_y_opponent = lidarData->y_adv + deltaYB3;;
            lidarData->readLidar_d_opponent = lidarData->d_adv;
            lidarData->readLidar_a_opponent = moduloLidarMPIPI(-lidarData->a_adv);
        } else {

            lidarData->readLidar_x_robot =
                    2 - (lidarData->x_robot - 0.1 * cos(lidarData->orientation_robot) + deltaXB3);
            lidarData->readLidar_y_robot =
                    3 - (lidarData->y_robot - 0.1 * sin(lidarData->orientation_robot) + deltaYB3);
            lidarData->readLidar_theta_robot = moduloLidarMPIPI(lidarData->orientation_robot + M_PI);
            lidarData->readLidar_x_opponent = 2 - (lidarData->x_adv + deltaXB3);
            lidarData->readLidar_y_opponent = 3 - (lidarData->y_adv + deltaYB3);
            lidarData->readLidar_d_opponent = lidarData->d_adv;
            lidarData->readLidar_a_opponent = lidarData->a_adv;
        }


        lidarData->old_transfo_x = lidarData->transfo_x;
        lidarData->old_transfo_y = lidarData->transfo_y;
        lidarData->old_transfo_a = lidarData->transfo_a;

    }
    else{
        //si le robot est perdu : 2 possibilités :
        //      -soit il ne sait pas du tout repérer l'adversaire (d_adv mis a 400m par défaut)
        //      -soit il se repère grace aux odo et reconnait l'adversaire qd même → dans ce cas-là on rentre dans la condition ci-dessous
        if (lidarData->d_adv<100){

            lidarData->readLidar_d_opponent = lidarData->d_adv;
            lidarData->readLidar_a_opponent = lidarData->a_adv;
            //if (shared.color == TeamBlue) {
            if (false){
                lidarData->readLidar_x_opponent = lidarData->x_adv + deltaXB3;
                lidarData->readLidar_y_opponent = lidarData->y_adv + deltaYB3;;
            }
            else{
                lidarData->readLidar_x_opponent = 2 - (lidarData->x_adv + deltaXB3);
                lidarData->readLidar_y_opponent = 3 - (lidarData->y_adv + deltaYB3);
            }
        }
    }
    //if(lidarData->readLidar_x_robot<0.0||lidarData->readLidar_x_robot>2.08||lidarData->readLidar_y_robot<0.0||lidarData->readLidar_y_robot>3.1){
        //lidarData->readLidar_lost=true;
    //}
    fullScanPcqLost = false;    
    premierScan=false;
    /*for (size_t i = 0; i < 6; i+=2)
    {
        printf("%f %f ",lidarData->beaconAdv[i],lidarData->beaconAdv[i]*180/M_PI  );

    }*/
    
      
}

void init_lidar(LidarData *lidarData) {
    lidarData->readLidar_x_robot = 0.0;
    lidarData->readLidar_y_robot = 0.0;
    lidarData->readLidar_theta_robot = 0.0;
    lidarData->readLidar_x_opponent = 400.0;
    lidarData->readLidar_y_opponent = 0.0;
    lidarData->readLidar_d_opponent = 400.0;
    lidarData->readLidar_a_opponent = 0.0;
    lidarData->readLidar_lost = true;


    lidarData->transfo_a = 0;
    lidarData->transfo_x = 0;
    lidarData->transfo_y = 0;

    lidarData->countObj_adv = 0;

    lidarData->x_robot = -0.7 + 0.95;
    lidarData->y_robot = -1.35 + 1.594;
    lidarData->orientation_robot = -M_PI / 2;

    lidarData->x_adv = 400;
    lidarData->y_adv = 0;
    lidarData->d_adv = 400;
    lidarData->a_adv = 0;

    lidarData->beaconAdv = new double[8]{0, 0, 0, 0, 0, 0, 0, 0};

    lidarData->x_odo = 0.0;
    lidarData->y_odo = 0.0;
    lidarData->theta_odo = 0.0;

    lidarData->old_x_adv = 0.7;
    lidarData->old_y_adv = 0.7;
    return;
}

void clear_lidar(LidarData *lidarData) {
    delete[] (lidarData->beaconAdv);
    delete[] (lidarData);
}