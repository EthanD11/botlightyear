#include "lidarTop.h"
#include <chrono>

//facteur si mouvent brusque
int facteurLost =1;

bool fullScanPcqLost = false;


///global variable to find out the size of the file
size_t arraySize = 8000;

/// distance between 2 beacons on the same side
double dref1 = 2 * 0.950;

/// distance between 2 beacons not on the same side
double dref2 = sqrt((0.95) * (0.95) + (1.594 * 2) * (2 * 1.594));

/// angle isocele pour transfo inverse
double angleIsocele = acos(dref1/(2*dref2)); 

bool analyseDetail = false;
bool analyseDetail_objet = false;
bool analyseRotationBalise = false;

///precison pour le deplacement effectué-> permet d'etre augmenté si on est perdu
double precisionPredef = 0.07;


///decalage balise et coin de la table (x_reel = x_B3 + delta)
double deltaXB3 = 0.05;
double deltaYB3 = -0.09;

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
    if (analyseRotationBalise){
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
    //TODO check si ok angle
    lidarData->orientation_robot = M_PI - orientation + atan2(lidarData->x_robot, lidarData->y_robot)-M_PI/2;

    while (lidarData->orientation_robot>M_PI){
        lidarData->orientation_robot-=2*M_PI;
    }

    while (lidarData->orientation_robot<-M_PI){
        lidarData->orientation_robot+=2*M_PI;
    }


    delete (beacon1);
    delete (beacon2);
    delete (beacon3);
    return;

}

/**
 * Find the avdersaire and save its position relative to the map and its position relative to the robot in lidarData
 * @param anglesAdv coordinates relative to the robot of possible opponents
 * @param distancesAdv coordinates relative to the robot of possible opponents;
 * @param lidarData structure with useful data
 *
 */
int Adversary(double *anglesAdv, double *distancesAdv, LidarData *lidarData) {
    ///transfo contains 4 elem : deltax, deltay, angle of rotation, the number of elements in possible opponents (number of elements in *anglesAdv)
    int size = lidarData->countObj_adv;

    ///maximum table dimensions
    double xmax = 2.0;
    double ymax = 3.0;

    ///coordinates in xy after one and two transformations (translations then rotation)
    double xtemp;
    double ytemp;
    double xobj;
    double yobj;

    for (int i = 0; i < size; ++i) {
        /// transformation identical to that of beacons and robots
        xtemp = (distancesAdv[i] * std::cos(-anglesAdv[i])) - lidarData->transfo_x;
        ytemp = (distancesAdv[i] * std::sin(-anglesAdv[i])) - lidarData->transfo_y;

        xobj = (cos(lidarData->transfo_a) * xtemp - sin(lidarData->transfo_a) * ytemp);
        /// check whether the x coordinate is valid (on the table)
        if (xobj > 0.001 && xobj < xmax) {
            yobj = (sin(lidarData->transfo_a) * xtemp + cos(lidarData->transfo_a) * ytemp);

            ///check whether the y coordinate is valid (on the table)
            if (yobj > 0.001 && yobj < ymax) {
                /// if the object is on the table, it's our opponent,
                /// we save its coordinates in the new base (xy based on beacon3)
                /// and the original coordinates (relative to the robot, distance and angle)
                lidarData->x_adv = xobj;
                lidarData->y_adv = yobj;
                lidarData->d_adv = distancesAdv[i];
                lidarData->a_adv = anglesAdv[i];
                /// we assume that the lidar can only see one object on the table
                /// and that it is therefore automatically the robot
                return 0;
            }
        }
    }
    /// We haven't found the opponent, by default the coordinates remain in 0
    return 1;
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
        if (0.2 < distances[i] && distances[i] < distMax) {

            /// no previous object: a new object to be initialized
            if (!objet) {
                objet = true;
                a1 = angles[i];
                d1 = distances[i];
            } else {
                // object present before: if distance small enough it's the same (delta<2cm) -> nothing to do
                // delta >2cm : new object -> check if the previous one is a beacon

                //TODO check le count gap max possible : prendre donnée d'un poteau le plus porche possible et voir combien d'elem ca prend
                // lien avec d2 ok ??

                if (std::abs(d2 - distances[i]) > 0.02 * countGap) {//TODO CHECK CE PROB :  && countGap/d2<10) {
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
        }
    }
    /// we save the number of elements that could possibly be beacon (opponent)
    Adversary(aObj_adv, dObj_adv, lidarData);
    ///we assume that we can only find 3 points corresponding to our beacons once
    delete (dObj_adv);
    delete (aObj_adv);
    return;
}

void xyToBeacon(LidarData* lidarData){
    double x = lidarData->x_odo;
    double y = lidarData->y_odo;
    double theta = lidarData->theta_odo;
    
    
    double a1,a2,a3,d1,d2,d3, alpha;
    alpha = atan2(x,y);
    a3 = M_PI - theta+ alpha;
    d3 = std::sqrt(x*x+y*y);
    d2 = std::sqrt(d3*d3+dref1*dref1-2*d3*dref1*cos(alpha));
    d1 = std::sqrt(d3*d3+dref2*dref2-2*d3*dref2*cos(angleIsocele-alpha));
    a2 = a3-M_PI/2+alpha-atan2(dref1-x,y);
    a1 = a3 + acos((d3*d3+d1*d1-dref2*dref2)/(2*d1*d3));

    while(a1<0){
        a1+=2*M_PI;
    }
    while(a1>2*M_PI){
        a1-=2*M_PI;
    } 
    while(a2<0){
        a2+=2*M_PI;
    }
    while(a2>2*M_PI){
        a2-=2*M_PI;
    }  
    while(a3<0){
        a3+=2*M_PI;
    }
    while(a3>2*M_PI){
        a3-=2*M_PI;
    }

    lidarData->beaconAdv[1]=d1;
    lidarData->beaconAdv[3]=d2;
    lidarData->beaconAdv[5]=d3;
    lidarData->beaconAdv[0]=a1;
    lidarData->beaconAdv[2]=a2;
    lidarData->beaconAdv[4]=a3;


    return;
}

/**
 * From the raw lidar data, save in lidarData the coordinates of the robot according to the defined plane and the coordinates of the opponent
 * @param angles : array of size max 8192 with angle in degree
 * @param distances : array of size max 8192 with distances in m
 * @param quality : array of size max 8192 with quality
 * @param lidarData : structure with useful data (previous data) and where we save the new data
 * @param fullScan : if true: performs a full scan with no position prediction,
 *                  if false: performs a more accurate scan, mimicked by an estimate of the position of the beacons and the opponent
 * if (fullScan) we need data in lidarData->beaconAdv : table of 8 with the angles and distances of the 3 beacons and the opponent (a1, d1, a2, d2, a3, d3, a, d)
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
    //TODO reflechir pcq pas ideal pour adversaire
    if (!fullScan){distMax = std::max(std::max(lidarData->beaconAdv[1], lidarData->beaconAdv[3]), lidarData->beaconAdv[5])+0.5;}

    ///objet==true : object detected at probable distance
    bool objet;

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

    ///if we make a full scan we don't know de position of the robot -> analysis of the full data
    ///if we don't make a full scan : we know a estimation of the opponent and the beacons -> 4 smalls for loop
    int nbBoucle = 1;
    int *origine = new int[4]{0, 0, 0, 0};
    int *fin = new int[4]{(int) arraySize, 0, 0, 0};

    double deltaDemiAlpha;
    double angleStart;
    double angleEnd;
    if (!fullScan) {
        nbBoucle = 4;
        ///bubble sort : the order of the beacons is important for next
        for (int i = 0; i < 4 - 1; ++i) {
            for (int j = 0; j < 4 - i - 1; ++j) {
                if (lidarData->beaconAdv[2 * j] > lidarData->beaconAdv[(j + 1) * 2]) {
                    double temp = lidarData->beaconAdv[2 * j];
                    lidarData->beaconAdv[2 * j] = lidarData->beaconAdv[2 * (j + 1)];
                    lidarData->beaconAdv[2 * (j + 1)] = temp;

                    temp = lidarData->beaconAdv[2 * j + 1];
                    lidarData->beaconAdv[2 * j + 1] = lidarData->beaconAdv[2 * (j + 1) + 1];
                    lidarData->beaconAdv[2 * (j + 1) + 1] = temp;
                }
            }
        }

        ///calculating the limits where we will look
        for (int i = 0; i < 4; ++i) {
            deltaDemiAlpha = 5 * std::tan(0.05 / lidarData->beaconAdv[2 * i + 1]);
            angleStart = lidarData->beaconAdv[2 * i] - deltaDemiAlpha*facteurLost;
            angleEnd = lidarData->beaconAdv[2 * i] + deltaDemiAlpha*facteurLost;
            origine[i] = arraySize * angleStart / (2 * M_PI);
            fin[i] = arraySize * angleEnd / (2 * M_PI);
            while (fin[i] - origine[i] < 70) {
                origine[i] -= 10;
                fin[i] += 10;
            }
            if (analyseDetail) {
                printf("origine fin  %d %d \n", origine[i], fin[i]);
            }

        }

        ///union find
        ///if 2 objects are too close, the two intervals will overlap and there is a risk of seeing the objects twice
        int i = 0;
        while (i < nbBoucle - 1) {
            if (origine[i + 1] <= fin[i]) {
                fin[i] = fin[i + 1];
                for (int j = i + 1; j < nbBoucle - 1; ++j) {
                    fin[j] = fin[j + 1];
                    origine[j] = origine[j + 1];
                }
                nbBoucle--;
            } else {
                i++;
            }
        }
    }


    /// count of missing data+1
    /// useful for knowing the difference in maximum distance when a lot of data is lost between 2 elements
    int countGap = 1;
    for (int k = 0; k < nbBoucle; ++k) {
        //if an object is between 0 and 360°, it may be detected twice, but this is not a problem.
        objet = false;
        int i;
        for (int p = origine[k]; p < fin[k]; ++p) {
            i = p;
            if (i < 0) {
                i += arraySize;
            }


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
                    //TODO check le count gap max possible : prendre donnée d'un poteau le plus porche possible et voir combien d'elem ca prend
                    // lien avec d2 ok ??

                    if (std::abs(d2 - distances[i]) > 0.02 * countGap) {//TODO CHECK CE PROB :  && countGap/d2<10) {
                        //what we detect is a new object

                        ///size of object previously detected
                        size = std::sqrt(d2 * d2 + d1 * d1 - 2 * d2 * d1 * std::cos(a2 - a1));
                        //TODO DELETE DELETE DELETE DELETE
                        //if (size < 0.065 && (fullScan || (((std::abs(d1-olddistB1)<0.1)&&(std::abs(a1-oldAngB1)<2.0))|| ((std::abs(d1-olddistB2)<0.1)&&(std::abs(a1-oldAngB2)<2.0))|| ((std::abs(d1-olddistB3)<0.1)&&(std::abs(a1-oldAngB3)<2.0))))){// && size > 0.015) {
                        if (size < 0.065 && (fullScan || (((std::abs(d1 - olddistB1) < 0.2)) ||
                                                          ((std::abs(d1 - olddistB2) < 0.2)) ||
                                                          ((std::abs(d1 - olddistB3) < 0.2))))) {// && size > 0.015) {
                            //beacon width = 5cm (estimate smaller than 6.5cm)
                            ///it may be a beacon, its data is saved
                            aObj[countObj] = (a1 + a2) / 2;
                            dObj[countObj] = (d1 + d2) / 2;
                            countObj++;
                        }
                        if (size < 0.11) {//adv ?
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
                if (size < 0.065 && d1 > 0) {
                    aObj[countObj] = (a1 + a2) / 2;
                    dObj[countObj] = (d1 + d2) / 2;
                    countObj++;
                }
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
            }
        }
        if (analyseDetail) {
            printf("count : %d %d\n", countObj, countObj_adv);
        }
    }


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

    ///recup balise perdue
    int countObjPlusOld = countObj;
    bool B1 = false;
    bool B2 = false;
    bool B3 = false;
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
    //if (!B1&&B2&&B3){
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
    //}
    //if (B1&&!B2&&B3){
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
    //}
    //if (B1&&B2&&!B3){
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

    //}
    ///recup balise perdue fin


    int startb3;
    int stopb3;

    int startb2;
    int stopb2;
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
                    if (std::abs((db1 + db2 + db3) - dref1 - 2 * dref2) < 0.2) {//TODO check precision
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
                            delete (db);
                            delete (yrp);
                            delete (xrp);
                            delete (abeac);
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
                                precision *= 10;
                            }
                            if (lidarData->x_robot > 0 && lidarData->x_robot < 2 && lidarData->y_robot > 0 &&
                                lidarData->y_robot < 3 && ((fullScan&&!fullScanPcqLost) || (
                                    std::abs(lidarData->x_robot - oldXRobot) < precision &&
                                    std::abs(lidarData->y_robot - oldYRobot) < precision))) {

                                /// we save the number of elements that could possibly be beacon (opponent)
                                lidarData->countObj_adv = countObj_adv;
                                int foundAdv = Adversary(aObj_adv, dObj_adv, lidarData);
                                if (foundAdv == 1) {
                                    lidarPerduAdv(angles, distances, lidarData);
                                }

                                lidarData->beaconAdv[0] = aObj[b1];
                                lidarData->beaconAdv[1] = dObj[b1];
                                lidarData->beaconAdv[2] = aObj[b2];
                                lidarData->beaconAdv[3] = dObj[b2];
                                lidarData->beaconAdv[4] = aObj[b3];
                                lidarData->beaconAdv[5] = dObj[b3];
                                lidarData->beaconAdv[6] = lidarData->a_adv;
                                lidarData->beaconAdv[7] = lidarData->d_adv;
                                ///we assume that we can only find 3 points corresponding to our beacons once
                                delete (dObj);
                                delete (aObj);
                                delete (dObj_adv);
                                delete (aObj_adv);
                                delete (origine);
                                delete (fin);
                                if (analyseDetail) {
                                    printf("found :)\n");
                                }
                                lidarData->readLidar_lost=false;
                                return;
                            }
                        }
                    }
                }
            }
        }
    }

    //We haven't found our position, by default the coordinates remain in 0
    delete (dObj);
    delete (aObj);
    delete (dObj_adv);
    delete (aObj_adv);
    delete (origine);
    delete (fin);
    if (analyseDetail) {
        printf("not found :( %f %f\n", lidarData->x_robot, lidarData->y_robot);
    }
    lidarData->x_robot= oldXRobot;
    lidarData->y_robot= oldYRobot;
    lidarData->orientation_robot= oldOrientationRobot;
    lidarData->readLidar_lost=true;
    return;
}

void lidarGetRobotPosition(LidarData *lidarData, int i, bool fullScan, bool fromOdo) {
    double *angles = new double[8000];
    double *distances = new double[8000];
    double *quality = new double[8000];
    size_t *as = new size_t[2]{8000, 8000};
    updateDataTop(angles, distances, quality, as);
    //updateDataFile(angles, distances, quality, "testLidarMobile/" + std::to_string(i), as);
    arraySize = as[0];
    if (fromOdo){
        xyToBeacon(lidarData);
    }
    if (analyseDetail) {
        printf("size : %ld\n", as[0]);
    }
    if(i==0){
        fullScan=true;
    }

    checkBeacon(angles, distances, quality, lidarData, fullScan);
    if (lidarData->readLidar_lost){
        facteurLost=5;
        fullScan=true;
        fullScanPcqLost = true;
        checkBeacon(angles, distances, quality, lidarData, fullScan);
        fullScanPcqLost = false;
        facteurLost=1;
    }

    delete (angles);
    delete (distances);
    delete (quality);
    delete (as);

    if(!lidarData->readLidar_lost){
    //2 des positions pour etre centré au niveau des roues sauf la distance et l'angle de l'adversaire
    lidarData->readLidar_x_robot = lidarData->x_robot-0.1*sin(lidarData->orientation_robot)+deltaXB3;
    lidarData->readLidar_y_robot = lidarData->y_robot-0.1*cos(lidarData->orientation_robot)+deltaYB3;
    lidarData->readLidar_theta_robot = lidarData->orientation_robot;
    lidarData->readLidar_x_opponent = lidarData->x_adv+deltaXB3;
    lidarData->readLidar_y_opponent = lidarData->y_adv+deltaYB3;;
    lidarData->readLidar_d_opponent = lidarData->d_adv;
    lidarData->readLidar_a_opponent = lidarData->a_adv;}
}

void init_lidar(LidarData *lidarData) {
    lidarData->readLidar_x_robot=0.0;
    lidarData->readLidar_y_robot = 0.0; 
    lidarData->readLidar_theta_robot = 0.0;
    lidarData->readLidar_x_opponent = 0.0;
    lidarData->readLidar_y_opponent = 0.0;
    lidarData->readLidar_d_opponent = 0.0;
    lidarData->readLidar_a_opponent = 0.0;
    lidarData->readLidar_lost = true;


    lidarData->transfo_a = 0;
    lidarData->transfo_x = 0;
    lidarData->transfo_y = 0;

    lidarData->countObj_adv = 0;

    lidarData->x_robot = -0.7 + 0.95;
    lidarData->y_robot = -1.35 + 1.594;
    lidarData->orientation_robot = M_PI / 2;

    lidarData->x_adv = 0;
    lidarData->y_adv = 0;
    lidarData->d_adv = 0;
    lidarData->a_adv = 0;

    lidarData->beaconAdv = new double[8]{0, 0, 0, 0, 0, 0, 0, 0};

    lidarData->x_odo = 0.0;
    lidarData->y_odo = 0.0;
    lidarData->theta_odo = 0.0;

    return;
}

void clear_lidar(LidarData *lidarData) {
    delete (lidarData->beaconAdv);
    delete(lidarData);
}