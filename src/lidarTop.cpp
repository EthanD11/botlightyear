#include "lidarTop.h"
#include <chrono>


///global variable to find out the size of the file
size_t arraySize = 8000;


///size beacon
double largeurMatAdvers = 0.17;

///useful for print
bool analyseDetail = false;
bool analyseDetail_objet = false;
bool analyseRotationBalise = false;


///tolerance on the edge of the table
double tolTable = 0.05;

///table size
double tableX = 2 + tolTable;
double tableY = 3 + tolTable;
double minTableX = - tolTable;
double minTableY = - tolTable;

///tolerance to avoid mistaking beacons for opponents
double tolBalise = 0.1;


/**
 * if we don't found adversary from previous data we check everywhere
 * @param angles array with angle in radian from lidar
 * @param distances array with distances in m from lidar
 * @param lidarData structure with previous data and where we save current data
 */

int foundAdvWithOdo(double *anglesAdv, double *distancesAdv, LidarData *lidarData) {
    //recalculates transformations to find the opponent
    ///transfo contains 4 elem : deltaX, deltaY, angle of rotation, the number of elements in possible opponents (number of elements in *anglesAdv)
    int size = lidarData->countObj_adv;

    ///coordinates in xy after one and two transformations (translations then rotation)
    double xobj;
    double yobj;
	double minDistAuRobot = 5.0;

    for (int i = 0; i < size; ++i) {
        /// transformation identical to that of beacons and robots
        double gamma = lidarData->orientation_robot - anglesAdv[i];
        xobj = lidarData->x_robot + distancesAdv[i] * std::cos(gamma);
        yobj = lidarData->y_robot + distancesAdv[i] * std::sin(gamma);
        if (xobj > minTableX && xobj < tableX) {
            
            ///check whether the y coordinate is valid (on the table)
            ///deplacement de l'adversaire
            if (yobj > minTableY && yobj < tableY) {
                bool premcondition = (yobj < tolBalise) && ((xobj < tolBalise) || (abs(xobj - 1) < tolBalise) || (xobj > 2 - tolBalise)); // 
                bool seccondition = (yobj > 3 - tolBalise) && ((xobj < tolBalise) || (abs(xobj - 1) < tolBalise) || (xobj > 2 - tolBalise));
                bool troicondition = (abs(yobj - 1.5) < 0.5) && xobj < 0.18;
                if (!(premcondition || seccondition || troicondition)) {
                    /// if the object is on the table, it's our opponent,
                    /// we save its coordinates in the new base (xy based on beacon3)
                    /// and the original coordinates (relative to the robot, distance and angle)
                    double dist = hypot((xobj - lidarData->x_robot), (yobj - lidarData->y_robot));
                    if (minDistAuRobot > dist) {
                        lidarData->x_adv = xobj;
                        lidarData->y_adv = yobj;
                        lidarData->d_adv = distancesAdv[i];
                        lidarData->a_adv = anglesAdv[i];
                        minDistAuRobot = dist;
                    }
                }
                /// we assume that the lidar can only see one object on the table
                /// and that it is therefore automatically the robot
            }
        }
    }
    /// We haven't found the opponent, by default the coordinates remain in 0
    return 1;
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
    ///par défaut a 3.55 mais peut être diminuer en fonction des distances des balises précédentes
    double distMax = 3.6;

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
    double *dObj_adv = new double[arraySize];
    double *aObj_adv = new double[arraySize];

    /// number of items in list
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
                // delta >3cm : new object -> check if the previous one is a beacon
                if (std::abs(d2 - distances[i]) > 0.03) {
                    //what we detect is a new object

                    ///size of object previously detected
                    size = std::sqrt(d2 * d2 + d1 * d1 - 2 * d2 * d1 * std::cos(a2 - a1));
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
            if (countGap > 5) {
                if (objet) {
                    size = std::sqrt(d2 * d2 + d1 * d1 - 2 * d2 * d1 * std::cos(a2 - a1));
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
    lidarData->countObj_adv = countObj_adv;
    foundAdvWithOdo(aObj_adv, dObj_adv, lidarData);
    return;
}

void lidarGetRobotPosition(LidarData *lidarData, int i, bool fullScan, bool fromOdo) {
    lidarData->readLidar_lost = false;
    lidarData->old_transfo_x = lidarData->x_adv;
    lidarData->old_transfo_y = lidarData->y_adv;
    double *angles = new double[8000];
    double *distances = new double[8000];
    double *quality = new double[8000];
    size_t *as = new size_t[2]{8000, 8000};

	lidarData->orientation_robot = lidarData->theta_odo;
	lidarData->x_robot = lidarData->x_odo + 0.1 * cos(lidarData->orientation_robot);
	lidarData->y_robot = lidarData->y_odo + 0.1 * sin(lidarData->orientation_robot);

    updateDataTop(angles, distances, quality, as);
    arraySize = as[0];

    if (analyseDetail) {
        printf("size : %ld\n", as[0]);
    }
    if (i == 0) {
        fullScan = true;
    }
    checkBeacon(angles, distances, quality, lidarData, fullScan);


    delete[] (angles);
    delete[] (distances);
    delete[] (quality);
    delete[] (as);
    if (!lidarData->readLidar_lost) {
        //2 of the positions to be centred at wheel level except the distance and angle of the opponent
		///save Data
		lidarData->readLidar_x_robot = lidarData->x_robot - 0.1 * cos(lidarData->orientation_robot);//plus utile
		lidarData->readLidar_y_robot = lidarData->y_robot - 0.1 * sin(lidarData->orientation_robot);//plus utile
		lidarData->readLidar_theta_robot = moduloLidarMPIPI(lidarData->orientation_robot);
		lidarData->readLidar_x_opponent = lidarData->x_adv;
		lidarData->readLidar_y_opponent = lidarData->y_adv;
		lidarData->readLidar_d_opponent = lidarData->d_adv;
		lidarData->readLidar_a_opponent = moduloLidarMPIPI(-lidarData->a_adv);

		///Save old
        lidarData->old_transfo_x = lidarData->transfo_x;
        lidarData->old_transfo_y = lidarData->transfo_y;
        lidarData->old_transfo_a = lidarData->transfo_a;

    } else {
        //if the robot is lost: 2 possibilities:
        // -either it can't spot the opponent at all (d_adv set to 400m by default)
        // -or it can find its way thanks to the odo and recognises the opponent anyway → in which case the condition below applies.
        if (lidarData->d_adv < 100) {
			//normally never enters here
            lidarData->readLidar_d_opponent = lidarData->d_adv;
            lidarData->readLidar_a_opponent = lidarData->a_adv;
			lidarData->readLidar_x_opponent = lidarData->x_adv;
			lidarData->readLidar_y_opponent = lidarData->y_adv;

        }
    }
    lidarData->readLidar_lost=true;

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

    lidarData->x_robot = 0.0;
    lidarData->y_robot = 0.0;
    lidarData->orientation_robot = -M_PI / 2;

    lidarData->x_adv = 400;
    lidarData->y_adv = 0;
    lidarData->d_adv = 400;
    lidarData->a_adv = 0;

    lidarData->beaconAdv = new double[8]{0, 0, 0, 0, 0, 0, 0, 0};

    lidarData->x_odo = 0.0;
    lidarData->y_odo = 0.0;
    lidarData->theta_odo = 0.0;

    lidarData->old_x_adv = 0.9;
    lidarData->old_y_adv = 0.9;
    return;
}

void clear_lidar(LidarData *lidarData) {
    delete[] (lidarData->beaconAdv);
    delete[] (lidarData);
}
