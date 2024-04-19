#include "lidarBottom.h"
#include "lidar.h"
#include <cmath>
//TODO ADAPT POUR YELLOW
static double sizePlant = 0.051;
int arraysize = 8000;
bool printplotpy = false;

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
 * @param obstacle : ?
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
    //Etape 1 : coordonées des zones
    zoneInPolar(robot, plantZone);
    //a cette etape, plantzone->distance,->angle sont rempli

    //Etape 2 : est-ce face à nous ?
    double deltaAngle;
    for (int i = 0; i < 6; ++i) {
        deltaAngle = atan(0.3 / plantZone[i]->distance);//on estime le rayon a 25cm et donc 30 par sécurité
        plantZone[i]->startAngle = plantZone[i]->angle - deltaAngle;
        plantZone[i]->endAngle = plantZone[i]->angle + deltaAngle;

        ///Modulo
        plantZone[i]->startAngle = moduloLidarZero2PI(plantZone[i]->startAngle);
        plantZone[i]->endAngle = moduloLidarZero2PI(plantZone[i]->endAngle);

        if (plantZone[i]->angle < M_PI / 4.0 || plantZone[i]->angle > 7.0 * M_PI / 4.0) {
            plantZone[i]->isVisible = true;
        } else {
            plantZone[i]->isVisible = false;
        }
    }
    /// à cette étape, on sait les angles de depart et de fin des zones ainsi que si la zone est accessible

    bool objet = false;
    int start, stop, i;
    int countGap = 0;
    double size, a1, a2, d1, d2;
    for (int zp = 0; zp < 6; ++zp) {
        if (plantZone[zp]->isVisible) {
            start = (int) (plantZone[zp]->startAngle / (2.0 * M_PI) * arraysize);
            //(int) plantZone[zp].startAngle/(2*M_PI)*arraysize;
            stop = (int) (plantZone[zp]->endAngle / (2.0 * M_PI) * arraysize);
            //(int) plantZone[zp].endAngle/(2*M_PI)*arraysize;
            objet = false;
            if (start > stop) {
                //utile si plante entre 355 et 5° par exemple
                stop += arraysize;
            }
            for (int j = start; j < stop; ++j) {
                i = j % arraysize;//utile si plante entre 355 et 5° par exemple
                /// check if the object is potentially on the table
                if (std::abs(distances[i] - plantZone[zp]->distance) < 0.3) {
                    if (!objet) {
                        /// no previous object: a new object to be initialized
                        objet = true;
                        a1 = angles[i];
                        d1 = distances[i];
                    } else {
                        // object present before: if distance small enough it's the same (delta<3cm) -> nothing to do
                        // delta >3cm : new object

                        if (std::abs(d2 - distances[i]) > 0.03) {
                            //what we detect is a new object
                            plantZone[zp]->aPlant[plantZone[zp]->numberPlant] = (a1 + a2) / 2.0;
                            plantZone[zp]->dPlant[plantZone[zp]->numberPlant] = (d1 + d2) / 2.0;
                            double gamma = robot->theta + plantZone[zp]->aPlant[plantZone[zp]->numberPlant];
                            double dist = plantZone[zp]->dPlant[plantZone[zp]->numberPlant];
                            plantZone[zp]->xPlant[plantZone[zp]->numberPlant] = robot->x + dist*std::cos(gamma);
                            plantZone[zp]->yPlant[plantZone[zp]->numberPlant] = robot->y + dist*std::sin(gamma);
                            /*printf("robot x = %f y = %f theta = %f\n", robot->x, robot->y, robot->theta );
                            printf("detect d= %f a=%f\n",plantZone[zp]->dPlant[plantZone[zp]->numberPlant],plantZone[zp]->aPlant[plantZone[zp]->numberPlant] );
                            printf("calcul gamma %f \n", gamma);*/


                            plantZone[zp]->numberPlant++;
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

                } else {
                    countGap++;
                    if (objet && countGap / d2 > 10) {
                        objet = false;
                        plantZone[zp]->aPlant[plantZone[zp]->numberPlant] = (a1 + a2) / 2.0;
                        plantZone[zp]->dPlant[plantZone[zp]->numberPlant] = (d1 + d2) / 2.0;
                        double gamma = robot->theta + plantZone[zp]->aPlant[plantZone[zp]->numberPlant];
                        double dist = plantZone[zp]->dPlant[plantZone[zp]->numberPlant];
                        plantZone[zp]->xPlant[plantZone[zp]->numberPlant] = robot->x + dist*std::cos(gamma);
                        plantZone[zp]->yPlant[plantZone[zp]->numberPlant] = robot->y + dist*std::sin(gamma);
                        /*printf("robot x = %f y = %f theta = %f\n", robot->x, robot->y, robot->theta );
                        printf("detect d= %f a=%f\n",plantZone[zp]->dPlant[plantZone[zp]->numberPlant],plantZone[zp]->aPlant[plantZone[zp]->numberPlant] );
                        printf("calcul gamma %f \n", gamma);*/
                        plantZone[zp]->numberPlant++;
                        countGap = 1;
                    }
                }
            }
            if (objet) {
                plantZone[zp]->aPlant[plantZone[zp]->numberPlant] = (a1 + a2) / 2.0;
                plantZone[zp]->dPlant[plantZone[zp]->numberPlant] = (d1 + d2) / 2.0;
                double gamma = robot->theta + plantZone[zp]->aPlant[plantZone[zp]->numberPlant];
                double dist = plantZone[zp]->dPlant[plantZone[zp]->numberPlant];
                plantZone[zp]->xPlant[plantZone[zp]->numberPlant] = robot->x + dist*std::cos(gamma);
                plantZone[zp]->yPlant[plantZone[zp]->numberPlant] = robot->y + dist*std::sin(gamma);
                /*printf("robot x = %f y = %f theta = %f\n", robot->x, robot->y, robot->theta );
                printf("detect d= %f a=%f\n",plantZone[zp]->dPlant[plantZone[zp]->numberPlant],plantZone[zp]->aPlant[plantZone[zp]->numberPlant] );
                printf("calcul gamma %f \n", gamma);*/
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
    size_t *asize = new size_t[2]{8000, 8000};

    //updateDataBottom(angles, distances, quality, asize);
    updateDataFile(angles, distances, quality, "DataTest/DataP/J-1/testBottom2.txt", asize);
    arraysize = asize[0];
    lidarGetPlantPosition(robot, angles, distances, obj, asize[0], plantZonePolar);

    for (int i = 0; i < 6; ++i) {
        double dmin = 10;
        for (int j = 0; j < plantZonePolar[i]->numberPlant; ++j) {
            if (plantZonePolar[i]->dPlant[j]<dmin){
                plantZonePolar[i]->xClosestPlant = plantZonePolar[i]->xPlant[j];
                plantZonePolar[i]->yClosestPlant = plantZonePolar[i]->yPlant[j];
            }
        }
    }
    /*for (int i = 0; i < 6; ++i) {
        printf("%d \n", plantZonePolar[i]->numberPlant);
        for (int j = 0; j < plantZonePolar[i]->numberPlant; ++j) {
            printf("%f %f\n", plantZonePolar[i]->dPlant[j],plantZonePolar[i]->aPlant[j]);
            printf("%f %f\n", plantZonePolar[i]->xPlant[j],plantZonePolar[i]->yPlant[j]);
        }
    }*/

    delete[] (angles);
    delete[] (distances);
    delete[] (quality);
    delete[] (obj);
    delete[] (robot);
    delete[] (asize);
    printf("fin bottomlidar\n");
    return 1;
}

/**
 *
 * @param polarCoord structure for init (already allocate)
 */
void initBottomLidar(PlantZone **polarCoord) {
    //TODO check si modif ok
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
        polarCoord[i]->aPlant = new double[6];
        polarCoord[i]->dPlant = new double[6];
        polarCoord[i]->xPlant = new double[6];
        polarCoord[i]->yPlant = new double[6];
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


//----------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------

/**
 * is called by the following function to determine the position of the robot relative to the lidarBottom
 * @param angles
 * @param distances
 * @param arraySize
 * @param return_x
 * @param return_y
 * @param return_theta
 */
void lidarGetDistanceWall(double* angles, double* distances,size_t arraySize, double* return_x,double* return_y,double* return_theta) {

    //trouver les deux angles,
    //calcule la distance
    //trouver la position du robot et son orientation
    int pos1 = (int) (60.0 / (360.0) * arraySize);
    int pos2 = (int) (80.0 / (360.0) * arraySize);
    int face = 0;
    double d1, d2, dface, a1,a2, aface;
    while (distances[face]==0){
        face++;
    }
    dface=distances[face];
    aface=angles[face];

    while (distances[pos1]==0){
        pos1++;
    }
    d1=distances[pos1];
    a1=angles[pos1];


    while (distances[pos2]==0){
        pos2++;
    }
    d2=distances[pos2];
    a2=angles[pos2];



    double mur = sqrt(d1*d1+d2*d2-2*d1*d2* cos((a2-a1)));
    double betha = acos((d2*d2-d1*d1-mur*mur)/(-2*d1*mur));
    double xp = d1*sin(betha);
    printf("%f %f\n", betha*180/M_PI,a1*180/M_PI);
    double orientation = M_PI-( a1-betha);
    double x = xp-sin(orientation)*dLidarCentre;
    double y = dface-cos(orientation)*dLidarCentre;
    
    *return_x = 2.0-x;
    *return_y = y;
    while (orientation>M_PI){
        orientation-=2*M_PI;
    }  
    while (orientation<-M_PI){
        orientation+=2*M_PI;
    }
    *return_theta = orientation;
    return;
}


/**
 * To be used when positioning the solar panel, returns the values of x,y and theta in the inputs,
 * bearing in mind that y is not always reliable if there are objects between us and the wall.
 * @param return_x
 * @param return_y
 * @param return_theta
 */
void positionBottomLidarLeftFront(double* return_x,double* return_y,double* return_theta) {

    double *angles = new double[8000];
    double *distances = new double[8000];
    double *quality = new double[8000];

    size_t *asize = new size_t[2]{8000, 8000};
    auto started = std::chrono::high_resolution_clock::now();
    updateDataBottom(angles, distances, quality, asize);
    //updateDataFile(angles, distances, quality, "DataTest/solar240410/panneausolaire.txt", asize);
    arraysize = asize[0];
    lidarGetDistanceWall(angles, distances, asize[0], return_x, return_y, return_theta);
    auto done = std::chrono::high_resolution_clock::now();
    std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(done - started).count() << "\n";

    delete (angles);
    delete (distances);
    delete (quality);
    delete (asize);
    return;
}

void calibrationLidarBottomLeftFront(double* angles, double* distances,size_t arraySize, double* return_x,double* return_y,double* return_theta) {
        //trouver les deux angles,
        //calcule la distance
        //trouver la position du robot et son orientation
        int pos1 = (int) (60.0 / (360.0) * arraySize);
        int pos2 = (int) (80.0 / (360.0) * arraySize);
        int face = 0;
        double d1, d2, dface, a1,a2, aface;
        while (distances[face]==0){
            face++;
        }
        dface=distances[face];
        aface=angles[face];

        while (distances[pos1]==0){
            pos1++;
        }
        d1=distances[pos1];
        a1=angles[pos1];


        while (distances[pos2]==0){
            pos2++;
        }
        d2=distances[pos2];
        a2=angles[pos2];


        double mur = sqrt(d1*d1+d2*d2-2*d1*d2* cos((a2-a1)));
        double betha = acos((d2*d2-d1*d1-mur*mur)/(-2*d1*mur));
        double xp = d1*sin(betha);
        printf("%f %f\n", betha*180/M_PI,a1*180/M_PI);
        double orientation = M_PI-( a1-betha);
        double x = xp-sin(orientation)*dLidarCentre;
        double y = dface-cos(orientation)*dLidarCentre;

        *return_x = 2.0-x;
        *return_y = y;
        *return_theta = orientation;
        return;

}
//TODO faire idem pour les 4 coins et check orientations voulues


void calibrationYellowR1(double* angles, double* distances,size_t arraySize, double* return_x,double* return_y,double* return_theta){
        //trouver les deux angles,
        //calcule la distance
        //trouver la position du robot et son orientation
        double dLidarCentre = 0.055;
        int pos2 = (int) (300.0 / (360.0) * arraySize);
        int pos1 = (int) (280.0 / (360.0) * arraySize);
        int back = (int) (1.0/2.0*arraySize);
        double d1, d2, dback, a1, a2, aback;
        while (distances[back]==0){
            back++;
        }
        dback=distances[back];
        aback=angles[back];

        while (distances[pos1]==0){
            pos1++;
        }
        d1=distances[pos1];
        a1=angles[pos1];


        while (distances[pos2]==0){
            pos2++;
        }
        d2=distances[pos2];
        a2=angles[pos2];


        double mur = sqrt(d1*d1+d2*d2-2*d1*d2* cos((a2-a1)));
        double betha = acos((d2*d2-d1*d1-mur*mur)/(-2*d1*mur));
        double xp = d1*sin(betha);
        printf("%f xp \n",xp);
        double orientation = moduloLidarMPIPI(a1-M_PI/2-betha);
        double x = xp-sin(orientation)*dLidarCentre;
        double y = dback-cos(orientation)*dLidarCentre;

        *return_x = x;
        *return_y = 3.0-y;
        *return_theta = orientation;
        return;
}


void calibrationBottom(double* return_x,double* return_y,double* return_theta){

    double *angles = new double[8000];
    double *distances = new double[8000];
    double *quality = new double[8000];

    size_t *asize = new size_t[2]{8000, 8000};
    auto started = std::chrono::high_resolution_clock::now();
    //updateDataBottom(angles, distances, quality, asize);
    updateDataFile(angles, distances, quality, "DataTest/solar240410/panneausolaire.txt", asize);
    arraysize = asize[0];
    //TODO faire case pour savoir quelle fonction appeler
    calibrationYellowR1(angles, distances, asize[0], return_x, return_y, return_theta);
    auto done = std::chrono::high_resolution_clock::now();
    std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(done - started).count() << "\n";

    delete (angles);
    delete (distances);
    delete (quality);
    delete (asize);
    return;

}
