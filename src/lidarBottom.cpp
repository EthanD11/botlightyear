#include "lidarBottom.h"
#include "lidar.h"
#include <cmath>

static double sizePlant = 0.05;
int arraysize = 8000;
bool printplotpy = false;

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

///coordonnées en x,y selon le repère balise b3=(0,0)
Point* zoneP;

double calculateDistance(Point a, Point b) {
    return std::sqrt(std::pow(b.x - a.x, 2) + std::pow(b.y - a.y, 2));
}

double calculateAngle(Point a, Point b) {
    return std::atan2(b.y - a.y, b.x - a.x);
}

void zoneInPolar(Point* robot, PlantZone** polarCoord) {
    for (int i = 0; i < 6; ++i) {
        polarCoord[i]->distance = calculateDistance(*robot, zoneP[i]);
        polarCoord[i]->angle = calculateAngle(*robot, zoneP[i]) - robot->theta;
        ///angle compris entre -PI et PI
        polarCoord[i]->angle = std::fmod((polarCoord[i]->angle +M_PI), (2*M_PI))-M_PI;
    }
    if (printplotpy){
        printf("[%f, %f, %f, %f, %f, %f], ",  polarCoord[0]->angle, polarCoord[1]->angle, polarCoord[2]->angle, polarCoord[3]->angle, polarCoord[4]->angle, polarCoord[5]->angle);
        printf("[%f, %f, %f, %f, %f, %f]\n\n",  polarCoord[0]->distance, polarCoord[1]->distance, polarCoord[2]->distance, polarCoord[3]->distance, polarCoord[4]->distance, polarCoord[5]->distance);
    }

    return;
}

void lidarGetPlantPosition(Point* robot, double* angles, double* distances, double* obstacle, int arraysize, PlantZone** plantZone){
    /*
     * ______________________
     * |                     |
     * |                     |
     * |     2       3       |
     * |                     |
     * |  1             4    |
     * |                     |
     * |      6       5      |
     * |                     |
     * |_____________________|
     */

    //Etape 1 : coordonées des zones
    zoneInPolar(robot, plantZone);
    //a cette etape, plantzone->distance,->angle sont rempli

    //Etape 2 : est-ce face à nous ?
    double deltaAngle;
    for (int i = 0; i < 6; ++i) {
        deltaAngle = atan(0.3/plantZone[i]->distance);//on estime le rayon a 25cm et donc 30 par sécurité
        plantZone[i]->startAngle = plantZone[i]->angle-deltaAngle;
        plantZone[i]->endAngle = plantZone[i]->angle+deltaAngle;

        ///Modulo
        while (plantZone[i]->startAngle<0){
            plantZone[i]->startAngle +=2*M_PI;
        }while (plantZone[i]->startAngle>2*M_PI){
            plantZone[i]->startAngle -=2*M_PI;
        }while (plantZone[i]->endAngle<0){
            plantZone[i]->endAngle +=2*M_PI;
        }while (plantZone[i]->endAngle>2*M_PI){
            plantZone[i]->endAngle -=2*M_PI;
        }
        //end Modulo

        //TODO CHECK THIS
        /*
         * plantZone[i].startAngle= std::fmod((plantZone[i].startAngle +M_PI), (2*M_PI))-M_PI
         * plantZone[i].endAngle= std::fmod((plantZone[i].endAngle +M_PI), (2*M_PI))-M_PI
         */

        if (plantZone[i]->angle<M_PI/4 || plantZone[i]->angle>7*M_PI/4){
            plantZone[i]->isAccessible = true;
        } else{
            plantZone[i]->isAccessible = false;
        }
    }
    /// à cette étape, on sait les angles de depart et de fin des zones ainsi que si la zone est accessible

    //TODO DELETE TEST
    //for (int i = 0; i < 6; ++i) {
        //plantZone[i].startAngle = 355*M_PI/180;
        //plantZone[i].endAngle = 5*M_PI/180;
        //plantZone[i].distance = 1.2;
        //if (i==0){
            //plantZone[i].isAccessible = true;
        //} else{
            //plantZone[i].isAccessible = false;
        //}
    //}
    //int start = (int) 355/360*arraysize;
    //int stop = (int) (360+5)/360*arraysize;
    //double dist = 1.2;

    // int des données utiles
    bool objet = false;
    int start, stop, i;
    int countGap = 0;
    double size, a1, a2, d1, d2;
    for (int zp = 0; zp < 6; ++zp) {
        if (plantZone[zp]->isAccessible){
            //TODO améliorer le start stop car parfois imprecision °
            start =  (int) (plantZone[zp]->startAngle/(2*M_PI)*arraysize);//(int) plantZone[zp].startAngle/(2*M_PI)*arraysize;
            stop =  (int) (plantZone[zp]->endAngle/(2*M_PI)*arraysize);//(int) plantZone[zp].endAngle/(2*M_PI)*arraysize;
            objet = false;
            if (start>stop){
                //utile si plante entre 355 et 5° par exemple
                stop+=arraysize;
            }
            for (int j = start; j < stop; ++j) {
                i = j%arraysize;//utile si plante entre 355 et 5° par exemple
                /// check if the object is potentially on the table
                if (std::abs(distances[i]-plantZone[zp]->distance)<0.3){
                    if (!objet){
                        /// no previous object: a new object to be initialized
                        objet = true;
                        a1 = angles[i];
                        d1 = distances[i];
                    } else{
                        // object present before: if distance small enough it's the same (delta<3cm) -> nothing to do
                        // delta >3cm : new object

                        if (std::abs(d2 - distances[i]) > 0.03) {
                            //what we detect is a new object
                            plantZone[zp]->aPlant[plantZone[zp]->numberPlant]= (a1 + a2) / 2;
                            plantZone[zp]->dPlant[plantZone[zp]->numberPlant]= (d1 + d2) / 2;
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

                } else{
                    countGap ++;
                    if (objet && countGap/d2>10){
                        objet = false;
                        plantZone[zp]->aPlant[plantZone[zp]->numberPlant]= (a1 + a2) / 2;
                        plantZone[zp]->dPlant[plantZone[zp]->numberPlant]= (d1 + d2) / 2;
                        plantZone[zp]->numberPlant++;
                        countGap = 1;
                    }
                }
            }
            if (objet){
                plantZone[zp]->aPlant[plantZone[zp]->numberPlant]= (a1 + a2) / 2;
                plantZone[zp]->dPlant[plantZone[zp]->numberPlant]= (d1 + d2) / 2;
                plantZone[zp]->numberPlant++;
            }
        }
    }
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

int getNumberOfPlantInZone(double x_robot, double y_robot, double theta_robot, int* zone, PlantZone** plantZonePolar){
    double* angles = new double[8000];
    double* distances = new double[8000];
    double* quality = new double[8000];
    double* obj = new double[8000];
    Point *robot = new Point[sizeof(Point)];
    robot->x = x_robot;
    robot->y = y_robot;
    robot->theta = theta_robot;
    size_t *asize = new size_t[2]{8000, 8000};
    updateDataTop(angles, distances, quality, asize);
    arraysize = asize[0];
    lidarGetPlantPosition(robot, angles, distances, obj, asize[0],plantZonePolar );
    return 1;
}

void initBottomLidar(PlantZone** polarCoord){
    zoneP = new Point[6]{{-0.6, 0}, {-0.3, 0.5},{0.3, 0.5}, {0.6, 0}, {0.3, -0.5},{-0.3, -0.5}};

    polarCoord = new PlantZone*[6];
    for (int i=0; i<6;i++){
        polarCoord[i] = new PlantZone[sizeof(PlantZone)];

    }

}