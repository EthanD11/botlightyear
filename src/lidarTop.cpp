//
// Created by pauline on 20/03/24.
//

#include "lidarTop.h"

///global variable to find out the size of the file
size_t arraySize = 3000;

void rotationPosition(double *db, double *x, double *y, double * robot, double* transfo, double* anglesBeacons) {
    ///x,y coordinates of the 4 elements
    double *beacon1;
    double *beacon2;
    double *beacon3;

    /// Save beacon such that :
    /// ---------D1---------
    /// |                  |
    /// |                  |
    /// |                  |
    /// |                  |
    /// |                  |
    /// |                  |
    /// D3----------------D2
    ///(0,0)
    double orientation;

    for (int i = 0; i < 3; ++i) {
        if (std::abs(db[(i+2)%3]-db[(i+1)%3])<0.3){
            beacon1 = new double[2]{x[i],y[i]};
            beacon2 = new double[2]{x[(i+1)%3],y[(i+1)%3]};
            beacon3 = new double[2]{x[(i+2)%3],y[(i+2)%3]};

            ///to determine the orientation of the robot on the table relative to beacon3
            orientation = anglesBeacons[(i+2)%3];
        }
    }


    ///Transformation so that D3 is in 0.0
    ///save transfo
    transfo[0] = beacon3[0];
    transfo[1] = beacon3[1];

    beacon1[1]-=beacon3[1];
    beacon2[1]-=beacon3[1];
    robot[1]-=beacon3[1];
    beacon3[1]-=beacon3[1];

    printf("snif\n");


    beacon1[0]-=beacon3[0];
    beacon2[0]-=beacon3[0];
    robot[0]-=beacon3[0];
    beacon3[0]-=beacon3[0];



    ///calculating the angle of rotation for the second transformation
    double alpha = atan(std::abs(beacon2[1])/std::abs(beacon2[0]));
    if (((beacon2[0]>0) & (beacon2[1]>0))){
        alpha *=-1;
    }
    if (((beacon2[0]<0) & (beacon2[1]>0))){
        alpha=M_PI-alpha;
        alpha*=-1;
    }
    if (((beacon2[0]<0) & (beacon2[1]<0))){
        alpha=M_PI-alpha;
    }

    ///save transfo
    transfo[2] = alpha;

    ///Rotation so that the table is in positive x, y
    double xtemp = beacon2[0];
    double ytemp = beacon2[1];
    beacon2[0] = cos(alpha)*xtemp - sin(alpha)*ytemp;
    beacon2[1] = sin(alpha)*xtemp + cos(alpha)*ytemp;

    xtemp = beacon1[0];
    ytemp = beacon1[1];
    beacon1[0] = cos(alpha)*xtemp - sin(alpha)*ytemp;
    beacon1[1] = sin(alpha)*xtemp + cos(alpha)*ytemp;

    xtemp = robot[0];
    ytemp = robot[1];
    robot[0] = cos(alpha)*xtemp - sin(alpha)*ytemp;
    robot[1] = sin(alpha)*xtemp + cos(alpha)*ytemp;

    ///to determine the orientation of the robot on the table
    //TODO check si ok angle
    robot[2] = orientation - 180 + (atan(robot[1]/robot[0])*180/M_PI);

    while (robot[2]>360){
        robot[2]-=360;
    }
    while (robot[2]<0){
        robot[2]+=360;
    }

    ///angle between 0 and 360° at present but we want between -180 and 180
    robot[2] -= 180;

    ///plus facile pour estimer position
    robot[3]= orientation;
    return;

}

void Adversary(double *anglesAdv, double *distancesAdv, double *transfo, double* adversaryCoordinates){
    ///transfo contains 4 elem : deltax, deltay, angle of rotation, the number of elements in possible opponents (number of elements in *anglesAdv)
    int size = (int) transfo[3];

    ///maximum table dimensions
    //TODO local change because balise pas au bon endroit
    double xmax = 2.0;
    double ymax = 3.0;

    ///coordinates in xy after one and two transformations (translations then rotation)
    double xtemp;
    double ytemp;
    double xobj;
    double yobj;

    for (int i = 0; i < size; ++i) {
        /// transformation identical to that of beacons and robots
        xtemp = (distancesAdv[i] * std::cos(-anglesAdv[i]))-transfo[0];
        ytemp = (distancesAdv[i] * std::sin(-anglesAdv[i]))-transfo[1];

        xobj = (cos(transfo[2])*xtemp - sin(transfo[2])*ytemp);
        printf("  %f %f\n", distancesAdv[i], anglesAdv[i]);
        /// check whether the x coordinate is valid (on the table)
        if (xobj>0.001 && xobj<xmax){
            //printf("%f ", xobj);
            yobj = (sin(transfo[2])*xtemp + cos(transfo[2])*ytemp);
            //printf("%f \n", yobj);

            ///check whether the y coordinate is valid (on the table)
            if (yobj>0.001 && yobj<ymax){

                /// if the object is on the table, it's our opponent,
                /// we save its coordinates in the new base (xy based on beacon3)
                /// and the original coordinates (relative to the robot, distance and angle)
                adversaryCoordinates[0]=xobj;
                adversaryCoordinates[1]=yobj;
                adversaryCoordinates[2]=distancesAdv[i];
                adversaryCoordinates[3]=anglesAdv[i];

                /// we assume that the lidar can only see one object on the table
                /// and that it is therefore automatically the robot
                return;
            }
        }
    }
    /// We haven't found the opponent, by default the coordinates remain in 0
    return;
}

void checkBeacon(double *angles, double *distances, double *quality, double *robot, double* adversaryCoordinates, bool fullScan, double* previousBeaconAdv) {
    robot[0]= 0;
    robot[1]=0;
    ///objet==true : object detected at probable distance
    bool objet;

    ///distance and angle of the first and last points of an object
    double d1 = 0;
    double d2 = 0;
    double a1 = 0;
    double a2 = 0;

    /// size of detected object
    double size;

    //TODO : size too large: to be improved ?
    /// list of objects that are possible beacon
    double *dObj = new double[arraySize];
    double *aObj = new double[arraySize];
    double *dObj_adv = new double[arraySize];
    double *aObj_adv = new double[arraySize];

    /// number of items in list
    int countObj = 0;
    int countObj_adv = 0;
    int oldcountObj = 0;

    ///if we make a full scan we don't know de position of the robot -> analysis of the full data
    ///if we don't make a full scan : we know a estimation of the opponent and the beacons -> 4 smalls for loop
    int nbBoucle = 1;
    size_t* origine = new size_t[4]{0,0,0,0};
    size_t * fin = new size_t [4]{arraySize,0,0,0};
    int* nbObjetParInterval = new int[4]{1,1,1,1};

    double deltaDemiAlpha;
    double angleStart;
    double angleEnd;
    if (!fullScan){

        nbBoucle = 4;
        //TODO si arraysize bien maj
        ///bubble sort : the order of the beacons is important for next
        for (int i = 0; i < 4 - 1; ++i) {
            for (int j = 0; j < 4 - i - 1; ++j) {
                if (previousBeaconAdv[2*j] > previousBeaconAdv[(j + 1)*2]) {
                    double temp = previousBeaconAdv[2*j];
                    previousBeaconAdv[2*j] = previousBeaconAdv[2*(j+1)];
                    previousBeaconAdv[2*(j+1)]=temp;

                    temp = previousBeaconAdv[2*j+1];
                    previousBeaconAdv[2*j+1] = previousBeaconAdv[2*(j+1)+1];
                    previousBeaconAdv[2*(j+1)+1] = temp;
                }
            }
        }


        ///calculating the limits where we will look
        for (int i = 0; i < 4; ++i) {
            deltaDemiAlpha = std::tan(0.05/previousBeaconAdv[2*i+1]);
            angleStart = previousBeaconAdv[2*i]-3*deltaDemiAlpha;
            angleEnd = previousBeaconAdv[2*i]+3*deltaDemiAlpha;
            origine[i] = arraySize*angleStart/(2*M_PI);
            fin[i] = arraySize*angleEnd/(2*M_PI);
            while (fin[i]-origine[i]<25){
                origine[i]-=5;
                fin[i]+=5;
            }
        }

        ///union find
        ///if 2 objects are too close, the two intervals will overlap and there is a risk of seeing the objects twice
        int i = 0;

        while(i<nbBoucle-1){
            if (origine[i+1]<=fin[i]){
                fin[i]=fin[i+1];
                nbObjetParInterval[1]++;
                for (int j = i+1; j < nbBoucle-1; ++j) {
                    fin[j]= fin[j+1];
                    origine[j] = origine[j+1];
                }
                nbBoucle--;
            }else{
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
        oldcountObj = countObj+countObj_adv;

        for (size_t i = origine[k]; i < fin[k]; ++i) {
            /// check if the object is potentially on the table
            if (0.2 < distances[i] && distances[i] < 3.45) {//TODO check max et min possible
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
                    if (std::abs(d2 - distances[i]) > 0.02*countGap && countGap/d2<10) {
                        //what we detect is a new object

                        ///size of object previously detected
                        size = std::sqrt(d2 * d2 + d1 * d1 - 2 * d2 * d1 * std::cos(a2 - a1));

                        if (size < 0.055){// && size > 0.015) {
                            //beacon width = 5cm (estimate smaller than 5.5cm)
                            //TODO : better precision  ??

                            ///it may be a beacon, its data is saved
                            aObj[countObj] = (a1 + a2) / 2;
                            dObj[countObj] = (d1 + d2) / 2;
                            countObj++;
                        }
                        if (size < 0.11){//adv ?
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
                if (size < 0.055 && d1>0 ){
                    aObj[countObj] = (a1 + a2) / 2;
                    dObj[countObj] = (d1 + d2) / 2;
                    countObj++;
                }
                if (size < 0.15&& d1>0){
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
            } else if (distances[i]==0) {
                countGap += 1;
            }
        }


        //TODO check si objet a la fin pour bien l'enregistré
        if (nbObjetParInterval[k]==2 && oldcountObj+3>countObj+countObj_adv){//plus 3 pcq si juste balise : ça ferra +1 ds obj et +1 dans adv => +2
            aObj[countObj]=aObj[countObj-1];
            if (std::abs(dObj[countObj-1]-previousBeaconAdv[2*k+1])<std::abs(dObj[countObj-1]-previousBeaconAdv[2*(k+1)+1])){
                dObj[countObj] = previousBeaconAdv[2*(k+1)+1]-previousBeaconAdv[2*k+1]+dObj[countObj-1];
            }else{
                dObj[countObj] = previousBeaconAdv[2*(k)+1]-previousBeaconAdv[2*(k+1)+1]+dObj[countObj-1];
            }
            countObj++;
        }

    }


    // We now have a list of objects whose size could match that of a beacon,
    // and based on their positions we'll determine which 3 objects are a beacon

    /// distance between 2 beacons on the same side
    double dref1 = 2*0.950;

    /// distance between 2 beacons not on the same side
    double dref2 = sqrt((0.95)*(0.95)+(1.594*2)*(2*1.594));

    ///coordinates and distances of the 3 beacons
    double x1, x2, x3, y1, y2, y3, db1, db2, db3;
    
    ///transformation a appliquer
    double* transfo = new double[4]{0,0,0,0};

    ///b1, b2, b3  : 3 beacons
    for (int b1 = 0; b1 < countObj; ++b1) {
        for (int b2 = b1 + 1; b2 < countObj; ++b2) {
            for (int b3 = b2 + 1; b3 < countObj; ++b3) {
                // TODO MINUS
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
                    printf("dref %f %f \n", dref2, dref1);
                    /// 2 size sides ok (3rd also ok because sum ok)
                    if (((std::abs(db1 - dref1) < 0.15) | (std::abs(db2 - dref1) < 0.15) | (std::abs(db3 - dref1) < 0.15))&&((std::abs(db1 - dref2) < 0.15 )| (std::abs(db2 - dref2) < 0.15) | (std::abs(db3 - dref2) < 0.15))&&((std::abs(db1-db2)<0.3)||(std::abs(db3-db2)<0.3)||(std::abs(db1-db3)<0.3))) {
                        rotationPosition(new double[3]{db1, db2, db3} , new double[3]{x1,x2,x3}, new double[3]{y1,y2,y3}, robot, transfo, new double[3]{aObj[b1],aObj[b2],aObj[b3]});

                        /// we save in blabla the number of elements that could possibly be beacon (opponent)
                        transfo[3] = countObj_adv;
                        Adversary(aObj_adv,dObj_adv,transfo, adversaryCoordinates);


                        previousBeaconAdv[0] = aObj[b1];
                        previousBeaconAdv[1] = dObj[b1];
                        previousBeaconAdv[2] = aObj[b2];
                        previousBeaconAdv[3] = dObj[b2];
                        previousBeaconAdv[4] = aObj[b3];
                        previousBeaconAdv[5] = dObj[b3];
                        previousBeaconAdv[6] = adversaryCoordinates[3];
                        previousBeaconAdv[7] = adversaryCoordinates[2];
                        ///we assume that we can only find 3 points corresponding to our beacons once
                        return;
                    }
                }
            }
        }
    }

    //We haven't found our position, by default the coordinates remain in 0
    return;
}

void lidarGetRobotPosition(double * robot, double* adv, double* beaconAdv) {
    //StartLidar();
    double* angles = new double[3000];
    double* distances = new double[3000];
    double* quality = new double[3000];
    //updateData(angles, distances, quality, 5000);
    updateDataFile(angles, distances, quality, "jsp.txt", new size_t[2]{5000,5000});
    checkBeacon(angles, distances, quality, robot, adv, true, beaconAdv);
    //DataToFile("testBottom1.txt");
    //StopLidar();
}



int main(int argc, char *argv[]) {
    //TODO diff entre les 2 lidars
    // connaitre leur noms
    // communication entre les 2 ?
    // meme orientation ?
    // alignement ?
    // comment det la position ? centre robot, pince, coin, lidar (haut, bas) ?
    double *robot = new double[4]{0, 0, 0, 0};
    double *adv = new double[4]{0, 0, 0, 0};
    double *beaconAdv = new double[8]{0, 0, 0, 0, 0, 0, 0, 0};

    lidarGetRobotPosition(robot, adv, beaconAdv);
    printf("\n robot at x=%f; y=%f; orientation=%f; %f radian beacon3\n", robot[0], robot[1], robot[2], robot[3]);
    printf("Adversary at x=%f; y=%f\n", adv[0], adv[1]);
    printf("adv at %f m; %f degree\n", adv[2], adv[3] * 180 / M_PI);
    for (int i = 0; i < 8; ++i) {
        printf("%f, ", beaconAdv[i]);
    }
    printf("\n");
    return 0;
}