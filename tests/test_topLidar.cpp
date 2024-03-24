#include "lidarTop.h"

int main(int argc, char *argv[]) {
    //TODO diff entre les 2 lidars
    // connaitre leur noms
    // communication entre les 2 ?
    // meme orientation ?
    // alignement ?
    // comment det la position ? centre robot, pince, coin, lidar (haut, bas) ?
    double *robot = new double[4]{0, 0, 0, 0};
    double *adv = new double[4]{0, 0, 0, 0};
    double *beaconAdv = new double[8]{0.198968, 2.880000, 0.000000, 0.680000, 1.815142, 1.630000, 3.857178, 0.450000};
    StartLidar();
    //DataToFile("jsp.txt");
    
    lidarGetRobotPosition(robot, adv, beaconAdv);
    printf("\n robot at x=%f; y=%f; orientation=%f; %f radian beacon3\n", robot[0], robot[1], robot[2], robot[3]);
    printf("Adversary at x=%f; y=%f\n", adv[0], adv[1]);
    printf("adv at %f m; %f degree\n", adv[2], adv[3] * 180 / M_PI);
    printf("beaconAdv: [%f, %f, %f, %f, %f, %f, %f, %f]", beaconAdv[0], beaconAdv[1], beaconAdv[2], beaconAdv[3], beaconAdv[4], beaconAdv[5], beaconAdv[6], beaconAdv[7]);
    for (int i = 0; i < 8; ++i) {
        printf("%f, ", beaconAdv[i]);
    }
    StopLidar();
    printf("\n");
    return 0;
}