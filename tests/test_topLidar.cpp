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
    double *beaconAdv = new double[8]{1.983773, 0.843000, 4.426254, 1.163000, 6.234721, 2.878000, 0.000000, 0.000000};
    StartLidar();
    int counterror = 0;
    for (size_t i = 0; i < 40; i++)
    {
    DataToFile("jsp.txt");
    printf("save\n");
    printf("\nboucle : %ld \n", i);
    lidarGetRobotPosition(robot, adv, beaconAdv);
    printf("\n robot at x=%f; y=%f; orientation=%f; %f radian beacon3\n", robot[0], robot[1], robot[2], robot[3]);
    printf("Adversary at x=%f; y=%f\n", adv[0], adv[1]);
    printf("adv at %f m; %f degree\n", adv[2], adv[3] * 180 / M_PI);
    printf("beaconAdv: [%f, %f, %f, %f, %f, %f, %f, %f]\n", beaconAdv[0], beaconAdv[1], beaconAdv[2], beaconAdv[3], beaconAdv[4], beaconAdv[5], beaconAdv[6], beaconAdv[7]);
    if (robot[0]<0.0001){
        counterror++;
    }
    }
    

    StopLidar();
    printf("counterror : %d\n", counterror);
    return 0;
}