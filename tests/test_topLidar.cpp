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
    double *beaconAdv = new double[8]{0.155987, 1.828000, 1.858849, 0.293500, 4.988889, 3.038000, 0.000000, 0.000000};
    double *beaconAdvold = new double[8]{0.155987, 1.828000, 1.858849, 0.293500, 4.988889, 3.038000, 0.000000, 0.000000};
    double *robotOld = new double[2]{0,0};
    StartLidar();
    int counterror = 0;
    int perdu = 0;
    for (size_t i = 0; i < 10; i++){
        for (size_t j = 0; j < 8; j++)
        {
            beaconAdvold[j]=beaconAdv[j];//recup si perdu
        }
        for (size_t k = 0; k < 2; k++)
        {
            robotOld[k]=robot[k];
        }
        

    
    DataToFile("jsp.txt");
    printf("\nboucle : %ld \n", i);
    lidarGetRobotPosition(robot, adv, beaconAdv);
    if (robot[0]<0.0001||robot[0]>2||robot[1]<0.001||robot[1]>3){
        printf("ooooooooooooooooo\n");
        for (size_t l = 0; l < 8; l++)
        {
            beaconAdv[i]=beaconAdvold[i];//recup si perdu
        }
        for (size_t m = 0; m < 2; m++)
        {
            robot[m]= robotOld[m];
        }
        
        perdu++;
    }

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
    printf("perdu : %d\n", perdu);
    return 0;
}