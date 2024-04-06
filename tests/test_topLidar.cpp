#include "lidarTop.h"
#include <chrono>


int main(int argc, char *argv[]) {
    //double *beaconAdv = new double[8]{11.4*M_PI/180, 2.88, 78/180*M_PI, 0.68, 104*M_PI/180, 1.63, 221*M_PI/180, 0.45};
    StartLidar();
    auto started = std::chrono::high_resolution_clock::now();
    LidarData *lidarData = new LidarData[sizeof(LidarData)];
    init_lidar(lidarData);

    for (int i = 0; i < 0; ++i) {
        lidarGetRobotPosition(lidarData, i);
        printf("\nboucle %d\n", i);
        printf(" robot at x=%f; y=%f; orientation=%f\n", lidarData->x_robot, lidarData->y_robot, lidarData->orientation_robot);
        printf("Adversary at d=%f; a=%f\n", lidarData->d_adv, lidarData->a_adv);
    }
    lidarData->x_odo = 0.2;
    lidarData->y_odo = 0.2;
    lidarData->theta_odo = M_PI/2;


    lidarGetRobotPosition(lidarData, 5, false,true);
    printf(" robot at x=%f; y=%f; orientation=%f\n", lidarData->x_robot, lidarData->y_robot, lidarData->orientation_robot);
    printf("Adversary at d=%f; a=%f\n", lidarData->d_adv, lidarData->a_adv);

    StopLidar();
    printf("\n");
    clear_lidar(lidarData);
    delete (lidarData);


    auto done = std::chrono::high_resolution_clock::now();
    std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(done-started).count()<< "\n";
    
    return 0;
}