#include "lidarTop.h"
#include <chrono>


int main(int argc, char *argv[]) {
    //double *beaconAdv = new double[8]{11.4*M_PI/180, 2.88, 78/180*M_PI, 0.68, 104*M_PI/180, 1.63, 221*M_PI/180, 0.45};
    //StartLidarTop();
    auto started = std::chrono::high_resolution_clock::now();
    LidarData *lidarData = new LidarData[sizeof(LidarData)];
    init_lidar(lidarData);


    //test from xy robot
    lidarData->x_odo = 0.2;
    lidarData->y_odo = 1.8;
    lidarData->theta_odo = -0.1;

    lidarGetRobotPosition(lidarData, 2, false, false);
    // DataToFileTop("jsp1.txt");

    printf(" robot at x=%f; y=%f; orientation=%f\n", lidarData->readLidar_x_robot, lidarData->readLidar_y_robot,
           lidarData->readLidar_theta_robot*180/M_PI);
    printf("Adversary at d=%f; a=%f x=%f y=%f\n", lidarData->readLidar_d_opponent, lidarData->readLidar_a_opponent, lidarData->readLidar_x_opponent, lidarData->readLidar_y_opponent);

    for (int i = 0; i < 0; ++i) {
        lidarGetRobotPosition(lidarData, i);
        printf("\nboucle %d\n", i);
        printf(" robot at x=%f; y=%f; orientation=%f\n", lidarData->readLidar_x_robot, lidarData->readLidar_y_robot,
               lidarData->readLidar_theta_robot);
        printf("Adversary at d=%f; a=%f\n", lidarData->readLidar_d_opponent, lidarData->readLidar_a_opponent);
    }

    sleep(0);
    //StopLidarTop();
    printf("\n");
    clear_lidar(lidarData);


    auto done = std::chrono::high_resolution_clock::now();
    std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(done - started).count() << "\n";

    return 0;
}