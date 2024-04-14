#include "lidarBottom.h"
#include <chrono>

bool testpython = false;

int main(int argc, char *argv[]) {
    //double *beaconAdv = new double[8]{11.4*M_PI/180, 2.88, 78/180*M_PI, 0.68, 104*M_PI/180, 1.63, 221*M_PI/180, 0.45};
    StartLidarBottom();
    auto started = std::chrono::high_resolution_clock::now();

    double x, y, theta;

    positionBottomLidarLeftFront(&x,&y,&theta);
    printf("%f %f %f\n", x, y, theta);

    StopLidarBottom();
    printf("\n");

    auto done = std::chrono::high_resolution_clock::now();
    std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(done - started).count() << "\n";

    return 0;
}