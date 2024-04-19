#include "lidarBottom.h"
#include <chrono>

bool testpython = false;

int main(int argc, char *argv[]) {
    //double *beaconAdv = new double[8]{11.4*M_PI/180, 2.88, 78/180*M_PI, 0.68, 104*M_PI/180, 1.63, 221*M_PI/180, 0.45};
    StartLidarBottom();
    auto started = std::chrono::high_resolution_clock::now();
    int *zone = new int[6]{1, 1, 1, 1, 1, 1};
    PlantZone **plantZonePolar = new PlantZone *[6 * sizeof(PlantZone)];

    initBottomLidar(plantZonePolar);


    int res = getNumberOfPlantInAllZone(0.390,0.382,65.501*M_PI/180.0, zone, plantZonePolar);
    DataToFileBottom("testBottom.txt");
    // int res =0;
    if (res == 1) {
        if (testpython) {
            for (size_t i = 0; i < 6; i++) {
                printf("plt.polar(%f, %f, \".\")\n", plantZonePolar[i]->angle, plantZonePolar[i]->distance);
            }
            for (size_t i = 0; i < 6; i++) {
                printf("%d ", plantZonePolar[i]->numberPlant);
            }
        }
    }


    StopLidarBottom();
    printf("\n");

    auto done = std::chrono::high_resolution_clock::now();
    std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(done - started).count() << "\n";

    return 0;
}