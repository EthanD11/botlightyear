#include "lidar.h"


int main(int argc, char const *argv[]) {
    StartLidar();
    for (size_t i = 119; i < 119; i++) {
        //sleep(10);
        DataToFile("testBottom" + std::to_string(i) + ".txt");
        printf("turn\n");
    }

    StopLidar();
    return 0;
}
