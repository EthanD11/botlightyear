#include "odometry.h"
#include <stdio.h>
#include <unistd.h>
#include <cmath>

//#define TEST_TICKS
#define TEST_POS_TRACK

SPIBus spiBus = SPIBus();
Odometry odo = Odometry(&spiBus);

int main(int argc, char const *argv[])
{
    
    odo.reset();
    #ifdef TEST_TICKS
    int left, right;
    printf("Place left and right odos between 10 000 and 100 000\n");
    while(1){
        odo.get_ticks(&left,&right);
        printf("left : %d right : %d\r\n", left, right);
        usleep(5000);
        fflush(stdout);
        usleep(5000);
        
        if (left > 10000 && left < 100000 && right > 10000 && right < 100000) break;

    }
    #endif

    #ifdef TEST_POS_TRACK
    double x = 0, y = 0, theta = 0;
    odo.set_pos(x, y, theta);
    while (1)
    {
        odo.get_pos(&x,&y,&theta);
        printf("%.3f,%.3f,%.3f\n", x, y, theta*180/3.1416);
        usleep(50000);
    }
    #endif

    return 0;
    // 2.794, -0.075, 2.670, -0.162, 2.647, -0.007, 2.816
}
