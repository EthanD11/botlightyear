#include "SPI_Modules.h"
#include <stdio.h>
#include <unistd.h>

//#define TEST_TICKS
#define TEST_POS_TRACK

int main(int argc, char const *argv[])
{
    
    init_spi();
    odo_reset();
    #ifdef TEST_TICKS
    int left, right;
    printf("Place left and right odos between 10 000 and 100 000\n");
    while(1){
        odo_get_tick(&left,&right);
        printf("left : %d right : %d\r\n", left, right);
        lguSleep(5e-3);
        fflush(stdout);
        lguSleep(5e-3);
        
        if (left > 10000 && left < 100000 && right > 10000 && right < 100000) break;

    }
    #endif

    #ifdef TEST_POS_TRACK
    double x = 0, y = 0, theta = 0;
    odo_set_pos(x, y, theta);
    while (1)
    {
        odo_get_pos(&x,&y,&theta);
        printf("%.3f,%.3f,%.3f\n", x, y, theta*180/3.1416);
        lguSleep(0.05);
    }
    #endif

    return 0;

}
