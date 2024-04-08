#include "../headers/SPI_Modules.h"
#include <stdio.h>

#define IDLE


const double deg_to_rads = M_PI/180;

int main(int argc, char const *argv[])
{
    init_spi(); 
    lguSleep(0.5);
    teensy_idle();
    close_spi();
    return 0;
}
