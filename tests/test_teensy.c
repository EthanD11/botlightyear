#include "SPI_Modules.h"
#include <stdio.h>


int main(int argc, char const *argv[])
{
    init_spi(); 
    //if (Teensy_Handle < 0) return -1;

    double speed_left = 0.5; 
    double speed_right = 0.5; 
    speed_control(speed_left, speed_right); 


    double x = 0.25; 
    double y = 0; 
    double t = 0; 
    double xr = 0.3; 
    double yr = 0; 
    double tr = 0;

    position_control(x, y, t, xr, yr, tr); 

    close_spi();
    return 0;
}
