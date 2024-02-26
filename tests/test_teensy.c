#include "SPI_Modules.h"
#include <stdio.h>

//#define SPEED_CONTROL
#define POSITION_CONTROL

const double deg_to_rads = 3.141593/180;

int main(int argc, char const *argv[])
{
    init_spi(); 

    #ifdef SPEED_CONTROL
    double speed_left, speed_right; 

    
    speed_left = 0; 
    speed_right = 0; 
    teensy_spd_ctrl(speed_left, speed_right); 
    lguSleep(2);

    speed_left = 0; 
    speed_right = 0; 
    teensy_spd_ctrl(speed_left, speed_right); 
 

    #endif

    #ifdef POSITION_CONTROL
    double x = 0; 
    double y = 0; 
    double t = 0*deg_to_rads; 
    double xr = 0.4; 
    double yr = 0.4; 
    double tr = 0*deg_to_rads;

    teensy_pos_ctrl(x, y, t, xr, yr, tr);
    close_spi();
    return 0;
    lguSleep(6);
    x = 0; 
    y = 0; 
    t = 0*deg_to_rads; 
    xr = 1; 
    yr = 0; 
    tr = 0*deg_to_rads;

    teensy_pos_ctrl(x, y, t, xr, yr, tr);
    #endif

    close_spi();
    return 0;
}
