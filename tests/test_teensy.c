#include "SPI_Modules.h"
#include <stdio.h>

//#define SPEED_CONTROL
// #define POSITION_CONTROL
#define PATH_FOLLOWING
//#define IDLE

const double deg_to_rads = 3.141593/180;

int main(int argc, char const *argv[])
{
    init_spi(); 

    #ifdef SPEED_CONTROL
    double speed_left, speed_right; 
    speed_left = 0.01; 
    speed_right = 0.01; 
    teensy_spd_ctrl(speed_left, speed_right); 
    lguSleep(10);
    teensy_idle();
    #endif

    #ifdef POSITION_CONTROL
    double x = 1; 
    double y = 0; 
    double t = 45*deg_to_rads; 
    double xr = 0; 
    double yr = 0; 
    double tr = 0*deg_to_rads;

    teensy_pos_ctrl(x, y, t, xr, yr, tr);
    #endif

    #ifdef PATH_FOLLOWING
    int ncheckpoints = 4;
    double x[4] = {1,2,3,4};
    double y[4] = {4,3,2,1};
    double theta = 0.0;

    teensy_path_following(x, y, ncheckpoints, theta);
    #endif

    #ifdef IDLE
    teensy_idle();
    #endif

    close_spi();
    return 0;
}
