#include "../headers/SPI_Modules.h"
#include <stdio.h>

// #define POSITION_CONTROL
// #define PATH_FOLLOWING
// #define IDLE
// #define SET_POSITION
#define ASK_MODE


const double deg_to_rads = 3.141593/180;

int main(int argc, char const *argv[])
{
    init_spi(); 

    #ifdef POSITION_CONTROL
    double xr = 1.22; 
    double yr = 0; 
    double tr = 1.0;

    teensy_pos_ctrl(xr, yr, tr);
    #endif

    #ifdef PATH_FOLLOWING
    int ncheckpoints = 5;
    double x[5] = {0.0,0.4,0.8,0.4,0.0};
    double y[5] = {1.5,1.7,1.5,1.3,1.5};
    double theta_start =    -1.23;
    double theta_end = 2.56;

    teensy_path_following(x, y, ncheckpoints, theta_start, theta_end);
    #endif

    #ifdef SET_POSITION
    double x = 0.123456789;
    double y = 0.987654321;
    double theta = -3.123456789;
    teensy_set_position(x, y, theta);
    #endif

    #ifdef IDLE
    teensy_idle();
    #endif

    #ifdef ASK_MODE
    teensy_ask_mode();
    #endif

    close_spi();
    return 0;
}
