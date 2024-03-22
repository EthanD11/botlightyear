#include "../headers/SPI_Modules.h"
#include <stdio.h>

// #define POSITION_CONTROL
// #define PATH_FOLLOWING
// #define IDLE
// #define SET_POSITION
// #define SPEED_CONTROL
// #define DC_CONTROL
#define ASK_STATE


const double deg_to_rads = 3.141593/180;

int main(int argc, char const *argv[])
{
    init_spi(); 
    lguSleep(1);

    #ifdef POSITION_CONTROL
    double x = 0; 
    double y = 0; 
    double t = 0;
    double xr = 1.22; 
    double yr = 0; 
    double tr = 45*deg_to_rads;

    teensy_set_position(x, y, t);
    teensy_pos_ctrl(xr, yr, tr);
    //teensy_spd_ctrl(0.3,0.3);
    lguSleep(5);
    teensy_idle();
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

    #ifdef SPEED_CONTROL
    teensy_spd_ctrl(0.2, 0.12345);
    #endif

    #ifdef DC_CONTROL
    teensy_constant_dc(-255,255);
    #endif

    #ifdef ASK_STATE
    teensy_ask_mode();
    #endif

    close_spi();
    return 0;
}
