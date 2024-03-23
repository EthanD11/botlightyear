#include "../headers/SPI_Modules.h"
#include <stdio.h>

// #define POSITION_CONTROL
#define PATH_FOLLOWING
// #define IDLE
// #define SET_POSITION
// #define SPEED_CONTROL
// #define DC_CONTROL
// #define ASK_STATE
// #define SET_POS_CTRL_GAINS
// #define SET_PATH_FOLLOWER_GAINS

const double deg_to_rads = 3.141593/180;

int main(int argc, char const *argv[])
{
    init_spi(); 
    lguSleep(1);

    #ifdef POSITION_CONTROL
    double x = 0; 
    double y = 0; 
    double t = 0;
    double xr = 0.5; 
    double yr = 0; 
    double tr = 0*deg_to_rads;

    teensy_set_position(x, y, t);
    teensy_pos_ctrl(xr, yr, tr);
    #endif

    #ifdef PATH_FOLLOWING
    int ncheckpoints = 5;
    double x[5] = {0.0,0.4,0.8,0.4,0.0};
    double y[5] = {1.5,1.7,1.5,1.3,1.5};
    double theta_start =   0.;
    double theta_end = M_PI;
    double vref = 0.4;
    double dist_goal_reached = 0.1;
    teensy_set_position(0, 1.5, 0);
    teensy_path_following(x, y, ncheckpoints, theta_start, theta_end, vref, dist_goal_reached);
    #endif

    #ifdef SET_POSITION
    double x = 0;
    double y = 0;
    double theta = 0;
    teensy_set_position(x, y, theta);
    #endif

    #ifdef IDLE
    teensy_idle();
    #endif

    #ifdef SPEED_CONTROL
    teensy_spd_ctrl(0, 0);
    #endif

    #ifdef DC_CONTROL
    teensy_set_position(0, 0, 0);
    teensy_constant_dc(0,0);
    #endif

    #ifdef SET_POS_CTRL_GAINS
    double kp = 1.0;
    double ka = 4.0;
    double kb = -0.5;
    double kw = 5.6;
    teensy_set_position_controller_gains(kp, ka, kb, kw);
    #endif

    #ifdef SET_PATH_FOLLOWER_GAINS
    double kt = 3.0;
    double kn = 1.0; // 0 < kn <= 1
    double kz = 80.0;
    double delta = 30e-3; // delta is in meters
    double sigma = 1.0;
    double epsilon = 150-3; // epsilon is in meters
    double wn = 0.3; // Command filter discrete cutoff frequency
    double kv_en = 10;
    teensy_set_path_following_gains(kt, kn, kz, delta, sigma, epsilon, wn, kv_en);
    #endif

    #ifdef ASK_STATE
    teensy_ask_mode();
    #endif

    close_spi();
    return 0;
}
