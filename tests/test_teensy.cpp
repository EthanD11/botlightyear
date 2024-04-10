#include "teensy.h"
#include <stdio.h>
#include <unistd.h>
#include <cmath>

// #define POSITION_CONTROL
// #define PATH_FOLLOWING
// #define IDLE
// #define SET_POSITION
#define SPEED_CONTROL
// #define DC_CONTROL
#define ASK_STATE
// #define SET_POS_CTRL_GAINS
// #define SET_PATH_FOLLOWER_GAINS

#ifdef DC_CONTROL
    #define DC_LEFT 50
    #define DC_RIGHT 50
#endif

SPIBus spiBus = SPIBus();
Teensy teensy = Teensy(&spiBus);

const double deg_to_rads = M_PI/180;

int main(int argc, char const *argv[])
{
    usleep(500000);

    #ifdef POSITION_CONTROL
    double x = 0; 
    double y = 0; 
    double t = 0;
    double xr = 0.2; 
    double yr = 0; 
    double tr = -60*deg_to_rads;

    teensy.set_position(x, y, t);
    teensy.pos_ctrl(xr, yr, tr);
    #endif

    #ifdef PATH_FOLLOWING
    double kt = 2.0;
    double kn = 0.32; // 0 < kn <= 1
    double kz = 30.0;
    double delta = 15e-3; // delta is in meters
    double sigma = 0.0;
    double epsilon = M_PI/8; // epsilon is in radians
    double wn = 0.25; // Command filter discrete cutoff frequency
    double kv_en = 12;
    teensy.set_path_following_gains(kt, kn, kz, sigma, epsilon, kv_en, delta, wn);
    usleep(100000);
    int ncheckpoints = 2;
    double x[5] = {0.0,0.4,0.8,0.4,0.0};
    double y[5] = {1.5,1.7,1.5,1.3,1.5};
    // double x[5] = {0.0,0.4};
    // double y[5] = {1.5,1.5};
    double theta_start =   0.;
    double theta_end = 0;
    double vref = 0.3;
    double dist_goal_reached = 0.2;
    teensy.set_position(0, 1.5, 0);
    usleep(100000);
    teensy.path_following(x, y, ncheckpoints, theta_start, theta_end, vref, dist_goal_reached);
    #endif

    #ifdef SET_POSITION
    double x = 0;
    double y = 0;
    double theta = 0;
    teensy.set_position(x, y, theta);
    #endif

    #ifdef IDLE
    teensy.idle();
    #endif

    #ifdef SPEED_CONTROL
    teensy.spd_ctrl(0.4, 0.4);
    #endif

    #ifdef DC_CONTROL
    teensy.set_position(0, 0, 0);
    teensy.constant_dc(DC_LEFT,DC_RIGHT);
    #endif

    #ifdef SET_POS_CTRL_GAINS
    double kp = 1.0;
    double ka = 4.0;
    double kb = -0.5;
    double kw = 8.0;
    teensy.set_position_controller_gains(kp, ka, kb, kw);
    #endif

    #ifdef SET_PATH_FOLLOWER_GAINS
    double kt = 2.0;
    double kn = 0.32; // 0 < kn <= 1
    double kz = 30.0;
    double delta = 15e-3; // delta is in meters
    double sigma = 0.0;
    double epsilon = M_PI/8; // epsilon is in radians
    double wn = 0.25; // Command filter discrete cutoff frequency
    double kv_en = 12;
    teensy.set_path_following_gains(kt, kn, kz, sigma, epsilon, kv_en, delta, wn);
    teensy.idle();
    #endif

    #ifdef ASK_STATE
    usleep(100000);
    teensy.ask_mode();
    #endif

    return 0;
}
