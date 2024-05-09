#include "teensy.h"
#include "odometry.h"
#include "SPI_bus.h"
#include "servos.h"
#include "steppers.h"

#include <stdio.h>
#include <unistd.h>
#include <cmath>
#include <lgpio.h>

// #define POSITION_CONTROL
// #define PATH_FOLLOWING
// #define IDLE
// #define SET_POSITION
// #define SPEED_CONTROL
#define DC_CONTROL
// #define ASK_STATE
// #define SET_POS_CTRL_GAINS
// #define SET_PATH_FOLLOWER_GAINS

#ifdef DC_CONTROL
    #define DC_LEFT 60
    #define DC_RIGHT -56
#endif
// Calib adz left: 90
// Calib adz left: 86
SPIBus spiBus = SPIBus();
GPIOPins pins = GPIOPins(); 
Teensy teensy = Teensy(&spiBus, &pins);
Flaps servoFlaps = Flaps(&spiBus);
Odometry odo = Odometry(&spiBus);

const double deg_to_rads = M_PI/180;

int main(int argc, char const *argv[])
{
    usleep(500000);

    #ifdef POSITION_CONTROL
    double kp = 0.5;
    double ka = 3.0;
    double kb = -1.0;
    double kw = 5.0;
    teensy.set_position_controller_gains(kp, ka, kb, kw);

    double x = 0; 
    double y = 0; 
    double t = 0;
    double xr = 0.8; 
    double yr = 0.0; 
    double tr = 0*deg_to_rads;
    double xpos = 0, ypos = 0, thetapos = 0;

    teensy.set_position(x, y, t);
    odo.set_pos(x, y, t);
    lguSleep(0.1);
    teensy.pos_ctrl(xr, yr, tr);

    lguSleep(0.1);
    while (true) {
        odo.get_pos(&xpos, &ypos, &thetapos);
        teensy.set_position(xpos, ypos, thetapos);
        lguSleep(0.5);
    }
    #endif

    #ifdef PATH_FOLLOWING
    double kp = 0.8;
    double ka = 8.0;
    double kb = -2.0;
    double kw = 8.0;
    teensy.set_position_controller_gains(kp, ka, kb, kw);

    double kt = 0.005;
    double kn = 0.7; // 0 < kn <= 1
    double kz = 20.0;
    double delta = 80e-3; // delta is in meters
    double sigma = 2.;
    double epsilon = M_PI/8; // epsilon is in radians
    double wn = 0.2; // Command filter discrete cutoff frequency
    double kv_en = 0.;
    teensy.set_path_following_gains(kt, kn, kz, sigma, epsilon, kv_en, delta, wn);
    lguSleep(0.1);
    int ncheckpoints = 5;
    double x[5] = {0.1,0.5,1.0,0.5,0.1};
    double y[5] = {1.5,1.3,1.5,1.8,1.5};
    // double x[5] = {0.1,1.2};
    // double y[5] = {1.5,1.5};
    double theta_start = 0.;
    double theta_end = M_PI;
    double vref = 0.25;
    double dist_goal_reached = 0.4;

    double xpos = 0, ypos = 0, thetapos = 0;
    odo.set_pos(x[0], y[0], 0);
    lguSleep(0.5);
    teensy.set_position(x[0], y[0], 0);
    lguSleep(1);

    teensy.pos_ctrl(x[0], y[0], atan2(y[1]-y[0], x[1]-x[0]));
    lguSleep(2);

    teensy.path_following(x, y, ncheckpoints, theta_start, theta_end, vref, dist_goal_reached);
    lguSleep(0.2);
    while (teensy.ask_mode() != ModePositionControlOver) {
        odo.get_pos(&xpos, &ypos, &thetapos);
        teensy.set_position(xpos, ypos, thetapos);
        lguSleep(0.4);
    }
    #endif

    #ifdef SET_POSITION
    double x = 0;
    double y = 0;
    double theta = 0;
    teensy.set_position(x, y, theta);
    #endif

    #ifdef IDLE
    teensy.idle();

    double xpos = 0, ypos = 0, thetapos = 0;
    teensy.set_position(1.6, 3.651e-3, M_PI/2);
    odo.set_pos(1.6, 3.651e-3, M_PI/2);
    while (true) {
        odo.get_pos(&xpos, &ypos, &thetapos);
        lguSleep(0.1);
        teensy.set_position(xpos, ypos, thetapos);
    }
    #endif

    #ifdef SPEED_CONTROL
    teensy.spd_ctrl(0.3, 0.3);
    #endif

    #ifdef DC_CONTROL
    teensy.set_position(0, 0, 0);
    servoFlaps.raise();
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
    teensy.ask_mode();
    printf("%d\n", teensy.ask_mode());
    #endif

    return 0;
}
