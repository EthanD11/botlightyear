#include "teensy.h"
#include "odometry.h"
#include <stdio.h>
#include <unistd.h>
#include <cmath>
#include <lgpio.h>


SPIBus spiBus = SPIBus();
GPIOPins pins = GPIOPins(); 
Teensy teensy = Teensy(&spiBus, &pins);
Odometry odo = Odometry(&spiBus);

const double deg_to_rads = M_PI/180;


int main() {
    usleep(500000);

    double kp = 0.8;
    double ka = 4.0;
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
    int ncheckpoints = 7;
    double x[7] = {1.0,0.4,0.4,1.0,1.6,1.6,1.0};
    double y[7] = {0.4,1.0,2.0,2.6,2.0,1.0,0.4};
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
}