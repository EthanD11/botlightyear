#include "action_displacement.h" 
#include "decision.h"
#include <stdio.h>
#include <cmath>


Teensy *teensy  = shared.teensy;

#define vref 0.25                 // [m/s] Speed reference for path following
#define dist_goal_reached 0.40    // [m] Distance tolerance to goal for path following

void displacement_action() {
    path_following_to_action();
}

uint8_t path_following_to_action() {

    // Set path following from path planning (decision)
    int ncheckpoints = (int) decision.path->nNodes; 
    double *x = decision.path->x; 
    double *y = decision.path->y; 

    double theta_start = decision.path->thetaStart; 
    double theta_end = decision.path->thetaEnd; 

    double kp = 0.8;
    double ka = 2.5;
    double kb = -0.8;
    double kw = 4.0;
    teensy->set_position_controller_gains(kp, ka, kb, kw);

    double kt = 2.0;
    double kn = 0.3; // 0 < kn <= 1
    double kz = 25.0;
    double delta = 20e-3; // delta is in meters
    double sigma = 10;
    double epsilon = M_PI/8; // epsilon is in radians
    double wn = 0.2; // Command filter discrete cutoff frequency
    double kv_en = 0.;
    teensy->set_path_following_gains(kt, kn, kz, sigma, epsilon, kv_en, delta, wn);

    teensy->path_following(x, y, ncheckpoints, theta_start, theta_end, vref, dist_goal_reached);

    // Check Teensy mode
    usleep(100000);
    while ((teensy->ask_mode()) != ModePositionControlOver) {
        usleep(1000);
    } 

    // Idle
    teensy->idle(); 

    return 0; 

}