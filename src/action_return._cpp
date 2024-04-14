#include "action_return.h"
#include "decision.h"

extern SharedVariables shared; 
extern decision_t decision; 


#define vref 0.25                 // [m/s] Speed reference for path following
#define dist_goal_reached 0.25    // [m] Distance tolerance to goal for path following


void path_following_to_base() {

    Teensy teensy   = shared.teensy;
    Graph graph     = shared.graph; 

    // Set path following from path planning (decision)
    int ncheckpoints = (int) decision.path->nNodes; 
    double *x = decision.path->x; 
    double *y = decision.path->y; 

    double theta_start = decision.path->thetaStart; 
    double theta_end = decision.path->thetaEnd; 

    teensy.path_following(x, y, ncheckpoints, theta_start, theta_end, vref, dist_goal_reached);

    // Check Teensy mode
    while ((teensy.ask_mode()) != ModePositionControlOver) { 
            usleep(1000);
    } 

    // Idle
    teensy.idle(); 

    // Score update
    shared.score += 10;

}