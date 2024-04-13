#include "action_displacement.h" 

extern SharedVariables shared; 
extern decision_t decision; 

Teensy teensy   = shared.teensy;
Graph graph     = shared.graph; 

#define vref = 0.25                 // [m/s] Speed reference for path following
#define dist_goal_reached = 0.25    // [m] Distance tolerance to goal for path following

void displacement_action() {
    path_following_to_action();
}

uint8_t path_following_to_action() {

    // Set path following from path planning (decision)
    int ncheckpoints = (int) decision.path->nNodes; 
    double x[ncheckpoints] = decision.path->x; 
    double y[ncheckpoints] = decision.path->y; 

    double theta_start = decision.path->theta_start; 
    double theta_end = decision.path->theta_end; 

    teensy.path_following(x, y, ncheckpoints, theta_start, theta_end, vref, dist_goal_reached);

    // Check Teensy mode
    while ((teensy.ask_mode()) != ModePositionControlOver) { 
            uSleep(1000);
    } 

    // Idle
    teensy.idle(); 

    return 0; 

}