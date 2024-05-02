#ifndef _POSITION_CONTROL_H_
#define _POSITION_CONTROL_H_

#include "../../utils.h"
#include "../localization/localization.h"
#include <stdlib.h>
#include <stdio.h>

#define POSITION_TOL_IN 5e-3 // Tolerance to enter the goal reached state
#define ANGULAR_TOL (2*M_PI/180)
#define ANGULAR_SPEED_TOL 1e-3

typedef struct PositionController {
    
    double xref, yref, theta_ref; // Input position reference
    double speed_refl, speed_refr; // Output speed references

    double kp;  // Proportional coefficient for distance error
    double ka;  // Proportional coefficient for direction error
    double kb;  // Proportional coefficient for orientation error
    double kw; // Propoortional coefficient for orientation error when position is reached
    // double position_tol;     // Acceptable static error on position (m)
    // double drift_tol;   // Acceptable drift from reference position when reorienting (m)
    double angular_tol;     // Acceptable static error on orientation

    int flag_position_reached;
    int flag_angular_position_reached;

    double omega_ref, vref;
} PositionController; 

PositionController *init_position_controller();
// TODO FREE

void control_position(
    PositionController *position_controller,
    RobotPosition *robot_position
);

void set_position_controller_gains(PositionController *position_controlelr,
    double kp, double ka, double kb, double kw);

inline void set_ref(PositionController *position_controller, double xref, double yref, double theta_ref) {
    position_controller->xref = xref;
    position_controller->yref = yref;
    position_controller->theta_ref = theta_ref;
}
inline void set_xref(PositionController *position_controller, double xref) {
    position_controller->xref = xref;
}
inline void set_yref(PositionController *position_controller, double yref) {
    position_controller->yref = yref;
}
inline void set_theta_ref(PositionController *position_controller, double theta_ref) {
    position_controller->theta_ref = theta_ref;
}

inline double get_speed_refl(PositionController *position_controller) {
    return position_controller->speed_refl;
}
inline double get_speed_refr(PositionController *position_controller) {
    return position_controller->speed_refr;
}

#endif