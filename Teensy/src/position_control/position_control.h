#ifndef _POSITION_CONTROL_H_
#define _POSITION_CONTROL_H_

#include "../../utils.h"
#include "../localization/localization.h"
#include <stdlib.h>
#include <stdio.h>

typedef struct PositionController {
    
    double xref, yref, theta_ref; // Input position reference
    double speed_refl, speed_refr; // Output speed references

    double kp;  // Proportional coefficient for distance error
    double ka;  // Proportional coefficient for direction error
    double kb;  // Proportional coefficient for orientation error
    double kw; // Propoortional coefficient for orientation error when position is reached
    double position_tol;     // Acceptable static error on position (m)
    double drift_tol;   // Acceptable drift from reference position when reorienting (m)
    double angular_tol;     // Acceptable static error on orientation

    int flag_position_reached;
} PositionController;

PositionController *init_position_controller();
void control_position(
    PositionController *position_controller,
    RobotPosition *robot_position
);

#endif