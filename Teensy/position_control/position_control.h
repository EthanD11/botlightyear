#ifndef _POSITION_CONTROL_H_
#define _POSITION_CONTROL_H_

#include "utils.h"

typedef struct PositionController {
    double speed_refl, speed_refr; // Output speed references of the position controller

    double kp;  // Proportional coefficient for distance error
    double ka;  // Proportional coefficient for direction error
    double kb;  // Proportional coefficient for orientation error
    double position_tol;     // Acceptable static error on position (m)
    double drift_tol;   // Acceptable drift from reference position when reorienting (m)
    double angular_tol;     // Acceptable static error on orientation

    int flag_position_reached;
} PositionController;

PositionController *init_position_controller();
void position_control(
    PositionController *position_controller,
    double xref, double yref,
    double xpos, double ypos, double theta
);

#endif