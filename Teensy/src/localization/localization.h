#ifndef _LOCALIZATION_H
#define _LOCALIZATION_H

#include <Encoder.h>
#include "../../utils.h"

typedef struct RobotPosition {
    double x, y, theta;
    double vfwd, vrot, omega;
    double speed_left, speed_right;

    double dt;

    Encoder *enc_r, *enc_l;
    double old_tick_left, old_tick_right;
    
} RobotPosition;

RobotPosition* init_robot_position(double x0, double y0, double theta_0); 
void free_robot_position(RobotPosition *robot_position);
void update_localization(RobotPosition *robot_position);

#endif