#include <Encoder.h>

#ifndef _LOCALIZATION_H
#define _LOCALIZATION_H

typedef struct RobotPosition {
    double x, y, theta;
    double vfwd, omega;

    double control_time, current_time, dt;

    Encoder enc_r, enc_l;
    double old_tick_left, old_tick_right;
    double speed_left, speed_right;
    
} RobotPosition;

void init_robot_position(RobotPosition *robot_position, double x0, double y0, double theta_0); 
void update_localization(RobotPosition *robot_position);

#endif