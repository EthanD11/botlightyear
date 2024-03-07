#include <Encoder.h>

#ifndef _LOCALIZATION_H
#define _LOCALIZATION_H

typedef struct RobotPosition {
    double x, y, theta;
    double vfwd, omega;
    double speed_left, speed_right;

    double dt;

    Encoder enc_r, enc_l;
    double old_tick_left, old_tick_right;
    
} RobotPosition;

RobotPosition* init_robot_position(double x0, double y0, double theta_0); 
void free_robot_position(RobotPosition *robot_position);
void update_localization(RobotPosition *robot_position);
inline void reset_encoders(RobotPosition *robot_position) {
    robot_position->enc_l.write(0); 
    robot_position->enc_r.write(0); 
    robot_position->old_tick_left = 0; 
    robot_position->old_tick_right = 0;
}

#endif