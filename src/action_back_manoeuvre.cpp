#include "action_back_manoeuvre.h"

void ActionBACK::do_action(){
    // 1) Initiate pos control gains
    shared.teensy->set_position_controller_gains(
        0.8, // kp
        8.0, // ka
        2.0, // kb
        6.0 // kw
    );

    // 2) Get position
    double xpos, ypos, thetapos;
    shared.get_robot_pos(&xpos, &ypos, &thetapos);

    // 3) Compute new position 
    double xnew, ynew, thetanew;
    xnew = xpos - cos(thetapos)*this->backward_dist;
    ynew = ypos - sin(thetapos)*this->backward_dist;
    thetanew = thetapos;

    // 4) While new position is too close to wall, reduce backward_dist and recompute new position
    while (this->too_close_to_wall(xnew, ynew) && (backward_dist > dist_useless)) {
        backward_dist = backward_dist - 0.01;
        xnew = xpos - cos(thetapos)*backward_dist;
        ynew = ypos - sin(thetapos)*backward_dist;
    }

    // 5) If robot is far away enough from a wall, do back manoeuver
    bool manoeuver_useless = (backward_dist <= dist_useless);
    if (!manoeuver_useless) {
        // 5.1) Do position control to computed position
        shared.teensy->pos_ctrl(xnew, ynew, thetanew);
        
        // 5.2) Wait for position control to be over
        while ((shared.teensy->ask_mode()) != ModePositionControlOver) usleep(10000);
    }
}

bool ActionBACK::too_close_to_wall(double x, double y) {
    if ((x       < safety_distance_to_pami_side_wall) ||
        (y       < safety_distance_to_walls)          ||
        (2.0 - x < safety_distance_to_walls)          ||
        (3.0 - y < safety_distance_to_walls)) 
    {
        return true;
    }

    return false;
}