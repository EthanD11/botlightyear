#include "action_clearpots.h"

void ActionClearPots::do_action() {
    double x = 0, y = 0, theta = 0;
    double xgoal, ygoal, theta_goal;
    shared.get_robot_pos(&x,&y,&theta);
    shared.teensy->set_position_controller_gains(1.2, 4.0, 0., 2.0);
    if (shared.color == TeamYellow) {
        xgoal = x + 0.7;
        ygoal = y - 0.03;
        theta_goal = 0; 
    } else {
        xgoal = x + 0.7;
        ygoal = y + 0.03;
        theta_goal = 0;
    }
    shared.teensy->pos_ctrl(xgoal, ygoal, theta_goal);
    usleep(1500000);
    // if (shared.color == TeamYellow) {
    //     xgoal = 0.225;
    //     ygoal = 0.275;
    //     theta_goal = 0;
    // } else {
    //     xgoal = 22.5;
    //     ygoal = 0.25;
    //     theta_goal = 0;
    // }
    // shared.teensy->pos_ctrl(xgoal, ygoal, theta_goal);
    // usleep(4000000);
    // shared.teensy->idle();
}