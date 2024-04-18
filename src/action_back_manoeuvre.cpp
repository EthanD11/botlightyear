#include "action_back_manoeuvre.h"

void ActionBACK::do_action(){
    // 1) Initiate pos control gains
    shared.teensy->set_position_controller_gains(
        0.8, // kp
        8.0, // ka
        2.0, // kb
        6.0 // kw
    );

    // 2) Initiate position control to computed position
    shared.teensy->pos_ctrl(
        this->xnew, 
        this->ynew, 
        this->thetanew);
}