#include "action_back_manoeuvre.h"

void ActionBACK::do_action(){
    // 3) Initiate position control to computed position
    shared.teensy->pos_ctrl(
        this->xnew, 
        this->ynew, 
        this->thetanew);
}