#include "action_clearpot.h" 


void ActionClearPot::do_action() {
    if (action_position_control(this->x_end, this->y_end, this->theta_end) == -1) return;
}