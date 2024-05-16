#ifndef BLY_ACT_RETURN_H
#define BLY_ACT_RETURN_H

#include "actions.h"

/* PATH_FOLLOWING: */
//void path_following_to_base(); 

class ActionBackToBase : public Action {
    public: 
        ActionBackToBase(graph_path_t* graph_path, bool posePlant = false) : Action(ReturnToBase, true, graph_path) {
            this->needs[0] = (uint8_t)posePlant;  // SptrPlate
            this->needs[1] = (uint8_t)posePlant;  // StprSlider
            this->needs[2] = (uint8_t)posePlant;  // StprFlaps
            this->needs[3] = 0;  // Dxl1
            this->needs[4] = 0;  // LidarBottom
        }
        ~ActionBackToBase() {}
        void do_action();
};

#endif