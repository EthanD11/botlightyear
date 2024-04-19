#ifndef BLY_ACT_RETURN_H
#define BLY_ACT_RETURN_H

#include "actions.h"

/* PATH_FOLLOWING: */
//void path_following_to_base(); 

class ActionBackToBase : public Action {
    public: 
        ActionBackToBase(graph_path_t* graph_path) : Action(ReturnToBase, true, graph_path) {
            this->needs[0] = 0;  // SptrPlate
            this->needs[1] = 0;  // StprSlider
            this->needs[2] = 0;  // StprFlaps
            this->needs[3] = 0;  // Dxl1
            this->needs[4] = 0;  // LidarBottom
        }
        ~ActionBackToBase() {}
        void do_action();
};

#endif