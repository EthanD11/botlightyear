#ifndef BLY_ACT_CLEARPOTS_H
#define BLY_ACT_CLEARPOTS_H

#include "actions.h"

class ActionClearPots : public Action {
    public: 
        ActionClearPots (graph_path_t* graph_path) : Action(Displacement, false, graph_path) {
            this->needs[0] = 0;  // SptrPlate
            this->needs[1] = 0;  // StprSlider
            this->needs[2] = 0;  // StprFlaps
            this->needs[3] = 0;  // Dxls
            this->needs[4] = 0;  // LidarBottom
        }
        ~ActionClearPots() {}
        void do_action();
};

#endif