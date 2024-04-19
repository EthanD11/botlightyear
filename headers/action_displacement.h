#ifndef BLY_ACT_DISPLACEMENT_H
#define BLY_ACT_DISPLACEMENT_H

#include "actions.h"

// void displacement_action(); 

class ActionDisplacement : public Action {
    public: 
        ActionDisplacement (graph_path_t* graph_path) : Action(Displacement, true, graph_path) {
            this->needs[0] = 0;  // SptrPlate
            this->needs[1] = 0;  // StprSlider
            this->needs[2] = 0;  // StprFlaps
            this->needs[3] = 0;  // Dxls
            this->needs[4] = 0;  // LidarBottom
        }
        ~ActionDisplacement() {}
        void do_action();
};

#endif