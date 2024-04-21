#ifndef BLY_ACT_BULLDOZER_H
#define BLY_ACT_BULLDOZER_H

#include "actions.h"

// void displacement_action(); 

class ActionBulldozer : public Action {
    public: 
        ActionBulldozer (graph_path_t* graph_path) : Action(Bulldozer, true, graph_path) {
            this->needs[0] = 0;  // SptrPlate
            this->needs[1] = 0;  // StprSlider
            this->needs[2] = 0;  // StprFlaps
            this->needs[3] = 0;  // Dxls
            this->needs[4] = 0;  // LidarBottom
        }
        ~ActionBulldozer() {}
        void do_action();
};

#endif