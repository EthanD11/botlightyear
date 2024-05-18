#ifndef BLY_ACT_CLEARPOTS_H
#define BLY_ACT_CLEARPOTS_H

#include "actions.h"

class ActionClearPots : public Action {
    public: 
        /**
        * @brief Action to clear pots at the start of the game
        * @param path The path structure to get to the node associated with the end of the clearing 
        * (NULL for now, only position control is done)
        * Only works when starting from "BottomRight" for blue team and "BottomLeft" base for yellow team
        */
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