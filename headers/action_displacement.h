#ifndef BLY_ACT_DISPLACEMENT_H
#define BLY_ACT_DISPLACEMENT_H

#include "actions.h"

// void displacement_action(); 

class ActionDisplacement : public Action {
    public: 
        ActionDisplacement (graph_path_t* graph_path) : Action(Displacement, true, graph_path) {
        }
        void do_action();
};

#endif