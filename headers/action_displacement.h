#ifndef BLY_ACT_DISPLACEMENT_H
#define BLY_ACT_DISPLACEMENT_H

#include "actions.h"

// void displacement_action(); 

class ActionDisplacement : public Action {
    public: 
        ActionDisplacement (graph_path_t* path) : Action(Displacement, true, path) {
        }
        ~ActionDisplacement() {}
        void do_action();
};

#endif