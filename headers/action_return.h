#ifndef BLY_ACT_RETURN_H
#define BLY_ACT_RETURN_H

#include "actions.h"

/* PATH_FOLLOWING: */
//void path_following_to_base(); 

class ActionBackToBase : public Action {
    public: 
        ActionBackToBase(graph_path_t* graph_path) : Action(ReturnToBase, true, graph_path) {
        }
        ~ActionBackToBase() {}
        void do_action();
};

#endif