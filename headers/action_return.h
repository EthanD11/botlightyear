#ifndef BLY_ACT_RETURN_H
#define BLY_ACT_RETURN_H

#include "shared_variables.h"
#include "actions.h"

/* PATH_FOLLOWING: */
void path_following_to_base(); 

class ActionBackToBase : public Action {
    public: 
        void do_action (); 
        ActionBackToBase(graph_path_t* graph_path) : Action(ReturnToBase, true, graph_path) {
        }
};

#endif