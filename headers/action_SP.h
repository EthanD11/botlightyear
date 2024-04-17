#ifndef BLY_ACTION_SP_H
#define BLY_ACTION_SP_H


#include "actions.h"


class ActionSP : public Action {
    private: 
        uint8_t sp_counter;
        bool reserved;
    public: 
        ActionSP (graph_path_t* graph_path, uint8_t sp_number, bool is_reserved) : Action(TurnSP, true, graph_path) { 
            sp_counter = sp_number;
            reserved = is_reserved;
        }
        ~ActionSP() {}
        void do_action();        
};

#endif