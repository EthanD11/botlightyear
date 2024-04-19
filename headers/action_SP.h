#ifndef BLY_ACTION_SP_H
#define BLY_ACTION_SP_H


#include "actions.h"

typedef enum {
    Forward, 
    Backward
} sp_direction_t


class ActionSP : public Action {
    private: 
        uint8_t sp_counter;
        bool reserved;  
        sp_direction_t sp_direction; 
    public: 
        ActionSP (graph_path_t* graph_path, uint8_t sp_number, bool is_reserved, sp_direction_t direction) : Action(TurnSP, true, graph_path) { 
            sp_counter = sp_number;
            reserved = is_reserved;
            sp_direction = direction; 
            this->needs[0] = 0;  // SptrPlate
            this->needs[1] = 0;  // StprSlider
            this->needs[2] = 0;  // StprFlaps
            this->needs[3] = 0;  // Dxl1 (0 for now, permissive)
            this->needs[4] = 0;  // Dxl2 (0 for now, permissive)
            this->needs[5] = 0;  // LidarBottom
        }
        ~ActionSP() {}
        void do_action();        
};

#endif