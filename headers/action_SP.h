#ifndef BLY_ACTION_SP_H
#define BLY_ACTION_SP_H


#include "actions.h"

typedef enum {
    Forward, 
    Backward
} sp_direction_t;


class ActionSP : public Action {
    private: 
        uint8_t sp_counter;
        bool reserved;  
        sp_direction_t sp_direction; 
    public: 
        /**
        * @brief Action to turn solar panels (by batches of three maximum)
        * @param path The path structure to get to the node associated with the solar panels (the middle one of the 3)
        * @param sp_number The number of solar panels to turn (from 1 up to 3)
        * @param is_reserved Tells if the solar panels are reserved or not. Non-reserved panels will be turned using the camera orientation information
        * @param direction The direction in which the robot does the solar panels (Forward or Backward) from the robot's pov
        */
        ActionSP (graph_path_t* graph_path, uint8_t sp_number, bool is_reserved, sp_direction_t direction) : Action(TurnSP, true, graph_path) { 
            sp_counter = sp_number;
            reserved = is_reserved;
            sp_direction = direction; 
            this->needs[0] = 0;  // SptrPlate
            this->needs[1] = 0;  // StprSlider
            this->needs[2] = 0;  // StprFlaps
            this->needs[3] = 0;  // Dxls (0 for now, permissive)
            this->needs[4] = 0;  // LidarBottom
        }
        ~ActionSP() {}
        void do_action();        
};

#endif