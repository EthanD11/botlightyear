#ifndef BLY_ACT_RETURN_H
#define BLY_ACT_RETURN_H

#include "actions.h"

#define EMPTY_PLANTS_ON_RETURN

/* PATH_FOLLOWING: */
//void path_following_to_base(); 

class ActionBackToBase : public Action {
    #ifdef EMPTY_PLANTS_ON_RETURN
    private:
        uint8_t nbPlants;
    public:
        
        ActionBackToBase(graph_path_t* graph_path, uint8_t nbPlants) : Action(ReturnToBase, true, graph_path) {
            this->needs[0] = 0;  // SptrPlate
            this->needs[1] = 0;  // StprSlider
            this->needs[2] = 0;  // StprFlaps
            this->needs[3] = 0;  // Dxl1
            this->needs[4] = 0;  // LidarBottom
            this->nbPlants = nbPlants;
        }

    #else
    public: 
        ActionBackToBase(graph_path_t* graph_path) : Action(ReturnToBase, true, graph_path) {
            this->needs[0] = 0;  // SptrPlate
            this->needs[1] = 0;  // StprSlider
            this->needs[2] = 0;  // StprFlaps
            this->needs[3] = 0;  // Dxl1
            this->needs[4] = 0;  // LidarBottom
        }
    #endif
        ~ActionBackToBase() {}
        void do_action();
};

#endif