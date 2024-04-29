#ifndef BLY_ACT_POTS_H
#define BLY_ACT_POTS_H

#include "actions.h"




class ActionPots : public Action
{
private: 
    uint8_t potCounter; 
public:
    ActionPots(graph_path_t *path, uint8_t potNumber) : Action(TakePots, true, path) {
        this->potCounter = potNumber;
        this->needs[0] = 1;  // SptrPlate
        this->needs[1] = 1;  // StprSlider
        this->needs[2] = 1;  // StprFlaps
        this->needs[3] = 0;  // Dxls
        this->needs[4] = 0;  // LidarBottom (for now, permissive)
    }
    ~ActionPots() {}
    void do_action();
};

#endif