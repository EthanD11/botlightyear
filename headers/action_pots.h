#ifndef BLY_ACT_POTS_H
#define BLY_ACT_POTS_H

#include "actions.h"




class ActionPots : public Action
{
private: 
    uint8_t potCounter; 
    bool removePot4;
    bool freeThGarden;
public:
    // remove pot 
    // potNumber: 1, 2, 3, 4 is the number of the pot to take
    // removeDoublePot: true if you want to withdraw the double pot after taking either pot 3 or 5 
    //                  (won't happen if you don't take one of the neighboring pots!!)
    // freeTheGarden: true if you want to completely free the planter from all remaining pots after taking the pot
    ActionPots(graph_path_t *path, uint8_t potNumber,bool removeDoublePot = false,bool freeTheGarden= false) : Action(TakePots, true, path) {
        this->potCounter = potNumber;
        this->removePot4 = removeDoublePot;
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