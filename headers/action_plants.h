#ifndef BLY_ACT_PLANTS_H
#define BLY_ACT_PLANTS_H

#include "actions.h"

class ActionPlants : public Action
{
private: 
    uint8_t plantCounter; 
public:
    ActionPlants(graph_path_t *path, uint8_t plantNumber) : Action(DepositPlanter, true, path) {
        this->plantCounter = plantNumber;
        this->needs[0] = 1;  // SptrPlate
        this->needs[1] = 1;  // StprSlider
        this->needs[2] = 1;  // StprFlaps
        this->needs[3] = 0;  // Dxl1
        this->needs[4] = 0;  // Dxl2
        this->needs[5] = 1;  // LidarBottom
    }
    ~ActionPlants() {}
    void do_action();
};
#endif