#ifndef BLY_ACT_PLANTS_H
#define BLY_ACT_PLANTS_H

#include "actions.h"

class ActionPlants : public Action
{
private: 
    uint8_t plantCounter; 
public:
    ActionPlants(graph_path_t *path, uint8_t plantNumber) : Action(DepositPlanter, true, path) {
        plantCounter = plantNumber;
    }
    ~ActionPlants() {}
    void do_action();
};
#endif