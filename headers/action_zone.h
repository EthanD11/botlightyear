#ifndef BLY_ACT_ZONE_H
#define BLY_ACT_ZONE_H

#include "actions.h"

class ActionZone : public Action
{
private:
    uint8_t nbPlants; // Number of plants to drop
public:
    ActionZone(graph_path_t *path, uint8_t nbPlants) : Action(DepositPlanter, true, path) {
        this->nbPlants = nbPlants;
    }
    ~ActionZone() {}
    void do_action();
};
#endif