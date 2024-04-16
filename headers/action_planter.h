#ifndef BLY_ACT_PLANTER_H
#define BLY_ACT_PLANTER_H

#include "actions.h"

typedef enum _preference : int8_t
{
    SideLeft = 1,
    SideMiddle = 0,
    SideRight = -1
} planter_side_t;

class ActionPlanter : public Action
{
private:
    planter_side_t preference; // Set plants in on a certain side first, conditions the way pot clearing is done
    uint8_t nbPlants;
    // Will clear away pots if needsPotClear != 0,
    // from left to right if needsPotClear == SideLeft
    // from right to left if needsPotClear == SideRight
    // no clearing if needsPotClear == SideMiddle (0)
    planter_side_t needsPotClear;
public:
    ActionPlanter(graph_path_t *path, uint8_t nbPlants, planter_side_t preference, planter_side_t needsPotClear) : Action(DepositPlanter, true, path) {
        this->nbPlants = nbPlants;
        this->preference = preference;
        this->needsPotClear = needsPotClear;
    }
    void do_action();
};

#endif