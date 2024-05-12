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
    planter_side_t preference; // Set plants in on a certain side first
    uint8_t nbPlants;
    // Will clear away pots if needsPotClear != 0,
    // from left to right if needsPotClear == SideLeft
    // from right to left if needsPotClear == SideRight
    // no clearing if needsPotClear == SideMiddle (0)
    planter_side_t needsPotClear;
    uint8_t planterIdx; 
public:
    ActionPlanter(graph_path_t *path, uint8_t nbPlants, planter_side_t preference, uint8_t planterIdx, planter_side_t needsPotClear = SideMiddle) : Action(DepositPlanter, true, path) {
        this->nbPlants = nbPlants;
        this->preference = preference;
        this->needsPotClear = needsPotClear;
        this->planterIdx = planterIdx; 
        this->needs[0] = 1;  // SptrPlate
        this->needs[1] = 1;  // StprSlider
        this->needs[2] = 1;  // StprFlaps
        this->needs[3] = 0;  // Dxls
        this->needs[4] = 0;  // LidarBottom

    }
    ~ActionPlanter() {}
    void do_action();
};

#endif