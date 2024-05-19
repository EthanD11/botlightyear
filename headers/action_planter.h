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
    /**
    * @brief Action to put plants (or plants in pots) in a planter
    * @param path The path structure to get to the node associated with the chosen planter
    * @param nbPlants The number of plants (in pots or not) to unload (for a maximum of 3). Warning : the number of plants 
    * to drop has to be available in the current storage ! Plants will be unloaded following storage's order of unloading
    * @param preference The first planter's side that will be dropped into (SideLeft, SideMiddle or SideRight)
    * @param planterIdx The index associated with the planter in sharedVariables : 0 if reserved planter, 1 if planter next to the reserved one, 
    * 2 if planter on the other side of the map ("Adversary's side")
    * @param needsPotClear Defines if an action needs pot clearing before drop. If yes, specifies the direction of clearing : 
    * from left to right if needsPotClear == SideLeft and from right to left if needsPotClear == SideRight. No clearing if needsPotClear == SideMiddle (default)
    */
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