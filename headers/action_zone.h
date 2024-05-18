#ifndef BLY_ACT_ZONE_H
#define BLY_ACT_ZONE_H

#include "actions.h"

class ActionZone : public Action
{
private:
    uint8_t nbPlants; // Number of plants to drop
    uint8_t zoneIdx;  // Zone index related to shared.zonesDone
public:
    /**
    * @brief Action to put plants (or plants in pots) in a zone
    * @param path The path structure to get to the node associated with the chosen zone
    * @param nbPlants The number of plants (in pots or not) to unload. Warning : the number of plants 
    * to drop has to be available in the current storage ! Plants will be unloaded following storage's order of unloading
    * @param zoneIdx The index associated with the planter in sharedVariables : 0 if reserved zone, 1 if the other zone on the "Robot's side", 
    * 2 if zone on the other side of the map ("Adversary's side")
    */
    ActionZone(graph_path_t *path, uint8_t nbPlants, uint8_t zoneIdx) : Action(DepositPlanter, true, path) {
        this->nbPlants = nbPlants;
        this->zoneIdx = zoneIdx; 
        this->needs[0] = 0;  // SptrPlate
        this->needs[1] = 0;  // StprSlider
        this->needs[2] = 0;  // StprFlaps
        this->needs[3] = 0;  // Dxls (0 for now, permissive)
        this->needs[4] = 0;  // LidarBottom
    }
    ~ActionZone() {}
    void do_action();
};
#endif