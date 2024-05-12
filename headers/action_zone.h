#ifndef BLY_ACT_ZONE_H
#define BLY_ACT_ZONE_H

#include "actions.h"

class ActionZone : public Action
{
private:
    uint8_t nbPlants; // Number of plants to drop
    uint8_t zoneIdx;  // Zone index related to shared.zonesDone
public:
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