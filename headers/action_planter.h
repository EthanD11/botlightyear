#ifndef BLY_ACT_PLANTER_H
#define BLY_ACT_PLANTER_H

#include "actions.h"

class ActionPlanter : public Action
{
public:
    ActionPlanter(graph_path_t *path) : Action(DepositPlanter, true, path) {}
    void do_action();
};

#endif