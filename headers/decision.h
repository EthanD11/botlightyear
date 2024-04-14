#ifndef BLY_DECISION_H
#define BLY_DECISION_H

#include "shared_variables.h"
#include <stdint.h>

typedef enum _action : uint8_t
{
    GameFinished,
    ReturnToBase,
    Displacement,
    TakePlants,
    TakePots,
    TurnSP,
    DepositZone,
    DepositPlanter, 
    TestAction
} action_t;

typedef struct _decision
{
    action_t actionType;
    graph_path_t *path;
} decision_t;

void make_decision(decision_t *decision);

extern decision_t decision; 

#endif