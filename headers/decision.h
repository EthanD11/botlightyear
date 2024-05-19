#ifndef BLY_DECISION_H
#define BLY_DECISION_H

#include "actions.h"

/**
 * @brief Takes a decision for the robot's next action
 * @return A pointer to the next action (Action class derived) that will be executed by the robot
*/
Action* make_decision();

#endif