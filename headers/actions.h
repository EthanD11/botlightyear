#ifndef BLY_ACTIONS_H
#define BLY_ACTIONS_H

#include "action_displacement.h"
#include "action_planter.h"
#include "action_plants.h"
#include "action_pots.h"
#include "action_return.h"
#include "action_SP.h"
#include "action_zone.h"

/* UTILS: PATH_FOLLOWING_TO_ACTION */
void path_following_to_action(path_t *path); 

/* UTILS: POSITION_CONTROL */
void action_position_control(double x_end, double y_end, double theta_end); 

#endif