#ifndef BLY_ACTIONS_H
#define BLY_ACTIONS_H

#include "action_displacement.h"
#include "action_planter.h"
#include "action_plants.h"
#include "action_pots.h"
#include "action_return.h"
#include "action_SP.h"
#include "action_zone.h"
#include <cmath>
#include <unistd.h>

#define vref 0.25                 // [m/s] Speed reference for path following
#define dist_goal_reached 0.40    // [m] Distance tolerance to goal for path following


/* UTILS: PATH_FOLLOWING_TO_ACTION */
int8_t path_following_to_action(graph_path_t *path); 

/* UTILS: POSITION_CONTROL */
int8_t action_position_control(double x_end, double y_end, double theta_end); 

#endif