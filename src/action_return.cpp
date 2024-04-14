#include "action_return.h"
#include "decision.h"
#include "actions.h"


#define vref 0.25                 // [m/s] Speed reference for path following
#define dist_goal_reached 0.25    // [m] Distance tolerance to goal for path following


void path_following_to_base() {

    if (path_following_to_action(decision->path) == -1) return;

    // Idle
    teensy->idle(); 

    // Score update
    shared.score += 10;

}