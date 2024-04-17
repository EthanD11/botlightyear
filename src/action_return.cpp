#include "action_return.h"


void ActionBackToBase::do_action() {

    if (abs(path->x[path->nNodes]) > 0.5)
        path->x[path->nNodes] += 0.1*(1-2*(path->x[path->nNodes] < 1.0));

    if (path_following_to_action(path) == -1) return;
    // Idle
    shared.teensy->idle(); 
    // Score update
    shared.score += 10;
}