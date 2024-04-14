#include "action_return.h"
#include "decision.h"
#include "actions.h"



void path_following_to_base() {

    if (path_following_to_action(decision.path) == -1) return;

    // Idle
    shared.teensy->idle(); 

    // Score update
    shared.score += 10;

}