#include "action_return.h"


void ActionBackToBase::do_action() {
    if (path_following_to_action(path) == -1) return;
    // Idle
    shared.teensy->idle(); 
    // Score update
    shared.score += 10;
}