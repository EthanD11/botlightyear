#include "action_return.h"


void ActionBackToBase::do_action() {
    path_following_to_action(path); 
    // Idle
    shared.teensy->idle(); 
    // Score update
    shared.score += 10;
}