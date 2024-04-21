#include "action_bulldozer.h" 
#include <stdio.h>


// Teensy *teensy  = shared.teensy;

void ActionBulldozer::do_action() {
    shared.steppers->flaps_move(FlapsOpen, CALL_BLOCKING);
    if (path_following_to_action(path) == -1) return;
    // if (shared.color = Team)
}


// void displacement_action() {
//     if (path_following_to_action(decision.path) == -1) return;
// }