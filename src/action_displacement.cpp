#include "action_displacement.h" 
#include <stdio.h>


// Teensy *teensy  = shared.teensy;

void ActionDisplacement::do_action() {
    if (path_following_to_action(path) == -1) return;
}


// void displacement_action() {
//     if (path_following_to_action(decision.path) == -1) return;
// }