#include "action_displacement.h" 
#include "actions.h"
#include "decision.h"
#include <stdio.h>


// Teensy *teensy  = shared.teensy;

void ActionDisplacement::do_action() {
    path_following_to_action(path);
}


// void displacement_action() {
//     path_following_to_action(decision.path);
// }