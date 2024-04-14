#include "action_displacement.h" 
#include "actions.h"
#include "decision.h"
#include <stdio.h>


Teensy *teensy  = shared.teensy;



void displacement_action() {
    path_following_to_action(decision.path);
}