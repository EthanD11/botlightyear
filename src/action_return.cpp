#include "action_return.h"
#include <stdio.h>
#include <cmath>
//#define EMPTY_PLANT

void ActionBackToBase::do_action() {

    #ifdef EMPTY_PLANT
    if (posePlant && get_content_count(ContainsStrongPlant) > 0) {


    } else {
        shared.goingToBase = 1;
        if (action_position_control(path->x[path->nNodes-1] + 0.1, path->y[path->nNodes-1] + ((shared.color == TeamYellow) ? 0.3 : (-0.03) ), path->thetaEnd) == -1) return; 
        printf("Position of back to base: (%f,%f)\n", path->x[path->nNodes-1] + 0.1, path->y[path->nNodes-1] + ((shared.color == TeamYellow) ? 0.3 : (-0.03) ));
        // Idle
        shared.teensy->idle(); 
        shared.backToBaseDone = 1; 
        // Score update
        shared.score += 10;
    }
    


    #else

    /*if (abs(path->x[path->nNodes-1]) > 0.5)
        path->x[path->nNodes-1] += 0.1*(1-2*(path->x[path->nNodes-1] < 1.0));

    if (path_following_to_action(path) == -1) return;*/
    double x_pos, y_pos, theta_pos; 
    shared.get_robot_pos(&x_pos, &y_pos, &theta_pos);
    shared.teensy->set_position_controller_gains(0.7,4.0,-1.0,2.0);
    shared.goingToBase = 1;

    if (hypot(x_pos - path->x[path->nNodes-1], y_pos - path->y[path->nNodes-1]) < 1.5) {
        if (action_position_control(path->x[path->nNodes-1] + 0.1, path->y[path->nNodes-1] + ((shared.color == TeamYellow) ? 0.3 : (-0.03) ), path->thetaEnd) == -1) return; 
        printf("Position of back to base: (%f,%f)\n", path->x[path->nNodes-1] + 0.1, path->y[path->nNodes-1] + ((shared.color == TeamYellow) ? 0.3 : (-0.03) ));
    } else {
        if (path_following_to_action(path) == -1) return; 
    }
    // Idle
    shared.teensy->idle(); 
    shared.backToBaseDone = 1; 
    // Score update
    shared.score += 10;
    
    #endif
}