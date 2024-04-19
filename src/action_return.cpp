#include "action_return.h"

//#define EMPTY_PLANT

void ActionBackToBase::do_action() {

    #ifdef EMPTY_PLANT

    if (get_content_count(ContainsStrongPlant) > 0) {


    } else {
        if (abs(path->x[path->nNodes-1]) > 0.5)
        path->x[path->nNodes-1] += 0.1*(1-2*(path->x[path->nNodes-1] < 1.0));

        if (path_following_to_action(path) == -1) return;
        // Idle
        shared.teensy->idle(); 
        // Score update
        shared.score += 10;
    }

    #else

    if (abs(path->x[path->nNodes-1]) > 0.5)
        path->x[path->nNodes-1] += 0.1*(1-2*(path->x[path->nNodes-1] < 1.0));

    if (path_following_to_action(path) == -1) return;
    // Idle
    shared.teensy->idle(); 
    // Score update
    shared.score += 10;
    
    #endif
}