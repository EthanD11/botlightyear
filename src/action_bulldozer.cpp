#include "action_bulldozer.h" 
#include <stdio.h>


// Teensy *teensy  = shared.teensy;

void ActionBulldozer::do_action() {
    shared.steppers->flaps_move(FlapsOpen);
    shared.servoFlaps->deploy();

    if (path_following_to_action(path) == -1) return;

    uint8_t target;
    double xpos, ypos, thetapos;
    double thetaEnd;
    // graph_path *path;
    
    // Select correct arrival zone
    uint8_t nNodes = 3;
    void *temp = malloc(sizeof(graph_path_t) + nNodes*(sizeof(uint8_t) + 2*sizeof(double)));
    graph_path_t *path = (graph_path_t*) temp;
    path->idNodes = ((uint8_t *) temp) + sizeof(graph_path_t);
    path->x = (double *) (((uint8_t*)temp) + sizeof(graph_path_t) + nNodes*sizeof(uint8_t));
    path->y = (double *) (((uint8_t*)temp) + sizeof(graph_path_t) + nNodes*(sizeof(uint8_t) + sizeof(double)));
    
    path->nNodes = nNodes;
    uint8_t bulldoPath[3]; 


    if (shared.color == TeamBlue) { 
        target = 2;
        thetaEnd = M_PI/2;

        // path->x[0] = shared.graph->nodes[42].x; 
        bulldoPath[0] = 0;
        bulldoPath[1] = 0;
        bulldoPath[2] = 0;
        // shared.graph->update_obstacle(22,1);
        // shared.graph->update_obstacle(19,1);
        // shared.graph->update_obstacle(13,1);
    } else {// TeamYellow 
        target = 43;
        thetaEnd = -M_PI/2;

        bulldoPath[0] = 0;
        bulldoPath[1] = 0;
        bulldoPath[2] = 0;
        // shared.graph->update_obstacle(19,1);
        // shared.graph->update_obstacle(22,1);
        // shared.graph->update_obstacle(30,1);
    }
    for (uint8_t i = 0; i<nNodes; i++) {
        path->idNodes[i] = shared.graph->nodes[bulldoPath[i]].id; 
        path->x[i] = shared.graph->nodes[bulldoPath[i]].x; 
        path->y[i] = shared.graph->nodes[bulldoPath[i]].y; 
    }
    // Launch BULLDO
    
    // path = shared.graph->compute_path(xpos, ypos, &target, 1);
    if (path != NULL) {
        path->thetaEnd = thetaEnd;
    }
    // if (shared.color == TeamBlue) { 
    //     shared.graph->update_obstacle(22,0);
    //     shared.graph->update_obstacle(19,0);
    //     shared.graph->update_obstacle(13,0);
    // } else {// TeamYellow 
    //     shared.graph->update_obstacle(19,0);
    //     shared.graph->update_obstacle(22,0);
    //     shared.graph->update_obstacle(30,0);
    // }
    if (path_following_to_action(path) == -1) return;
    
    shared.bulldoDone = 1;
    // Go nicely backward
    double xnew, ynew, thetanew;
    double backward_dist = 0.2;
    xnew = xpos - cos(thetapos)*backward_dist;
    ynew = ypos - sin(thetapos)*backward_dist;
    thetanew = thetapos;
    if (action_position_control(xnew, ynew, thetanew) == -1) return;

    // A half turn to before finishing action
    thetanew = thetapos + M_PI;
    if (action_position_control(xnew, ynew, thetanew) == -1) return;

}


// void displacement_action() {
//     if (path_following_to_action(decision.path) == -1) return;
// }