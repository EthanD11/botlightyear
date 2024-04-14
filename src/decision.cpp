#include "decision.h"

#include <stdlib.h> // For random numbers
#include <stdbool.h>
#include <stdio.h>

#define TESTS


int8_t time_gotobase = 20; 


bool in_array(uint8_t * arr, uint8_t len, uint8_t val) {
    for (uint8_t i = 0; i<len; i++) {
        if (arr[i] == val) {
            return true; 
        }
    }
    return false; 
}

double getThetaEnd(uint8_t * arrNodes, double * arrThetas , uint8_t len, uint8_t val) {
    for (uint8_t i = 0; i<len; i++) {
        if (arrNodes[i] == val) {
            return arrThetas[i]; 
        }
    }
    return 0; 
}

void make_decision(decision_t *decision) {
    

    int8_t current_time = shared.update_and_get_timer();
    double x_pos, y_pos, theta_pos, dist_from_currentNode; 
    shared.get_robot_pos(&x_pos, &y_pos, &theta_pos); 
    graph_path_t* path;
    uint8_t currentNode = shared.graph->identify_pos(x_pos, y_pos, &dist_from_currentNode);  // Get the closest node from the robot 

    #ifdef TESTS

    // Some code here to do the path planning for the test action
    uint8_t target = shared.graph->plants[rand()%6]; // get random plant node
    shared.graph->node_level_update(target, 0, DISABLE_PROPAGATION);
    path = shared.graph->compute_path(currentNode, &target, 1, 0);
    path->thetaStart = 0; 
    path->thetaEnd = 0; 
    decision->actionType = TestAction; 
    decision->path = path;
    return;
    #endif

    // First check if time is over : 
    if (current_time < 0) {
        decision->actionType = GameFinished; 
        return; 
    }

    // Second check if time is running out : 
    if (current_time < time_gotobase) {
        // Get the closest base from the robot pov
        path = shared.graph->compute_path(currentNode, shared.graph->friendlyBases, 3,0);
        path->thetaStart = theta_pos; 
        path->thetaEnd = getThetaEnd(shared.graph->friendlyBases, shared.graph->friendlyBasesTheta, 3, path->target); 
        decision->actionType = ReturnToBase; 
        return; 
    }

    // Strategy : for now, only cycling between random plant nodes and going back to closest base

    if (in_array(shared.graph->plants, 6, currentNode)) {
        // If it's in a plant node, go back to a base 
        path = shared.graph->compute_path(currentNode, shared.graph->friendlyBases, 3,0);
        path->thetaStart = theta_pos; 
        path->thetaEnd = getThetaEnd(shared.graph->friendlyBases, shared.graph->friendlyBasesTheta, 3, path->target); 

    } else {
        // Else, go to a random plant node 
        uint8_t target = shared.graph->plants[rand()%6]; // get random plant node
        shared.graph->node_level_update(target, 0, DISABLE_PROPAGATION);
        path = shared.graph->compute_path(currentNode, &target, 1, 0);
        path->thetaStart = theta_pos; 
        path->thetaEnd = 0; 
    }
    decision->actionType = Displacement; 
    decision->path = path;
    return; 
}