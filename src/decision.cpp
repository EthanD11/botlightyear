#include "decision.h"

#include <stdlib.h> // For random numbers


#define TRUE 1
#define FALSE 0



extern SharedVariables shared;

int8_t time_gotobase = 80; 


uint8_t in_array(uint8_t * arr, uint8_t len, uint8_t val) {
    for (uint8_t i = 0; i<len; i++) {
        if (arr[i] == val) {
            return TRUE; 
        }
    }
    return FALSE; 
}

void make_decision(decision_t *decision) {

    int8_t current_time = shared.update_and_get_timer(); 
    double x_pos, y_pos, theta_pos, dist_from_currentNode; 
    shared.get_robot_pos(&x_pos, &y_pos, &theta_pos); 
    graph_path_t* path;
    uint8_t currentNode = shared.graph.identify_pos(x_pos, y_pos, &dist_from_currentNode);  // Get the closest node from the robot 


    // First check if time is running out : 
    if (current_time > time_gotobase) {
       

        // Get the closest base from the robot pov
        path = shared.graph.compute_path(currentNode, shared.graph.friendlyBases, 3,0);
        //path->theta_start = theta_pos; 

        decision->actionType = ReturnToBase; 
        return; 
    }


    if (in_array(shared.graph.plants, 6, currentNode)) {
        // If it's in a plant node, go back to a base 
        path = shared.graph.compute_path(currentNode, shared.graph.friendlyBases, 3,0);
        //path->theta_start = theta_pos; 

    } else {
        // Else, go to a random plant node 
        uint8_t target = shared.graph.plants[rand()%6]; // get random plant node
        path = shared.graph.compute_path(currentNode, &target, 1, 0);
    }
    decision->actionType = Displacement; 
    return; 
}