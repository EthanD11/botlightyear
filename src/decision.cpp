#include "decision.h"
#include "action_displacement.h"
#include "action_planter.h"
#include "action_plants.h"
#include "action_pots.h"
#include "action_return.h"
#include "action_SP.h"
#include "action_zone.h"

#include <stdlib.h> // For random numbers
#include <stdbool.h>
#include <stdio.h>
#include <cmath>

#define TESTS

class ActionGameFinished : public Action {
    public :
        ActionGameFinished() : Action(GameFinished, false, NULL) {}
        void do_action () {} 
};

class ActionWait: public Action {
    public :
        ActionWait() : Action(Wait, false, NULL) {}
        void do_action () {
            usleep(5000);
        } 
};

class ActionTest : public Action {
    public :
        ActionTest() : Action(TestAction, false, NULL) {}
        void do_action () {} 
};

Action* possible_actions[10]; 
uint8_t n_possible_actions;

int8_t time_gotobase = 20; 


// ------------ UTILS --------------

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
// void make_decision(decision_t *decision) {


// Create the different actions that can be taken (in order !) with respect to the strategy
void decide_possible_actions() {

    int8_t current_time = shared.update_and_get_timer();
    double x_pos, y_pos, theta_pos, dist_from_currentNode; 
    shared.get_robot_pos(&x_pos, &y_pos, &theta_pos); 
    graph_path_t* path;
    x_pos += 0.15*cos(theta_pos);
    y_pos += 0.15*sin(theta_pos);
    uint8_t currentNode = shared.graph->identify_pos(x_pos, y_pos, &dist_from_currentNode);

    #ifdef TESTS
    uint8_t target = 16;
    path = shared.graph->compute_path(x_pos, y_pos, &target, 1);
    if (path != NULL) {
        path->thetaStart = theta_pos; 
        path->thetaEnd = -M_PI/2;  
    }
    
    possible_actions[0] = new ActionSP(path, 3, true); 
    n_possible_actions = 1; 
    return;
    #endif

    if (current_time < 0) {
        possible_actions[0] = new ActionGameFinished(); 
        n_possible_actions = 1; 
        return; 
    }

    // Second check if time is running out : 
    if (current_time < time_gotobase) {
        // Get the closest base from the robot pov
        path = shared.graph->compute_path(x_pos, y_pos, shared.graph->friendlyBases, 3);
        if (path != NULL) {
            path->thetaStart = theta_pos; 
            path->thetaEnd = getThetaEnd(shared.graph->friendlyBases, shared.graph->friendlyBasesTheta, 3, path->target); 
        }
        possible_actions[0] = new ActionBackToBase(path); 
        n_possible_actions = 1; 
        return; 
    }
    // Strategy : for now, only cycling between random plant nodes and going back to closest base

    if (in_array(shared.graph->plants, 6, currentNode)) {
        // If it's in a plant node, go back to a base 
        path = shared.graph->compute_path(x_pos, y_pos, shared.graph->friendlyBases, 3);
        if (path != NULL) {
            path->thetaStart = theta_pos; 
            path->thetaEnd = getThetaEnd(shared.graph->friendlyBases, shared.graph->friendlyBasesTheta, 3, path->target); 
        }
        possible_actions[0] = new ActionBackToBase(path); 
        n_possible_actions = 1; 

    } else {
        // Else, go to a random plant node 
        uint8_t target = shared.graph->plants[rand()%6]; // get random plant node
        path = shared.graph->compute_path(x_pos, y_pos, &target, 1);       
        if (path != NULL) {
            path->thetaStart = theta_pos; 
            path->thetaEnd = 0; 
        }
        
        possible_actions[0] = new ActionDisplacement(path); 
        n_possible_actions = 1; 
    }

}
bool check_validity(Action* action) {
    if (action->needs_path && action->path == NULL) return false; 
    for (uint8_t i=0; i<6; i++) {
        if (action->needs[i]==1 && shared.valids[i]==0) return false; 
    }
    return true; 
}
    
Action* make_decision() {
    decide_possible_actions(); 
    Action* selected_action = NULL; 

    for (uint8_t i=0; i<n_possible_actions; i++) {
        if (selected_action == NULL) {
            if (!check_validity(possible_actions[i])) {
                delete possible_actions[i]; 
            } else {
                selected_action = possible_actions[i];
            }
        } else {
            delete possible_actions[i]; 
        }
    }
    if (selected_action != NULL) {
        return selected_action;
    } else {
        return new ActionWait(); 
    }
    
}


