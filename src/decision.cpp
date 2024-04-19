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
#include <algorithm>

#define TESTS

//#define FINAL_STRATEGY
//#define HOMOLOGATION

class ActionGameFinished : public Action {
    public :
        ActionGameFinished() : Action(GameFinished, false, NULL) {
            this->needs[0] = 0;  // SptrPlate
            this->needs[1] = 0;  // StprSlider
            this->needs[2] = 0;  // StprFlaps
            this->needs[3] = 0;  // Dxls
            this->needs[4] = 0;  // LidarBottom
        }
        void do_action () {} 
};

class ActionWait: public Action {
    public :
        ActionWait() : Action(Wait, false, NULL) {
            this->needs[0] = 0;  // SptrPlate
            this->needs[1] = 0;  // StprSlider
            this->needs[2] = 0;  // StprFlaps
            this->needs[3] = 0;  // Dxls
            this->needs[4] = 0;  // LidarBottom
        }
        void do_action () {
            double x = 0,y = 0;
            shared.get_robot_pos(&x,&y,NULL);
            uint8_t node = shared.graph->identify_pos(x,y,NULL);
            printf("Current node : %d with level %d\n", node, shared.graph->nodes[node].level);
            usleep(300000);
        } 
};

class ActionTest : public Action {
    public :
        ActionTest() : Action(TestAction, false, NULL) {
            this->needs[0] = 0;  // SptrPlate
            this->needs[1] = 0;  // StprSlider
            this->needs[2] = 0;  // StprFlaps
            this->needs[3] = 0;  // Dxls
            this->needs[4] = 0;  // LidarBottom
        }
        void do_action () {} 
};

Action* possible_actions[10]; 
uint8_t n_possible_actions;

int8_t time_gotobase = 15; 
int8_t time_sp = 40; 
int8_t time_sp_reserved = 25;


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
    n_possible_actions = 0; 
    int8_t remaining_time = shared.update_and_get_timer();
    double x_pos = 0, y_pos = 0, theta_pos = 0, dist_from_currentNode = 0; 
    shared.get_robot_pos(&x_pos, &y_pos, &theta_pos); 
    graph_path_t* path;
    x_pos += 0.15*cos(theta_pos);
    y_pos += 0.15*sin(theta_pos);
    uint8_t currentNode = shared.graph->identify_pos(x_pos, y_pos, &dist_from_currentNode);

    #ifdef TESTS
    /*static uint8_t base = 0;
    uint8_t target;
    if (base) target = shared.graph->friendlyBases[rand()%3];
    else target = shared.graph->plants[rand()%6];
    base = !base; 
    path = shared.graph->compute_path(x_pos, y_pos, &target, 1);
    if (path != NULL) {
        path->thetaStart = theta_pos; 
        path->thetaEnd = -M_PI/2;  
    }
    
    possible_actions[0] = new ActionDisplacement(path); */

    // ---------- SP TEST -----------
    // uint8_t target = 26;
    // shared.graph->update_obstacle(27,1);
    // path = shared.graph->compute_path(x_pos, y_pos, &target, 1);
    // shared.graph->update_obstacle(27,0);
    // if (path != NULL) {
    //     path->thetaStart = theta_pos; 
    //     path->thetaEnd = -M_PI_2;  
    // }
    
    // possible_actions[0] = new ActionSP(path, 3, true, Forward); 
    // n_possible_actions = 1; 

    // ---------- Plants TEST -----------

    uint8_t target = 31;
    shared.graph->update_obstacle(31,0);
    path = shared.graph->compute_path(x_pos, y_pos, &target, 1);
    //shared.graph->update_obstacle(27,1);
    if (path != NULL) {
        path->thetaStart = theta_pos; 
        path->thetaEnd = 0;  
    }
    shared.graph->update_obstacle(31,1);
    possible_actions[0] = new ActionPlants(path, 1); 
    n_possible_actions = 1; 
    return;
    #endif

    #ifdef HOMOLOGATION 
    if (shared.graph->identify_pos(x_pos, y_pos, NULL) == 0) {
        possible_actions[0] = new ActionGameFinished(); 
        n_possible_actions = 1; 
        return;
    }
    uint8_t target = 0;
    path = shared.graph->compute_path(x_pos, y_pos, &target, 1);
    if (path == NULL) {
        n_possible_actions = 0;
        return;
    }
    path->thetaStart = theta_pos; 
    path->thetaEnd = M_PI/2;  
    possible_actions[0] = new ActionDisplacement(path); 
    n_possible_actions = 1; 
    return;
    #endif

    if (remaining_time < 0) {
        possible_actions[0] = new ActionGameFinished(); 
        n_possible_actions = 1; 
        return; 
    }
    
    double xPos = 0, yPos = 0, thetaPos = 0;
    shared.get_robot_pos(&xPos, &yPos, &thetaPos);

    // Second check if time is running out : 
    if (remaining_time < time_gotobase) {
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
    #ifndef FINAL_STRATEGY
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

    #else 
    // ----------- ENDGAME -----------------
    if (remaining_time < time_sp) { //Time to switch to solar panels
        
        if ((remaining_time < time_sp_reserved) && (shared.SPsDone[1]==0)) {
            if (shared.color == TeamBlue) {
                shared.graph->update_obstacle(41,1);
                uint8_t target = 39;
                path = shared.graph->compute_path(x_pos, y_pos, &target, 1);
                shared.graph->update_obstacle(41,0);
            } else {
                shared.graph->update_obstacle(7,1);
                uint8_t target = 6;
                path = shared.graph->compute_path(x_pos, y_pos, &target, 1);
                
                shared.graph->update_obstacle(7,0);
            }
            if (path != NULL) {
                path->thetaStart = theta_pos; 
                path->thetaEnd = -M_PI_2; 
            }
            possible_actions[n_possible_actions] = new ActionSP(path, 3, true, (shared.color == TeamBlue) ? Forward : Backward); 
            n_possible_actions++;

        } else {
            if(shared.SPsDone[0]==0) {
                shared.graph->update_obstacle(27,1);
                uint8_t target = 26;
                path = shared.graph->compute_path(x_pos, y_pos, &target, 1);
                if (path != NULL) {
                    path->thetaStart = theta_pos; 
                    path->thetaEnd = -M_PI_2; 
                }
                shared.graph->update_obstacle(27,0);
        
                possible_actions[n_possible_actions] = new ActionSP(path, 3, false, Forward); 
                n_possible_actions++;
            }
            

            if (shared.SPsDone[1]==0) {
                if (shared.color == TeamBlue) {
                    shared.graph->update_obstacle(41,1);
                    uint8_t target = 39;
                    path = shared.graph->compute_path(x_pos, y_pos, &target, 1);
                    shared.graph->update_obstacle(41,0);
                } else {
                    shared.graph->update_obstacle(7,1);
                    uint8_t target = 6;
                    path = shared.graph->compute_path(x_pos, y_pos, &target, 1);
                    shared.graph->update_obstacle(7,0);
                }
                if (path != NULL) {
                    path->thetaStart = theta_pos; 
                    path->thetaEnd = -M_PI_2; 
                }
                possible_actions[n_possible_actions] = new ActionSP(path, 3, true, (shared.color == TeamBlue) ? Forward : Backward); 
                n_possible_actions++;
            }
            
        }
        
    }

    // --------- EARLY GAME -------------
    // Early part of the game : actions with plants
    int8_t current_plant_count = get_content_count(ContainsWeakPlant); 
    if (current_plant_count ==0) { // Go take plants quicc
        for(uint8_t i = 0; i<6; i++) {
            shared.graph->update_obstacle(shared.graph->plants[i], 0); 
        }
        path = shared.graph->compute_path(x_pos, y_pos, shared.graph->plants, 6); 
        if (path != NULL) {
            path->thetaStart = theta_pos; 
            path->thetaEnd = 0; // gets updated within ActionPlants anyways, no need to worry about it :)
        }
        possible_actions[n_possible_actions] = new ActionPlants(path,6); // Plant number "could" be modulated with time 
        n_possible_actions++; 


        for(uint8_t i = 0; i<6; i++) {
            shared.graph->update_obstacle(shared.graph->plants[i], 1); 
        }
        return; 
    } else {
        uint8_t i=0; 
        if (shared.plantersDone[0] == 0) {
            // Reserved planter action if not done yet
            path = shared.graph->compute_path(x_pos, y_pos, &shared.graph->friendlyPlanters[0], 1); 
            if (path != NULL) {
                path->thetaStart = theta_pos; 
                path->thetaEnd = shared.graph->friendlyPlantersTheta[0];
            }
            possible_actions[n_possible_actions] = new ActionPlanter(path, std::max((current_plant_count-2)/2,1), SideMiddle, SideMiddle);
            n_possible_actions++;
        }
        if (shared.plantersDone[1] == 0) {
            // Second planter action if not done yet
            path = shared.graph->compute_path(x_pos, y_pos, &shared.graph->friendlyPlanters[1], 1); 
            if (path != NULL) {
                path->thetaStart = theta_pos; 
                path->thetaEnd = shared.graph->friendlyPlantersTheta[1];
            }
            planter_side_t planter_side = (shared.color==TeamBlue) ? SideRight : SideLeft; 
            planter_side_t pot_clear_side = (shared.color==TeamBlue) ? SideRight : SideLeft; 
            possible_actions[n_possible_actions] = new ActionPlanter(path, std::max((current_plant_count-2)/2,1), planter_side, pot_clear_side);
            n_possible_actions++;
        }
        if (shared.zonesDone[0] == 0) {
            // Reserved zone action if not done yet
            path = shared.graph->compute_path(x_pos, y_pos, &shared.graph->friendlyBases[0], 1); 
            if (path != NULL) {
                path->thetaStart = theta_pos; 
                path->thetaEnd = shared.graph->friendlyBases[0]; 
            }
            possible_actions[n_possible_actions] = new ActionZone(path, 1);
            n_possible_actions++;
        }
    }

    #endif

}
bool check_validity(Action* action) {

    if (action->needs_path && action->path == NULL) {
        printf("Invalid path !\n "); 

        return false; 
    }
    // printf("Validities = "); 
    // for (uint8_t i=0; i<5; i++) {
    //     printf("%d \n", shared.valids[i]);
    //     if (action->needs[i]==1 && shared.valids[i]==0) {
    //         printf("Invalid need : %d \n", i); 
    //         return false; 
    //     }
    // }
    // printf("\n");
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


