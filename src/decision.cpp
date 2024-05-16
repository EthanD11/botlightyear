#include "decision.h"
#include "actions.h"
#include "action_displacement.h"
#include "action_clearpots.h"
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

// #define TESTS
#define PLANT_STRATEGY
// #define RANDOM
// #define SP_STRATEGY
// #define FINAL_STRATEGY

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
        void do_action () {
            shared.teensy->idle(); 
        } 
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
            /*/*double x = 0,y = 0;
            shared.get_robot_pos(&x,&y,NULL);
            uint8_t node = shared.graph->identify_pos(x,y,NULL);
            printf("Current node : %d with level %d\n", node, shared.graph->nodes[node].level);*/
            int8_t remaining_time = shared.update_and_get_timer();
            printf("Waiting... Time = %d\n", remaining_time); 
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

class ActionBlockSps : public Action {
    public :
        ActionBlockSps() : Action(BlockSPs, false, NULL) {
            this->needs[0] = 0;  // SptrPlate
            this->needs[1] = 0;  // StprSlider
            this->needs[2] = 0;  // StprFlaps
            this->needs[3] = 0;  // Dxls
            this->needs[4] = 0;  // LidarBottom
        }
        void do_action () {
            if (action_position_control(shared.graph->nodes[26].x, shared.graph->nodes[26].y, -M_PI_2) == -1) return; 
            shared.spBlockDone = 1; 
        } 
};




Action* possible_actions[10]; 
uint8_t n_possible_actions;

uint8_t plantZonesValid[6]; 
uint8_t plantZonesCount = 0;

#ifdef SP_STRATEGY
int16_t time_gotobase = 20;
int16_t time_sp_reserved = 50;
int16_t time_sp = 1000; 

#endif

#ifdef FINAL_STRATEGY
static bool hasClearedPots = false; 
static bool hasDonePlanters = false; 
int16_t time_gotobase = 20;
int16_t time_sp_reserved = 30;
int16_t time_sp = 50; 
#endif 

#ifdef PLANT_STRATEGY
static bool hasTakenPots = false; 
static bool hasDonePlanters = false; 
int16_t time_gotobase = 15;
int16_t time_sp = -1;
int16_t time_sp_reserved = -1;
int16_t time_pot = 20; 
#endif

#ifdef TESTS
static bool hasTakenPots = false; 
int16_t time_gotobase = -100;
#endif

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

void update_valid_plant_zones() {
    plantZonesCount = 0; 
    for (uint8_t i = 0; i<6; i++) {
        if (shared.plantCounts[i] != 0) {
            plantZonesValid[plantZonesCount] = shared.graph->plants[i]; 
            plantZonesCount +=1; 
        }
    }
}
uint8_t get_plantZoneIdx(uint8_t targetPlantZone) {
    for (uint8_t i = 0; i<6; i++) {
        if (shared.graph->plants[i] == targetPlantZone) {
            return i; 
        }
    }
    printf("Error : invalid plant zone target node !\n"); 
    return 0;
}
// void make_decision(decision_t *decision) {


// Create the different actions that can be taken (in order !) with respect to the strategy
void decide_possible_actions() {
    printf("----- Deciding action -----\n\n");
    n_possible_actions = 0; 
    int16_t remaining_time = shared.update_and_get_timer();
    printf("remaining_time = %d\n", remaining_time);
    double x_pos = 0, y_pos = 0, theta_pos = 0, dist_from_currentNode = 0; 
    shared.get_robot_pos(&x_pos, &y_pos, &theta_pos); 
    graph_path_t* path;
    x_pos += 0.15*cos(theta_pos);
    y_pos += 0.15*sin(theta_pos);
    uint8_t currentNode = shared.graph->identify_pos(x_pos, y_pos, &dist_from_currentNode);

    // -----------------------------------------------------------------------------------------
    // --------------------------------- TESTS -------------------------------------------------
    // -----------------------------------------------------------------------------------------


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
    
    // possible_actions[0] = new ActionSP(path, 3, false, Forward); 
    // n_possible_actions = 1; 
    // return;

    // ---------- Pots TEST -----------
    // if (!hasTakenPots) {
    //     path = shared.graph->compute_path(x_pos, y_pos, shared.graph->pots, 6);
    
    //     if (path != NULL) {
    //         path->thetaStart = theta_pos; 
    //         path->thetaEnd = getThetaEnd(shared.graph->pots, shared.graph->potsTheta, 6, path->target);
    //     } else {
    //         printf("Path is NULL\n");
    //     }
    //     possible_actions[0] = new ActionPots(path, 2,false,true); 
    //     n_possible_actions = 1; 
    //     hasTakenPots = true; 
    //     return;
    // }
    // ---------- Plants TEST -----------

    uint8_t target = 31;
    
    shared.graph->update_obstacle(target,0);
    path = shared.graph->compute_path(x_pos, y_pos, &target, 1);
    int whichZone = 0;
    //shared.graph->update_obstacle(27,1);
    if (path != NULL) {
        path->thetaStart = theta_pos; 
        path->thetaEnd = 0; // Angle is recomputed in Action Plants  
        whichZone = get_plantZoneIdx(path->target);
    } else {
        printf("Path is NULL\n");
    }
    // shared.graph->update_obstacle(target,1);

    possible_actions[0] = new ActionPlants(path, 6, whichZone); 
    n_possible_actions = 1; 
    return;

    // ---------- Planters TEST -----------
    
    
    // path = shared.graph->compute_path(x_pos, y_pos, shared.graph->friendlyPlanters, 3);
    // update_plate_content(SlotM3, ContainsWeakPlant); 
    // update_plate_content(Slot3, ContainsWeakPlant); 

    // if (path != NULL) {
    //     path->thetaStart = theta_pos; 
    //     path->thetaEnd = getThetaEnd(shared.graph->friendlyPlanters, shared.graph->friendlyPlantersTheta, 3, path->target);  
    // } else {
    //     printf("Path is NULL\n");
    // }
    // // shared.graph->update_obstacle(target,1);
    // possible_actions[0] = new ActionPlanter(path, 3, SideRight, 0, SideMiddle); 
    // n_possible_actions = 1; 
    // return;
    
    #endif   

    #ifdef HOMOLOGATION 
    uint8_t target = shared.graph->friendlyBases[2];
    if (shared.graph->identify_pos(x_pos, y_pos, NULL) == target) {
        printf("In position !\n"); 
        possible_actions[0] = new ActionGameFinished(); 
        n_possible_actions = 1; 
        return;
    }
    path = shared.graph->compute_path(x_pos, y_pos, &target, 1);
    if (path == NULL) {
        printf("No path \n");
        n_possible_actions = 0;
        return;
    }
    path->thetaStart = theta_pos; 
    path->thetaEnd = shared.graph->friendlyBasesTheta[2];  
    possible_actions[0] = new ActionDisplacement(path); 
    n_possible_actions = 1; 
    printf("New path \n");
    return;
    #endif

    if (remaining_time <= 1) {
        possible_actions[0] = new ActionGameFinished(); 
        n_possible_actions = 1; 
        return; 
    }

    // Second check if time is running out : 

    if (shared.backToBaseDone==1) { // Check to not cycle endlessly to back to base, goes into idle mode and waits the timer
        n_possible_actions = 0; 
        return; 
    }
    if (remaining_time < time_gotobase) {
        printf("GOES TO BASE \n");

        // Get the closest base from the robot pov
        // printf("Searching for bases %d %d %d\n", shared.graph->friendlyBases[0], shared.graph->friendlyBases[1], shared.graph->friendlyBases[2]);
        // printf("Bases levels %d %d %d\n", shared.graph->nodes[shared.graph->friendlyBases[0]].level, shared.graph->nodes[shared.graph->friendlyBases[1]].level, shared.graph->nodes[shared.graph->friendlyBases[2]].level);
        shared.graph->level = 1;
        path = shared.graph->compute_path(x_pos, y_pos, shared.graph->friendlyBases+1, 2);
        if (path != NULL) {
            path->thetaStart = theta_pos;  
            #ifdef SP_STRATEGY
            path->thetaEnd = -M_PI/2.0;
            #else
            path->thetaEnd = getThetaEnd(shared.graph->friendlyBases, shared.graph->friendlyBasesTheta, 3, path->target);
            #endif
        }
        possible_actions[0] = new ActionBackToBase(path); 
        n_possible_actions = 1; 
        return; 
    }

    // -----------------------------------------------------------------------------------------
    // --------------------------------- SP STRATEGY ----------------------------------------
    // -----------------------------------------------------------------------------------------

    #ifdef SP_STRATEGY
    // ----------- ENDGAME -----------------
    // time_sp_reserved = 40; 
    // if (shared.SPsDone[1]==0) {
    //     printf("DOES SP RESERVED \n");
    //     if (shared.color == TeamBlue) {
    //         shared.graph->update_obstacle(41,1);
    //         uint8_t target = 39;
    //         path = shared.graph->compute_path(x_pos, y_pos, &target, 1);
    //         shared.graph->update_obstacle(41,0);
    //     } else {
    //         shared.graph->update_obstacle(7,1);
    //         uint8_t target = 6;
    //         path = shared.graph->compute_path(x_pos, y_pos, &target, 1);
    //         shared.graph->update_obstacle(7,0);
    //     }
    //     double xa = 0, ya = 0;
    //     shared.get_adv_pos(&xa,&ya,NULL,NULL);
    //     printf("Adversary at %d %d\n", xa, ya);
    //     if (path != NULL) {
    //         path->thetaStart = theta_pos; 
    //         path->thetaEnd = -M_PI_2; 
    //     }
    //     possible_actions[n_possible_actions] = new ActionSP(path, 3, true, Forward); //(shared.color == TeamBlue) ? Forward : Backward 
    //     n_possible_actions++;

    // } else {
    if(shared.SPsDone[0]==0) {
        printf("TRIES SP COMMON \n");
        shared.graph->update_obstacle(27,1);
        shared.graph->update_obstacle(25,1);
        uint8_t target = 26;
        path = shared.graph->compute_path(x_pos, y_pos, &target, 1);
        if (path != NULL) {
            path->thetaStart = theta_pos; 
            path->thetaEnd = -M_PI_2; 
        }
        shared.graph->update_obstacle(27,0);
        shared.graph->update_obstacle(25,0);

        possible_actions[n_possible_actions] = new ActionSP(path, 3, false, Forward); 
        n_possible_actions++;
    }
    // } else {
    //     if (shared.spBlockDone == 0) {
    //         printf("DOES SP BLOCK \n");
    //         possible_actions[0] = new ActionBlockSps(); 
    //         n_possible_actions = 1;
    //     } else {
    //         n_possible_actions = 0;
    //     } 
    //     return;
    // }
        

    if (shared.SPsDone[1]==0) {
        printf("TRIES SP RESERVED \n");
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
        possible_actions[n_possible_actions] = new ActionSP(path, 3, true, Forward); //(shared.color == TeamBlue) ? Forward : Backward
        n_possible_actions++;
    }
        



    return;
    #endif
    #ifdef RANDOM
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
    return; 
    #endif
    // -----------------------------------------------------------------------------------------
    // --------------------------------- PLANT STRATEGY ----------------------------------------
    // -----------------------------------------------------------------------------------------

    #ifdef PLANT_STRATEGY
    printf("Remaining time %d\n", remaining_time);
    if (!hasDonePlanters && (shared.plantersDone[0] !=0) && (shared.plantersDone[1] !=0)) {
        hasDonePlanters = true; 
    }

    if ((remaining_time < time_sp) && hasDonePlanters) { //Time to switch to solar panels OR everything with plants is already done
        
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
            possible_actions[n_possible_actions] = new ActionSP(path, 3, true, Forward); //(shared.color == TeamBlue) ? Forward : Backward 
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
                possible_actions[n_possible_actions] = new ActionSP(path, 3, true, Forward); //(shared.color == TeamBlue) ? Forward : Backward
                n_possible_actions++;
            }
            
        }
        return;
    }



    // --------- EARLY GAME -------------
    if (!hasTakenPots && get_content_count(ContainsPot) !=0) {
        hasTakenPots = true; 
    }
    
    if (!hasTakenPots) {
        path = shared.graph->compute_path(x_pos, y_pos, shared.graph->pots, 6);
        if (path != NULL) {
            path->thetaStart = theta_pos; 
            path->thetaEnd = getThetaEnd(shared.graph->pots, shared.graph->potsTheta, 6, path->target);
        } else {
            printf("Path is NULL\n");
        }
        possible_actions[n_possible_actions] = new ActionPots(path, 2,false,true); 
        n_possible_actions ++;
    }

    int8_t current_plant_count = get_content_count(ContainsWeakPlant); 
    if (current_plant_count ==0) { // Go take plants quicc
        uint8_t plantZoneIdx=0; 
        update_valid_plant_zones(); 
        for(uint8_t i = 0; i<6; i++) {
            shared.graph->update_obstacle(shared.graph->plants[i], 0); 
        }

        path = shared.graph->compute_path(shared.graph->nodes[shared.graph->friendlyBases[0]].x, 
                                            shared.graph->nodes[shared.graph->friendlyBases[0]].y, 
                                            plantZonesValid, plantZonesCount);
        if (path !=NULL) {
            uint8_t target = path->target;
            free(path); 
            path = shared.graph->compute_path(x_pos, y_pos, &target, 1); 
            if (path != NULL) {
                path->thetaStart = theta_pos; 
                path->thetaEnd = 0; // gets updated within ActionPlants anyways, no need to worry about it :)
                plantZoneIdx = get_plantZoneIdx(path->target); 
            }
        }
        
        possible_actions[n_possible_actions] = new ActionPlants(path,3, plantZoneIdx); // Plant number "could" be modulated with time 
        n_possible_actions++; 

        for(uint8_t i = 0; i<plantZonesCount; i++) {
            shared.graph->update_obstacle(plantZonesValid[i], 1); 
        }
        return; 
    } else {

        if (shared.plantersDone[0] == 0) {
            // Reserved planter action if not done yet
            path = shared.graph->compute_path(x_pos, y_pos, &shared.graph->friendlyPlanters[0], 1); 
            if (path != NULL) {
                path->thetaStart = theta_pos; 
                path->thetaEnd = shared.graph->friendlyPlantersTheta[0];
            }
            // possible_actions[n_possible_actions] = new ActionPlanter(path, std::max((current_plant_count-2)/2,1), SideMiddle, SideMiddle);
            possible_actions[n_possible_actions] = new ActionPlanter(path, 1, SideMiddle, 0, SideMiddle);
            
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
            planter_side_t pot_clear_side = SideMiddle; //(shared.color==TeamBlue) ? SideRight : SideLeft; 
            // possible_actions[n_possible_actions] = new ActionPlanter(path, std::max((current_plant_count-2)/2,1), planter_side, pot_clear_side);
            possible_actions[n_possible_actions] = new ActionPlanter(path, 1, planter_side, 1, pot_clear_side);
            n_possible_actions++;
        }
    }

    for (int i=0; i < 2; i++) {
        printf("plantersDone[%d] = %d\n", i, shared.plantersDone[i]);
    }

    return;
    #endif

    // -----------------------------------------------------------------------------------------
    // --------------------------------- FINAL STRATEGY ----------------------------------------
    // -----------------------------------------------------------------------------------------

    #ifdef FINAL_STRATEGY 
    if (!hasDonePlanters && (shared.plantersDone[0] !=0) && (shared.plantersDone[1] !=0)) {
        hasDonePlanters = true; 
    }

    if ((remaining_time < time_sp) || hasDonePlanters) { //Time to switch to solar panels OR everything with plants is already done
        
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
            possible_actions[n_possible_actions] = new ActionSP(path, 3, true, Forward); //(shared.color == TeamBlue) ? Forward : Backward 
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
                possible_actions[n_possible_actions] = new ActionSP(path, 3, true, Forward); //(shared.color == TeamBlue) ? Forward : Backward
                n_possible_actions++;
            }
            
        }
        return;
    }



    // --------- EARLY GAME -------------
    
    if (!hasClearedPots) {
        path = shared.graph->compute_path(x_pos, y_pos, shared.graph->pots, 6);
        if (path != NULL) {
            path->thetaStart = theta_pos; 
            path->thetaEnd = getThetaEnd(shared.graph->pots, shared.graph->potsTheta, 6, path->target);
        } else {
            printf("Path is NULL\n");
        }
        possible_actions[n_possible_actions] = new ActionClearPots(NULL);  
        n_possible_actions ++;
        hasClearedPots = true;
        return;
    }

    int8_t current_plant_count = get_content_count(ContainsWeakPlant); 
    if (current_plant_count ==0) { // Go take plants quicc
        uint8_t plantZoneIdx=0; 
        update_valid_plant_zones(); 
        for(uint8_t i = 0; i<6; i++) {
            shared.graph->update_obstacle(shared.graph->plants[i], 0); 
        }

        path = shared.graph->compute_path(shared.graph->nodes[shared.graph->friendlyBases[0]].x, 
                                            shared.graph->nodes[shared.graph->friendlyBases[0]].y, 
                                            plantZonesValid, plantZonesCount);
        if (path !=NULL) {
            uint8_t target = path->target;
            free(path); 
            path = shared.graph->compute_path(x_pos, y_pos, &target, 1); 
            if (path != NULL) {
                path->thetaStart = theta_pos; 
                path->thetaEnd = 0; // gets updated within ActionPlants anyways, no need to worry about it :)
                plantZoneIdx = get_plantZoneIdx(path->target); 
            }
        }
        
        possible_actions[n_possible_actions] = new ActionPlants(path, 2, plantZoneIdx); // Plant number "could" be modulated with time 
        n_possible_actions++; 

        for(uint8_t i = 0; i<plantZonesCount; i++) {
            shared.graph->update_obstacle(plantZonesValid[i], 1); 
        }
        return; 
    } else {

        if (shared.plantersDone[0] == 0) {
            // Reserved planter action if not done yet
            path = shared.graph->compute_path(x_pos, y_pos, &shared.graph->friendlyPlanters[0], 1); 
            if (path != NULL) {
                path->thetaStart = theta_pos; 
                path->thetaEnd = shared.graph->friendlyPlantersTheta[0];
            }
            // possible_actions[n_possible_actions] = new ActionPlanter(path, std::max((current_plant_count-2)/2,1), SideMiddle, SideMiddle);
            possible_actions[n_possible_actions] = new ActionPlanter(path, 1, SideMiddle, 0, SideMiddle);
            
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
            planter_side_t pot_clear_side = SideMiddle; //(shared.color==TeamBlue) ? SideRight : SideLeft; 
            // possible_actions[n_possible_actions] = new ActionPlanter(path, std::max((current_plant_count-2)/2,1), planter_side, pot_clear_side);
            possible_actions[n_possible_actions] = new ActionPlanter(path, 1, planter_side, 1, pot_clear_side);
            n_possible_actions++;
        }
    }

    return;
    #endif

}
bool check_validity(Action* action) {

    if (action->needs_path && action->path == NULL) {
        printf("Invalid path for action : %d\n", action->action_type); 

        return false; 
    }
    printf("Validities = "); 
    for (uint8_t i=0; i<5; i++) {
        printf("%d ", shared.valids[i]);
        if (action->needs[i]==1 && shared.valids[i]==0) {
            printf("\n Invalid need : %d for action : %d\n", i, action->action_type); 
            return false; 
        }
    }
    printf("\n");
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
                printf("Deciding to do choice %d \n", i);
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


