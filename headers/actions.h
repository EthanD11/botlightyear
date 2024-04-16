#ifndef BLY_ACTIONS_H
#define BLY_ACTIONS_H

#include "shared_variables.h"
#include <cmath>
#include <unistd.h>

#define vref 0.25                 // [m/s] Speed reference for path following
#define dist_goal_reached 0.40    // [m] Distance tolerance to goal for path following

typedef enum _action : uint8_t
{
    GameFinished,
    ReturnToBase,
    Displacement,
    TakePlants,
    TakePots,
    TurnSP,
    DepositZone,
    DepositPlanter, 
    TestAction, 
    Wait
} action_t;

class Action {
        
    public: 
        virtual void do_action() = 0; 
        action_t action_type; 
        bool needs_path; 
        graph_path_t* path; 
        Action(action_t type, bool needsPath, graph_path_t* graph_path) {
            action_type = type;
            needs_path = needsPath;
            path = graph_path;
        }
};


/* UTILS: PATH_FOLLOWING_TO_ACTION */
int8_t path_following_to_action(graph_path_t *path); 

/* UTILS: POSITION_CONTROL */
int8_t action_position_control(double x_end, double y_end, double theta_end); 

/* UTILS: GET PLATE SLOT*/
int8_t get_plate_slot_ID(storage_slot_t slot); 

int8_t get_next_unloaded_plate_slot (storage_content_t content);

int8_t get_next_free_plate_slot (storage_content_t content); 
/* UTILS: CLOSER_IN_PATH
uint8_t closer_in_path(graph_path_t *path, double xr, double yr); 

/* UTILS: ADVERSARY_IN_PATH
uint8_t adversary_in_path(graph_path_t *path, uint8_t closer_node_id); */

#endif