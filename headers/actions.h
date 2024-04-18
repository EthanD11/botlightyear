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
    BackManoeuvre,
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
        uint8_t needs[6]; 
        Action(action_t type, bool needsPath, graph_path_t* graph_path) {
            action_type = type;
            needs_path = needsPath;
            path = graph_path;
        }
        virtual ~Action() {}
};


/* UTILS: PATH_FOLLOWING_TO_ACTION */
int8_t path_following_to_action(graph_path_t *path); 

/* UTILS: POSITION_CONTROL */
int8_t action_position_control(double x_end, double y_end, double theta_end); 

/**
 * @brief Get the plate slot associated with the plate slot ID
 * @param slot the slot ID
 * @return the plate slot number associated to the stepper position of the plate
*/
int8_t get_plate_slot(storage_slot_t slotID); 

/**
 * @brief Get the next slot ID to unload from the plate
 * @param content the type of object it has to unload
 * @return the slot ID of the object to unload, and SlotInvalid = -1 if not found any
*/
storage_slot_t get_next_unloaded_slot_ID(storage_content_t content);

/**
 * @brief Get the next free slot ID to load to the plate
 * @param content the type of object it has to be free from (plant or pot)
 * @return the slot ID of the object to load, and SlotInvalid = -1 if not found any
*/
storage_slot_t get_next_free_slot_ID (storage_content_t content); 

/**
 * @brief Updates the content of the plate slot ID
 * @param slotID the id of the slot to update
 * @param content the type of object to update (ContainsNothing to reset slot)
*/
void update_plate_content(storage_slot_t slotID, storage_content_t content); 

/**
 * @brief Returns the angle in radians in between [-pi, pi]
*/
double periodic_angle(double angle); 

/**
 * @brief Returns the difference in radians theta_a - theta_b, while resolving discontinuity issues
 * Angles need to be in the range [-pi, pi]
*/
double trigo_diff(double theta_a, double theta_b);


#endif