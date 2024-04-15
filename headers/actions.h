#ifndef BLY_ACTIONS_H
#define BLY_ACTIONS_H

#include "action_displacement.h"
#include "action_planter.h"
#include "action_plants.h"
#include "action_pots.h"
#include "action_return.h"
#include "action_SP.h"
#include "action_zone.h"
#include <cmath>
//#include <unistd.h>

#define vref 0.25                 // [m/s] Speed reference for path following
#define dist_goal_reached 0.40    // [m] Distance tolerance to goal for path following


/* UTILS: PATH_FOLLOWING_TO_ACTION */
int8_t path_following_to_action(graph_path_t *path); 

/* UTILS: POSITION_CONTROL */
int8_t action_position_control(double x_end, double y_end, double theta_end); 


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

/* UTILS: CLOSER_IN_PATH */
uint8_t closer_in_path(graph_path_t *path, double xr, double yr); 

/* UTILS: ADVERSARY_IN_PATH */
uint8_t adversary_in_path(graph_path_t *path, uint8_t closer_node_id); 

#endif