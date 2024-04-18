#ifndef BLY_ACTION_BACK_H
#define BLY_ACTION_BACK_H


#include "actions.h"


class ActionBACK : public Action { 
    private:
        double const safety_distance_to_walls = 0.2; // [m]
        double const safety_distance_to_pami_side_wall = 0.3; 
        double const dist_useless = 0.05;
        double backward_dist;
        bool too_close_to_wall(double x, double y);

    public: 

        /**
         * @brief: Botlightyear moves backward with a specified distance, it.
         * The action manages collisions with walls.
         * @param backward_dist The distance botlightyear will move backward
        */
        ActionBACK (double backward_dist) : Action(BackManoeuvre, false, NULL) {
            this->backward_dist = backward_dist;
        }
        ~ActionBACK() {};
        void do_action();
};

#endif