#ifndef BLY_ACTION_BACK_H
#define BLY_ACTION_BACK_H


#include "actions.h"


class ActionBACK : public Action { 
    private:
        double xnew, ynew, thetanew;
        double const safety_distance_to_walls = 0.2; // [m]
        double const safety_distance_to_pami_side_wall = 0.3; 
        double const dist_useless = 0.05;
        bool manoeuver_useless;
        bool too_close_to_wall(double x, double y) {
            if ((x < safety_distance_to_pami_side_wall) ||
                (y < safety_distance_to_walls) ||
                (2.0 - x < safety_distance_to_walls) ||
                (3.0 - y < safety_distance_to_walls)
                ) {
                return true;
            }
            return false;
        }

    public: 

        /**
         * @brief: Botlightyear moves backward with a specified distance, it
         * can also re-orient itself with a specified angular displacement. The action
         * manages collisions with walls.
         * @param backward_dist The distance botlightyear will move backward
         * @param angular_displacement: The angular displacement that botlightyear will realise during the back manoeuver
        */
        ActionBACK (double backward_dist) : Action(BackManoeuvre, false, NULL) { 
            // 1) Get position
            // 2) While new position too close to wall, reduce backward dist
            double xpos, ypos, thetapos;
            shared.get_robot_pos(&xpos, &ypos, &thetapos);

            double xnew, ynew, thetanew;
            xnew = xpos - cos(thetapos)*backward_dist;
            ynew = ypos - sin(thetapos)*backward_dist;
            thetanew = thetapos;

            while (this->too_close_to_wall(xnew, ynew) && (backward_dist > dist_useless)) {
                backward_dist = backward_dist - 0.01;
                xnew = xpos - cos(thetapos)*backward_dist;
                ynew = ypos - sin(thetapos)*backward_dist;
            }

            if (backward_dist <= dist_useless) {
                this->manoeuver_useless = true;
            } else {
                this->manoeuver_useless = false;
            }


        }
        ~ActionBACK() {};
        void do_action();
};

#endif