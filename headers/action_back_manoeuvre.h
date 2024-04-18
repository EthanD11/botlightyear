#ifndef BLY_ACTION_BACK_H
#define BLY_ACTION_BACK_H


#include "actions.h"


class ActionBACK : public Action { 
    private:
        double xnew, ynew, thetanew; 
    public: 

        /**
         * @brief: Botlightyear moves backward with a specified distance, it
         * can also re-orient itself with a specified angular displacement. The action
         * manages collisions with walls.
         * @param backward_dist The distance botlightyear will move backward
         * @param angular_displacement: The angular displacement that botlightyear will realise during the back manoeuver
        */
        ActionBACK (double backward_dist, double angular_displacement) : Action(BackManoeuvre, false, NULL) { 
            // Compute position from robot current position ang specified angular displacement
            double xpos, ypos, thetapos;
            shared.get_robot_pos(&xpos, &ypos, &thetapos);
            this->xnew = xpos - cos(thetapos)*backward_dist;
            this->ynew = ypos - sin(thetapos)*backward_dist;
            this->thetanew = thetapos + angular_displacement;
        }
        ~ActionBACK() {};
        void do_action();
};

#endif