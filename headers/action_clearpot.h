#ifndef BLY_ACT_CLEARPOT_H
#define BLY_ACT_CLEARPOT_H

#include "actions.h"


class ActionClearPot : public Action {
    private:
        double x_end; 
        double y_end; 
        double theta_end; 
    public: 
        ActionClearPot (double x_end, double y_end, double theta_end) : Action(ClearPot, true, graph_path) {
            this->needs[0] = 0;  // SptrPlate
            this->needs[1] = 0;  // StprSlider
            this->needs[2] = 0;  // StprFlaps
            this->needs[3] = 0;  // Dxls
            this->needs[4] = 0;  // LidarBottom
            this->x_end = x_end; 
            this->y_end = y_end;
            this->theta_end = theta_end;
        }
        ~ActionClearPot() {}
        void do_action();
};

#endif