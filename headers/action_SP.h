#ifndef BLY_ACTION_SP_H
#define BLY_ACTION_SP_H

#include "dynamixels.h"
#include "cameraTag.h"
#include "shared_variables.h"
#include "actions.h"

void solar_panel_pc(); 

//void positionCtrlIterative(); 

void turn_solar_panel(bool reserved, uint8_t sp_counter);

/* TURN_SOLAR_PANEL_RESERVED: Action sequence to turn one or more solar panels, reserved (no angle estimation)
   uint8_t sp_counter: number of solar panels to turn */
void turn_solar_panel_reserved(uint8_t sp_counter);

/* TURN_SOLAR_PANEL_PUBLIC: Action sequence to turn one or more solar panels, public (angle estimation through camera thread)
   uint8_t sp_counter: number of solar panels to turn */
void turn_solar_panel_public(uint8_t sp_counter); 

/* TURN_SOLAR_PANEL_PUBLIC: Action sequence to turn one or more solar panels, public and reserved (angle estimation through camera thread)
   uint8_t sp_counter: number of solar panels to turn */
void turn_solar_panel_all(uint8_t sp_counter); 


class ActionSP : public Action {
    private: 
        uint8_t sp_counter;
        bool reserved;
    public: 
        void do_action (); 
        ActionSP (graph_path_t* graph_path, uint8_t sp_number, bool is_reserved) : Action(TurnSP, true, graph_path) { 
            sp_counter = sp_number;
            reserved = is_reserved;
        }
};

#endif