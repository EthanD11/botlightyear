#ifndef BLY_ACTION_SP_H
#define BLY_ACTION_SP_H

#include "dynamixels.h"
#include "cameraTag.h"
#include "shared_variables.h"


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


#endif