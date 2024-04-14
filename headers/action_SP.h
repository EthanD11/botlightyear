#ifndef BLY_ACTION_SP_H
#define BLY_ACTION_SP_H

#include "dynamixels.h"
#include "cameraTag.h"
#include "shared_variables.h"


/* SOLAR_PANEL_PC: Position Control for Solar Panels 
   x, y, theta: robot position at the start of the displacement action
   xa, ya, ta : robot position actuated by thread TopLidar (odometry)
   Position Control from one solar panel to another, while checking current position from desired one
   While displacement, reset dxl 8 (wheel) to init position for left-right turn */
void solar_panel_pc(); 

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