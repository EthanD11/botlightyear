#ifndef BLY_DYNAMIXELS_H
#define BLY_DYNAMIXELS_H

#include "colors.h"

typedef enum {
    Up, 
    Down,
    Mid,
} position_t;

typedef enum {
    CCW,
    CW,
} direction_t;

int dxl_init_port(); 
void dxl_close_port(); 

int dxl_ping(int ID, float PROTOCOL); 
void dxl_idle(int ID, float PROTOCOL);

int dxl_deploy(position_t position);
void dxl_multiturn(direction_t direction);
void dxl_position(double goal_pos); 
int dxl_turn(team_color_t team, double angle); 
void dxl_reset_sp(); 

void solar_panel(team_color_t team, double angle); 

#endif