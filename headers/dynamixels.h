#ifndef BLY_DYNAMIXELS_H
#define BLY_DYNAMIXELS_H

typedef enum {
    Up, 
    Down,
    Mid,
} position_t;

typedef enum {
    CCW,
    CW,
} direction_t;

typedef enum {
    Blue, 
    Yellow,
} team_t; 

void dxl_init_port(); 
void dxl_close_port(); 

void dxl_ping(int ID, float PROTOCOL); 
void dxl_idle(int ID, float PROTOCOL);

void dxl_deploy(position_t position);
void dxl_multiturn(direction_t direction);
void dxl_position(double goal_pos); 
void dxl_turn(team_t team, double angle); 
void dxl_init_sp(); 

void solar_panel(team_t team, double angle); 

#endif