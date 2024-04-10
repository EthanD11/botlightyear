#ifndef BLY_DYNAMIXELS_H
#define BLY_DYNAMIXELS_H

typedef enum {
    Plant, 
    Pot, 
    Open, 
    Close,
} object_t; 

typedef enum {
    Up, 
    Down,
    MidPlant,
    MidPot, 
} position_t;

typedef enum {
    CCW,
    CW,
} direction_t;

typedef enum {
    UpS,
    DownS,
} sp_position_t;

typedef enum {
    Blue, 
    Yellow,
} team_t; 

void dxl_init_port(); 
void dxl_close_port(); 

void dxl_ping(int ID, float PROTOCOL); 
void dxl_idle(int ID, float PROTOCOL);

void gripper(object_t objet);
void position_gripper(position_t position);

void position_solar(sp_position_t position);
void multiturn_solar(direction_t direction);
void turn_solar(team_t team, double pres_angle); 
void single_turn_sp(double goal_pos); 
void init_sp(); 

void solar_panel(team_t team, double angle); 

#endif