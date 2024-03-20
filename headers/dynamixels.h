#ifndef MECATROMINIBOT_DYNAMIXELS_H
#define MECATROMINIBOT_DYNAMIXELS_H

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
    cw,
} direction_t;

typedef enum {
    Up,
    Down,
} position_s;

void dxl_init_port(); 
void dxl_close_port(); 

void dxl_ping(int ID, float PROTOCOL); 
void dxl_idle(int ID, float PROTOCOL);

void gripper(object_t);
void position_gripper(position_t);

void position_solar(position_s);
void multiturn_solar(direction_t);

#endif