#ifndef MECATROMINIBOT_SOLAR_PANELS_H
#define MECATROMINIBOT_SOLAR_PANELS_H

void init_port(); 
void ping_dxl(int ID, float protocol);

void deploy_solar_panel();
void raise_solar_panel();
void multi_turn_solar_panel(); 

void deploy_solar_panel();
void raise_solar_panel();
void multi_turn_solar_panel();

void openG(); 
void open_gripper(); 
void close_gripper(); 

void raiseG_withoutreturn(); 
void deployG_withoutreturn(); 

void close_port();

#endif
