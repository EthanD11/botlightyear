#ifndef MECATROMINIBOT_SOLAR_PANELS_H
#define MECATROMINIBOT_SOLAR_PANELS_H

void ax_init_port(); 
void xl_init_port(); 
void ax_ping(int ID);
void xl_ping(int ID);

void deploy_solar_panel();
void raise_solar_panel();
void multi_turn_solar_panel(); 

void deploy_solar_panel();
void raise_solar_panel();
void multi_turn_solar_panel();

void open_gripper(); 
void close_gripper(); 
void close_gripper_plant();

void raise_gripper(); 
void deploy_gripper(); 
void mid_gripper();


void ax_close_port();
void xl_close_port();

#endif
