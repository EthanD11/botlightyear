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
void multi_turn_solar_panel_cw();
void multi_turn_solar_panel_ccw(); 
void position_solar_panel(); 
void position_solar_panel2();

void open_gripper(); 
void close_gripper(); 
void close_gripper_plant();
void close_gripper_pot(); 

void raise_gripper(); 
void deploy_gripper(); 
void mid_gripper();

void idle(int ID, float protocol);

void ax_close_port();
void xl_close_port();

#endif
