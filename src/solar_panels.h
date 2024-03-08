#ifndef MECATROMINIBOT_SOLAR_PANELS_H
#define MECATROMINIBOT_SOLAR_PANELS_H

void init_port(); 
void ping_dxl(int ID, float protocol);

void deployP();
void raiseP();
void multi_turnP(); 

void openG(); 
void openG_withoutreturn(); 
void closeG_withoutreturn(); 

void raiseG_withoutreturn(); 
void deployG_withoutreturn(); 

void close_port();

#endif
