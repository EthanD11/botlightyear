#include "action_SP.h"

double xinit, yinit, tinit = 0; 
shared.set_robot_pos(&xinit, &yinit, &tinit); 

int main(int argc, char const *argv[]) {
    dxl_init_port();
    
    dxl_ping(6, 1.0);
    dxl_ping(8, 1.0);

    bool reserved = true; 
    turn_solar_panel(reserved, 3); 

    dxl_close_port();

    return 0;
}