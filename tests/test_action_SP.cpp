#include "action_SP.h"

SharedVariables shared = SharedVariables();  

double xinit, yinit, tinit = 0; 

int main(int argc, char const *argv[]) {

    shared.set_robot_pos(xinit, yinit, tinit); 

    dxl_init_port();
    
    dxl_ping(6, 1.0);
    dxl_ping(8, 1.0);

    bool reserved = true; 
    turn_solar_panel(reserved, 2); 

    printf("End of action, score: %d\n", shared.score); 
    dxl_close_port();

    return 0;
}