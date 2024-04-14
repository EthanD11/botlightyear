#include "action_SP.h"

SharedVariables shared = SharedVariables();  

double xinit, yinit, tinit = 0; 

int main(int argc, char const *argv[]) {

    shared.set_robot_pos(xinit, yinit, tinit); 
    Odometry odo = shared.odo; 
    odo.set_pos(xinit, yinit, tinit); 
    printf("Shared address %d \n", &shared); 

    //dxl_init_port();
    
    //dxl_ping(6, 1.0);
    //dxl_ping(8, 1.0);

    for (size_t i = 0; i < 3; i++)
    {
        solar_panel_pc();
    }
    
    

/*    bool reserved = true; 
    turn_solar_panel(reserved, 2);
    turn_solar_panel(reserved, 2); 

    dxl_close_port();*/

    return 0;
}