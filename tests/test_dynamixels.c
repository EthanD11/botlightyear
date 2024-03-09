#include "dynamixels.h"
#include <unistd.h>

int main(int argc, char const *argv[])
{
    init_port();
    
    ping_dxl(1, 2.0);
    ping_dxl(3, 2.0);
    ping_dxl(6, 1.0);
    ping_dxl(8, 1.0);

    //SOLAR PANELS
    /*deploy_solar_panel(); 
    sleep(0.5);
    multi_turn_solar_panel();
    sleep(0.5);
    raise_solar_panel();*/
    

    //GRIPPER OPEN-CLOSE
    /*open_gripper(); 
    sleep(1); 
    close_gripper();*/

    //GRIPPER UP-DOWN
    /*raiseG_withoutreturn(); 
    sleep(1); 
    deployG_withoutreturn();*/

    close_port();
    return 0;
}