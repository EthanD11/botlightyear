#include "dynamixels.h"
#include <unistd.h>

int main(int argc, char const *argv[])
{
    init_port();
    
    //SOLAR PANELS
    deploy_solar_panel(); 
    sleep(0.5);
    multi_turn_solar_panel();
    sleep(0.5);
    raise_solar_panel();

    //GRIPPER OPEN-CLOSE
    /*openG_withoutreturn(); 
    sleep(1); 
    closeG_withoutreturn();*/

    //GRIPPER UP-DOWN
    /*raiseG_withoutreturn(); 
    sleep(1); 
    deployG_withoutreturn();*/

    close_port();
    return 0;
}