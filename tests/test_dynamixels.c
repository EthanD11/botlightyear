#include "../src/dynamixels.h"
#include <unistd.h>

int main(int argc, char const *argv[])
{
    ax_init_port();
    //xl_init_port();

    xl_ping(1);
    xl_ping(3);
    ax_ping(6);
    ax_ping(8);

    //SOLAR PANELS
    /*deploy_solar_panel(); 
    sleep(0.5);
    multi_turn_solar_panel();
    sleep(0.5);
    raise_solar_panel();*/
    

    //GRIPPER OPEN-CLOSE
    /*open_gripper(); 
    close_gripper();*/

    //GRIPPER UP-DOWN
    /*raise_gripper(); 
    deploy_gripper();*/

    //GRIPPER HELLO
    mid_gripper();
    deploy_gripper(); 
    mid_gripper();
    deploy_gripper(); 
    open_gripper(); 
    close_gripper(); 
    open_gripper(); 
    close_gripper(); 
    raise_gripper();

    //TAKE PLANT
    /*open_gripper(); 
    deploy_gripper(); */
    /*sleep(2);
    close_gripper_plant();
    sleep(10);
    open_gripper(); 
    sleep(2);
    close_gripper(); 
    raise_gripper();*/


    ax_close_port();
    //xl_close_port();
    return 0;
}