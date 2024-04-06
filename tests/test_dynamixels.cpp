#include "dynamixels.h"
#include <unistd.h>
#include <cstdio>

#define SOLAR_PANELS
//#define EXTENDED_SOLAR_PANELS
// #define GRIPPER_OPEN_CLOSE
//#define GRIPPER_UP_DOWN
//#define GRIPPER_HELLO
//#define TAKE_PLANT
//#define TAKE_POT

int main(int argc, char const *argv[])
{

    // Init ports, and test ping
    dxl_init_port();
    //dxl_ping(1, 2.0);
    //dxl_ping(3, 2.0);
    dxl_ping(6, 1.0);
    dxl_ping(8, 1.0);


    #ifdef SOLAR_PANELS
    position_solar(DownS);
    sleep(0.5);
    turn_solar(Yellow, 180);
    sleep(0.5);
    position_solar(UpS);
    sleep(0.5); 
    init_sp();
    #endif

    #ifdef EXTENDED_SOLAR_PANELS
    position_solar(DownS);
    sleep(0.5);
    multiturn_solar(CCW);
    sleep(1);
    multiturn_solar(CW);
    sleep(0.5);
    position_solar(UpS);
    #endif

    #ifdef GRIPPER_OPEN_CLOSE
    gripper(Open);
    gripper(Close);
    #endif

    #ifdef GRIPPER_UP_DOWN
    position_gripper(MidPlant);
    sleep(1);
    position_gripper(Up);
    sleep(1);
    position_gripper(Down);
    #endif

    #ifdef GRIPPER_HELLO
    position_gripper(MidPlant);
    position_gripper(Down);
    position_gripper(MidPlant);
    position_gripper(Down);
    gripper(Open);
    gripper(Close);
    gripper(Open);
    gripper(Close);
    position_gripper(Up);
    sleep(1);
    position_gripper(Down);
    #endif

    #ifdef TAKE_PLANT
    gripper(Open);
    sleep(2);
    gripper(Plant);
    sleep(5);
    gripper(Open);
    sleep(2);
    gripper(Close);
    #endif

    #ifdef TAKE_POT

    gripper(Open);
    sleep(2);
    gripper(Pot);
    sleep(5);
    gripper(Open);
    sleep(2);
    gripper(Close);
    #endif

    dxl_idle(1, 2.0);
    dxl_idle(3, 2.0);
    dxl_idle(6, 1.0);
    dxl_idle(8, 1.0);

    dxl_close_port();
    return 0;
}