#include "dynamixels.h"
#include <unistd.h>
#include <cstdio>

//#define SOLAR_PANELS
//#define EXTENDED_SOLAR_PANELS
//#define GRIPPER_OPEN_CLOSE
//#define GRIPPER_UP_DOWN
#define GRIPPER_HELLO
//#define TAKE_PLANT
//#define TAKE_POT

int main(int argc, char const *argv[])
{

    // Init ports, and test ping
    //#if defined(SOLAR_PANELS) || defined(EXTENDED_SOLAR_PANELS)
    if (ax_init_port() != 0) return -1;
    if (ax_ping(6) != 0) return -1;
    if (ax_ping(8) != 0) return -1;
    //#endif

    //#if defined(GRIPPER_OPEN_CLOSE) || defined(GRIPPER_UP_DOWN) || defined(GRIPPER_HELLO) || defined(TAKE_PLANT) || defined(TAKE_POT) 
    if (xl_init_port() != 0) return -1;
    if (xl_ping(1) != 0) return -1;
    if (xl_ping(3) != 0) return -1;
    //#endif

    #ifdef SOLAR_PANELS
    deploy_solar_panel();
    sleep(0.5);
    //multi_turn_solar_panel_ccw();
    position_solar_panel();
    sleep(0.5);
    raise_solar_panel();
    #endif

    #ifdef EXTENDED_SOLAR_PANELS
    deploy_solar_panel();
    sleep(0.5);
    multi_turn_solar_panel_cw();
    sleep(1);
    multi_turn_solar_panel_ccw();
    sleep(1);
    position_solar_panel();
    sleep(0.5);
    raise_solar_panel();
    #endif

    #ifdef GRIPPER_OPEN_CLOSE
    open_gripper();
    close_gripper();
    #endif

    #ifdef GRIPPER_UP_DOWN
    mid_gripper();
    sleep(1);
    raise_gripper();
    sleep(1);
    deploy_gripper();
    #endif

    #ifdef GRIPPER_HELLO
    sleep(5);
    mid_gripper();
    deploy_gripper();
    mid_gripper();
    deploy_gripper();
    open_gripper();
    close_gripper();
    open_gripper();
    close_gripper();
    raise_gripper();
    sleep(1);
    deploy_gripper();
    #endif

    #ifdef TAKE_PLANT
    open_gripper();
    deploy_gripper();
    sleep(4);
    close_gripper_plant();
    sleep(10);
    open_gripper();
    sleep(2);
    raise_gripper();
    close_gripper();
    #endif

    #ifdef TAKE_POT
    open_gripper();
    deploy_gripper();
    sleep(3);
    close_gripper_pot();
    sleep(5);
    open_gripper();
    close_gripper();

    open_gripper();
    sleep(2);
    close_gripper_pot();
    sleep(2);
    open_gripper();
    sleep(0.5);
    close_gripper();
    #endif

    //#if defined(SOLAR_PANELS) || defined(EXTENDED_SOLAR_PANELS)
    idle(6, 1.0);
    idle(8, 1.0);
    ax_close_port();
    //#endif
    //#if defined(GRIPPER_OPEN_CLOSE) || defined(GRIPPER_UP_DOWN) || defined(GRIPPER_HELLO) || defined(TAKE_PLANT)
    idle(1, 2.0);
    idle(3, 2.0);
    xl_close_port();
    //#endif
    return 0;
}