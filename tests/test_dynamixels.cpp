#include "dynamixels.h"
#include <unistd.h>
#include <cstdio>

#define FINAL_SP
//#define SOLAR_PANELS
//#define EXTENDED_SOLAR_PANELS


int main(int argc, char const *argv[])
{

    // Init ports and ping
    dxl_init_port();
    dxl_ping(6, 1.0);
    dxl_ping(8, 1.0);


    #ifdef FINAL_SP
    // dxl_reset_sp(); 
    solar_panel(TeamBlue, 90);
    // dxl_deploy(Down); 
    // sleep(0.5);
    // dxl_position(700); 
    // sleep(0.5); 
    // dxl_deploy(Up);
    #endif

    #ifdef SOLAR_PANELS
    dxl_deploy(Down);
    sleep(0.5);
    dxl_position(780);
    sleep(1);
    // dxl_deploy(Mid);
    // sleep(0.5);
    // dxl_deploy(Down);
    // sleep(0.5);
    // dxl_position(790);
    // sleep(0.5);
    dxl_deploy(Up);
    sleep(0.5); 
    dxl_reset_sp();
    #endif

    #ifdef EXTENDED_SOLAR_PANELS
    dxl_position(Down);
    sleep(0.5);
    dxl_multiturn(CCW);
    sleep(1);
    dxl_multiturn(CW);
    sleep(0.5);
    dxl_position(Up);
    #endif

    //dxl_idle(6, 1.0);
    //dxl_idle(8, 1.0);

    dxl_close_port();
    return 0;
}