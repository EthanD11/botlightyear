#include "SPI_Modules.h"
#include "dynamixels.h"
#include <unistd.h>
#include <cstdio>



int main(int argc, char const *argv[])
{

    // Init ports, and test ping
    init_spi();
    dxl_init_port();
    dxl_ping(1, 2.0);
    dxl_ping(3, 2.0);
    dxl_ping(6, 1.0);
    dxl_ping(8, 1.0);

    flaps_servo_cmd(FlapsRaise);
    stpr_setup_speed(StprFlaps,100,600); 
    stpr_setup_speed(StprPlate,60,500); 
    stpr_setup_speed(StprSlider,300,400);
    stpr_reset_all(); 
    stpr_calibrate_all();

    sleep(5); 


    flaps_servo_cmd(FlapsDeploy);
    flaps_move(FlapsPlant);
    lguSleep(3);
    flaps_move(FlapsOpen);
    gripper(Open);
    position_gripper(Down);
    lguSleep(1);
    slider_move(SliderLow);
    lguSleep(3);
    gripper(Plant); 
    lguSleep(0.5);
    slider_move(SliderPlate);
    lguSleep(2);
    plate_move(2);
    lguSleep(2);
    //position_gripper(Down);
    gripper(Open); 
    position_gripper(Up);
    plate_move(0);

    lguSleep(5);

    slider_move(SliderHigh); 
    plate_move(2);
    gripper(Open);
    lguSleep(2);
    position_gripper(Down);
    lguSleep(3);
    slider_move(SliderTake); 
    lguSleep(2); 
    gripper(Plant); 
    slider_move(SliderHigh); 
    lguSleep(1); 
    plate_move(0); 
    lguSleep(2); 
    slider_move(SliderDeposit); 
    lguSleep(3); 
    gripper(Open); 
    lguSleep(2); 
    slider_move(SliderPlate); 
    lguSleep(1);
    gripper(Close); 




    dxl_idle(1, 2.0);
    dxl_idle(3, 2.0);
    dxl_idle(6, 1.0);
    dxl_idle(8, 1.0);

    dxl_close_port();
    spi_close();
    return 0;
}