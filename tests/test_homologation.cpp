#include "SPI_bus.h"
#include "servos.h"
#include "steppers.h"
#include "dynamixels.h"
#include <unistd.h>
#include <cstdio>

SPIBus spi_bus = SPIBus(); 
Steppers steppers = SPIUser(&spi_bus); 

Flaps servo_flaps = SPIUser(&spi_bus); 

int main(int argc, char const *argv[])
{

    // Init ports, and test ping
    init_spi();
    dxl_init_port();
    dxl_ping(1, 2.0);
    dxl_ping(3, 2.0);
    dxl_ping(6, 1.0);
    dxl_ping(8, 1.0);

    servo_flaps.raise();
    steppers.setup_speed(StprFlaps,100,600); 
    steppers.setup_speed(StprPlate,60,500); 
    steppers.setup_speed(StprSlider,300,400);
    steppers.reset_all(); 
    steppers.calibrate_all();

    sleep(5); 


    servo_flaps.deploy();
    steppers.flaps_move(FlapsPlant);
    lguSleep(3);
    steppers.flaps_move(FlapsOpen);
    gripper(Open);
    position_gripper(Down);
    lguSleep(1);
    steppers.slider_move(SliderLow);
    lguSleep(3);
    gripper(Plant); 
    lguSleep(0.5);
    steppers.slider_move(SliderPlate);
    lguSleep(2);
    steppers.plate_move(2);
    lguSleep(2);
    //position_gripper(Down);
    gripper(Open); 
    position_gripper(Up);
    steppers.plate_move(0);

    lguSleep(5);

    steppers.slider_move(SliderHigh); 
    steppers.plate_move(2);
    gripper(Open);
    lguSleep(2);
    position_gripper(Down);
    lguSleep(3);
    steppers.slider_move(SliderTake); 
    lguSleep(2); 
    gripper(Plant); 
    steppers.slider_move(SliderHigh); 
    lguSleep(1); 
    steppers.plate_move(0); 
    lguSleep(2); 
    steppers.slider_move(SliderDeposit); 
    lguSleep(3); 
    gripper(Open); 
    lguSleep(2); 
    steppers.slider_move(SliderPlate); 
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