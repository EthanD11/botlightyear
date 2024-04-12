#include "old_dynamixels.h"
#include "SPI_bus.h"
#include "steppers.h"
#include "servos.h"
#include <unistd.h>

SPIBus spiBus = SPIBus();
GPIOPins pins = GPIOPins(); 
Steppers steppers = Steppers(&spiBus, &pins); 
Flaps servoFlaps = Flaps(&spiBus) 

int main(int argc, char const *argv[])
{
    ax_init_port();

    if (xl_ping(1) != 0) return -1;
    if (xl_ping(3) != 0) return -1;
    if (ax_ping(6) != 0) return -1;
    if (ax_ping(8) != 0) return -1;

    servoFlaps.raise(); 

    steppers.reset_all(); 
    steppers.setup_speed(StprFlaps,5,10); 
    steppers.setup_speed(StprPlate,2,10); 
    steppers.setup_speed(StprSlider,4,10);
    steppers.calibrate_all();
    sleep(10);

    
    servoFlaps.deploy();
    steppers.flaps_move(FlapsPot);
    sleep(3);
    steppers.flaps_move(FlapsOpen);
    sleep(1);
    servoFlaps.raise();
    open_gripper();
    deploy_gripper();
    steppers.slider_move(SliderLow);
    sleep(6);
    close_gripper_pot();
    sleep(1);
    steppers.slider_move(SliderPlate);
    sleep(6);
    steppers.plate_move(2);
    sleep(4); 
    open_gripper(); 
    raise_gripper(); 
    close_gripper();
    steppers.plate_move(0);
    sleep(5); 

    open_gripper();
    deploy_gripper(); 
    servoFlaps.deploy();
    steppers.flaps_move(FlapsPot); 
    sleep(3); 
    steppers.flaps_move(FlapsOpen); 
    sleep(1); 
    servoFlaps.raise();
    steppers.slider_move(SliderLow); 
    sleep(6); 
    close_gripper_pot(); 
    sleep(1); 
    steppers.slider_move(SliderPlate); 
    sleep(6); 
    steppers.plate_move(1); 
    sleep(3); 
    open_gripper(); 
    raise_gripper(); 
    steppers.plate_move(0);
    sleep(3);

    open_gripper();
    deploy_gripper();
    servoFlaps.deploy();
    steppers.flaps_move(FlapsPlant); 
    sleep(3); 
    steppers.flaps_move(FlapsOpen);
    sleep(1);
    servoFlaps.raise();
    steppers.slider_move(SliderLow); 
    sleep(6); 
    close_gripper_plant(); 
    sleep(1);
    steppers.slider_move(SliderPlate); 
    sleep(6); 
    mid_gripper(); 
    steppers.plate_move(2); 
    sleep(4); 
    deploy_gripper(); 
    open_gripper();
    raise_gripper();
    close_gripper();
    steppers.plate_move(0); 
    sleep(3);

    open_gripper();
    deploy_gripper();
    servoFlaps.deploy();
    steppers.flaps_move(FlapsPlant); 
    sleep(3); 
    steppers.flaps_move(FlapsOpen);
    sleep(1);
    servoFlaps.raise();
    steppers.slider_move(SliderLow); 
    sleep(6); 
    close_gripper_plant(); 
    sleep(1);
    steppers.slider_move(SliderPlate); 
    sleep(6); 
    mid_gripper(); 
    steppers.plate_move(1); 
    sleep(4); 
    deploy_gripper(); 
    open_gripper();
    raise_gripper();
    close_gripper();
    steppers.plate_move(0); 
    sleep(3);

    steppers.plate_move(1); 
    open_gripper(); 
    deploy_gripper();
    sleep(5);
    steppers.slider_move(SliderTake);
    sleep(3); 
    close_gripper_pot(); 
    sleep(1);
    mid_gripper(); 
    steppers.plate_move(0); 
    sleep(2);
    deploy_gripper(); 
    steppers.slider_move(SliderLow); 
    sleep(5); 
    open_gripper(); 
    raise_gripper(); 
    steppers.slider_move(SliderTake); 
    //steppers.reset(StprPlate); 
    
    idle(1, 2.0);
    idle(3, 2.0);
    ax_close_port();
    xl_close_port();
    spi_close();
    
    return 0;
}