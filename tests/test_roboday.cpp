#include "dynamixels.h"
#include "SPI_Modules.h"
#include <unistd.h>

void calibrateAll() {
    stpr_calibrate(StprFlaps);
    stpr_calibrate(StprPlate);
    stpr_calibrate(StprSlider);
}

void resetAll() {
    stpr_reset(StprFlaps);
    stpr_reset(StprPlate);
    stpr_reset(StprSlider);
}

int main(int argc, char const *argv[])
{
    init_spi();
    ax_init_port();

    if (xl_ping(1) != 0) return -1;
    if (xl_ping(3) != 0) return -1;
    if (ax_ping(6) != 0) return -1;
    if (ax_ping(8) != 0) return -1;

    servo_cmd(ServoRaise);

    resetAll(); 
    stpr_setup_speed(5,10,StprFlaps); 
    stpr_setup_speed(2,10,StprPlate); 
    stpr_setup_speed(4,10,StprSlider);
    calibrateAll();
    sleep(10);

    
    servo_cmd(ServoDeploy); 
    flaps_move(FlapsPot);
    sleep(3);
    flaps_move(FlapsOpen);
    sleep(1);
    servo_cmd(ServoRaise);
    open_gripper();
    deploy_gripper();
    slider_move(SliderLow);
    sleep(6);
    close_gripper_pot();
    sleep(1);
    slider_move(SliderPlate);
    sleep(6);
    plate_move(2);
    sleep(4); 
    open_gripper(); 
    raise_gripper(); 
    close_gripper();
    plate_move(0);
    sleep(5); 

    open_gripper();
    deploy_gripper(); 
    servo_cmd(ServoDeploy); 
    flaps_move(FlapsPot); 
    sleep(3); 
    flaps_move(FlapsOpen); 
    sleep(1); 
    servo_cmd(ServoRaise); 
    slider_move(SliderLow); 
    sleep(6); 
    close_gripper_pot(); 
    sleep(1); 
    slider_move(SliderPlate); 
    sleep(6); 
    plate_move(1); 
    sleep(3); 
    open_gripper(); 
    raise_gripper(); 
    plate_move(0);
    sleep(3);

    open_gripper();
    deploy_gripper();
    servo_cmd(ServoDeploy); 
    flaps_move(FlapsPlant); 
    sleep(3); 
    flaps_move(FlapsOpen);
    sleep(1);
    servo_cmd(ServoRaise); 
    slider_move(SliderLow); 
    sleep(6); 
    close_gripper_plant(); 
    sleep(1);
    slider_move(SliderPlate); 
    sleep(6); 
    mid_gripper(); 
    plate_move(2); 
    sleep(4); 
    deploy_gripper(); 
    open_gripper();
    raise_gripper();
    close_gripper();
    plate_move(0); 
    sleep(3);

    open_gripper();
    deploy_gripper();
    servo_cmd(ServoDeploy); 
    flaps_move(FlapsPlant); 
    sleep(3); 
    flaps_move(FlapsOpen);
    sleep(1);
    servo_cmd(ServoRaise); 
    slider_move(SliderLow); 
    sleep(6); 
    close_gripper_plant(); 
    sleep(1);
    slider_move(SliderPlate); 
    sleep(6); 
    mid_gripper(); 
    plate_move(1); 
    sleep(4); 
    deploy_gripper(); 
    open_gripper();
    raise_gripper();
    close_gripper();
    plate_move(0); 
    sleep(3);

    plate_move(1); 
    open_gripper(); 
    deploy_gripper();
    sleep(5);
    slider_move(SliderTake);
    sleep(3); 
    close_gripper_pot(); 
    sleep(1);
    mid_gripper(); 
    plate_move(0); 
    sleep(2);
    deploy_gripper(); 
    slider_move(SliderLow); 
    sleep(5); 
    open_gripper(); 
    raise_gripper(); 
    slider_move(SliderTake); 
    //stpr_reset(StprPlate); 
    
    idle(1, 2.0);
    idle(3, 2.0);
    ax_close_port();
    xl_close_port();
    close_spi();
    
    return 0;
}