#include "../src/dynamixels.h"
#include "SPI_Modules.h"
#include <unistd.h>

int main(int argc, char const *argv[])
{
    int init = init_spi();
    ax_init_port();

    xl_ping(1);
    xl_ping(3);
    ax_ping(6);
    ax_ping(8);

    servo_raise();

    resetAll(); 
    setupStepperSpeed(5,10,Flaps); 
    setupStepperSpeed(2,10,Plate); 
    setupStepperSpeed(4,10,Slider);
    calibrateAll();

    sleep(10);
    servo_deploy(); 
    moveFlaps(Pot);
    sleep(3);
    moveFlaps(Open);
    deploy_gripper();
    open_gripper();
    moveSlider(Bas);
    sleep(6);
    close_gripper_pot();
    sleep(1);
    moveSlider(Plateau);
    sleep(6);
    PositionPlateau(2);
    sleep(4); 
    open_gripper(); 
    raise_gripper(); 
    close_gripper();
    PositionPlateau(0);
    sleep(5); 
    open_gripper();
    deploy_gripper();
    servo_deploy(); 
    moveFlaps(Plant); 
    sleep(3); 
    moveFlaps(Open);
    moveSlider(Bas); 
    sleep(6); 
    close_gripper_plant(); 
    sleep(1);
    moveSlider(Plateau); 
    sleep(6); 
    mid_gripper(); 
    PositionPlateau(2); 
    sleep(4); 
    deploy_gripper(); 
    open_gripper();
    raise_gripper();
    close_gripper();

    //resetStepperModule(Plate); 
    
    ax_close_port();
    close_spi();
    
    return 0;
}

void calibrateAll() {
    calibrateStepper(Flaps);
    calibrateStepper(Plate);
    calibrateStepper(Slider);
}

void resetAll() {
    resetStepperModule (Flaps);
    resetStepperModule (Plate);
    resetStepperModule (Slider);
}