#include "dynamixels.h"
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
    setupStepperSpeed(500,1000,Flaps); 
    setupStepperSpeed(200,1000,Plate); 
    setupStepperSpeed(400,1000,Slider);
    calibrateAll();
    sleep(10);

    
    servo_deploy(); 
    moveFlaps(Pot);
    sleep(3);
    moveFlaps(Open);
    sleep(1);
    servo_raise();
    open_gripper();
    deploy_gripper();
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
    moveFlaps(Pot); 
    sleep(3); 
    moveFlaps(Open); 
    sleep(1); 
    servo_raise(); 
    moveSlider(Bas); 
    sleep(6); 
    close_gripper_pot(); 
    sleep(1); 
    moveSlider(Plateau); 
    sleep(6); 
    PositionPlateau(1); 
    sleep(3); 
    open_gripper(); 
    raise_gripper(); 
    PositionPlateau(0);
    sleep(3);

    open_gripper();
    deploy_gripper();
    servo_deploy(); 
    moveFlaps(Plant); 
    sleep(3); 
    moveFlaps(Open);
    sleep(1);
    servo_raise(); 
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
    PositionPlateau(0); 
    sleep(3);

    open_gripper();
    deploy_gripper();
    servo_deploy(); 
    moveFlaps(Plant); 
    sleep(3); 
    moveFlaps(Open);
    sleep(1);
    servo_raise(); 
    moveSlider(Bas); 
    sleep(6); 
    close_gripper_plant(); 
    sleep(1);
    moveSlider(Plateau); 
    sleep(6); 
    mid_gripper(); 
    PositionPlateau(1); 
    sleep(4); 
    deploy_gripper(); 
    open_gripper();
    raise_gripper();
    close_gripper();
    PositionPlateau(0); 
    sleep(3);

    PositionPlateau(1); 
    open_gripper(); 
    deploy_gripper();
    sleep(5);
    moveSlider(Take);
    sleep(3); 
    close_gripper_pot(); 
    sleep(1);
    mid_gripper(); 
    PositionPlateau(0); 
    sleep(2);
    deploy_gripper(); 
    moveSlider(Bas); 
    sleep(5); 
    open_gripper(); 
    raise_gripper(); 
    moveSlider(Take); 
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