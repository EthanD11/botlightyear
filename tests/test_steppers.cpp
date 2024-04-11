#include "SPI_bus.h"
#include "GPIO.h"
#include "steppers.h"
#include "servos.h"
#include <unistd.h> 

//#define RESET_CALIBRATE
// #define SETUP_CUSTOM_SPEED_OLD
// #define SETUP_CUSTOM_SPEED_NEW
#define DEMO_S6
#define TESTS

SPIBus spi_bus = SPIBus();
GPIOPins pins = GPIOPins(); 

Steppers steppers = Steppers(&spi_bus, &pins); 
Flaps servo_flaps = Flaps(&spi_bus); 
void TakePotCHAIN() {
    gripper_holder_cmd(HolderIdle);
    gripper_deployer_cmd(DeployerIdle);
    stpr_reset_all();
    stpr_calibrate(StprPlate, CALL_BLOCKING); 
    stpr_calibrate(StprSlider, CALL_BLOCKING); 


    plate_move(0, CALL_BLOCKING); 
    gripper_deployer_cmd(DeployerDeploy);

    gripper_holder_cmd(HolderClosed);
    slider_move(SliderIntermediateLow, CALL_BLOCKING);
    gripper_holder_cmd(HolderOpen);


    slider_move(SliderLow, CALL_BLOCKING); 
    sleep(2);

    gripper_holder_cmd(HolderClosed);

    slider_move(SliderHigh, CALL_BLOCKING);
    gripper_deployer_cmd(DeployerPot);

    plate_move(3, CALL_BLOCKING); 

    
    slider_move(SliderDepositPot, CALL_BLOCKING);
    gripper_deployer_cmd(DeployerDeploy); 
    sleep(2);
    gripper_holder_cmd(HolderOpen);
    sleep(2);
}

void demoPlate(){
    steppers.plate_move(-3);
    sleep(3);
    for(int i = -2; i<= 3; i++) {
        steppers.plate_move(i);
        sleep(2);
    }
    steppers.plate_move(0);
}

int main(int argc, char const *argv[])
{
    if (init_spi() != 0) return -1;  
    int val = init_spi2(); 
    if (val !=0) {
        printf("Error init spi2 : %d\n", init_spi2()); 
    }

    #ifdef TESTS
    // stpr_setup_speed(StprFlaps,500,800); 
    // stpr_setup_speed(StprPlate,60,500); 
    // stpr_setup_speed(StprSlider,300,600);
    // stpr_setup_speed(StprFlaps,100,400);
    // stpr_setup_calib_speed(400,500,StprFlaps);

    // gripper_holder_cmd(HolderIdle);
    // gripper_deployer_cmd(DeployerIdle);
    // stpr_reset_all();

    // stpr_calibrate(StprPlate, CALL_BLOCKING); 
    // stpr_calibrate(StprSlider, CALL_BLOCKING); 

    // sleep(5);
    // slider_move(SliderPlate, CALL_BLOCKING);
    // slider_move(SliderTake, CALL_BLOCKING);
    // slider_move(SliderDeposit, CALL_BLOCKING);
    // slider_move(SliderPlate, CALL_BLOCKING);
    //1800 : TAKE-PLANT, TAKE-POT, DEPOSIT-PLANT-not in pot, 
    //1000 : DEPOSIT_POT
    //5300 : butée : slider_low : take plant or pot

    // stpr_move(StprSlider, 5300, 0); 
    // plate_move(1, CALL_BLOCKING);
    // stpr_calibrate(StprSlider); 
    // slider_move(SliderDepositPot);
    // gripper_holder_cmd(HolderPot);
    // gripper_holder_cmd(HolderOpen);
    // gripper_deployer_cmd(DeployerPot);
    // gripper_deployer_cmd(DeployerDeploy);
    
    // stpr_move(StprPlate, 70, 1);
    // stpr_reset(StprPlate); 
    // stpr_calibrate(StprPlate, CALL_BLOCKING); 
    // plate_move(0, CALL_BLOCKING); 


    // gripper_deployer_cmd(DeployerDeploy);
    slider_move(SliderIntermediateLow);

    gripper_holder_cmd(HolderOpen);
    
    // TakePotCHAIN();

    // plate_move(3, CALL_BLOCKING);
    // plate_move(0, CALL_BLOCKING);
    // plate_move(3, CALL_BLOCKING);
    // plate_move(0, CALL_BLOCKING);
    // TakePotCHAIN(); 

    
    // steppers.setup_speed(StprPlate,60,500); //60 max
    // demoPlate();
    // steppers.plate_move(-3);
    // steppers.plate_move(3);

    

    
    // steppers.setup_speed(StprFlaps,100,400);
    // steppers.flaps_move(FlapsPlant);
    // steppers.flaps_move(FlapsOpen);
    // steppers.flaps_move(FlapsPot);
    // steppers.flaps_move(FlapsOpen);

    #endif

    #ifdef RESET_CALIBRATE
    steppers.reset_all(); 
    steppers.calibrate_all();
    sleep(10);
    #endif

    #ifdef SETUP_CUSTOM_SPEED_OLD
    steppers.setup_speed(StprFlaps,5,10); 
    steppers.setup_speed(StprPlate,2,10); 
    steppers.setup_speed(StprSlider,4,10);
    #endif 

    #ifdef SETUP_CUSTOM_SPEED_NEW
    steppers.setup_speed(StprFlaps,500,1000); 
    steppers.setup_speed(StprPlate,200,1000); 
    steppers.setup_speed(StprSlider,400,1000);
    steppers.setup_acc(StprPlate, 5);
    #endif

    #ifdef DEMO_S6
    servo_flaps.deploy(); 
    steppers.flaps_move(FlapsPlant);
    sleep(4);
    steppers.flaps_move(FlapsOpen);
    steppers.slider_move(SliderLow);
    sleep(2);
    servo_flaps.raise(); 
    sleep(3);
    steppers.slider_move(SliderPlate);
    sleep(5);
    demoPlate();
    sleep(5);
    //steppers.slider_move(SliderLow);
    steppers.move(StprFlaps, 600,0);
    sleep(5);
    steppers.plate_move(1);
    servo_flaps.deploy(); 
    
    sleep(5);
    servo_flaps.idle();
    steppers.reset_all(); 
    steppers.calibrate_all();
    #endif

    // gripper_holder_cmd(HolderIdle);
    // gripper_deployer_cmd(DeployerIdle);
    
    return 0;
}

