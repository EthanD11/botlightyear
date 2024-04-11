#include "SPI_Modules.h"
#include <stdio.h>
//#define RESET_CALIBRATE
//#define SETUP_CUSTOM_SPEED_OLD
//#define SETUP_CUSTOM_SPEED_NEW
// #define DEMO_S6
#define TESTS

// 8x (8 = ecrire)(x = stepper)
// 1 plateau
// 2 .. config vitesse
// 3 .. config reste
// 4 slider, rail lineaire
// 5 .. config vitesse
// 6 .. config reste
// 7 flaps stepper
// 8 .. config vitesse
// 9 .. config reste

// 00(mode) 0(Signe) 00...00(29->step)
// mode: 
//      00 OFF     
//      01 Idle
//      10 Calibre
//      11 Step + signe direction du stepper (horlogique/antihorlogique)

void TakePotCHAIN() {
    stpr_setup_speed(StprSlider,400,600);

    gripper_holder_cmd(HolderIdle);
    gripper_deployer_cmd(DeployerIdle);
    stpr_reset_all();
    stpr_calibrate(StprPlate, CALL_BLOCKING); 
    stpr_calibrate(StprSlider, CALL_BLOCKING); 


    plate_move(0, CALL_BLOCKING); 
    gripper_deployer_cmd(DeployerDeploy);
    gripper_holder_cmd(HolderOpenFull);

    // Go to intermediate low
    slider_move(SliderIntermediateLow, CALL_BLOCKING);
    // "Half open" holder
    gripper_holder_cmd(HolderOpen);

    // Go to the low
    slider_move(SliderLow, CALL_BLOCKING); 
    gripper_holder_cmd(HolderClosed);
    sleep(1); // to assure grip

    // go back up
    slider_move(SliderHigh, CALL_BLOCKING);
    gripper_deployer_cmd(DeployerPot);

    plate_move(3, CALL_BLOCKING); 

    // Go to deposit
    slider_move(SliderDepositPot, CALL_BLOCKING);
    gripper_deployer_cmd(DeployerDeploy); 
    sleep(2);
    gripper_holder_cmd(HolderOpenFull);
    sleep(2);
}

void demoPlate(){
    plate_move(-3);
    sleep(3);
    for(int i = -2; i<= 3; i++) {
        plate_move(i);
        sleep(2);
    }
    plate_move(0);
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
    // slider_move(SliderIntermediateLow);

    // gripper_holder_cmd(HolderOpen);
    
    TakePotCHAIN();

    // plate_move(3, CALL_BLOCKING);
    // plate_move(0, CALL_BLOCKING);
    // plate_move(3, CALL_BLOCKING);
    // plate_move(0, CALL_BLOCKING);
    // TakePotCHAIN(); 

    
    //stpr_setup_speed(StprPlate,60,500); //60 max
    //demoPlate();
    //plate_move(-3);
    //plate_move(3);

    

    /*
    flaps_move(FlapsPlant);
    flaps_move(FlapsOpen);
    flaps_move(FlapsPot);
    flaps_move(FlapsOpen);*/

    #endif

    #ifdef RESET_CALIBRATE
    stpr_reset_all(); 
    stpr_calibrate_all();
    lguSleep(10);
    #endif

    #ifdef SETUP_CUSTOM_SPEED_OLD
    stpr_setup_speed(StprFlaps,5,10); 
    stpr_setup_speed(StprPlate,2,10); 
    stpr_setup_speed(StprSlider,4,10);
    #endif 

    #ifdef SETUP_CUSTOM_SPEED_NEW
    stpr_setup_speed(StprFlaps,500,1000); 
    stpr_setup_speed(StprPlate,200,1000); 
    stpr_setup_speed(StprSlider,400,1000);
    stepper_setup_acc(StprPlate, 5);
    #endif

    #ifdef DEMO_S6
    servo_cmd(ServoDeploy); 
    flaps_move(FlapsPlant);
    sleep(4);
    flaps_move(FlapsOpen);
    slider_move(SliderLow);
    sleep(2);
    servo_cmd(ServoRaise);
    sleep(3);
    slider_move(SliderPlate);
    sleep(5);
    demoPlate();
    sleep(5);
    //slider_move(SliderLow);
    stpr_move(StprFlaps, 600,0);
    sleep(5);
    plate_move(1);
    servo_cmd(ServoDeploy); 
    
    sleep(5);
    servo_cmd(ServoIdle);
    stpr_reset_all(); 
    stpr_calibrate_all();
    #endif

    gripper_holder_cmd(HolderIdle);
    gripper_deployer_cmd(DeployerIdle);
    close_spi2(); 

    close_spi();
    
    return 0;
}

