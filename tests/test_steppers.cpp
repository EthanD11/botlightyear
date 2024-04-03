#include "SPI_Modules.h"

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

    #ifdef TESTS
    
    
    resetAll();
    calibrateAll();

    
    //stpr_setup_speed(60,500,StprPlate); //60 max
    //demoPlate();
    //plate_move(-3);
    //plate_move(3);

    

    /*
    stpr_setup_speed(100,400,StprFlaps);
    flaps_move(FlapsPlant);
    flaps_move(FlapsOpen);
    flaps_move(FlapsPot);
    flaps_move(FlapsOpen);*/

    #endif

    #ifdef RESET_CALIBRATE
    resetAll(); 
    calibrateAll();
    lguSleep(10);
    #endif

    #ifdef SETUP_CUSTOM_SPEED_OLD
    stpr_setup_speed(5,10,StprFlaps); 
    stpr_setup_speed(2,10,StprPlate); 
    stpr_setup_speed(4,10,StprSlider);
    #endif 

    #ifdef SETUP_CUSTOM_SPEED_NEW
    stpr_setup_speed(500,1000,StprFlaps); 
    stpr_setup_speed(200,1000,StprPlate); 
    stpr_setup_speed(400,1000,StprSlider);
    stepper_setup_acc(StprPlate, 5);
    #endif

    #ifdef DEMO_S6
    flaps_servo_cmd(FlapsDeploy); 
    flaps_move(FlapsPlant);
    sleep(4);
    flaps_move(FlapsOpen);
    slider_move(SliderLow);
    sleep(2);
    flaps_servo_cmd(FlapsRaise);
    sleep(3);
    slider_move(SliderPlate);
    sleep(5);
    demoPlate();
    sleep(5);
    //slider_move(SliderLow);
    stpr_move(StprFlaps, 600,0);
    sleep(5);
    plate_move(1);
    flaps_servo_cmd(FlapsDeploy); 
    
    sleep(5);
    flaps_servo_cmd(FlapsIdle);
    resetAll(); 
    calibrateAll();
    #endif

    close_spi();
    
    return 0;
}

