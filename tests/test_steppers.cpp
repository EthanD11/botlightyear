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
    
    
    init_spi2(); 
    stpr_reset_all();
    stpr_calibrate_all();


    plate_move(-3, CALL_BLOCKING);
    plate_move(0, CALL_BLOCKING);
    plate_move(3, CALL_BLOCKING);
    spi_close2(); 

    
    //stpr_setup_speed(StprPlate,60,500); //60 max
    //demoPlate();
    //plate_move(-3);
    //plate_move(3);

    

    /*
    stpr_setup_speed(StprFlaps,100,400);
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

    spi_close();
    
    return 0;
}

