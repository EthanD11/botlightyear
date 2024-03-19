#include "SPI_Modules.h"
<<<<<<< HEAD
void demoS6();
=======

//#define RESET_CALIBRATE
//#define SETUP_CUSTOM_SPEED_OLD
//#define SETUP_CUSTOM_SPEED_NEW
//#define DEMO_S6
#define TESTS

>>>>>>> f558d75f005c06c6bac1bed7ab13982647f476ae
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

<<<<<<< HEAD
int main(int argc, char const *argv[])
{
    int init = init_spi();  
    //calibrateStepper(Flaps);
    demoS6();
    //demoS6();

    
=======
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
    stpr_calibrate(StprPlate);
    stpr_calibrate(StprFlaps);

    
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
    resetAll(); 
    calibrateAll();
    #endif
>>>>>>> f558d75f005c06c6bac1bed7ab13982647f476ae

    close_spi();
    
    return 0;
}

<<<<<<< HEAD
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

void demoPlate(){
    steppers_t stepper = Plate; 
    setupStepperSpeed(2,10,stepper); 
    PositionPlateau(-3);
    sleep(4);
    for(int i = -2; i<= 3; i++) {
        PositionPlateau(i);
        sleep(2);
    }
    PositionPlateau(0);
}

void demoS6(){

    
    servo_raise();

    resetAll(); 
    setupStepperSpeed(5,10,Flaps); 
    setupStepperSpeed(2,10,Plate); 
    setupStepperSpeed(4,10,Slider);
    calibrateAll();
    sleep(10);
    servo_deploy(); 
    moveFlaps(Plant);
    sleep(4);
    moveFlaps(Open);
    moveSlider(Bas);
    sleep(2);
    servo_raise();
    sleep(3);
    moveSlider(Plateau);
    sleep(5);
    demoPlate();
    sleep(5);
    //moveSlider(Bas);
    moveStepperSteps(Flaps, 600,0);
    sleep(5);
    PositionPlateau(1);
    servo_deploy(); 
    
    sleep(5);
    servo_idle();
    resetAll(); 
    calibrateAll();
}
=======
>>>>>>> f558d75f005c06c6bac1bed7ab13982647f476ae
