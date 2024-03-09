#include "SPI_Modules.h"

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

int main(int argc, char const *argv[])
{
    int init = init_spi();
    
    setupStepperSpeed(5,10,Flaps); 
    setupStepperSpeed(2,10,Plate); 
    setupStepperSpeed(4,10,Slider);

    /*
    resetStepperModule (Flaps);
    resetStepperModule (Plate);
    resetStepperModule (Slider);*/

    //steppers_t stepper = Plate; 
    steppers_t stepper = Slider; 
    position_slider_t pos = Bas;
    //positions_flaps_t position_flaps = Plant; 



    //resetStepperModule (stepper);
    /*
    calibrateStepper(Flaps);
    calibrateStepper(Plate);
    calibrateStepper(Slider);*/

    demoS6();
    
    close_spi();
    
    return 0;
}

void calibrateAll() {
    calibrateStepper(Flaps);
    calibrateStepper(Plate);
    calibrateStepper(Slider);
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
    calibrateAll();
    sleep(20);
    moveFlaps(Plant);
    sleep(4);
    moveFlaps(Open);
    sleep(1);
    moveSlider(Bas);
    sleep(5);
    moveSlider(Plateau);
    sleep(5);
    demoPlate();
    sleep(8);
    moveSlider(Bas);
}