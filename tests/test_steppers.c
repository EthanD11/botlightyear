#include "SPI_Modules.h"


#define REDUCTION_PLATEAU 8.5
#define ANGLE_OUVERTURE_PLATEAU 103.33
#define TIC_STEPPER_PLATEAU 1600

typedef enum {
    Plate, 
    Slider, 
    Flaps
} steppers_t;

typedef enum {
    Open, 
    Plant, 
    Pot
} positions_flaps_t;

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

    
    //steppers_t stepper = Plate; 
    steppers_t stepper = Flaps; 
    positions_flaps_t position_flaps = Plant; 

    setupStepperSpeed(5,10,stepper); 
    sleep(1);    
    //resetStepperModule (stepper);
    calibrateStepper(Flaps);
    calibrateStepper(Plate);
    calibrateStepper(Slider);


    //resetStepperModule (stepper)
    /*
    setupStepperSpeed(5,10,stepper); 
    sleep(1);
    calibrateStepper(stepper); */
    //moveFlaps(position_flaps); 
    //moveFlapsSteps(2050); //2220 for plant, 2050 for pot
    //PositionPlateau(0);
    //demoPlateS6();
    
    
    
    return 0;
}

void demoPlateS6(){
    steppers_t stepper = Plate; 
    setupStepperSpeed(2,10,stepper); 
    PositionPlateau(-3);
    sleep(4);
    for(int i = -2; i<= 3; i++) {
        int pos = 0;
        PositionPlateau(i);
        sleep(2);
    }
    PositionPlateau(0);
}
/*
void moveFlapsSteps (int steps) {
    int DE0_handle = lgSpiOpen(0, SPI_DE0, SPI_SPEED_HZ_DEFAULT, SPI_MODE_DEFAULT);
    int numberHexa1 = steps/65536; //division entiere
    int numberHexa2 = (steps-numberHexa1*65536) /256;
    int numberHexa3 = steps-numberHexa1*65536-numberHexa2*256;

    char send[] = {0x87,0xE0,numberHexa1,numberHexa2,numberHexa3};
    lgSpiWrite(DE0_handle, send, 5); //5 = le nombre de bit a ecrire
    lgSpiClose(DE0_handle);
}*/

void moveStepperSteps(steppers_t stepperName, int steps, int neg) {
    int DE0_handle = lgSpiOpen(0, SPI_DE0, SPI_SPEED_HZ_DEFAULT, SPI_MODE_DEFAULT);
    int request; 
    int direction; 

    switch (stepperName) {
        case Plate :
            request = 0x81; 
            direction = (neg == 0) ? 0xC0 : 0xE0; 
            break;
        case Slider :
            request = 0x84; 
            direction = 0xE0;
            break;
        case Flaps :
            request = 0x87; 
            direction = 0xE0;
            break;
        default : 
            request = 0; 
            direction = 0;
            printf("Error : not a stepper"); 
            break; 
    }   

    int steps1 = steps/65536; 
    int steps2 = (steps-steps1*65536) /256;
    int steps3 = steps-steps1*65536-steps2*256;

    char send[] = {request,direction,steps1,steps2,steps3};
    lgSpiWrite(DE0_handle, send, 5);
    lgSpiClose(DE0_handle);
}

void moveFlaps (positions_flaps_t pos) {
    int steps; 
    switch (pos)
    {
    case Open :
        steps = 0; 
        break;
    case Plant :
        steps = 2220; 
        break;
    case Pot :
        steps = 2050; 
        break;  
    default:
        steps = 0; 
        break;
    }
    moveStepperSteps(Flaps, steps, 0); 
}

void PositionPlateau(int pot){
    //pot est une variable allant de -3 Ã  3 avec 0 la position de repos
    int direction = 0;
    if (pot ==0){
        moveStepperSteps(Plate, 0, 0);   
    } else {
        if (pot<0) {
            pot = -pot;
            direction = 1;
        }
        pot = pot - 1;
        double anglePlateau = (ANGLE_OUVERTURE_PLATEAU)/2 + (pot)* (360-ANGLE_OUVERTURE_PLATEAU)/5;
        double angleStepper = anglePlateau * REDUCTION_PLATEAU;
        double ticStepper = angleStepper/360 * TIC_STEPPER_PLATEAU;
        moveStepperSteps(Plate,(int)ticStepper,direction);
    }   

}


void setupStepperSpeed (int nominalSpeed, int calibrationSpeed, steppers_t stepper) {
    int DE0_handle = lgSpiOpen(0, SPI_DE0, SPI_SPEED_HZ_DEFAULT, SPI_MODE_DEFAULT);
    int calibrationSpeed1 = calibrationSpeed/256;
    int calibrationSpeed2 = calibrationSpeed-calibrationSpeed1*256;
    int nominalSpeed1= nominalSpeed/256;
    int nominalSpeed2 = nominalSpeed-nominalSpeed1*256;
    int request; 
    switch (stepper) {
        case Plate :
            request = 0x82; 
            break;
        case Slider :
            request = 0x85; 
            break;
        case Flaps :
            request = 0x88; 
            break;
        default : 
            request = 0; 
            printf("Error : not a stepper"); 
            break; 
    }
    char send[] = {request,nominalSpeed1,nominalSpeed2, calibrationSpeed1, calibrationSpeed2};//2 premier byte pour vitesse de plateau et 2 dernier vitesse calibration
    lgSpiWrite(DE0_handle, send, 5); 
    lgSpiClose(DE0_handle);
}

void calibrateStepper(steppers_t stepper) {
    int DE0_handle = lgSpiOpen(0, SPI_DE0, SPI_SPEED_HZ_DEFAULT, SPI_MODE_DEFAULT);
    int request; 
    int calibDir;
    switch (stepper) {
        case Plate :
            request = 0x81; 
            calibDir = 0x80; 
            break;
        case Slider :
            request = 0x84; 
            calibDir = 0xA0;
            break;
        case Flaps :
            request = 0x87; 
            calibDir = 0xA0;
            break;
        default : 
            request = 0; 
            calibDir = 0x80;
            printf("Error : not a stepper"); 
            break; 
    }
    char send[] = {request,calibDir,0,0,0};
    lgSpiWrite(DE0_handle, send, 5);
    lgSpiClose(DE0_handle);
}

void resetStepperModule (steppers_t stepper) {
    int DE0_handle = lgSpiOpen(0, SPI_DE0, SPI_SPEED_HZ_DEFAULT, SPI_MODE_DEFAULT);
    int request; 
    switch (stepper) {
        case Plate :
            request = 0x83; 
            break;
        case Slider :
            request = 0x86; 
            break;
        case Flaps :
            request = 0x89; 
            break;
        default : 
            request = 0; 
            printf("Error : not a stepper"); 
            break; 
    } 
    char send[] = {request,0,1,0,0};
    lgSpiWrite(DE0_handle, send, 5);
    lgSpiClose(DE0_handle);
    sleep(1);
    
    send[2] = 0; 
    DE0_handle = lgSpiOpen(0, SPI_DE0, SPI_SPEED_HZ_DEFAULT, SPI_MODE_DEFAULT);
    lgSpiWrite(DE0_handle, send, 5);
    lgSpiClose(DE0_handle);
}
