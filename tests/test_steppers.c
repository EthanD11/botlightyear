#include "SPI_Modules.h"
#include <unistd.h>


#define REDUCTION_PLATEAU 8.5
#define ANGLE_OUVERTURE_PLATEAU 96.72
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

int main(int argc, char const *argv[])
{
    //steppers_t stepper = Plate; 
    steppers_t stepper = Slider; 
    positions_flaps_t position_flaps = Open; 
    
    setupStepperSpeed(5,10,stepper); 
    //sleep(1);

    calibrateStepper(stepper); 
    //moveFlaps(position_flaps); 
    //moveFlapsSteps(2050); //2220 for plant, 2050 for pot

    /*int DE0_handle = lgSpiOpen(0, SPI_DE0, SPI_SPEED_HZ_DEFAULT, SPI_MODE_DEFAULT);
    char send[] = {0x87,0xE0,0,0b00000011,0b00100000};
    lgSpiWrite(DE0_handle, send, 5); //5 = le nombre de bit a ecrire
    lgSpiClose(DE0_handle);*/


    /*for(int i = -3; i<= 3; i++) {
        int pos = 0;
        PositionPlateau(i);
        sleep(5);
    }*/

    
    //PositionPlateau(0);

    return 0;
}
void moveFlapsSteps (int steps) {
    int DE0_handle = lgSpiOpen(0, SPI_DE0, SPI_SPEED_HZ_DEFAULT, SPI_MODE_DEFAULT);
    int numberHexa1 = steps/65536; //division entiere
    int numberHexa2 = (steps-numberHexa1*65536) /256;
    int numberHexa3 = steps-numberHexa1*65536-numberHexa2*256;

    char send[] = {0x87,0xE0,numberHexa1,numberHexa2,numberHexa3};
    lgSpiWrite(DE0_handle, send, 5); //5 = le nombre de bit a ecrire
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
    moveFlapsSteps(steps); 
}
void PositionPlateau(int pot){
    //pot est une variable allant de -3 à 3 avec 0 la position de repos
    int DE0_handle = lgSpiOpen(0, SPI_DE0, SPI_SPEED_HZ_DEFAULT, SPI_MODE_DEFAULT);
    char send[]= {0,0,0,0,0};
    
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
    int direction = 192;// 0xC0
    if (pot ==0){
        //send = {0x81,0xC0,0,0,0}; // Reset position to 0
        send[0] = 0x81;
        send[1] = 0xC0;
        
    } else if (pot != 0){
        if (pot<0) {
            direction = 224;//0xE0; 1110 0000
            pot = -pot;
        }
        pot = pot - 1;
        int anglePlateau = (ANGLE_OUVERTURE_PLATEAU)/2 + (pot)* (360-ANGLE_OUVERTURE_PLATEAU)/6;
        int angleStepper = anglePlateau * REDUCTION_PLATEAU;
        int ticStepper = angleStepper/360 * TIC_STEPPER_PLATEAU;
        int numberHexa1 = ticStepper/65536; //division entiere
        int numberHexa2 = (ticStepper-numberHexa1*65536) /256;
        int numberHexa3 = ticStepper-numberHexa1*65536-numberHexa2*256;
        //send[] = {0x81,direction,numberHexa1,numberHexa2,numberHexa3};
        send[0] = 0x81;
        send[1] = direction;
        send[2] = numberHexa1;
        send[3] = numberHexa2;
        send[4] = numberHexa3;
    }   
    //char send[] = {0x87,0x40,0,0,0}; // IDLE 
    //char send[] = {0x83,0x80,0,0,0}; // calibre
    //char send[] = {0x83,0xC0,0,0b00000110,0b01000000}; // FULL TURN = 1600 steps pour stepper d'Arnaud (17hs16-2004s1)
    //char send[] = {0x83,0xC0,0,0b00011001,0}; // 4 full turns to test precision
    //char send[] = {0x83,0xC0,0,0b00110010,0}; // 8 full turns to test precision
    //char send[] = {0x83,0xC0,0,0b01100100,0}; // 16 full turns to test precision
    //char send[] = {0x83,0xDF,0xFF,0xF9,0xC0 }; // -1 full turn = -1600
    lgSpiWrite(DE0_handle, send, 5); //5 = le nombre de bit a ecrire
    lgSpiClose(DE0_handle);
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
            calibDir = 0x80;
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
/*
int main(int argc, char const *argv[])
{   
    // 10 -> 8a ecrire dans le control des switch
    //si marche pas, envoie un 1 pour trigger en regardant toute les valeurs des switch en mode cyclic
    int DE0_handle = lgSpiOpen(0, SPI_DE0, SPI_SPEED_HZ_DEFAULT, SPI_MODE_DEFAULT);
    char send[] = {0x8a,0,0,0,1};
    lgSpiWrite(DE0_handle, send, 5);
    lgSpiClose(DE0_handle);
    return 0;
}*/