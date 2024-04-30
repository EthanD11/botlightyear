#include "SPI_bus.h"
#include "GPIO.h"
#include "steppers.h"
#include "servos.h"
#include "teensy.h"
#include <unistd.h> 
#include <stdio.h>
#include <cmath>
//#define RESET_CALIBRATE
//#define SETUP_CUSTOM_SPEED_OLD
//#define SETUP_CUSTOM_SPEED_NEW
//#define DEMO_S6
//#define PLANTER
#define POT
//#define TESTS

SPIBus spi_bus = SPIBus();
GPIOPins pins = GPIOPins(); 

Steppers* steppers = new Steppers(&spi_bus, &pins); 
GripperHolder* holder = new GripperHolder(&spi_bus); 
GripperDeployer* deployer = new GripperDeployer(&spi_bus); 
Flaps* servoFlaps = new Flaps(&spi_bus); 

Teensy *teensy = new Teensy(&spi_bus, &pins);


double mappingOnPi(double angle){
    double angleMapped = angle;
    if(angleMapped < -M_PI){
        angleMapped = 2*M_PI + angleMapped;
    }
    if(angleMapped > M_PI){
        angleMapped = angleMapped - 2*M_PI;
    }
    printf("angle initiale : %f angle mappé : %f\n", angle,angleMapped);
    return angleMapped;
}

void TakePotCHAIN(int slotNumber, int numeroPot = 1) {
    //numeroPot :
    //   _____
    //  / 1 2 \ 
    // | 3 4 5 |
    //position des pot de la premiere zone bleu
    double posPotX = 0.6125;
    double posPotY = 0.035;
    double posPotTheta = M_PI_2;
    //teensy->set_position(posPotX,posPotY,mappingOnPi(posPotTheta-M_PI));
    //printf("Starting angle : %f \n", mappingOnPi(posPotTheta-M_PI)); 
    //parametre approach/prise pot 
    double betaPot1 = M_PI_2/3;
    double distanceRoue = 0.3;
    double deltaApproach = 0.2;
    double rayonPot = 0.08;

    double posPotXApproach;
    double posPotYApproach;
    double posPotThetaApproach;
    double posPotXPrise;
    double posPotYPrise;
    double posPotThetaPrise;

    deployer->deploy();
    holder->open();
    //steppers->setup_speed(StprSlider, 300,600);
    //steppers->reset_all(); 
    //steppers->calibrate(StprPlate, CALL_BLOCKING);
    //steppers->calibrate(StprSlider, CALL_BLOCKING); 
    switch (numeroPot)
    {
    case 1 :
        posPotXApproach = posPotX+(distanceRoue+deltaApproach+rayonPot)*cos(betaPot1+posPotTheta);
        posPotYApproach = posPotY+(distanceRoue+deltaApproach+rayonPot)*sin(betaPot1+posPotTheta);
        posPotThetaApproach = posPotTheta+betaPot1-M_PI;
        posPotXPrise = posPotX+(distanceRoue+rayonPot)*cos(betaPot1+posPotTheta);
        posPotYPrise = posPotY+(distanceRoue+rayonPot)*sin(betaPot1+posPotTheta);
        posPotThetaPrise = posPotTheta+betaPot1-M_PI;
        break;
    case 2 :
        posPotXApproach = posPotX+(distanceRoue+deltaApproach+rayonPot)*cos(-betaPot1+posPotTheta);
        posPotYApproach = posPotY+(distanceRoue+deltaApproach+rayonPot)*sin(-betaPot1+posPotTheta);
        posPotThetaApproach = posPotTheta-betaPot1-M_PI;
        posPotXPrise = posPotX+(distanceRoue+rayonPot)*cos(-betaPot1+posPotTheta);
        posPotYPrise = posPotY+(distanceRoue+rayonPot)*sin(-betaPot1+posPotTheta);
        posPotThetaPrise = posPotTheta-betaPot1-M_PI;
        break;
    default:
        break;
    }
    //remapping pour -PI a +PI
    posPotThetaApproach = mappingOnPi(posPotThetaApproach);
    posPotThetaPrise = mappingOnPi(posPotThetaPrise);
    //-----approche grossiere-----
    printf("approche grossiere : %f, %f, %f ,distance :%f\n", posPotXApproach, posPotYApproach, posPotThetaApproach,distanceRoue+deltaApproach+rayonPot);
    teensy->pos_ctrl(posPotXApproach,posPotYApproach,posPotThetaApproach); 
    sleep(10);
    servoFlaps->deploy();
    steppers->flaps_move(FlapsIntermediatePot); 
    steppers->slider_move(SliderPreparePot);
    //-----approche précise-----
    printf("approche précise : %f, %f, %f, distance :%f\n", posPotXPrise, posPotYPrise, posPotThetaPrise,distanceRoue+rayonPot);
    //teensy->pos_ctrl(posPotX+(distanceRoue)*(betaPot1),posPotY+(distanceRoue)*(betaPot1),posPotTheta+betaPot1-M_PI);
    teensy->pos_ctrl(posPotXPrise,posPotYPrise,posPotThetaPrise);
    sleep(10);    
    //-----prise pot-----
    printf("debut chaine cinématique\n");
    steppers->flaps_move(FlapsPot,CALL_BLOCKING);
    steppers->flaps_move(FlapsIntermediatePot,CALL_BLOCKING);
    steppers->flaps_move(FlapsOpen);
    holder->open_full();
    steppers->slider_move(SliderLow,CALL_BLOCKING);
    holder->hold_pot();//fermeture pot

    //remonte
    steppers->slider_move(SliderHigh, CALL_BLOCKING);
    steppers->plate_move(slotNumber, CALL_BLOCKING); 

    deployer->pot_deposit();
    steppers->slider_move(SliderDepositPot, CALL_BLOCKING);
    deployer->deploy();
    steppers->slider_move(SliderStorage, CALL_BLOCKING);
    holder->open_full();
    deployer->half();
    steppers->slider_move(SliderHigh,CALL_BLOCKING);
    steppers->plate_move(0, CALL_BLOCKING);
    holder->idle();
    deployer->idle();
}



int main(int argc, char const *argv[])
{
    printf("Test is running\n");
    //steppers->setup_all_speeds(); 
    steppers->reset_all(); 

    // steppers->calibrate(StprPlate, CALL_BLOCKING, NULL); 

    steppers->calibrate_all(CALL_BLOCKING, NULL);
    steppers->plate_move(0,CALL_BLOCKING);
    printf("Go ! \n");
    teensy->set_position(0.612,0.60,-M_PI_2); //position des pot de la premiere zone bleu
    //steppers->flaps_move(FlapsIntermediatePot,CALL_BLOCKING);
    // holder->hold_pot();
    // steppers->slider_move(SliderHigh, CALL_BLOCKING);
    // steppers->plate_move(2,CALL_BLOCKING);
    // sleep(4);
    // //deployer->pot_deposit();
    // steppers->slider_move(SliderDepositPot, CALL_BLOCKING);
    // deployer->deploy();
    // holder->open();
    #ifdef POT
    TakePotCHAIN(2,1);
    #endif

    sleep(1);
    holder->idle();
    deployer->idle();
    teensy->idle();
    return 0;
}

