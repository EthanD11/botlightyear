#include "steppers.h"
#include <stdio.h>

// Steppers
#define FALSE 0

#define PLATEAU_REDUCTION 8.5
#define PLATEAU_ANGLE_OUVERTURE 103.33
#define PLATEAU_TIC_STEPPER 1600

void Steppers::move(steppers_t stepperName, uint32_t steps, uint8_t neg, uint8_t blocking) {
    char request; 
    char direction; 

    switch (stepperName) {
        case StprPlate :
            request = 0x82; 
            direction = (neg == 0) ? 0xC0 : 0xE0; 
            break;
        case StprSlider :
            request = 0x86; 
            direction = 0xE0;
            break;
        case StprFlaps :
            request = 0x8A; 
            direction = 0xE0;
            break;
        default : 
            request = 0; 
            direction = 0;
            printf("Error : not a stepper %d \n", stepperName); 
            return; 
    }   

    char steps1 = (steps & 0xFF0000) >> 16; 
    char steps2 = (steps & 0xFF00) >> 8;
    char steps3 = steps & 0xFF;

    char send[] = {request,direction,steps1,steps2,steps3};
    bus->lock();
    bus->DE0_write(send);
    bus->unlock();

    if (blocking == CALL_BLOCKING) {
        
        GPIO_t stepper_gpio; 
        switch (stepperName) {
            case StprPlate :
                stepper_gpio = StprPlateGPIO; 
                break;
            case StprSlider :
                stepper_gpio = StprSliderGPIO; 
                break;
            case StprFlaps :
                stepper_gpio = StprFlapsGPIO; 
                break;
            default : 
                printf("Error : not a stepper %d \n", stepperName); 
                return; 
        }
        pins->wait_for_gpio_value(stepper_gpio, 1, 10000); 
    }
}

void Steppers::calibrate(steppers_t stepperName, uint8_t blocking, uint8_t*valids) {
    char request; 
    char calibDir;
    switch (stepperName) {
        case StprPlate :
            request = 0x82; 
            calibDir = 0x80; 
            plate_pos = 0; 
            break;
        case StprSlider :
            request = 0x86; 
            calibDir = 0xA0;
            slider_pos = SliderHigh; 
            break;
        case StprFlaps :
            request = 0x8A; 
            calibDir = 0xA0;
            flaps_pos = FlapsOpen; 
            break;
        default : 
            request = 0; 
            calibDir = 0x80;
            printf("Error : not a stepper %d \n", stepperName); 
            return; 
    }
    char send[] = {request,calibDir,0,0,0};
    bus->lock();
    bus->DE0_write(send);
    bus->unlock();

    if (blocking == CALL_BLOCKING) {
        
        GPIO_t stepper_gpio; 
        uint8_t validityID;
        switch (stepperName) {
            case StprPlate :
                stepper_gpio = StprPlateGPIO; 
                validityID = 0; 
                break;
            case StprSlider :
                stepper_gpio = StprSliderGPIO; 
                validityID = 1; 
                break;
            case StprFlaps :
                stepper_gpio = StprFlapsGPIO; 
                validityID = 2; 
                break;
            default : 
                printf("Error : not a stepper %d \n", stepperName); 
                return; 
        }
        int8_t wait_res = pins->wait_for_gpio_value(stepper_gpio, 1, 20000); 
        if (valids != NULL) valids[validityID] = (wait_res == 0); 
    }   
}

void Steppers::flaps_move(flaps_pos_t pos, uint8_t blocking) {
    uint32_t steps; 
    switch (pos)
    {
    case FlapsOpen :
        steps = 0; 
        break;
    case FlapsPlant :
        steps = 3250; 
        break;
    case FlapsApproachPlant:
        steps = 1000;
        break;
    case FlapsPot :
        steps = 2920; 
        break; 
    case FlapsIntermediatePot:
        steps = 1400;  //a iterer...
        break; 
    default:
        printf("Error : not a position %d \n", pos);
        steps = 0; 
        return;
    }
    move(StprFlaps, steps, 0, blocking); 
    flaps_pos = pos; 
}

void Steppers::slider_move(slider_pos_t pos, uint8_t blocking){
    int steps;
    switch(pos)
    {
    case SliderHigh :
        steps = 100;
        break;
    case SliderLow :
        steps = 5300;
        break;
    case SliderIntermediateLow: //position pour jardiniere
        steps = 3650;
        break;
    case SliderStorage : // Deposit plant, take plant, take pot
        steps = 1800;
        break;     
    case SliderDepositPot : 
        steps = 1300;
        break; 
    case SliderPreparePot :
        steps = 3000;
        break;
    default :
        printf("Error : not a position : %d \n", pos);
        steps = 0; 
        return;
    }
    move(StprSlider,steps,0, blocking);
    slider_pos = pos; 
}


void Steppers::plate_move(int8_t pot, uint8_t blocking){
    //pot est une variable allant de -3 a 3 avec 0 la position de repos
    int direction = 0;
    int offset = 70;
    if (pot == 0){
        move(StprPlate, 0, 0,blocking);   
    } else {
        if (pot < 0) {
            pot = -pot;
            direction = 1;
            offset = -70;
        }
        pot = pot - 1;
        double anglePlateau = (PLATEAU_ANGLE_OUVERTURE)/2 + (pot)* (360-PLATEAU_ANGLE_OUVERTURE)/5;
        double angleStepper = anglePlateau * PLATEAU_REDUCTION;
        double ticStepper = angleStepper/360 * PLATEAU_TIC_STEPPER -offset;
        move(StprPlate,(int)ticStepper,direction, blocking);
    }   
    plate_pos = pot; 
}


void Steppers::setup_speed(steppers_t stepperName, int nominalSpeed, int initialSpeed) {
    char initialSpeed1 = (initialSpeed & 0xFF00) >> 8; 
    char initialSpeed2 = initialSpeed & 0xFF;
    char nominalSpeed1= (nominalSpeed & 0xFF00) >> 8;
    char nominalSpeed2 = nominalSpeed & 0xFF;
    char request; 
    switch (stepperName) {
        case StprPlate :
            request = 0x83; 
            break;
        case StprSlider :
            request = 0x87; 
            break;
        case StprFlaps :
            request = 0x8B; 
            break;
        default : 
            request = 0; 
            printf("Error : not a stepper"); 
            return; 
    }
    
    char send[] = {request,nominalSpeed1,nominalSpeed2, initialSpeed1, initialSpeed2};
    bus->lock();
    bus->DE0_write(send);
    bus->unlock();
}


void Steppers::setup_calib_speed(steppers_t stepperName, int calibrationSpeed, int smallCalibrationSpeed) {
    char calibrationSpeed1 = calibrationSpeed/256;
    char calibrationSpeed2 = calibrationSpeed-calibrationSpeed1*256;
    char smallCalibrationSpeed1= smallCalibrationSpeed/256;
    char smallCalibrationSpeed2 = smallCalibrationSpeed-smallCalibrationSpeed1*256;
    char request; 
    switch (stepperName) {
        case StprPlate :
            request = 0x84; 
            break;
        case StprSlider :
            request = 0x88; 
            break;
        case StprFlaps :
            request = 0x8C; 
            break;
        default : 
            request = 0; 
            printf("Error : not a stepper %d \n", stepperName); 
            return; 
    }
    char send[] = {request,calibrationSpeed1, calibrationSpeed2, smallCalibrationSpeed1, smallCalibrationSpeed2};
    bus->lock();
    bus->DE0_write(send);
    bus->unlock();
}



void Steppers::reset(steppers_t stepperName) {
    char request;
    char request2; 


    // Stop steppers
    switch (stepperName) {
        case StprPlate :
            request = 0x82; 
            request2 = 0x85;
            break;
        case StprSlider :
            request = 0x86; 
            request2 = 0x89;
            break;
        case StprFlaps :
            request = 0x8A; 
            request2 = 0x8D;
            break;
        default : 
            request = 0; 
            request2 = 2; 
            printf("Error : not a stepper %d \n", stepperName); 
            return; 
    } 
    char send1[] = {request,0,0,0,0}; // set the Module command to Idle
    char send2[] = {request2,0,1,0,0}; // send 1 to reset the module completely

    bus->lock();
    bus->DE0_write(send1);
    bus->DE0_write(send2);
    send2[2] = 0; // send 0 to stop resetting
    bus->DE0_write(send2); 
    bus->unlock();
}

void Steppers::setup_acc(steppers_t stepperName, uint8_t accSteps) {
    char request;
    // Stop steppers
    switch (stepperName) {
        case StprPlate :
            request = 0x85;
            break;
        case StprSlider :
            request = 0x89;
            break;
        case StprFlaps :
            request = 0x8D;
            break;
        default : 
            request = 0; 
            printf("Error : not a stepper %d \n", stepperName);  
            return; 
    } 

    char send[] = {request,0,0,0,accSteps}; // send 1 to reset the module completely
    bus->lock();
    bus->DE0_write(send);
    bus->unlock();

}

void Steppers::setup_all_speeds() {
    setup_speed(StprPlate,80,400); 
    setup_speed(StprSlider,350,450);
    setup_speed(StprFlaps,200,300); 
}

void Steppers::calibrate_all(uint8_t blocking, uint8_t*valids) {
    calibrate(StprFlaps, blocking, valids);
    calibrate(StprPlate, blocking, valids);
    calibrate(StprSlider, blocking, valids);
}

void Steppers::reset_all() {
    reset(StprFlaps);
    reset(StprPlate);
    reset(StprSlider);
}