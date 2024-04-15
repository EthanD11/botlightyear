#include "action_plants.h"
#include <unistd.h>

/*
WARNING : TO BE TESTED FIRST
Takes plant and puts it to the plate storage specified
*/
void take_plant_kinematicChain(int8_t slotNumber) {
    GripperHolder* holder = shared.grpHolder; 
    GripperDeployer* deployer = shared.grpDeployer; 
    Steppers* steppers = shared.steppers; 

    steppers->slider_move(SliderHigh, CALL_BLOCKING);
    deployer->half(); 
    steppers->plate_move(0, CALL_BLOCKING); 

    deployer->deploy();

    holder->open_full();

    steppers->slider_move(SliderLow, CALL_BLOCKING); 

    holder->hold_plant();

    steppers->slider_move(SliderHigh, CALL_BLOCKING);
    deployer->half(); 

    steppers->plate_move(slotNumber, CALL_BLOCKING); 

    
    steppers->slider_move(SliderStorage, CALL_BLOCKING);
    deployer->deploy(); 
    usleep(100000);
    holder->open_full();
    usleep(100000);

    steppers->slider_move(SliderHigh, CALL_BLOCKING); 

    holder->idle();
    deployer->idle();
}