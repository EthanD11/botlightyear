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
#define PLANT
//#define TESTS

SPIBus spi_bus = SPIBus();
GPIOPins pins = GPIOPins(); 

Steppers* steppers = new Steppers(&spi_bus, &pins); 
GripperHolder* holder = new GripperHolder(&spi_bus); 
GripperDeployer* deployer = new GripperDeployer(&spi_bus); 
Flaps* servoFlaps = new Flaps(&spi_bus); 
Teensy *teensy = new Teensy(&spi_bus, &pins);


void take_plant_kinematicChain(int8_t slotNumber) {

    deployer->deploy(); 
    holder->open_full();
    //approach
    steppers->slider_move(SliderPreparePlant);
    steppers->flaps_move(FlapsApproachPlant);
    servoFlaps->deploy();
    sleep(5);

    // Align plant
    steppers->flaps_move(FlapsPlant, CALL_BLOCKING); 
    steppers->flaps_move(FlapsApproachPlant);

    // take de plant
    steppers->slider_move(SliderLow, CALL_BLOCKING);
    holder->hold_plant();    
    steppers->slider_move(SliderHigh);//tricks pour replier servo au milieu
    usleep(100000);
    deployer->half();
    shared.pins->wait_for_gpio_value(StprSliderGPIO, 1, 10000); 

    //mise dans plateau
    steppers->plate_move(slotNumber, CALL_BLOCKING); 
    steppers->slider_move(SliderIntermediatePlant,CALL_BLOCKING);
    deployer->deploy(); 
    steppers->slider_move(SliderStorage, CALL_BLOCKING);
    holder->open_full();
    usleep(100000);

    //remonte et remise en place
    deployer->half();
    steppers->slider_move(SliderHigh, CALL_BLOCKING); 
    steppers->plate_move(0, CALL_BLOCKING); 
    servoFlaps->raise();
    holder->idle();
    deployer->idle();
    // teensy->set_position_controller_gains(0.8,2.5,-1.5,1.0);
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

    #ifdef PLANT
    take_plant_kinematicChain(1);
    #endif

    sleep(1);
    holder->idle();
    deployer->idle();
    teensy->idle();
    return 0;
}

