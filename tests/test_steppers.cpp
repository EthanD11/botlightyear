#include "SPI_bus.h"
#include "GPIO.h"
#include "steppers.h"
#include "servos.h"
#include <unistd.h> 
#include <stdio.h>

//#define RESET_CALIBRATE
//#define SETUP_CUSTOM_SPEED_OLD
//#define SETUP_CUSTOM_SPEED_NEW
//#define DEMO_S6
#define TESTS

SPIBus spi_bus = SPIBus();
GPIOPins pins = GPIOPins(); 

Steppers steppers = Steppers(&spi_bus, &pins); 
GripperHolder holder = GripperHolder(&spi_bus); 
GripperDeployer deployer = GripperDeployer(&spi_bus); 

Flaps servoFlaps = Flaps(&spi_bus); 
void TakePotCHAIN() {
    holder.idle();
    deployer.idle();
    steppers.setup_speed(StprSlider, 300,600);
    steppers.reset_all(); 
    steppers.calibrate(StprPlate, CALL_BLOCKING);
    steppers.calibrate(StprSlider, CALL_BLOCKING); 


    steppers.plate_move(0, CALL_BLOCKING); 
    deployer.deploy();

    holder.open_full();
    steppers.slider_move(SliderIntermediateLow, CALL_BLOCKING);
    holder.open();


    steppers.slider_move(SliderLow, CALL_BLOCKING); 
    sleep(2);

    holder.hold_pot();

    steppers.slider_move(SliderHigh, CALL_BLOCKING);
    deployer.pot_deposit();

    steppers.plate_move(3, CALL_BLOCKING); 

    
    steppers.slider_move(SliderDepositPot, CALL_BLOCKING);
    deployer.deploy(); 
    sleep(2);
    holder.open();
    sleep(2);
}

void demoPlate(){
    steppers.plate_move(-3);
    sleep(3);
    for(int i = -2; i<= 3; i++) {
        steppers.plate_move(i);
        sleep(2);
    }
    steppers.plate_move(0);
}

int main(int argc, char const *argv[])
{
    #ifdef TESTS

    // holder.idle();
    // deployer.idle();
    // steppers.reset_all();
    
    // TakePotCHAIN(); 



    // steppers.setup_all_speeds(); 
    // steppers.reset_all(); 
    steppers.calibrate(StprPlate, CALL_BLOCKING); 
    steppers.calibrate(StprSlider, CALL_BLOCKING); 

    //steppers.calibrate(StprFlaps, CALL_BLOCKING); 
    deployer.half();
    holder.hold_plant(); 

    steppers.plate_move(-3, CALL_BLOCKING); 
    steppers.plate_move(0, CALL_BLOCKING); 
    steppers.plate_move(3, CALL_BLOCKING); 
    steppers.plate_move(0, CALL_BLOCKING); 
    // sleep(2);
    deployer.deploy(); 

    // steppers.slider_move(SliderStorage, CALL_BLOCKING); 
    // steppers.slider_move(SliderLow, CALL_BLOCKING); 
    // steppers.slider_move(SliderDepositPot, CALL_BLOCKING);
    // steppers.slider_move(SliderHigh, CALL_BLOCKING); 
    

    // steppers.flaps_move(FlapsPlant, CALL_BLOCKING); 
    // steppers.flaps_move(FlapsOpen, CALL_BLOCKING); 
    // steppers.flaps_move(FlapsPot, CALL_BLOCKING); 
    // steppers.flaps_move(FlapsOpen, CALL_BLOCKING); 



    #endif

    #ifdef RESET_CALIBRATE
    steppers.reset_all(); 
    steppers.calibrate_all();
    sleep(10);
    #endif

    #ifdef SETUP_CUSTOM_SPEED_OLD
    steppers.setup_speed(StprFlaps,5,10); 
    steppers.setup_speed(StprPlate,2,10); 
    steppers.setup_speed(StprSlider,4,10);
    #endif 

    #ifdef SETUP_CUSTOM_SPEED_NEW
    steppers.setup_speed(StprFlaps,500,1000); 
    steppers.setup_speed(StprPlate,200,1000); 
    steppers.setup_speed(StprSlider,400,1000);
    steppers.setup_acc(StprPlate, 5);
    #endif

    #ifdef DEMO_S6
    servoFlaps.deploy(); 
    steppers.flaps_move(FlapsPlant);
    sleep(4);
    steppers.flaps_move(FlapsOpen);
    steppers.slider_move(SliderLow);
    sleep(2);
    servoFlaps.raise(); 
    sleep(3);
    steppers.slider_move(SliderHigh);
    sleep(5);
    demoPlate();
    sleep(5);
    //steppers.slider_move(SliderLow);
    steppers.move(StprFlaps, 600,0);
    sleep(5);
    steppers.plate_move(1);
    servoFlaps.deploy(); 
    
    sleep(5);
    servoFlaps.idle();
    #endif

    sleep(1);
    holder.idle();
    deployer.idle();
    
    return 0;
}

