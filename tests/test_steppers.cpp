#include "SPI_bus.h"
#include "steppers.h"
#include <lgpio.h>

//#define RESET_CALIBRATE
// #define SETUP_CUSTOM_SPEED_OLD
// #define SETUP_CUSTOM_SPEED_NEW
#define DEMO_S6
#define TESTS

SPIBus spi_bus = SPIBus(); 

Steppers steppers = SPIUser(); 


void demoPlate(){
    steppers.plate_move(-3);
    lguSleep(3);
    for(int i = -2; i<= 3; i++) {
        steppers.plate_move(i);
        lguSleep(2);
    }
    steppers.plate_move(0);
}

int main(int argc, char const *argv[])
{

    #ifdef TESTS  
    
    init_spi2(); 
    steppers.reset_all();
    steppers.calibrate_all();


    steppers.plate_move(-3, CALL_BLOCKING);
    steppers.plate_move(0, CALL_BLOCKING);
    steppers.plate_move(3, CALL_BLOCKING);
    spi_close2(); 

    
    // steppers.setup_speed(StprPlate,60,500); //60 max
    // demoPlate();
    // steppers.plate_move(-3);
    // steppers.plate_move(3);

    

    
    // steppers.setup_speed(StprFlaps,100,400);
    // steppers.flaps_move(FlapsPlant);
    // steppers.flaps_move(FlapsOpen);
    // steppers.flaps_move(FlapsPot);
    // steppers.flaps_move(FlapsOpen);

    #endif

    #ifdef RESET_CALIBRATE
    steppers.reset_all(); 
    steppers.calibrate_all();
    lguSleep(10);
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
    servo_cmd(ServoDeploy); 
    steppers.flaps_move(FlapsPlant);
    lguSleep(4);
    steppers.flaps_move(FlapsOpen);
    steppers.slider_move(SliderLow);
    lguSleep(2);
    servo_cmd(ServoRaise);
    lguSleep(3);
    steppers.slider_move(SliderPlate);
    lguSleep(5);
    demoPlate();
    lguSleep(5);
    //slider_move(SliderLow);
    steppers.move(StprFlaps, 600,0);
    lguSleep(5);
    steppers.plate_move(1);
    servo_cmd(ServoDeploy); 
    
    lguSleep(5);
    servo_cmd(ServoIdle);
    steppers.reset_all(); 
    steppers.calibrate_all();
    #endif
    
    return 0;
}

