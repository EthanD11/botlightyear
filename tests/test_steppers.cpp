#include "SPI_bus.h"
#include "GPIO.h"
#include "steppers.h"
#include "servos.h"
#include <unistd.h> 

//#define RESET_CALIBRATE
// #define SETUP_CUSTOM_SPEED_OLD
// #define SETUP_CUSTOM_SPEED_NEW
#define DEMO_S6
#define TESTS

SPIBus spi_bus = SPIBus();
GPIOPins pins = GPIOPins(); 

Steppers steppers = Steppers(&spi_bus, &pins); 
GripperHolder holder = GripperHolder(&spi_bus); 
GripperDeployer deployer = GripperDeployer(&spi_bus); 

Flaps servo_flaps = Flaps(&spi_bus); 
void TakePotCHAIN() {
    holder.idle();
    deployer.idle();
    steppers.reset_all(); 
    steppers.reset(StprPlate, CALL_BLOCKING)
    steppers.reset(StprSlider, CALL_BLOCKING); 


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
    // steppers.setup_speed(StprFlaps,500,800); 
    // steppers.setup_speed(StprPlate,60,500); 
    // steppers.setup_speed(StprSlider,300,600);
    // steppers.setup_speed(StprFlaps,100,400);
    // steppers.setup_calib_speed(400,500,StprFlaps);

    // holder.idle();
    // deployer.idle();
    // steppers.reset_all();

    // steppers.calibrate(StprPlate, CALL_BLOCKING); 
    // steppers.calibrate(StprSlider, CALL_BLOCKING); 

    // sleep(5);
    // steppers.slider_move(SliderPlate, CALL_BLOCKING);
    // steppers.slider_move(SliderTake, CALL_BLOCKING);
    // steppers.slider_move(SliderDeposit, CALL_BLOCKING);
    // steppers.slider_move(SliderPlate, CALL_BLOCKING);
    //1800 : TAKE-PLANT, TAKE-POT, DEPOSIT-PLANT-not in pot, 
    //1000 : DEPOSIT_POT
    //5300 : butée : slider_low : take plant or pot

    // steppers.move(StprSlider, 5300, 0); 
    // steppers.plate_move(1, CALL_BLOCKING);
    // steppers.calibrate(StprSlider); 
    // steppers.slider_move(SliderDepositPot);
    // holder.hold_pot();
    // holder.open();
    // deployer.pot_deposit();
    // deployer.deploy();
    
    // steppers.reset(StprPlate); 
    // steppers.calibrate(StprPlate, CALL_BLOCKING); 
    // steppers.plate_move(0, CALL_BLOCKING); 


    // deployer.deploy();
    steppers.slider_move(SliderIntermediateLow);

    holder.open();
    

    // steppers.plate_move(3, CALL_BLOCKING);
    // steppers.plate_move(0, CALL_BLOCKING);
    // steppers.plate_move(3, CALL_BLOCKING);
    // steppers.plate_move(0, CALL_BLOCKING);
    // TakePotCHAIN(); 

    
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
    servo_flaps.deploy(); 
    steppers.flaps_move(FlapsPlant);
    sleep(4);
    steppers.flaps_move(FlapsOpen);
    steppers.slider_move(SliderLow);
    sleep(2);
    servo_flaps.raise(); 
    sleep(3);
    steppers.slider_move(SliderPlate);
    sleep(5);
    demoPlate();
    sleep(5);
    //steppers.slider_move(SliderLow);
    steppers.move(StprFlaps, 600,0);
    sleep(5);
    steppers.plate_move(1);
    servo_flaps.deploy(); 
    
    sleep(5);
    servo_flaps.idle();
    steppers.reset_all(); 
    steppers.calibrate_all();
    #endif

    // gripper_holder_cmd(HolderIdle);
    // gripper_deployer_cmd(DeployerIdle);
    
    return 0;
}

