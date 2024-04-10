#include "SPI_bus.h"
#include "servos.h"
#include "steppers.h"

SPIBus spi_bus = SPIBus(); 
GPIOPins pins = GPIOPins(); 
Steppers steppers = Steppers(&spi_bus, &pins); 

Flaps servo_flaps = Flaps(&spi_bus); 
GripperDeployer servo_gripper_deployer = GripperDeployer(&spi_bus); 
GripperHolder servo_gripper_holder = GripperHolder(&spi_bus);


int main(int argc, char const *argv[])
{
    // if (init_spi() != 0) return -1;
    // if (test_spi() != 0) exit(2);

    // steppers.reset_all();
    // sleep(1);
    // steppers.calibrate_all();

    // steppers.plate_move(1);
    // sleep(3);
    // steppers.reset_all();

    // servo_flaps.deploy(); 
    // sleep(3);
    // servo_flaps.raise();
    // sleep(1);

    servo_flaps.idle();
    
    // servo_gripper_holder.close(); 
    // sleep(3);
    // servo_gripper_holder.open(); 
    // sleep(3);
    // servo_gripper_holder.hold_plant(); 
    // sleep(3);
    // servo_gripper_holder.hold_pot(); 
    // sleep(1);
    servo_gripper_holder.idle();

    // servo_gripper_deployer.deploy();
    // sleep(3);
    // servo_gripper_deployer.half(); 
    // sleep(3);
    // servo_gripper_deployer.raise();
    // sleep(1);
    servo_gripper_deployer.idle(); 

    return 0;
}
