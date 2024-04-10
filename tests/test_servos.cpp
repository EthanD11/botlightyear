#include "SPI_bus.h"
#include "servos.h"
#include "steppers.h"
#include <lgpio.h>

SPIBus spi_bus = SPIBus(); 
Steppers steppers = SPIUser(&spi_bus); 

Flaps servo_flaps = SPIUser(&spi_bus); 
GripperDeployer servo_gripper_deployer = SPIUser(&spi_bus); 
GripperHolder servo_gripper_holder = SPIUser(&spi_bus);


int main(int argc, char const *argv[])
{
    // if (init_spi() != 0) return -1;
    // if (test_spi() != 0) exit(2);

    // steppers.reset_all();
    // lguSleep(1);
    // steppers.calibrate_all();

    // steppers.plate_move(1);
    // lguSleep(3);
    // steppers.reset_all();

    // servo_flaps.deploy(); 
    // lguSleep(3);
    // servo_flaps.raise();
    // lguSleep(1);

    servo_flaps.idle();
    
    // servo_gripper_holder.close(); 
    // lguSleep(3);
    // servo_gripper_holder.open(); 
    // lguSleep(3);
    // servo_gripper_holder.hold_plant(); 
    // lguSleep(3);
    // servo_gripper_holder.hold_pot(); 
    // lguSleep(1);
    servo_gripper_holder.idle();

    // servo_gripper_deployer.deploy();
    // lguSleep(3);
    // servo_gripper_deployer.half(); 
    // lguSleep(3);
    // servo_gripper_deployer.raise();
    // lguSleep(1);
    gripper_deployer_cmd(DeployerIdle);
    servo_gripper_deployer.idle(); 

    spi_close();
    return 0;
}
