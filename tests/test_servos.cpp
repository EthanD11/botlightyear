#include "SPI_bus.h"
#include "servos.h"
#include "steppers.h"
#include <unistd.h>

SPIBus spi_bus = SPIBus(); 
GPIOPins pins = GPIOPins(); 
Steppers steppers = Steppers(&spi_bus, &pins); 

Flaps servoFlaps = Flaps(&spi_bus); 
GripperDeployer servoGripperDeployer = GripperDeployer(&spi_bus); 
GripperHolder servoGripperHolder = GripperHolder(&spi_bus);


int main(int argc, char const *argv[])
{
    // for (size_t i = 0; i < 6; i++)
    // {
    //     servoFlaps.deploy(); 
    //     sleep(1);
    //     servoFlaps.raise();
    //     sleep(1);
    // }
    
   
    // servoFlaps.raise();
    // sleep(1);

    // servoFlaps.idle();
    
    // servoGripperHolder.close(); 
    // sleep(3);
    // servoGripperHolder.open(); 
    // sleep(3);
    // servoGripperHolder.open_full(); 
    // sleep(3);
    // servoGripperHolder.hold_plant(); 
    // sleep(3);
    // servoGripperHolder.hold_pot(); 
    // sleep(1);
    // servoGripperHolder.idle();

    servoGripperDeployer.deploy();
    
    sleep(3);
    // servoGripperDeployer.half(); 
    // sleep(3);
    // servoGripperDeployer.plantLift();
    // sleep(3);
    servoGripperDeployer.half();
    sleep(3);
    servoGripperDeployer.idle();

    // for (size_t i = 0; i < 5; i++)
    // {
    //     servoGripperHolder.open_full();
    //     sleep(1);
    //     servoGripperHolder.hold_pot();
    //     sleep(1);
    // }
    // servoGripperDeployer.deploy(); 
    // servoGripperHolder.close();
    // servoGripperHolder.open();
    // sleep(1);
    // servoGripperHolder.open_full();

    // sleep(2); 
    // servoGripperDeployer.idle();
    servoGripperHolder.idle();

    return 0;
}
