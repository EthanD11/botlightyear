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

    /*servoFlaps.deploy(); 
    sleep(3);
    servoFlaps.raise();
    sleep(1);

    servoFlaps.idle();
    
    servoGripperHolder.close(); 
    sleep(3);
    servoGripperHolder.open(); 
    sleep(3);
    servoGripperHolder.open_full(); 
    sleep(3);
    servoGripperHolder.hold_plant(); 
    sleep(3);
    servoGripperHolder.hold_pot(); 
    sleep(1);
    servoGripperHolder.idle();

    servoGripperDeployer.deploy();
    sleep(3);
    servoGripperDeployer.half(); 
    sleep(3);
    servoGripperDeployer.raise();
    sleep(1);
    servoGripperDeployer.idle(); */

    servoGripperHolder.open();

    return 0;
}
