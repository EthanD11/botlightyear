#include "teensy.h"
#include "SPI_bus.h"
#include "servos.h"
#include "steppers.h"
#include <unistd.h>
#include <stdio.h>

SPIBus spi_bus = SPIBus();
GPIOPins pins = GPIOPins(); 
Teensy teensy = Teensy(&spi_bus, &pins);

Flaps servoFlaps = Flaps(&spi_bus); 
GripperDeployer servoGripperDeployer = GripperDeployer(&spi_bus); 
GripperHolder servoGripperHolder = GripperHolder(&spi_bus);




int main(int argc, char const *argv[])
{
    teensy.idle();
    servoFlaps.idle();
    servoGripperHolder.idle();
    servoGripperDeployer.idle();
    servoGripperDeployer.idle();
    servoGripperHolder.idle();
    
    return 0;
}
