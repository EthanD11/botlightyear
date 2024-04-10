#include "SPI_bus.h"


SPIBus spi_bus = SPIBus(); 

int main(int argc, char const *argv[])
{
    char send[] = {0x81,0,0,0,1};
    spi_bus.lock(); 
    spi_bus.DE0_write(send); 
    spi_bus.unlock();
    return 0;
}
