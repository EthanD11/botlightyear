#include "teensy.h"
#include <stdio.h>

SPIBus spi = SPIBus();
Teensy teensy = Teensy(&spi);

int main(int argc, char const *argv[])
{
    teensy.idle();
    return 0;
}
