#include "teensy.h"
#include <stdio.h>

SPIBus spiBus = SPIBus();
GPIOPins pins = GPIOPins(); 
Teensy teensy = Teensy(&spiBus, &pins);

int main(int argc, char const *argv[])
{
    teensy.idle();
    return 0;
}
