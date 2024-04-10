#ifndef _BLY_SERVOS_H_
#define _BLY_SERVOS_H_

#include "SPI_bus.h"

class Flaps : SPIUser
{
public:
    void deploy();
    void raise();
    void idle();
};

#endif