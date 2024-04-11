#ifndef _BLY_ODOMETRY_H_
#define _BLY_ODOMETRY_H_

#include "SPI_bus.h"
#include <stdint.h>

#define ODO_TICKS_TO_M 1.7257283863713464e-05 // Conversion factor ticks to meters. (theoretical : pi*45e-3/8192; practical : 610e-3/(2**3+2**4+2**6+4*2**11))

class Odometry : private SPIUser
{
public:
    Odometry(SPIBus *bus) : SPIUser(bus) {}
    void reset();
    void get_pos(double *x, double *y, double *theta);
    void set_pos(double x, double y, double theta);
    void get_ticks(int32_t *ticksLeft, int32_t *ticksRight);
};

#endif