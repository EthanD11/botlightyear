#ifndef _BLY_SERVOS_H_
#define _BLY_SERVOS_H_

#include "SPI_bus.h"
#include <stdint.h>

class Flaps : private SPIUser
{
private:
void send_flaps_dutyCycle(uint16_t servo_flaps1_duty_cycle, uint16_t servo_flaps2_duty_cycle); 
public:
    Flaps(SPIBus *bus) : SPIUser(bus) {}
    void deploy();
    void raise();
    void idle();
};

class GripperDeployer : private SPIUser
{
private:
void send_dutyCycle(uint16_t duty_cycle); 
public:
    GripperDeployer(SPIBus *bus) : SPIUser(bus) {}
    void idle();
    void half();
    void deploy();
    void raise();
};

class GripperHolder : private SPIUser
{
private:
void send_dutyCycle(uint16_t duty_cycle); 
public:
    GripperHolder(SPIBus *bus) : SPIUser(bus) {}
    void idle();
    void close(); 
    void open(); 
    void hold_pot(); 
    void hold_plant(); 
};

#endif