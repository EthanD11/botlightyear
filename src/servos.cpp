#include "servos.h"

void Flaps::deploy() {
    char send[5];
    send[0] = 0x80;
    uint16_t servo_flaps1_duty_cycle = 680; 
    uint16_t servo_flaps2_duty_cycle = 620; 
    
    send[1] = (servo_flaps1_duty_cycle & 0xFF00) >> 8;
    send[2] = servo_flaps1_duty_cycle & 0xFF;

    send[3] = (servo_flaps2_duty_cycle & 0xFF00) >> 8;
    send[4] = servo_flaps2_duty_cycle & 0xFF;
    bus->lock();
    bus->DE0_write(send);
    bus->unlock();
}