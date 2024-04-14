#include "odometry.h"

// Converts words from big endian to little endian (and vice versa)
// https://codereview.stackexchange.com/questions/151049/endianness-conversion-in-c
static inline int32_t Reverse32(int32_t value) 
{
    return (((value & 0x000000FF) << 24) |
            ((value & 0x0000FF00) <<  8) |
            ((value & 0x00FF0000) >>  8) |
            ((value & 0xFF000000) >> 24));
}

void Odometry::reset() {
    char send[5]; send[0] = 0x7F;
    bus->lock();
    bus->DE0_write(send);
    bus->unlock();
}

void Odometry::get_pos(double *x, double *y, double *theta) {
    char sendx[5]; char sendy[5]; char sendt[5];
    char receivex[5]; char receivey[5]; char receivet[5];
    sendx[0] = 0x03; sendy[0] = 0x04; sendt[0] = 0x05;
    
    bus->lock();
    bus->DE0_xfer(sendx, receivex);
    bus->DE0_xfer(sendy, receivey);
    bus->DE0_xfer(sendt, receivet);
    bus->unlock();

    int32_t receive_int;
    float *receive_float = (float*) &receive_int;

    if (x != NULL) {
    receive_int = Reverse32(*(int32_t *)(&(receivex[1])));
    *x = (double) *receive_float;
    }
    if (y != NULL) {
    receive_int = Reverse32(*(int32_t *)(&(receivey[1])));
    *y = (double) *receive_float;
    }
    if (theta != NULL) {
    receive_int = Reverse32(*(int32_t *)(&(receivet[1])));
    *theta = (double) *receive_float;
    }
}

void Odometry::set_pos(double x, double y, double theta) {
    char sendx[5]; char sendy[5]; char sendt[5]; char sendr[5]; 
    float x_crop = (float) x, y_crop = (float) y, theta_crop = (float) theta;
    
    int32_t send_float;

    send_float = *((int32_t*)(&x_crop));
    sendx[0] = 0x90;
    sendx[1] = send_float >> 24;
    sendx[2] = (send_float & 0xFF0000) >> 16;
    sendx[3] = (send_float & 0xFF00) >> 8;
    sendx[4] = send_float & 0xFF;

    send_float = *((int32_t*) (&y_crop));
    sendy[0] = 0x91;
    sendy[1] = send_float >> 24;
    sendy[2] = (send_float & 0xFF0000) >> 16;
    sendy[3] = (send_float & 0xFF00) >> 8;
    sendy[4] = send_float & 0xFF;

    send_float = *((int32_t*) (&theta_crop));
    sendt[0] = 0x92;
    sendt[1] = send_float >> 24;
    sendt[2] = (send_float & 0xFF0000) >> 16;
    sendt[3] = (send_float & 0xFF00) >> 8;
    sendt[4] = send_float & 0xFF;

    sendr[0] = 0x7F;

    bus->lock();
    bus->DE0_write(sendx);
    bus->DE0_write(sendy);
    bus->DE0_write(sendt);
    bus->DE0_write(sendr);
    bus->unlock();
}

void Odometry::get_ticks(int32_t *ticksLeft, int32_t *ticksRight) {
    char sendl[5]; sendl[0] = 0x01;
    char sendr[5]; sendr[0] = 0x02;
    char receivel[5];
    char receiver[5];
    bus->lock();
    bus->DE0_xfer(sendl, receivel);
    bus->DE0_xfer(sendr, receiver);
    bus->unlock();

    // left
    *ticksLeft = *((int32_t *)(&(receivel[1])));
    *ticksLeft = Reverse32(*ticksLeft);
    
    // right
    *ticksRight = *((int32_t *)(&(receiver[1])));
    *ticksRight = Reverse32(*ticksRight);
}