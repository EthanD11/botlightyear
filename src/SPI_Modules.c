#include "SPI_Modules.h"

unsigned int DE0_handle, Teensy_handle;
const unsigned int sonar_GPIO_Trig = 16;
const unsigned int sonar_GPIO_Echo = 19;

// Converts words from big endian to little endian (and vice versa)
// https://codereview.stackexchange.com/questions/151049/endianness-conversion-in-c
static inline int32_t Reverse32(int32_t value) 
{
    return (((value & 0x000000FF) << 24) |
            ((value & 0x0000FF00) <<  8) |
            ((value & 0x00FF0000) >>  8) |
            ((value & 0xFF000000) >> 24));
}

void init_spi() {
    DE0_handle = spiOpen(SPI_DE0, SPI_SPEED_HZ_DEFAULT, SPI_MODE_DEFAULT);
    Teensy_handle = spiOpen(SPI_TEENSY, SPI_SPEED_HZ_DEFAULT, SPI_MODE_DEFAULT);
}

void close_spi() {
    spiClose(DE0_handle);
    spiClose(Teensy_handle);
}

// ############################################
// -------- Odometers and Encoders ------------
// ############################################

void get_odo_tick(int32_t *tick_left, int32_t *tick_right) {

    // left
    char send[] = {0x03,0,0,0,0};
    char receive[5];
    spiXfer(DE0_handle, send, receive, 5);
    *tick_left = *((int32_t *)(&(receive[1])));
    *tick_left = Reverse32(*tick_left);
    
    // right
    send[0] = 0x04;
    spiXfer(DE0_handle, send, receive, 5);
    *tick_right = *((int32_t *)(&(receive[1])));
    *tick_right = Reverse32(*tick_right);

}

void get_odo_tick_fast(int32_t *tick_left, int32_t *tick_right) {

    
    char send[] = {0x20,0,0,0,0};
    char receive[5];
    spiXfer(DE0_handle, send, receive, 5);
    
    *tick_left  = (receive[1] << 8) + (receive[2] << 16);
    *tick_right = (receive[3] << 8) + (receive[4] << 16);

}

void get_enc_spd(int32_t *spd_left, int32_t*spd_right) {
    // left
    char send[] = {0x01,0,0,0,0};
    char receive[5];
    spiXfer(DE0_handle, send, receive, 5);
    *spd_left = *((int32_t *)(&(receive[1])));
    *spd_left = Reverse32(*spd_left);

    // right
    send[0] = 0x02;
    spiXfer(DE0_handle, send, receive, 5);
    *spd_right = *((int32_t *)(&(receive[1])));
    *spd_right = Reverse32(*spd_right);

}

void get_enc_spd_fast(int32_t *spd_left, int32_t *spd_right) {

    
    char send[] = {0x10,0,0,0,0};
    char receive[5];
    spiXfer(DE0_handle, send, receive, 5);

    *spd_left  = (receive[1] << 16) + (receive[2] << 24);
    *spd_right = (receive[3] << 16) + (receive[4] << 24);

}

void odo_enc_reset() {
    char send[] = {0x7F,0,0,0,0};
    char receive[5];
    spiXfer(DE0_handle, send, receive, 5);
}

// ############################
// -------- Sonars ------------
// ############################

double sonar_ask() {

    gpioTrigger(sonar_GPIO_Trig, 11, 1);
    time_sleep(6e-2);

    char send[] = {0x05,0,0,0,0};
    char receive[5];
    spiXfer(DE0_handle, send, receive, 5);

    // Retrieve travel time (round trip time)
    uint32_t RTT = *((uint32_t*) &(receive[1]));
    RTT = Reverse32(RTT);
    return RTT*3.44e-6; // Compute distance from travel time (d = RTT*v_sound/2)

}

void init_sonar() {
    gpioSetMode(sonar_GPIO_Trig, PI_OUTPUT);
}