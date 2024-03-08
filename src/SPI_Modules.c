#include "SPI_Modules.h"

int DE0_handle, Teensy_handle;
const unsigned int sonar_GPIO_Trig = 16;
const unsigned int sonar_GPIO_Echo = 19;
const double x_max = 3.0; 
const double y_max = 2.0; 
const double t_max = 2*3.141593; 
const double speed_max = 1.0; 
int servo_deployed = 0;
// TODO : calibrate with flaps
const char servo_left_dc_deployed = 19;
const char servo_left_dc_raised = 30;
const char servo_right_dc_deployed = 19;
const char servo_right_dc_raised = 5;

// Converts words from big endian to little endian (and vice versa)
// https://codereview.stackexchange.com/questions/151049/endianness-conversion-in-c
static inline int32_t Reverse32(int32_t value) 
{
    return (((value & 0x000000FF) << 24) |
            ((value & 0x0000FF00) <<  8) |
            ((value & 0x00FF0000) >>  8) |
            ((value & 0xFF000000) >> 24));
}

int init_spi() {
    DE0_handle = lgSpiOpen(0, SPI_DE0, SPI_SPEED_HZ_DEFAULT, SPI_MODE_DEFAULT);
    Teensy_handle = lgSpiOpen(0, SPI_TEENSY, SPI_SPEED_HZ_DEFAULT, SPI_MODE_DEFAULT);
    return (DE0_handle < 0) | (Teensy_handle  < 0); 
}

void close_spi() {
    lgSpiClose(DE0_handle);
    lgSpiClose(Teensy_handle);
}

// ############################################
// -------- Odometers and Encoders ------------
// ############################################

void get_odo_tick(int32_t *tick_left, int32_t *tick_right) {

    // left
    char send[] = {0x03,0,0,0,0};
    char receive[5];
    lgSpiXfer(DE0_handle, send, receive, 5);
    *tick_left = *((int32_t *)(&(receive[1])));
    *tick_left = Reverse32(*tick_left);
    
    // right
    send[0] = 0x04;
    lgSpiXfer(DE0_handle, send, receive, 5);
    *tick_right = *((int32_t *)(&(receive[1])));
    *tick_right = Reverse32(*tick_right);

}

/*
void get_odo_tick_fast(int32_t *tick_left, int32_t *tick_right) {

    
    char send[] = {0x20,0,0,0,0};
    char receive[5];
    lgSpiXfer(DE0_handle, send, receive, 5);
    
    *tick_left  = (receive[1] << 8) + (receive[2] << 16);
    *tick_right = (receive[3] << 8) + (receive[4] << 16);

}*/

/*void get_enc_spd(int32_t *spd_left, int32_t*spd_right) {
    // left
    char send[] = {0x01,0,0,0,0};
    char receive[5];
    lgSpiXfer(DE0_handle, send, receive, 5);
    *spd_left = *((int32_t *)(&(receive[1])));
    *spd_left = Reverse32(*spd_left);

    // right
    send[0] = 0x02;
    lgSpiXfer(DE0_handle, send, receive, 5);
    *spd_right = *((int32_t *)(&(receive[1])));
    *spd_right = Reverse32(*spd_right);

}*/

/*
void get_enc_spd_fast(int32_t *spd_left, int32_t *spd_right) {

    
    char send[] = {0x10,0,0,0,0};
    char receive[5];
    lgSpiXfer(DE0_handle, send, receive, 5);

    *spd_left  = (receive[1] << 16) + (receive[2] << 24);
    *spd_right = (receive[3] << 16) + (receive[4] << 24);

}*/

void odo_reset() {
    char send[] = {0x7F,0,0,0,0};
    char receive[5];
    lgSpiXfer(DE0_handle, send, receive, 5);
}

// ############################
// -------- Sonars ------------
// ############################
/*
double sonar_ask() {

    lgTxPulse(sonar_GPIO_Trig, 11, 1);
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
}*/

// ############################
// -------- Teensy ------------
// ############################

void teensy_pos_ctrl(double x, double y, double t, double xr, double yr, double tr) {
    // Compression to go to SPI
    char send[7];
    send[0] = (char) 3; 
    send[1] = (char) (x*255/x_max);   // x compressed
    send[2] = (char) (y*255/y_max);   // y compressed
    send[3] = (char) ((t+t_max/2)*255/t_max);   // t compressed
    send[4] = (char) (xr*255/x_max);  // xr compressed
    send[5] = (char) (yr*255/y_max);  // yr compressed
    send[6] = (char) ((tr+t_max/2)*255/t_max);  // tr compressed

    char receive[7];
    lgSpiXfer(Teensy_handle, send, receive, 7);

    #ifdef VERBOSE
    printf("Sending Position ctrl \n");
    for (int i = 0; i < 7; i++)
    {
        printf("%d, %d\n",send[i], receive[i]);
    }
    #endif
}

void teensy_spd_ctrl(double speed_left, double speed_right) {
    // Compression to go to SPI
    char send[3];
    char receive[3];
    send[0] = (char) 4; 
    send[1] = (char) (speed_left*255/speed_max);   // speed_left compressed
    send[2] = (char) (speed_right*255/speed_max);   // speed_right compressed

    lgSpiXfer(Teensy_handle, send, receive, 3);

    #ifdef VERBOSE
    printf("Sending Speed ctrl \n");
    for (int i = 0; i < 3; i++)
    {
        printf("%d, %d\n",send[i], receive[i]);
    }
    #endif
    
}

void teensy_idle() {
    char send = 0;
    lgSpiWrite(Teensy_handle, &send, 1);
}

int servo_toggle() {
    servo_deployed = 1-servo_deployed; // Invert value
    char send[5];
    send[0] = 0x80;
    send[3] = (servo_deployed) ? servo_left_dc_deployed : servo_left_dc_raised;
    send[4] = (servo_deployed) ? servo_right_dc_deployed : servo_right_dc_raised;
    lgSpiWrite(DE0_handle, send, 5);
    return servo_deployed;
}

void servo_idle() {
    char send[5];
    send[0] = 0x80;
    send[3] = 0;
    send[4] = 0;
    lgSpiWrite(DE0_handle, send, 5);
}