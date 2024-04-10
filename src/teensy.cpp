#include "teensy.h"
#include <stdint.h>
#include <stdio.h>
#include <cmath>

//#define VERBOSE

#define SATURATE(a,lb,ub) ((a) > (ub) ? (ub) : ((a) < (lb) ? (lb) : (a)))

typedef enum {
	QueryIdle,
	QueryDoPositionControl,
	QueryDoSpeedControl,
	QueryDoSetDutyCycle,
	QueryDoPathFollowing,
	QueryDoConstantDutyCycle,
	QueryAskState,
	QuerySetPosition,
	QuerySetPositionControlGains,
	QuerySetPathFollowerGains
} query_t;

void Teensy::path_following(double *x, double *y, int ncheckpoints,
                        double theta_start, double theta_end,
                        double vref, double dist_goal_reached) {

    size_t message_size = sizeof(char)*2 + sizeof(uint16_t)*(2*ncheckpoints+4);
    // Send vector. Needs to be malloced since it is variable size
    char send[message_size];

    // Send the query over a single byte
    char *send_query = (char *) send;
    send_query[0] = (char) QueryDoPathFollowing;

    // Send the number of points over a single byte. Hence limited to 255 points
    char *send_n =(char *) (send_query + sizeof(char));
    send_n[0] = (char) ncheckpoints;
    
    // Send each points over two bytes
    uint16_t *send_points = (uint16_t *) (send_n + sizeof(char)); // Send points over 2 bytes   
    for (int i = 0; i < ncheckpoints; i++)              send_points[i] = (uint16_t) (UINT16_MAX*(x[i]/2.0));
    for (int i = 0; i < ncheckpoints; i++) send_points[i+ncheckpoints] = (uint16_t) (UINT16_MAX*(y[i]/3.0));
    send_points[2*ncheckpoints]   = (uint16_t) (UINT16_MAX*((theta_start+M_PI)/(M_PI*2)));
    send_points[2*ncheckpoints+1] = (uint16_t) (UINT16_MAX*((theta_end+M_PI)/(M_PI*2)));
    send_points[2*ncheckpoints+2] = (uint16_t) (UINT16_MAX*(vref/2.0));
    send_points[2*ncheckpoints+3] = (uint16_t) (UINT16_MAX*(dist_goal_reached/3.0));

    #ifdef VERBOSE
    char receive[message_size];
    bus->lock();
    bus->Teensy_xfer(send, receive, message_size);
    bus->unlock();
    printf("Sending path following\n");
    for (size_t i = 0; i < message_size; i++)
    {
        printf("%d, %d\n", send[i], receive[i]);
    }
    #else
    bus->lock();
    bus->Teensy_write(send, message_size);
    bus->unlock();
    #endif

}

void Teensy::set_position(double x, double y, double theta) {

    char send[7];
    send[0] = (char) QuerySetPosition; 
    
    uint16_t *send_ref = (uint16_t *) (send + sizeof(char));
    send_ref[0] = (uint16_t) (UINT16_MAX*(x/2.0));  // xr compressed
    send_ref[1] = (uint16_t) (UINT16_MAX*(y/3.0));  // yr compressed
    send_ref[2] = (uint16_t) (UINT16_MAX*((theta+M_PI)/(M_PI*2)));  // tr compressed

    #ifdef VERBOSE
    char receive[7];
    bus->lock();
    bus->Teensy_xfer(send, receive, 7);
    bus->unlock();
    printf("Sending Set Position\n");
    for (int i = 0; i < 7; i++)
    {
        printf("%d, %d\n",send[i], receive[i]);
    }
    #else
    bus->lock();
    bus->Teensy_write(send, 7);
    bus->unlock();
    #endif

}

void Teensy::pos_ctrl(double xr, double yr, double theta_r) {
    // Compression to go to SPI
    char send[7];
    send[0] = (char) QueryDoPositionControl; 
    
    uint16_t *send_ref = (uint16_t *) (send + sizeof(char));
    send_ref[0] = (uint16_t) (UINT16_MAX*(xr/2.0));  // xr compressed
    send_ref[1] = (uint16_t) (UINT16_MAX*(yr/3.0));  // yr compressed
    send_ref[2] = (uint16_t) (UINT16_MAX*((theta_r+M_PI)/(M_PI*2)));  // tr compressed

    #ifdef VERBOSE
    char receive[7];
    bus->lock();
    bus->Teensy_xfer(send, receive, 7);
    bus->unlock();
    printf("Sending Position ctrl \n");
    for (int i = 0; i < 7; i++)
    {
        printf("%d, %d\n",send[i], receive[i]);
    }
    #else
    bus->lock();
    bus->Teensy_write(send, 7);
    bus->unlock();
    #endif
}

void Teensy::constant_dc(int dc_refl, int dc_refr) {
    // Compression to go to SPI
    char send[5];
    send[0] = (char) QueryDoConstantDutyCycle;
    uint16_t *send_ref = (uint16_t*) (send + sizeof(char));
    send_ref[0] = (uint16_t) (((double) SATURATE(dc_refl, -255,255))+255.0);  // speed_left compressed
    send_ref[1] = (uint16_t) (((double) SATURATE(dc_refr, -255, 255))+255.0);   // speed_right compressed

    #ifdef VERBOSE
    char receive[5];
    bus->lock();
    bus->Teensy_xfer(send, receive, 5);
    bus->unlock();
    printf("Sending constant DC ctrl \n");
    for (int i = 0; i < 5; i++)
    {
        printf("%d, %d\n",send[i], receive[i]);
    }
    #else
    bus->lock();
    bus->Teensy_write(send, 5);
    bus->unlock();
    #endif
}

void Teensy::idle() {
    char send = (char) QueryIdle;
    bus->lock();
    bus->Teensy_write(&send, 1);
    bus->unlock();
}

int Teensy::ask_mode() {
    char send[4];
    char receive[4];
    send[0] = QueryAskState;
    send[1] = 0; // Not needed
    send[2] = 1; // Not needed
    send[3] = 2; // Not needed
    bus->lock();
    bus->Teensy_xfer(send, receive, 4);
    bus->unlock();
    #ifdef VERBOSE
    printf("Asking state\n");
    for (int i = 0; i < 4; i++){
        printf("%d, %d\n", send[i], receive[i]);
    }
    #endif
    return (int) receive[2];
}

void Teensy::set_position_controller_gains(double kp, double ka, double kb, double kw) {
    // Compression to go to SPI
    char send[9];
    send[0] = (char) QuerySetPositionControlGains;

    uint16_t *send_ref = (uint16_t *) (send + sizeof(char));
    send_ref[0] = (uint16_t) ((kp/20)*UINT16_MAX);   // speed_left compressed
    send_ref[1] = (uint16_t) ((ka/20)*UINT16_MAX);   // speed_right compressed
    send_ref[2] = (uint16_t) ((-kb/20)*UINT16_MAX);   // speed_right compressed
    send_ref[3] = (uint16_t) ((kw/50)*UINT16_MAX);   // speed_right compressed

    #ifdef VERBOSE
    char receive[9];
    bus->lock();
    bus->Teensy_xfer(send, receive, 9);
    bus->unlock();
    printf("Sending Speed ctrl \n");
    for (int i = 0; i < 9; i++)
    {
        printf("%d, %d\n",send[i], receive[i]);
    }
    #else
    bus->lock();
    bus->Teensy_write(send, 9);
    bus->unlock();
    #endif
}

void Teensy::set_path_following_gains(double kt, double kn, double kz,
                                    double sigma, double epsilon, double kv_en, double delta, double wn) {

    // Compression to go to SPI
    char send[17];
    send[0] = (char) QuerySetPathFollowerGains;

    uint16_t *send_ref = (uint16_t *) (send + sizeof(char));
    send_ref[0] = (uint16_t) ((kt/50)*UINT16_MAX);   // speed_left compressed
    send_ref[1] = (uint16_t) ((kn)*UINT16_MAX);   // speed_right compressed
    send_ref[2] = (uint16_t) ((kz/200)*UINT16_MAX);   // speed_right compressed
    send_ref[3] = (uint16_t) ((sigma/20)*UINT16_MAX);   // speed_right compressed
    send_ref[4] = (uint16_t) ((epsilon)*UINT16_MAX);   // speed_left compressed
    send_ref[5] = (uint16_t) ((kv_en/100)*UINT16_MAX);   // speed_right compressed
    send_ref[6] = (uint16_t) ((delta)*UINT16_MAX);   // speed_right compressed
    send_ref[7] = (uint16_t) ((wn/100)*UINT16_MAX);   // speed_right compressed

    

    #ifdef VERBOSE
    char receive[17];
    bus->lock();
    bus->Teensy_xfer(send, receive, 17);
    bus->unlock();
    printf("Sending Speed ctrl \n");
    for (int i = 0; i < 17; i++)
    {
        printf("%d, %d\n",send[i], receive[i]);
    }
    #else
    bus->lock();
    bus->Teensy_write(send, 17);
    bus->unlock();
    #endif
}