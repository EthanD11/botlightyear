#include "SPI_Modules.h"

int DE0_handle, Teensy_handle;
//const unsigned int sonar_GPIO_Trig = 16;
//const unsigned int sonar_GPIO_Echo = 19;
const double x_max = 3.0; 
const double y_max = 2.0; 
const double t_max = 2*M_PI; 
const double speed_max = 1.0; 

const char servo_left_dc_deployed = 15;
const char servo_left_dc_raised = 21;
const char servo_right_dc_deployed = 23;
const char servo_right_dc_raised = 17;

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

// ----- Odometers -----

void odo_get_tick(int32_t *tick_left, int32_t *tick_right) {

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

void odo_reset() {
    char send[] = {0x7F,0,0,0,0};
    char receive[5];
    lgSpiXfer(DE0_handle, send, receive, 5);
}

// ----- Sonars -----

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

// ----- Teensy -----

void teensy_path_following(double *x, double *y, int ncheckpoints, double theta_current) {

    size_t message_size = sizeof(char)*2 + sizeof(uint16_t)*(2*ncheckpoints+1);
    // Send vector. Needs to be malloced since it is variabel size
    char *send = (char *) malloc(message_size);
    char *receive = (char *) malloc(message_size);

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
    send_points[2*ncheckpoints] = (uint16_t) (UINT16_MAX*((theta_current+M_PI)/(M_PI*2)));

    lgSpiXfer(Teensy_handle, send, receive, message_size);

    #ifdef VERBOSE
    printf("Sending path following\n");
    for (size_t i = 0; i < message_size; i++)
    {
        printf("%d, %d\n",send[i], receive[i]);
    }
    #endif

    free(send);
    free(receive);
}

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

void servo_cmd(servo_cmd_t command) {
    char send[5];
    send[0] = 0x80;
    switch (command)
    {

    case ServoIdle: 
        send[3] = 0;
        send[4] = 0;
        break;

    case ServoDeploy:
        send[3] = servo_left_dc_deployed;
        send[4] = servo_right_dc_deployed;
        break;
    
    case ServoRaise:
        send[3] = servo_left_dc_raised;
        send[4] = servo_right_dc_raised;
        break;

    default:
        return;
    }
    lgSpiWrite(DE0_handle, send, 5);
}

// ----- Steppers -----

// 8x (8 = ecrire)(x = stepper)
// 1 plateau
// 2 .. config vitesse
// 3 .. config reste
// 4 slider, rail lineaire
// 5 .. config vitesse
// 6 .. config reste
// 7 flaps stepper
// 8 .. config vitesse
// 9 .. config reste

// 00(mode) 0(Signe) 00...00(29->step)
// mode: 
//      00 OFF     
//      01 Idle
//      10 Calibre
//      11 Step + signe direction du stepper (horlogique/antihorlogique)

void stpr_move(steppers_t stepperName, uint32_t steps, int neg) {
    char request; 
    char direction; 

    switch (stepperName) {
        case StprPlate :
            request = 0x82; 
            direction = (neg == 0) ? 0xC0 : 0xE0; 
            break;
        case StprSlider :
            request = 0x86; 
            direction = 0xE0;
            break;
        case StprFlaps :
            request = 0x8A; 
            direction = 0xE0;
            break;
        default : 
            request = 0; 
            direction = 0;
            printf("Error : not a stepper"); 
            return; 
    }   

    char steps1 = (steps & 0xFF0000) >> 16; 
    char steps2 = (steps & 0xFF00) >> 8;
    char steps3 = steps & 0xFF;

    char send[] = {request,direction,steps1,steps2,steps3};
    lgSpiWrite(DE0_handle, send, 5);
}

void flaps_move(flaps_pos_t pos) {
    uint32_t steps; 
    switch (pos)
    {
    case FlapsOpen :
        steps = 0; 
        break;
    case FlapsPlant :
        steps = 2220; 
        break;
    case FlapsPot :
        steps = 2050; 
        break;  
    default:
        printf("Error : not a position \n");
        steps = 0; 
        return;
    }
    stpr_move(StprFlaps, steps, 0); 
}

void slider_move(slider_pos_t pos){
    int steps;
    switch(pos)
    {
    case SliderHigh :
        steps = 0;
        break;
    case SliderLow :
        steps = 5300;
        break;
    case SliderPlate :
        steps = 250;
        break; 
    case SliderTake :
        steps = 1500;
        break;      
    default :
        printf("Error : not a position \n");
        printf("%d\n", pos);
        steps = 0; 
        return;
    }
    stpr_move(StprSlider,steps,0);
}


void plate_move(int pot){
    //pot est une variable allant de -3 a 3 avec 0 la position de repos
    int direction = 0;
    if (pot ==0){
        stpr_move(StprPlate, 0, 0);   
    } else {
        if (pot<0) {
            pot = -pot;
            direction = 1;
        }
        pot = pot - 1;
        double anglePlateau = (PLATEAU_ANGLE_OUVERTURE)/2 + (pot)* (360-PLATEAU_ANGLE_OUVERTURE)/5;
        double angleStepper = anglePlateau * PLATEAU_REDUCTION;
        double ticStepper = angleStepper/360 * PLATEAU_TIC_STEPPER;
        stpr_move(StprPlate,(int)ticStepper,direction);
    }   

}


void stpr_setup_speed(int nominalSpeed, int initialSpeed, steppers_t stepper) {
    char initialSpeed1 = (initialSpeed & 0xFF00) >> 8; 
    char initialSpeed2 = initialSpeed & 0xFF;
    char nominalSpeed1= (nominalSpeed & 0xFF00) >> 8;
    char nominalSpeed2 = nominalSpeed & 0xFF;
    char request; 
    printf("Init speed 1 : %d, Init speed 2 : %d", initialSpeed1, initialSpeed2);
    switch (stepper) {
        case StprPlate :
            request = 0x83; 
            break;
        case StprSlider :
            request = 0x87; 
            break;
        case StprFlaps :
            request = 0x8B; 
            break;
        default : 
            request = 0; 
            printf("Error : not a stepper"); 
            return; 
    }
    
    char send[] = {request,nominalSpeed1,nominalSpeed2, initialSpeed1, initialSpeed2};//2 premier byte pour vitesse de plateau et 2 dernier vitesse calibration
    lgSpiWrite(DE0_handle, send, 5); 
}


void stpr_setup_calib_speed(int calibrationSpeed, int smallCalibrationSpeed, steppers_t stepper) {
    char calibrationSpeed1 = calibrationSpeed/256;
    char calibrationSpeed2 = calibrationSpeed-calibrationSpeed1*256;
    char smallCalibrationSpeed1= smallCalibrationSpeed/256;
    char smallCalibrationSpeed2 = smallCalibrationSpeed-smallCalibrationSpeed1*256;
    char request; 
    switch (stepper) {
        case StprPlate :
            request = 0x84; 
            break;
        case StprSlider :
            request = 0x88; 
            break;
        case StprFlaps :
            request = 0x8C; 
            break;
        default : 
            request = 0; 
            printf("Error : not a stepper"); 
            return; 
    }
    char send[] = {request,calibrationSpeed1, calibrationSpeed2, smallCalibrationSpeed1, smallCalibrationSpeed2};//2 premier byte pour vitesse de plateau et 2 dernier vitesse calibration
    lgSpiWrite(DE0_handle, send, 5); 
}

void stpr_calibrate(steppers_t stepper) {
    char request; 
    char calibDir;
    switch (stepper) {
        case StprPlate :
            request = 0x82; 
            calibDir = 0x80; 
            break;
        case StprSlider :
            request = 0x86; 
            calibDir = 0xA0;
            break;
        case StprFlaps :
            request = 0x8A; 
            calibDir = 0xA0;
            break;
        default : 
            request = 0; 
            calibDir = 0x80;
            printf("Error : not a stepper"); 
            return; 
    }
    char send[] = {request,calibDir,0,0,0};
    lgSpiWrite(DE0_handle, send, 5);
}

void stpr_reset(steppers_t stepper) {
    char request;
    char request2; 


    // Stop steppers
    switch (stepper) {
        case StprPlate :
            request = 0x82; 
            request2 = 0x85;
            break;
        case StprSlider :
            request = 0x86; 
            request2 = 0x89;
            break;
        case StprFlaps :
            request = 0x8A; 
            request2 = 0x8D;
            break;
        default : 
            request = 0; 
            request2 = 2; 
            printf("Error : not a stepper"); 
            return; 
    } 
    char send1[] = {request,0,0,0,0}; // set the Module command to Idle
    lgSpiWrite(DE0_handle, send1, 5);

    char send2[] = {request2,0,1,0,0}; // send 1 to reset the module completely
    lgSpiWrite(DE0_handle, send2, 5);
    //sleep(1);
    send2[2] = 0; 
    lgSpiWrite(DE0_handle, send2, 5); // send 0 to stop resetting

}

void stepper_setup_acc(steppers_t stepper, uint8_t acc) {
    char request;
    // Stop steppers
    switch (stepper) {
        case StprPlate :
            request = 0x85;
            break;
        case StprSlider :
            request = 0x89;
            break;
        case StprFlaps :
            request = 0x8D;
            break;
        default : 
            request = 0; 
            printf("Error : not a stepper"); 
            return; 
    } 

    char send[] = {request,0,0,0,acc}; // send 1 to reset the module completely
    lgSpiWrite(DE0_handle, send, 5);

}
