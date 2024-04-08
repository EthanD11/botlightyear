#include "SPI_Modules.h"
#include <pthread.h>

int DE0_handle, Teensy_handle, SPI2_handle;
pthread_mutex_t spi_mutex;
//const unsigned int sonar_GPIO_Trig = 16;
//const unsigned int sonar_GPIO_Echo = 19;

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
    pthread_mutex_init(&spi_mutex,NULL);
    return (DE0_handle < 0) | (Teensy_handle  < 0); 
}

void close_spi() {
    lgSpiClose(DE0_handle);
    lgSpiClose(Teensy_handle);
    pthread_mutex_destroy(&spi_mutex);
}

int test_spi() {

    // TODO : Add test communication with Teensy

    char send[] = {0,0,0,0,0};
    char receive[5];
    uint8_t failure = 0;
    lgSpiXfer(DE0_Handle, send, receive, 5);
    for (int i = 0; i < 5; i++)
    {
        if (receive[i] != i) {
            printf("SPI test 1 failed : receive[%d] == %d != %d\n",i,receive[i],i);
            failure = 1;
        }
    }
    
    if (failure) return failure;

    // store in register
    send[0] = 0x8F; send[1] = 0x05; send[2] = 0x04; send[3] = 0x03; send[4] = 0x02;
    lgSpiXfer(DE0_Handle, send, receive, 5);

    // read register
    send[0] = 0x0F; send[1] = 0x00; send[2] = 0x00; send[3] = 0x00; send[4] = 0x00;
    lgSpiXfer(DE0_Handle, send, receive, 5);
    if (receive[0] != 0x00) {printf("SPI test 2 failed : receive[%d] == %d != %d\n",0,receive[0],0x00); failure = 2;}
    if (receive[1] != 0x05) {printf("SPI test 2 failed : receive[%d] == %d != %d\n",1,receive[1],0x05); failure = 2;}
    if (receive[2] != 0x04) {printf("SPI test 2 failed : receive[%d] == %d != %d\n",2,receive[2],0x04); failure = 2;}
    if (receive[3] != 0x03) {printf("SPI test 2 failed : receive[%d] == %d != %d\n",3,receive[3],0x03); failure = 2;}
    if (receive[4] != 0x02) {printf("SPI test 2 failed : receive[%d] == %d != %d\n",4,receive[4],0x02); failure = 2;}
    return 0;
}

// ----- SPI 2 ------

uint8_t init_spi2() {
    int SPI2_handle = lgGpiochipOpen(4);
    if (SPI2_handle < 0) return 1;
    if (lgGpioSetUser(SPI2_handle, "Bot Lightyear") < 0) return 2;

    claim_gpio(StprSliderGPIO); 
    claim_gpio(StprPlateGPIO); 
    claim_gpio(StprFlapsGPIO); 
    claim_gpio(BpSwitchFlapsLeftGPIO); 
    claim_gpio(BpSwitchFlapsRightGPIO); 
    return 0;
}

void close_spi2() {
    free_gpio(StprSliderGPIO); 
    free_gpio(StprPlateGPIO); 
    free_gpio(StprFlapsGPIO); 
    free_gpio(BpSwitchFlapsLeftGPIO); 
    free_gpio(BpSwitchFlapsRightGPIO); 
    lgGpiochipClose(SPI2_handle);
}

uint8_t claim_gpio(uint8_t gpio_pin) {
    if (lgGpioClaimInput(SPI2_handle, LG_SET_PULL_NONE, gpio_pin) != 0) return 1;
    return 0;
}

void free_gpio(uint8_t gpio_pin) {
    lgGpioFree(SPI2_handle, gpio_pin);
}

// ----- Odometers -----

void odo_get_tick(int32_t *tick_left, int32_t *tick_right) {

    char send1[] = {0x01,0,0,0,0};
    char send2[] = {0x02,0,0,0,0};
    char receive1[5];
    char receive2[5];
    pthread_mutex_lock(&spi_mutex);
    lgSpiXfer(DE0_handle, send1, receive1, 5);
    lgSpiXfer(DE0_handle, send2, receive2, 5);
    pthread_mutex_unlock(&spi_mutex);

    // left
    *tick_left = *((int32_t *)(&(receive1[1])));
    *tick_left = Reverse32(*tick_left);
    
    // right
    *tick_right = *((int32_t *)(&(receive2[1])));
    *tick_right = Reverse32(*tick_right);

}

void odo_reset() {
    char send[] = {0x7F,0,0,0,0};
    pthread_mutex_lock(&spi_mutex);
    lgSpiWrite(DE0_handle, send, 5);
    pthread_mutex_unlock(&spi_mutex);
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

void teensy_path_following(double *x, double *y, int ncheckpoints, double theta_start, double theta_end, double vref, double dist_goal_reached) {

    size_t message_size = sizeof(char)*2 + sizeof(uint16_t)*(2*ncheckpoints+4);
    // Send vector. Needs to be malloced since it is variable size
    char send[message_size];
    char receive[message_size];

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

    pthread_mutex_lock(&spi_mutex);
    lgSpiXfer(Teensy_handle, send, receive, message_size);
    pthread_mutex_unlock(&spi_mutex);

    #ifdef VERBOSE
    printf("Sending path following\n");
    for (size_t i = 0; i < message_size; i++)
    {
        printf("%d, %d\n", send[i], receive[i]);
    }
    #endif
}

void teensy_set_position(double x, double y, double theta) {
    char send[7];
    send[0] = (char) QuerySetPosition; 
    
    uint16_t *send_ref = (uint16_t *) (send + sizeof(char));
    send_ref[0] = (uint16_t) (UINT16_MAX*(x/2.0));  // xr compressed
    send_ref[1] = (uint16_t) (UINT16_MAX*(y/3.0));  // yr compressed
    send_ref[2] = (uint16_t) (UINT16_MAX*((theta+M_PI)/(M_PI*2)));  // tr compressed

    char receive[7];
    pthread_mutex_lock(&spi_mutex);
    lgSpiXfer(Teensy_handle, send, receive, 7);
    pthread_mutex_unlock(&spi_mutex);

    #ifdef VERBOSE
    printf("Sending Set Position\n");
    for (int i = 0; i < 7; i++)
    {
        printf("%d, %d\n",send[i], receive[i]);
    }
    #endif
}

void teensy_pos_ctrl(double xr, double yr, double theta_r) {
    // Compression to go to SPI
    char send[7];
    send[0] = (char) QueryDoPositionControl; 
    
    uint16_t *send_ref = (uint16_t *) (send + sizeof(char));
    send_ref[0] = (uint16_t) (UINT16_MAX*(xr/2.0));  // xr compressed
    send_ref[1] = (uint16_t) (UINT16_MAX*(yr/3.0));  // yr compressed
    send_ref[2] = (uint16_t) (UINT16_MAX*((theta_r+M_PI)/(M_PI*2)));  // tr compressed

    char receive[7];
    pthread_mutex_lock(&spi_mutex);
    lgSpiXfer(Teensy_handle, send, receive, 7);
    pthread_mutex_unlock(&spi_mutex);

    #ifdef VERBOSE
    printf("Sending Position ctrl \n");
    for (int i = 0; i < 7; i++)
    {
        printf("%d, %d\n",send[i], receive[i]);
    }
    #endif
}


/*void teensy_spd_ctrl(double speed_left, double speed_right) {
    // Compression to go to SPI
    char send[5];
    char receive[5];
    send[0] = (char) QueryDoSpeedControl;

    uint16_t *send_ref = (uint16_t *) (send + sizeof(char));
    send_ref[0] = (uint16_t) ((speed_left/2.0)*UINT16_MAX);   // speed_left compressed
    send_ref[1] = (uint16_t) ((speed_right/2.0)*UINT16_MAX);   // speed_right compressed

    pthread_mutex_lock(&spi_mutex);
    lgSpiXfer(Teensy_handle, send, receive, 5);
    pthread_mutex_unlock(&spi_mutex);

    #ifdef VERBOSE
    printf("Sending Speed ctrl \n");
    for (int i = 0; i < 5; i++)
    {
        printf("%d, %d\n",send[i], receive[i]);
    }
    #endif
    
}*/

void teensy_constant_dc(int dc_refl, int dc_refr) {
    // Compression to go to SPI
    char send[5];
    char receive[5];
    send[0] = (char) QueryDoConstantDutyCycle;
    uint16_t *send_ref = (uint16_t*) (send + sizeof(char));
    send_ref[0] = (uint16_t) (((double) SATURATE(dc_refl, -255,255))+255.0);  // speed_left compressed
    send_ref[1] = (uint16_t) (((double) SATURATE(dc_refr, -255, 255))+255.0);   // speed_right compressed

    pthread_mutex_lock(&spi_mutex);
    lgSpiXfer(Teensy_handle, send, receive, 5);
    pthread_mutex_unlock(&spi_mutex);

    #ifdef VERBOSE
    printf("Sending constant DC ctrl \n");
    for (int i = 0; i < 5; i++)
    {
        printf("%d, %d\n",send[i], receive[i]);
    }
    #endif
}

void teensy_idle() {
    char send = (char) QueryIdle;
    pthread_mutex_lock(&spi_mutex);
    lgSpiWrite(Teensy_handle, &send, 1);
    pthread_mutex_unlock(&spi_mutex);
}

int teensy_ask_mode() {
    char send[4];
    char receive[4];
    send[0] = QueryAskState;
    send[1] = 0; // Not needed
    send[2] = 1; // Not needed
    send[3] = 2; // Not needed
    pthread_mutex_lock(&spi_mutex);
    int retval = lgSpiXfer(Teensy_handle, send, receive, 4);
    pthread_mutex_unlock(&spi_mutex);
    #ifdef VERBOSE
    printf("Asking state\n");
    printf("Xfer return value: %d\n", retval);
    for (int i = 0; i < 4; i++){
        printf("%d, %d\n", send[i], receive[i]);
    }
    #endif
    return (int) receive[2];
}

void teensy_set_position_controller_gains(double kp, double ka, double kb, double kw) {
    // Compression to go to SPI
    int message_size = 1 + 2*4;
    char send[9];
    char receive[9];
    send[0] = (char) QuerySetPositionControlGains;

    uint16_t *send_ref = (uint16_t *) (send + sizeof(char));
    send_ref[0] = (uint16_t) ((kp/20)*UINT16_MAX);   // speed_left compressed
    send_ref[1] = (uint16_t) ((ka/20)*UINT16_MAX);   // speed_right compressed
    send_ref[2] = (uint16_t) ((-kb/20)*UINT16_MAX);   // speed_right compressed
    send_ref[3] = (uint16_t) ((kw/50)*UINT16_MAX);   // speed_right compressed

    pthread_mutex_lock(&spi_mutex);
    lgSpiXfer(Teensy_handle, send, receive, message_size);
    pthread_mutex_unlock(&spi_mutex);

    #ifdef VERBOSE
    printf("Sending Speed ctrl \n");
    for (int i = 0; i < message_size; i++)
    {
        printf("%d, %d\n",send[i], receive[i]);
    }
    #endif
}

void teensy_set_path_following_gains(double kt, double kn, double kz, double sigma, double epsilon, double kv_en, double delta, double wn) {
    // Compression to go to SPI
    int message_size = 1 + 2*8;
    char send[17];
    char receive[17];
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

    pthread_mutex_lock(&spi_mutex);
    lgSpiXfer(Teensy_handle, send, receive, message_size);
    pthread_mutex_unlock(&spi_mutex);

    #ifdef VERBOSE
    printf("Sending Speed ctrl \n");
    for (int i = 0; i < message_size; i++)
    {
        printf("%d, %d\n",send[i], receive[i]);
    }
    #endif
}

void teensy_ask_pos(double *x, double *y, double *theta) {
    
}


// ----- Flaps servomotors -----

void flaps_servo_cmd(flaps_servo_cmd_t command) {
    char send[5];
    send[0] = 0x80;
    uint16_t servo_flaps1_duty_cycle; 
    uint16_t servo_flaps2_duty_cycle; 

    switch (command)
    {

    case FlapsIdle: 
        servo_flaps1_duty_cycle = 0;
        servo_flaps2_duty_cycle = 0;
        break;

    case FlapsDeploy:
        servo_flaps1_duty_cycle = 608;
        servo_flaps2_duty_cycle = 608;
        break;
    
    case FlapsRaise:
        servo_flaps1_duty_cycle = 736;
        servo_flaps2_duty_cycle = 512;
        break;

    default:
        return;
    }
    send[1] = (servo_flaps1_duty_cycle & 0xFF00) >> 8;
    send[2] = servo_flaps1_duty_cycle & 0xFF;

    send[3] = (servo_flaps2_duty_cycle & 0xFF00) >> 8;
    send[4] = servo_flaps2_duty_cycle & 0xFF;
    pthread_mutex_lock(&spi_mutex);
    lgSpiWrite(DE0_handle, send, 5);
    pthread_mutex_unlock(&spi_mutex);
}

void servo_gripper_holder_cmd(servo_gripper_holder_cmd_t command) {
    char send[5];
    send[0] = 0x8E;
    uint16_t servo_holder_duty_cycle; 
    switch (command) {
        case HolderIdle:
            servo_holder_duty_cycle = 0;
            break; 

        case HolderClosed:
            servo_holder_duty_cycle = 0;
            break; 

        case HolderPot:
            servo_holder_duty_cycle = 0;
            break;

        case HolderPlant:
            servo_holder_duty_cycle = 0;
            break;

        default:
            return;
    }
    send[3] = (servo_holder_duty_cycle & 0xFF00) >> 8;
    send[4] = servo_holder_duty_cycle & 0xFF;
    pthread_mutex_lock(&spi_mutex);
    lgSpiWrite(DE0_handle, send, 5);
    pthread_mutex_unlock(&spi_mutex);
}

void servo_gripper_deployer_cmd(servo_gripper_deployer_cmd_t command) {
    char send[5];
    send[0] = 0x8F;
    uint16_t servo_deployer_duty_cycle; 
    switch (command) {
        case DeployerIdle:
            servo_deployer_duty_cycle = 0;
            break; 

        case DeployerRaise:
            servo_deployer_duty_cycle = 0;
            break; 

        case DeployerDeploy:
            servo_deployer_duty_cycle = 0;
            break;

        default:
            return;
    }
    send[3] = (servo_deployer_duty_cycle & 0xFF00) >> 8;
    send[4] = servo_deployer_duty_cycle & 0xFF;
    pthread_mutex_lock(&spi_mutex);
    lgSpiWrite(DE0_handle, send, 5);
    pthread_mutex_unlock(&spi_mutex);
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

void stpr_move(steppers_t stepperName, uint32_t steps, uint8_t neg, uint8_t blocking = CALL_NON_BLOCKING) {
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
            printf("Error : not a stepper %d \n", stepperName); 
            return; 
    }   

    char steps1 = (steps & 0xFF0000) >> 16; 
    char steps2 = (steps & 0xFF00) >> 8;
    char steps3 = steps & 0xFF;

    char send[] = {request,direction,steps1,steps2,steps3};
    pthread_mutex_lock(&spi_mutex);
    lgSpiWrite(DE0_handle, send, 5);
    pthread_mutex_unlock(&spi_mutex);

     if (blocking = CALL_BLOCKING) {
        
        spi2_gpio_t stepper_gpio; 
        switch (stepperName) {
            case StprPlate :
                stepper_gpio = StprPlateGPIO; 
                break;
            case StprSlider :
                stepper_gpio = StprSliderGPIO; 
                break;
            case StprFlaps :
                stepper_gpio = StprFlapsGPIO; 
                break;
            default : 
                stepper_gpio = 0; 
                printf("Error : not a stepper %d \n", stepperName); 
                return; 
        }
        lguSleep(0.001); // Sleep to avoid timing conflicts with spi message
        int isIdle;
        do {
            isIdle = lgGpioRead(handle,4);
            if (isIdle < 0) {
                printf("Error : gpio %d not readable \n", stepper_gpio); 
                return;
            }
            lguSleep(0.001);
        } while (!isIdle);
    }
}

void stpr_calibrate(steppers_t stepperName, uint8_t blocking = CALL_NON_BLOCKING) {
    char request; 
    char calibDir;
    switch (stepperName) {
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
            printf("Error : not a stepper %d \n", stepperName); 
            return; 
    }
    char send[] = {request,calibDir,0,0,0};
    pthread_mutex_lock(&spi_mutex);
    lgSpiWrite(DE0_handle, send, 5);
    pthread_mutex_unlock(&spi_mutex);
    if (blocking = CALL_BLOCKING) {
        
        spi2_gpio_t stepper_gpio; 
        switch (stepperName) {
            case StprPlate :
                stepper_gpio = StprPlateGPIO; 
                break;
            case StprSlider :
                stepper_gpio = StprSliderGPIO; 
                break;
            case StprFlaps :
                stepper_gpio = StprFlapsGPIO; 
                break;
            default : 
                stepper_gpio = 0; 
                printf("Error : not a stepper %d \n", stepperName); 
                return; 
        }
        lguSleep(0.001); // Sleep to avoid timing conflicts with spi message
        int isIdle;
        do {
            isIdle = lgGpioRead(handle,4);
            if (isIdle < 0) {
                printf("Error : gpio %d not readable \n", stepper_gpio); 
                return;
            }
            lguSleep(0.001);
        } while (!isIdle);
    }
}

void flaps_move(flaps_pos_t pos, uint8_t blocking = CALL_NON_BLOCKING) {
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
        printf("Error : not a position %d \n", pos);
        steps = 0; 
        return;
    }
    stpr_move(StprFlaps, steps, 0, blocking); 
}

void slider_move(slider_pos_t pos, uint8_t blocking = CALL_NON_BLOCKING){
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
        steps = 350;
        break; 
    case SliderTake :
        steps = 1600;
        break;      
    case SliderDeposit : 
        steps = 5000;
        break; 
    default :
        printf("Error : not a position : %d \n", pos);
        printf("%d\n", pos);
        steps = 0; 
        return;
    }
    stpr_move(StprSlider,steps,0, blocking);
}


void plate_move(int8_t pot, uint8_t blocking = CALL_NON_BLOCKING){
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
        stpr_move(StprPlate,(int)ticStepper,direction, blocking);
    }   

}


void stpr_setup_speed(int nominalSpeed, int initialSpeed, steppers_t stepperName) {
    char initialSpeed1 = (initialSpeed & 0xFF00) >> 8; 
    char initialSpeed2 = initialSpeed & 0xFF;
    char nominalSpeed1= (nominalSpeed & 0xFF00) >> 8;
    char nominalSpeed2 = nominalSpeed & 0xFF;
    char request; 
    //printf("Init speed 1 : %d, Init speed 2 : %d", initialSpeed1, initialSpeed2);
    switch (stepperName) {
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
    
    char send[] = {request,nominalSpeed1,nominalSpeed2, initialSpeed1, initialSpeed2};
    pthread_mutex_lock(&spi_mutex);
    lgSpiWrite(DE0_handle, send, 5); 
    pthread_mutex_unlock(&spi_mutex);
}


void stpr_setup_calib_speed(int calibrationSpeed, int smallCalibrationSpeed, steppers_t stepperName) {
    char calibrationSpeed1 = calibrationSpeed/256;
    char calibrationSpeed2 = calibrationSpeed-calibrationSpeed1*256;
    char smallCalibrationSpeed1= smallCalibrationSpeed/256;
    char smallCalibrationSpeed2 = smallCalibrationSpeed-smallCalibrationSpeed1*256;
    char request; 
    switch (stepperName) {
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
            printf("Error : not a stepper %d \n", stepperName); 
            return; 
    }
    char send[] = {request,calibrationSpeed1, calibrationSpeed2, smallCalibrationSpeed1, smallCalibrationSpeed2};
    pthread_mutex_lock(&spi_mutex);
    lgSpiWrite(DE0_handle, send, 5); 
    pthread_mutex_unlock(&spi_mutex);
}



void stpr_reset(steppers_t stepperName) {
    char request;
    char request2; 


    // Stop steppers
    switch (stepperName) {
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
            printf("Error : not a stepper %d \n", stepperName); 
            return; 
    } 
    char send1[] = {request,0,0,0,0}; // set the Module command to Idle
    char send2[] = {request2,0,1,0,0}; // send 1 to reset the module completely

    pthread_mutex_lock(&spi_mutex);
    lgSpiWrite(DE0_handle, send1, 5);
    lgSpiWrite(DE0_handle, send2, 5); // send 0 to stop resetting
    send2[2] = 0; 
    //sleep(1);
    lgSpiWrite(DE0_handle, send2, 5); // send 0 to stop resetting
    pthread_mutex_unlock(&spi_mutex);

}

void stpr_setup_acc(steppers_t stepperName, uint8_t accSteps) {
    char request;
    // Stop steppers
    switch (stepperName) {
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
            printf("Error : not a stepper %d \n", stepperName);  
            return; 
    } 

    char send[] = {request,0,0,0,accSteps}; // send 1 to reset the module completely
    pthread_mutex_lock(&spi_mutex);
    lgSpiWrite(DE0_handle, send, 5);
    pthread_mutex_unlock(&spi_mutex);

}

void stpr_calibrate_all() {
    stpr_calibrate(StprFlaps);
    stpr_calibrate(StprPlate);
    stpr_calibrate(StprSlider);
}

void stpr_reset_all() {
    stpr_reset(StprFlaps);
    stpr_reset(StprPlate);
    stpr_reset(StprSlider);
}