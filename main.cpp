#include "SPI_Modules.h"
#include "lidarTop.h"
#include "dynamixels.h"
#include <unistd.h>
#include <cstdio>
#include <stdio.h>
#include <stdlib.h> 
#include <unistd.h>  
#include <pthread.h> 

pthread_t tid[2]; 
pthread_rwlock_t lock[2]; 

bool ADVERSARY_FLAG = false; 
bool ENDGAME = false; 

const double deg_to_rads = M_PI/180;
int32_t old_ticks_l = 0, old_ticks_r = 0;
double x = 0.0, y = 0.0, theta = 0.0;

typedef enum {
  ModeIdle, // No input from RPi, default is to remain still
  ModePositionControl,
  ModePathFollowingInit,
  ModePathFollowing,
  ModeSpeedControl,
  ModeConstantDC
} controlmode_t; 


void updateRobotPosition() {
    int32_t ticks_l, ticks_r; // Current values of ticks, left and right
    double step_l, step_r; // Distance traveled during the last iteration (m), left and right 
    double fwd, rot; // Forward and rotational components
    
    // Updating values via SPI
    odo_get_tick(&ticks_l, &ticks_r);
    step_l = (ticks_l - old_ticks_l) * ODO_TICKS_TO_M;
    step_r = (ticks_r - old_ticks_r) * ODO_TICKS_TO_M;
    old_ticks_l = ticks_l;
    old_ticks_r = ticks_r;

    // Forward & rotation component
    fwd = (step_l + step_r) / 2;
    rot = (step_r - step_l) / (ODO_WHEEL_L);

    // Estimate new position
    x += fwd * cos(theta + rot / 2);
    y += fwd * sin(theta + rot / 2);
    theta += rot;
}



void *homologation(void* v) {
    printf("Entering homologation thread \n");
    //Init ports
    init_spi();
    dxl_init_port();

    //Calibrate all steppers
    flaps_servo_cmd(FlapsRaise);
    stpr_setup_speed(100,600,StprFlaps); 
    stpr_setup_speed(60,500,StprPlate); 
    stpr_setup_speed(300,400,StprSlider);
    stpr_reset_all(); 
    stpr_calibrate_all();

    //Ping Dynamixels
    dxl_ping(1, 2.0);
    dxl_ping(3, 2.0);
    dxl_ping(6, 1.0);
    dxl_ping(8, 1.0);

    int handle = lgGpiochipOpen(4);
    if (handle < 0) exit(1);
    /*lgChipInfo_t chipinfo;
    if (lgGpioGetChipInfo(handle,&chipinfo) < 0) exit(1);
    printf("lines=%d name=%s label=%s\n", chipinfo.lines, chipinfo.name, chipinfo.label);*/

    if (lgGpioSetUser(handle, "Bot Lightyear") < 0) exit(2);

    if (lgGpioClaimInput(handle, LG_SET_PULL_NONE, 4) != 0) exit(3);

    /*lgLineInfo_t lInfo;
    if (lgGpioGetLineInfo(handle, 4, &lInfo) < 0) exit(4);
    printf("lFlags=%d name=%s user=%s offset=%d\n", lInfo.lFlags, lInfo.name, lInfo.user, lInfo.offset);*/

    printf("Waiting for setup... \n");

    int start;
    do {
        start = lgGpioRead(handle,4);
        if (start < 0) exit(5);
    } while (start);
    printf("Starting cord setup\n");
    
    lguSleep(1);

    printf("Waiting start of the game... \n");
    do {
        start = lgGpioRead(handle,4);
        if (start < 0) exit(5);
    } while (!start);

    lgGpioFree(handle, 4);
    lgGpiochipClose(handle);

    printf("Game started! \n");

    //Set initial robot position and path following useful variables

    //Path following : Go grab a plant
    if (!ADVERSARY_FLAG) {
        printf("No adversary, taking path following \n");
        // double kp = 1.0;
        // double ka = 4.0;
        // double kb = -0.5;
        // double kw = 10.0;
        // teensy_set_position_controller_gains(kp, ka, kb, kw);
        double kt = 2.0;
        double kn = 0.7; // 0 < kn <= 1
        double kz = 10.0;
        double delta = 40e-3; // delta is in meters
        double sigma = 0.0;
        double epsilon = 150e-3; // epsilon is in meters
        double wn = 0.3; // Command filter discrete cutoff frequency
        double kv_en = 10;
        teensy_set_path_following_gains(kt, kn, kz, sigma, epsilon, kv_en, delta, wn);
        lguSleep(0.1);
        double x0 = 0.035;
        double y0 = 0.2;
        double theta0 = 0.0;
        teensy_set_position(x0, y0, theta0);
        lguSleep(0.1);
        teensy_pos_ctrl(0.2, 0.2, theta0);
        // teensy_pos_ctrl(x0, y0, theta_start + (atan2(yr[1]-yr[0], xr[1]-xr[0])-theta_start)/2.0);
        lguSleep(5);

        int ncheckpoints = 3;
        double xr[3] = {0.2, 0.5, 0.7};
        double yr[3] = {0.2, 0.3, 0.7};
        double theta_start = 0.0;
        double theta_end = M_PI/2.0;
        double vref = 0.25;
        double dist_goal_reached = 0.05;
        teensy_path_following(xr, yr, ncheckpoints, theta_start, theta_end, vref, dist_goal_reached);
        /*while (((controlmode_t) teensy_ask_mode()) == ModePathFollowing) {
            printf("Moving \n");
            if (ADVERSARY_FLAG) {
                printf("Adversary found \n");
                teensy_idle();
                break;
                exit(1); 
            }
        };*/
    }

    //Grab the plant:
    lguSleep(2);
    teensy_ask_mode();
    lguSleep(2);
    teensy_ask_mode();
    lguSleep(2);
    teensy_ask_mode();
    lguSleep(2);
    teensy_ask_mode();
    /*lguSleep(2);
    teensy_ask_mode();
    lguSleep(2);
    teensy_ask_mode();
    lguSleep(2);
    teensy_ask_mode();*/

    sleep(3); 

    flaps_servo_cmd(FlapsDeploy);
    flaps_move(FlapsPlant);
    lguSleep(3);
    flaps_move(FlapsOpen);
    gripper(Open);
    position_gripper(Down);
    lguSleep(0.5);
    slider_move(SliderLow);
    lguSleep(2);
    gripper(Plant); 
    lguSleep(0.5);
    slider_move(SliderPlate);
    lguSleep(2);
    plate_move(2);
    lguSleep(2);
    //position_gripper(Down);
    gripper(Open); 
    position_gripper(Up);
    plate_move(0);
    


    //Path following: Go to planter
    if ((!ADVERSARY_FLAG)) {
        printf("No adversary, taking path following \n");
        int ncheckpoints = 2;
        double xr[2] = {0.7, 1};
        double yr[2] = {0.7, 2.5};
        double theta_start =   M_PI/2.0;
        double theta_end = M_PI/2.0;
        double vref = 0.2;
        double dist_goal_reached = 0.1;
        //teensy_set_position(xr[0], yr[0], theta_start);
        lguSleep(0.1);
        teensy_path_following(xr, yr, ncheckpoints, theta_start, theta_end, vref, dist_goal_reached);
        /*while (((controlmode_t) teensy_ask_mode()) == ModePathFollowing) {
            printf("Moving \n");
            if (ADVERSARY_FLAG) {
                printf("Adversary found \n");
                teensy_idle();
                break;
                exit(1); 
            }
        };*/
    }

    //Drop the plant
    slider_move(SliderHigh); 
    plate_move(2);
    gripper(Open);
    lguSleep(2);
    position_gripper(Down);
    lguSleep(3);
    slider_move(SliderTake); 
    lguSleep(2); 
    gripper(Plant); 
    slider_move(SliderHigh); 
    lguSleep(1); 
    plate_move(0); 
    lguSleep(2); 
    slider_move(SliderDeposit); 
    lguSleep(3); 
    gripper(Open); 
    lguSleep(2); 
    slider_move(SliderPlate); 
    lguSleep(1);
    gripper(Close); 
    flaps_servo_cmd(FlapsRaise);

    //Path following: Go to solar panels
    lguSleep(2);
    teensy_ask_mode();
    lguSleep(2);
    /*teensy_ask_mode();
    lguSleep(2);
    teensy_ask_mode();
    lguSleep(2);
    teensy_ask_mode();
    lguSleep(2);
    teensy_ask_mode();
    lguSleep(2);
    teensy_ask_mode();
    lguSleep(2);
    teensy_ask_mode();
    lguSleep(2);
    teensy_ask_mode();
    lguSleep(2);
    teensy_ask_mode();*/

    if ((!ADVERSARY_FLAG)) {
        printf("No adversary, taking path following \n");
        int ncheckpoints = 3;
        double xr[3] = {1, 1.6, 1.80};
        double yr[3] = {2, 1.5, 0.74};
        double theta_start =   M_PI/2.0;
        double theta_end = -1.01*M_PI/2.0;
        double vref = 0.2;
        double dist_goal_reached = 0.1;
        teensy_pos_ctrl(xr[0], yr[0], M_PI/8.0);
        lguSleep(4);
        teensy_pos_ctrl(xr[0], yr[0], -M_PI/4.0);
        lguSleep(5);
        teensy_path_following(xr, yr, ncheckpoints, -M_PI/2.0, theta_end, vref, dist_goal_reached);
        /*while (((controlmode_t) teensy_ask_mode()) == ModePathFollowing) {
            printf("Moving \n");
            if (ADVERSARY_FLAG) {
                printf("Adversary found \n");
                teensy_idle();
                break;
                exit(1); 
            }
        };*/
    }
    //Turn solar panel
    lguSleep(2);
    teensy_ask_mode();
    lguSleep(2);
    teensy_ask_mode();
    lguSleep(2);
    teensy_ask_mode();
    lguSleep(2);
    teensy_ask_mode();
    lguSleep(2);
    teensy_ask_mode();
    lguSleep(2);
    teensy_ask_mode();
    lguSleep(2);
    teensy_ask_mode();
    lguSleep(2);
    teensy_ask_mode();
    lguSleep(2);
    teensy_ask_mode();

    position_solar(DownS);
    sleep(1);
    multiturn_solar(CCW);
    sleep(1);
    position_solar(UpS);

    //Path following: Go to charging station
    if ((!ADVERSARY_FLAG)) {
        printf("No adversary, taking path following \n");
        int ncheckpoints = 2;
        double xr[2] = {1.80, 1.80};
        double yr[2] = {0.74, 0.3};
        double theta_start =   -M_PI/2.0;
        double theta_end = -M_PI/2.0;
        double vref = 0.2;
        double dist_goal_reached = 0.1;
        //teensy_set_position(xr[0], yr[0], theta_start);
        lguSleep(0.1);
        teensy_path_following(xr, yr, ncheckpoints, theta_start, theta_end, vref, dist_goal_reached);
        /*while (((controlmode_t) teensy_ask_mode()) == ModePathFollowing) {
            printf("Moving \n");
            if (ADVERSARY_FLAG) {
                printf("Adversary found \n");
                teensy_idle();
                break;
                exit(1); 
            }
        };*/
    }

    return 0;
}

void *topLidar(void* v) {
    printf("Entering topLidar thread \n");
    double *robot = new double[4]{0, 0, 0, 0};
    double *adv = new double[4]{0, 0, 0, 3.14};
    //TODO LIDAR INIT
    StartLidar();
    int i =0;
    while (!ENDGAME) {
        //lidarGetRobotPosition(robot, adv, beaconAdv);
        double adv_dist = adv[2]; 
        double adv_angle = adv[3];
        double limit_stop = 0.5; 
        if ((adv_dist < limit_stop) & (adv_angle < 0.79) & (adv_angle > (6.28-0.79))) {
            ADVERSARY_FLAG = true; 
            printf("Adversary detected\n");
        }
        i++;
    }

    StopLidar();
    return 0;
}
int main(int argc, char const *argv[]) { 
    
    int error;

    for (int i = 0; i < 2; i++) {
        if (pthread_rwlock_init(&lock[i], NULL) != 0) { 
        printf("Mutex init has failed : [%d] \n", i); 
        exit(1); 
        } 
    }

    error = pthread_create(&(tid[0]), NULL, homologation, NULL); 
    if (error != 0) {
        printf("\nThread can't be created :[0]"); 
        exit(1);
    }
    error = pthread_create(&(tid[1]), NULL, topLidar, NULL); 
    if (error != 0) {
        printf("\nThread can't be created :[1]"); 
        exit(1);
    }
    printf("1");
    pthread_join(tid[0], NULL); 
    pthread_join(tid[1], NULL); 
    printf("2");
    for (int i = 0; i < 2; i++) {
        pthread_rwlock_destroy(&lock[i]);
    }
    
    exit(0); 
}