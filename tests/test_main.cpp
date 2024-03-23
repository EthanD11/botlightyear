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

typedef struct {
    int32_t old_ticks_l;
    int32_t old_ticks_r;
    int32_t ticks_l;
    int32_t ticks_r;
    double step_l;
    double step_r;
    double omega_l;
    double omega_r;
    double fwd;
    double rot;
    double omega_refl;
    double omega_refr;
    double x;
    double y;
    double theta;
} Odometers;

typedef enum {
  ModeIdle, // No input from RPi, default is to remain still
  ModePositionControl,
  ModePathFollowingInit,
  ModePathFollowing,
  ModeSpeedControl,
  ModeConstantDC
} controlmode_t; 

void calibrateAll() {
    stpr_calibrate(StprFlaps);
    stpr_calibrate(StprPlate);
    stpr_calibrate(StprSlider);
}

void resetAll() {
    stpr_reset(StprFlaps);
    stpr_reset(StprPlate);
    stpr_reset(StprSlider);
}

void update_position(int32_t ticks_l, int32_t ticks_r) {

}



void *homologation(void* v) {
    printf("Entering homologation thread \n");
    //Init ports
    init_spi();
    //dxl_init_port();

    //Init useful variables
    /*int32_t old_ticks_l = 0, old_ticks_r = 0; // Old tick values from previous iteration, left and right
    int32_t ticks_l, ticks_r; // Current values of ticks, left and right
    double step_l, step_r; // Distance traveled during the last iteration (m), left and right 
    double omega_l, omega_r; // Current values of speed (rad_mot/s), left and right
    double fwd, rot; // Forward and rotational components
    double omega_refl, omega_refr; // Reference speed values, left and right

    //Init odometry
    Odometers* Odometers = (Odometers*) malloc(sizeof(Odometers));
    Odometers = {old_ticks_l, old_ticks_r, ticks_l, ticks_r, step_l, step_r, omega_l, omega_r, fwd, rot, omega_refl, omega_refr, 0, 0, 0};
    */

    //Calibrate all steppers
    /*servo_cmd(ServoRaise);
    resetAll(); 
    stpr_setup_speed(5,10,StprFlaps); 
    stpr_setup_speed(2,10,StprPlate); 
    stpr_setup_speed(4,10,StprSlider);
    calibrateAll();*/

    //Ping Dynamixels
    /*dxl_ping(1, 2.0);
    dxl_ping(3, 2.0);
    dxl_ping(6, 1.0);
    dxl_ping(8, 1.0);*/

    sleep(10);

    //Set initial robot position and path following useful variables

    //Path following : Go grab a plant
    if (!ADVERSARY_FLAG) {
        printf("no adv\n");
        double kt = 3.0;
        double kn = 1.0; // 0 < kn <= 1
        double kz = 80.0;
        double delta = 50e-3; // delta is in meters
        double sigma = 0.0;
        double epsilon = 150e-3; // epsilon is in meters
        double wn = 0.3; // Command filter discrete cutoff frequency
        double kv_en = 10;
        teensy_set_path_following_gains(kt, kn, kz, sigma, epsilon, kv_en, delta, wn);
        lguSleep(0.1);
        int ncheckpoints = 2;
        double xr[2] = {0.2, 0.7};
        double yr[2] = {0.7, 0.15};
        double theta_start =   0.;
        double theta_end = 0;
        double vref = 0.2;
        double dist_goal_reached = 0.1;
        teensy_set_position(0.2, 0.7, 0);
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

    //Grab the plant:
    /*servo_cmd(ServoDeploy);
    flaps_move(FlapsPlant);
    sleep(3);
    gripper(Open);
    position_gripper(Down);
    slider_move(SliderLow);
    sleep(6);
    gripper(Plant); 
    sleep(0.5);
    slider_move(SliderPlate);
    sleep(6);
    plate_move(2);
    sleep(4);
    gripper(Open); 
    position_gripper(Up);
    plate_move(0);*/

    //Path following: Go to solar panels

    //Turn solar panel

    //Path following: Go to planter

    //Drop the plant


    //Path following: Go to charging station
    
    ENDGAME = true; 
    return 0;
}

void *topLidar(void* v) {
    //double *robot = new double[4]{0, 0, 0, 0};
    double *adv = new double[4]{0, 0, 0, 3.14};
    //double *beaconAdv = new double[8]{0, 0, 0, 0, 0, 0, 0, 0};
    
    StartLidar();

    while (!ENDGAME) {
        //lidarGetRobotPosition(robot, adv, beaconAdv);
        double adv_dist = adv[2]; 
        double adv_angle = adv[3];
        double limit_stop = 0.5; 
        if ((adv_dist < limit_stop) & (adv_angle < 0.79) & (adv_angle > (6.28-0.79))) {
            ADVERSARY_FLAG = true; 
            printf("Adversary detected\n");
        }
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