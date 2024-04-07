#include "SPI_Modules.h"
#include "lidarTop.h"
#include "dynamixels.h"
#include <unistd.h>
#include <cstdio>
#include <stdio.h>
#include <stdlib.h> 
#include <unistd.h>  
#include <pthread.h>
#include <math.h>

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

    //Set initial robot position and path following useful variables
    double x0 = 0.035;
    double y0 = 0.2;
    double theta0 = 0.0;
    teensy_set_position(x0, y0, theta0);
    //Path following : Go grab a plant
    if (!ADVERSARY_FLAG) {
        printf("No adversary, taking path following \n");
        // double kp = 1.0;
        // double ka = 4.0;
        // double kb = -0.5;
        // double kw = 10.0;
        // teensy_set_position_controller_gains(kp, ka, kb, kw);
        double kt = 2.0;
        double kn = 0.32; // 0 < kn <= 1
        double kz = 30.0;
        double delta = 15e-3; // delta is in meters
        double sigma = 0.0;
        double epsilon = M_PI/8; // epsilon is in radians
        double wn = 0.25; // Command filter discrete cutoff frequency
        double kv_en = 12;
        //teensy_set_path_following_gains(kt, kn, kz, sigma, epsilon, kv_en, delta, wn);
        lguSleep(0.1);  
        
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
    lguSleep(2);
    


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
    StartLidar();
    LidarData *lidarData = new LidarData[sizeof(LidarData)];
    init_lidar(lidarData);
    int i = 0;
    while (!ENDGAME) {
        //DataToFile("testLidarMobile/"+std::to_string(i));
        lidarGetRobotPosition(lidarData, i);
            printf("\nboucle %d", i);
            printf(" robot at x=%f; y=%f; orientation=%f", lidarData->readLidar_x_robot, lidarData->readLidar_y_robot, lidarData->readLidar_theta_robot*180.0/M_PI);
            printf(" Adversary at d=%f; a=%f\n", lidarData->readLidar_d_opponent, lidarData->readLidar_a_opponent);
        i++;
        //printf("%f %f %f %f %f %f %f %f \n", lidarData->beaconAdv[0]*180.0/M_PI,lidarData->beaconAdv[1],lidarData->beaconAdv[2]*180.0/M_PI,lidarData->beaconAdv[3],lidarData->beaconAdv[4]*180.0/M_PI,lidarData->beaconAdv[5],lidarData->beaconAdv[6]*180.0/M_PI,lidarData->beaconAdv[7]);
        double adv_dist = lidarData->readLidar_d_opponent; 
        double adv_angle = lidarData->readLidar_a_opponent;
        double limit_stop = 0.5; 
        if ((adv_dist > 0.1) && (adv_dist < limit_stop) && ((adv_angle < 0.79) || (adv_angle > (6.28-0.79)))) {
            ADVERSARY_FLAG = true; 
            teensy_idle();
            printf("Adversary detected\n");
        }
    }

    StopLidar();
    clear_lidar(lidarData);
    delete (lidarData);
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