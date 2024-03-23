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

void *homologation() {
    //Init ports
    if (init_spi() != 0) exit(1);
    if (dxl_init_port() != 0) exit(1);

    //Calibrate all steppers
    servo_cmd(ServoRaise);
    resetAll(); 
    stpr_setup_speed(5,10,StprFlaps); 
    stpr_setup_speed(2,10,StprPlate); 
    stpr_setup_speed(4,10,StprSlider);
    calibrateAll();

    //Ping Dynamixels
    dxl_ping(1, 2.0);
    dxl_ping(3, 2.0);
    dxl_ping(6, 1.0);
    dxl_ping(8, 1.0);

    sleep(10);

    /* //Init Starting Switch
    int handle = lgGpiochipOpen(0);
    if (handle < 0) exit(1);
    int start = 0; 
    
    //Wait for starting switch
    while (start == 0) {
        start = lgGpioRead(handle, 4);
        sleep(0.5);
    }

    //Start the game
    while (FLAG == 0) {

    }*/

    




}

void *topLidar() {
    double *robot = new double[4]{0, 0, 0, 0};
    double *adv = new double[4]{0, 0, 0, 0};
    double *beaconAdv = new double[8]{0, 0, 0, 0, 0, 0, 0, 0};
    
    StartLidar();

    while (!ENDGAME) {
        lidarGetRobotPosition(robot, adv, beaconAdv);
        double adv_dist = adv[2]; 
        double adv_angle = adv[3];
        double limit_stop = 0.5; 
        if ((adv_dist < limit_stop) & (adv_dist < 0.79) & (adv_dist > (6.28-0.79))) {
            ADVERSARY_FLAG = true; 
            printf("Adversary detected\n");
        }
    }

    StopLidar();
    return 0;
}

int main() { 

    int error;

    for (int i = 0; i < 2; i++) {
        if (pthread_rwlock_init(&lock[i], NULL) != 0) { 
        printf("Mutex init has failed : [%d] \n", i); 
        exit(1); 
        } 
    }

    for (int i = 0; i < 2; i++) {
        error = pthread_create(&(tid[i]), NULL, &trythis, NULL); 
        if (error != 0) {
            printf("\nThread can't be created :[%s]", strerror(error)); 
            exit(1);
        }
    }

    pthread_join(tid[0], NULL); 
    pthread_join(tid[1], NULL); 

    for (int i = 0; i < 2; i++) {
        pthread_rwlock_destroy(&lock[i]);
    }
    
    exit(0); 
}