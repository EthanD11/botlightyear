#include "SPI_Modules.h"
#include "lidarTop.h"
#include "dynamixels.h"
#include <unistd.h>
#include <cstdio>
#include <stdio.h>
#include <stdlib.h> 
#include <unistd.h>  
#include <pthread.h> 


#define FLAG    0

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
    sleep(10);

    //Ping Dynamixels
    dxl_ping(1, 2.0);
    dxl_ping(3, 2.0);
    dxl_ping(6, 1.0);
    dxl_ping(8, 1.0);

    //Init Starting Switch
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

    }

}

void *topLidar() {
    double *robot = new double[4]{0, 0, 0, 0};
    double *adv = new double[4]{0, 0, 0, 0};
    double *beaconAdv = new double[8]{0, 0, 0, 0, 0, 0, 0, 0};
    StartLidar();
    DataToFile("jsp.txt");
    StopLidar();
    //lidarGetRobotPosition(robot, adv, beaconAdv);
    /*printf("\n robot at x=%f; y=%f; orientation=%f; %f radian beacon3\n", robot[0], robot[1], robot[2], robot[3]);
    printf("Adversary at x=%f; y=%f\n", adv[0], adv[1]);
    printf("adv at %f m; %f degree\n", adv[2], adv[3] * 180 / M_PI);
    for (int i = 0; i < 8; ++i) {
        printf("%f, ", beaconAdv[i]);
    }*/
    printf("\n");
    return 0;
}

int main() 
{ 
    pthread_t thread_id; 
    pthread_create(&thread_id, NULL, myThreadFun, NULL); 
    pthread_join(thread_id, NULL); 
    
    exit(0); 
}
