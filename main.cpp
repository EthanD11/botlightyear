#include "SPI_bus.h"
#include "steppers.h"
#include "servos.h"
#include "lidarTop.h"
#include "dynamixels.h"
#include "graph.h"
#include "action_variables.h"
#include <unistd.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h> 
#include <unistd.h>  
#include <pthread.h> 
#include <termios.h>

#define VERBOSE

#define ASCII_b 98
#define ASCII_B 66
#define ASCII_y 121
#define ASCII_Y 89

team_color_t color = NoTeam;

time_t t; // Time elapsed since the start of the game
time_t dt; // Time elapsed since the last control loop

pthread_t topLidarID;
uint8_t lidarEnd = 0;
pthread_rwlock_t graph_lock;

graph_path_t *currentPath = NULL;
uint8_t bases[3];

SPIBus spi_bus = SPIBus();
Steppers steppers = Steppers(&spi_bus);
Graph graph = Graph();

void ask_user_input_params() {
    printf("Which team do I play for ? Press 'b' for team blue, 'y' for team yellow\n");
    do {
        string input;
        std::cin >> input;
        char keyboard_input = input.at(0);
        if (keyboard_input == ASCII_b || keyboard_input == ASCII_B) { color = TeamBlue; break; }
        else if (keyboard_input == ASCII_y || keyboard_input == ASCII_Y ) { color = TeamYellow; break; }
        printf("Invalid input color : %c\n", keyboard_input);
    } while (1);
}

time_t init_main() {
    dxl_init();

    // if (init_spi() != 0) exit(2);
    // if (test_spi() != 0) exit(2);
    
    if (graph.init_from_file("./graphs/BL_V2.txt", color) != 0) exit(3);
    graph.node_level_update(graph.adversaryBases[0], 3, DISABLE_PROPAGATION);

    if (pthread_create(&topLidarID, NULL, topLidar, NULL) != 0) exit(4);

    steppers.reset_all();
    steppers.calibrate_all();
    
    int GPIOhandle = lgGpiochipOpen(4);
    if (GPIOhandle < 0) exit(5);
    if (lgGpioSetUser(GPIOhandle, "Bot Lightyear") < 0) exit(5);
    if (lgGpioClaimInput(GPIOhandle, LG_SET_PULL_NONE, 4) != 0) exit(5);

    
    
    time_t tStart;
    {
        #ifdef VERBOSE
        printf("Waiting for starting cord setup... \n");
        #endif
        int start;
        do {
            start = lgGpioRead(GPIOhandle,4);
            if (start < 0) exit(5);
        } while (start);
        #ifdef VERBOSE
        printf("Starting cord has been setup\n");
        #endif
        lguSleep(1);

        #ifdef VERBOSE
        printf("Waiting start of the game... \n");
        #endif
        do {
            start = lgGpioRead(GPIOhandle,4);
            if (start < 0) exit(5);
        } while (!start);
    }
    tStart = time(NULL);

    lgGpioFree(GPIOhandle, 4);
    lgGpiochipClose(GPIOhandle);

    #ifdef VERBOSE
    printf("Game started! \n");
    #endif
    return tStart;
}

void finish_main() {

    teensy_idle();

    lidarEnd = 1;
    pthread_join(topLidarID, NULL); 

    if (currentPath != NULL) free(currentPath);

    // spi_close();

    dxl_close();

}

void *topLidar(void* arg) {
    StartLidar();
    while (!lidarEnd) {

    }
    return NULL;
    StopLidar();
}

int main(int argc, char const *argv[])
{

    ask_user_input_params();

    // ------ INIT -----
    time_t tOld = 0;
    time_t tStart = init_main();
    // ----- GAME -----

    do {
        // Update time values
        t = time(NULL) - tStart;
        dt = t - tOld;

        // ...
    } while (t < 120);


    // ----- FINISH -----

    finish_main();

    exit(0);
}