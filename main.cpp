#include "shared_variables.h"
#include "dynamixels.h"
#include <unistd.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h> 
#include <unistd.h>  
#include <pthread.h> 
#include <termios.h>
#include <fcntl.h>
#include <lgpio.h>

#define VERBOSE

#define ASCII_b 98
#define ASCII_B 66
#define ASCII_y 121
#define ASCII_Y 89

pthread_t topLidarID;
uint8_t lidarEnd = 0;

SharedVariables shared = SharedVariables();


int getch()
{
#if defined(__linux__) || defined(__APPLE__)
  struct termios oldt, newt;
  int ch;
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  ch = getchar();
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  return ch;
#elif defined(_WIN32) || defined(_WIN64)
  return _getch();
#endif
}

void ask_user_input_params() {
    printf("Which team do I play for ? Press 'b' for team blue, 'y' for team yellow\n");
    do {
        int keyboard_input = getch();
        if (keyboard_input == ASCII_b || keyboard_input == ASCII_B) { shared.color = TeamBlue; break; }
        else if (keyboard_input == ASCII_y || keyboard_input == ASCII_Y ) { shared.color = TeamYellow; break; }
        printf("Invalid input color : %c\n", keyboard_input);
    } while (1);
}

void init_main() {

    dxl_init_port();


    if (shared.graph.init_from_file("./graphs/BL_V2.txt", shared.color) != 0) exit(3);
    shared.graph.node_level_update(shared.graph.adversaryBases[0], 3, DISABLE_PROPAGATION);

    if (pthread_create(&topLidarID, NULL, topLidar, NULL) != 0) exit(4);

    shared.steppers.reset_all();
    shared.steppers.calibrate_all();
    
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