#include "SPI_Modules.h"
#include "lidarTop.h"
#include "dynamixels.h"
#include "graph.h"
#include <unistd.h>
#include <cstdio>
#include <stdio.h>
#include <stdlib.h> 
#include <unistd.h>  
#include <pthread.h> 
#include <termios.h>
#include <time.h>

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

void *topLidar(void* arg) {
    StartLidar();
    while (!lidarEnd) {

    }
    return NULL;
    StopLidar();
}

int main(int argc, char const *argv[])
{

    printf("Which team do I play for ? Press 'b' for team blue, 'y' for team yellow\n")
    do {
        int keyboard_input = getch();
        if (keyboard_input == ASCII_b || keyboard_input == ASCII_B) { color = TeamBlue; break; }
        else if (keyboard_input == ASCII_y || keyboard_input == ASCII_Y ) { color = TeamYellow; break; }
        printf("Invalid input color : %c\n", (char) keyboard_input);
    } while (1);

    // ------ INIT -----

    dxl_init_port();
    dxl_ping(1, 2.0);
    dxl_ping(3, 2.0);
    dxl_ping(6, 1.0);
    dxl_ping(8, 1.0);

    if (init_spi() != 0) exit(2);
    if (test_spi() != 0) exit(2);
    
    if (init_graph_from_file("./graphs/BL_V2.txt", color) != 0) exit(3);

    if (pthread_create(&topLidarID, NULL, topLidar, NULL) != 0) exit(4);
    
    int GPIOhandle = lgGpiochipOpen(4);
    if (GPIOhandle < 0) exit(5);
    if (lgGpioSetUser(GPIOhandle, "Bot Lightyear") < 0) exit(5);
    if (lgGpioClaimInput(GPIOhandle, LG_SET_PULL_NONE, 4) != 0) exit(5);

    // ----- CALIBRATION -----

    stpr_reset_all();
    stpr_calibrate_all();


    if (color == BLUE) {
        graph_level_update(graph_bases[3])
    }

    static time_t tOld = 0;
    {
        printf("Waiting for starting cord setup... \n");
        int start;
        do {
            start = lgGpioRead(GPIOhandle,4);
            if (start < 0) exit(5);
        } while (start);
        printf("Starting cord has been setup\n");
        lguSleep(1);

        printf("Waiting start of the game... \n");
        do {
            start = lgGpioRead(GPIOhandle,4);
            if (start < 0) exit(5);
        } while (!start);
    }
    static time_t tStart = time(NULL);

    lgGpioFree(GPIOhandle, 4);
    lgGpiochipClose(GPIOhandle);

    printf("Game started! \n");

    // ----- GAME -----

    do {
        // Update time values
        t = time(NULL) - tStart;
        dt = t - tOld;

        // ...
    } while (t < 120);


    // ----- FINISH -----

    lidarEnd = 1;
    pthread_join(topLidarID, NULL); 

    if (currentPath != NULL) free(currentPath);
    free_graph();

    close_spi();

    dxl_idle(1, 2.0);
    dxl_idle(3, 2.0);
    dxl_idle(6, 1.0);
    dxl_idle(8, 1.0);
    dxl_close_port();

    exit(0);
}