#include "shared_variables.h"
#include "dynamixels.h"
#include "lidarTop.h"
#include "decision.h"
#include <unistd.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h> 
#include <unistd.h>  
#include <pthread.h> 
#include <termios.h>
#include <fcntl.h>

#define VERBOSE

#define ASCII_b 98
#define ASCII_B 66
#define ASCII_y 121
#define ASCII_Y 89

decision_t decision;

pthread_t localizerID;
uint8_t localizerEnd = 0; // Set to one to finish localizer thread

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

void init_and_wait_for_start() {

    dxl_init_port();
    dxl_ping(6,1.0);
    dxl_ping(8,1.0);

    if (shared.graph.init_from_file("./graphs/BL_V2.txt", shared.color) != 0) exit(3);
    shared.graph.node_level_update(shared.graph.adversaryBases[0], 3, DISABLE_PROPAGATION);

    if (pthread_create(&localizerID, NULL, localizer, NULL) != 0) exit(4);

    shared.steppers.reset_all();
    shared.steppers.calibrate_all();
    
    shared.start_timer();

}

void finish_main() {

    localizerEnd = 1;
    pthread_join(localizerID, NULL);

    shared.~SharedVariables();

    free(decision.path);

    dxl_close();

}

void *localizer(void* arg) {
    StartLidar();
    double x, y, theta, xAdv, yAdv, thetaAdv;
    while (!localizerEnd) {
        
        shared.teensy.set_position(x,y,theta);
        shared.set_robot_pos(x,y,theta);
        shared.set_adv_pos(xAdv,yAdv,thetaAdv);
        usleep(300000);
    }
    StopLidar();
    return NULL;
}

int main(int argc, char const *argv[])
{

    ask_user_input_params();

    // ------ INIT -----
    
    init_and_wait_for_start();

    // ----- GAME -----

    decision.path = malloc(sizeof(graph_path_t));
    do {
        make_decision(&decision);
    } while (decision.actionType != GameFinished);

    // ----- FINISH -----

    teensy.idle();
    steppers.reset_all();
    servoFlaps.idle(); grpDeployer.idle(); grpHolder.idle();
    dxl_idle(6, 1.0);
    dxl_idle(8, 1.0);

    printf("Game finished\n");
    // TODO : show score
    getch();

    finish_main();

    exit(0);
}