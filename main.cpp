#include "shared_variables.h"
#include "actions.h"
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
#include <cmath>

//#define VERBOSE
//#define TIME_MEAS

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
    printf("Which team do I play for ? Enter 'b' for team blue, 'y' for team yellow\n");
    string s; 
    do {
        std::cin >> s;
        char keyboard_input = s.at(0);
        if (keyboard_input == ASCII_b || keyboard_input == ASCII_B) { shared.color = TeamBlue; break; }
        else if (keyboard_input == ASCII_y || keyboard_input == ASCII_Y ) { shared.color = TeamYellow; break; }
        printf("Invalid input color : %c\n", keyboard_input);
    } while (1);

    printf("Please enter a starting base from the following : \n");
    if (shared.color == TeamBlue) {

        printf("'bottom right' (reserved), 'top right', 'middle left'\n");
        do {
            std::cin >> s;
            if (!s.compare("bottom right")) {
                shared.startingBaseID = shared.graph.friendlyBases[0];
                shared.odo.set_pos(0.225,0.035,M_PI_2);
                shared.set_robot_pos(0.225,0.035,M_PI_2);
                shared.teensy.set_position(0.225,0.035,M_PI_2);
                break;
            }
            if (!s.compare("top right")) {
                shared.startingBaseID = shared.graph.friendlyBases[1];
                shared.odo.set_pos(1.775,0.035,M_PI_2);
                shared.set_robot_pos(1.775,0.035,M_PI_2);
                shared.teensy.set_position(1.775,0.035,M_PI_2);
                break;
            }
            if (!s.compare("middle left")) {
                shared.startingBaseID = shared.graph.friendlyBases[2];
                shared.odo.set_pos(1.0,2.965,-M_PI_2);
                shared.set_robot_pos(1.0,2.965,-M_PI_2);
                shared.teensy.set_position(1.0,2.965,-M_PI_2);
                break;
            }
            printf("Invalid input base : %s\n", s);
        } while (1);

    } else {
        printf("'bottom left' (reserved), 'top left', 'middle right'\n");
        do {
            std::cin >> s;
            if (!s.compare("bottom left")) {
                shared.startingBaseID = shared.graph.friendlyBases[0];
                shared.odo.set_pos(0.225,2.965,-M_PI_2);
                shared.set_robot_pos(0.225,2.965,-M_PI_2);
                shared.teensy.set_position(0.225,2.965,-M_PI_2);
                break;
            }
            if (!s.compare("top left")) {
                shared.startingBaseID = shared.graph.friendlyBases[1];
                shared.odo.set_pos(1.775,2.965,-M_PI_2);
                shared.set_robot_pos(1.775,2.965,-M_PI_2);
                shared.teensy.set_position(1.775,2.965,-M_PI_2);
                break;
            }
            if (!s.compare("middle right")) {
                shared.startingBaseID = shared.graph.friendlyBases[2];
                shared.odo.set_pos(1.0,0.035,-M_PI_2);
                shared.set_robot_pos(1.0,0.035,-M_PI_2);
                shared.teensy.set_position(1.0,0.035,-M_PI_2);
                break;
            }
            printf("Invalid input base : %s\n", s);
        } while (1);
    }
}

void init_and_wait_for_start() {

    dxl_init_port();
    dxl_ping(6,1.0);
    dxl_ping(8,1.0);

    if (shared.graph.init_from_file("./graphs/BL_V3.txt", shared.color) != 0) exit(3);
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

    dxl_close_port();

}

void *localizer(void* arg) {
    //StartLidarTop();
    //LidarData lidarData;
    //init_lidar(&lidarData);
    double xOdo, yOdo, thetaOdo;
    double x, y, theta;
    //shared.get_robot_pos(&lidarData.x_odo, &lidarData.y_odo, &lidarData.theta_odo);
    while (!localizerEnd) {

        #ifdef TIME_MEAS
        clock_t start = clock();
        #endif

        //lidarGetRobotPosition(&lidarData, 0, false, lidarData.readLidar_lost);

        #ifdef TIME_MEAS
        clock_t lidarClock = clock();
        #endif

        shared.odo.get_pos(&xOdo, &yOdo, &thetaOdo);

        #ifdef TIME_MEAS
        clock_t odoClock = clock();
        #endif

        //x = (xOdo + lidarData.readLidar_x_robot)*0.5;
        //y = (yOdo + lidarData.readLidar_y_robot)*0.5;
        //theta = (thetaOdo + lidarData.readLidar_theta_robot)*0.5;

        shared.teensy.set_position(x,y,theta);

        #ifdef TIME_MEAS
        clock_t teensyClock = clock();
        #endif

        shared.set_robot_pos(x,y,theta);
        //shared.set_adv_pos(lidarData.readLidar_x_opponent,lidarData.readLidar_y_opponent);
        usleep(300000);
    }
    //StopLidarTop();
    return NULL;
}

int main(int argc, char const *argv[])
{

    ask_user_input_params();

    // ------ INIT -----
    
    init_and_wait_for_start();

    // ----- GAME -----

    decision.path = (graph_path_t *) malloc(sizeof(graph_path_t));
    do {
        make_decision(&decision);
        switch (decision.actionType)
        {
        case ReturnToBase :
            path_following_to_base(); 
            break;
        case Displacement :
            displacement_action(); 
            break;
        case TestAction :
            // Do test action here (looping forever if TESTS enable in decision.cpp)
            break; 
        default:
            break;
        }
    } while (decision.actionType != GameFinished);

    // ----- FINISH -----

    shared.teensy.idle();
    shared.steppers.reset_all();
    shared.servoFlaps.idle(); shared.grpDeployer.idle(); shared.grpHolder.idle();
    dxl_idle(6, 1.0);
    dxl_idle(8, 1.0);

    printf("Game finished\n");
    // TODO : show score
    getch();

    finish_main();

    exit(0);
}