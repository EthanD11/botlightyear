#include "shared_variables.h"
#include "actions.h"
#include "dynamixels.h"
#include "lidarTop.h"
#include "decision.h"
#include <unistd.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h> 
#include <time.h>
#include <unistd.h>  
#include <pthread.h> 
#include <termios.h>
#include <fcntl.h>
#include <cmath>
#include "oled.h"

#define VERBOSE
//#define TIME_MEAS

#define ASCII_b 98
#define ASCII_B 66
#define ASCII_y 121
#define ASCII_Y 89

pthread_t localizerID;
uint8_t localizerEnd = 0; // Set to one to finish localizer thread
void *localizer(void* arg);

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

        printf("'bottomright' (reserved), 'topright', 'middleleft'\n");
        do {
            std::cin >> s;
            if (!s.compare("bottomright")) {
                shared.startingBaseID = shared.graph->friendlyBases[0];
                shared.odo->set_pos(0.225,0.035,M_PI_2);
                shared.set_robot_pos(0.225,0.035,M_PI_2);
                shared.teensy->set_position(0.225,0.035,M_PI_2);
                break;
            }
            if (!s.compare("topright")) {
                shared.startingBaseID = shared.graph->friendlyBases[1];
                shared.odo->set_pos(1.775,0.035,M_PI_2);
                shared.set_robot_pos(1.775,0.035,M_PI_2);
                shared.teensy->set_position(1.775,0.035,M_PI_2);
                break;
            }
            if (!s.compare("middleleft")) {
                shared.startingBaseID = shared.graph->friendlyBases[2];
                shared.odo->set_pos(1.0,2.965,-M_PI_2);
                shared.set_robot_pos(1.0,2.965,-M_PI_2);
                shared.teensy->set_position(1.0,2.965,-M_PI_2);
                break;
            }
            std::cout << "Invalid input base : " << s << "\n";
        } while (1);

    } else {
        printf("'bottomleft' (reserved), 'topleft', 'middleright'\n");
        do {
            std::cin >> s;
            if (!s.compare("bottomleft")) {
                shared.startingBaseID = shared.graph->friendlyBases[0];
                shared.odo->set_pos(0.035,2.775,0);
                shared.set_robot_pos(0.035,2.775,0);
                shared.teensy->set_position(0.035,2.775,0);
                break;
            }
            if (!s.compare("topleft")) {
                shared.startingBaseID = shared.graph->friendlyBases[1];
                shared.odo->set_pos(1.775,2.965,-M_PI_2);
                shared.set_robot_pos(1.775,2.965,-M_PI_2);
                shared.teensy->set_position(1.775,2.965,-M_PI_2);
                break;
            }
            if (!s.compare("middleright")) {
                shared.startingBaseID = shared.graph->friendlyBases[2];
                shared.odo->set_pos(1.0,0.035,M_PI_2);
                shared.set_robot_pos(1.0,0.035,M_PI_2);
                shared.teensy->set_position(1.0,0.035,M_PI_2);
                break;
            }
            std::cout << "Invalid input base : " << s << "\n";
        } while (1);
    }
}

void init_and_wait_for_start() {

    oled_init(); 
    
    dxl_init_port();
    dxl_ping(6,1.0);
    dxl_ping(8,1.0);

    if (shared.graph->init_from_file("./graphs/BL_V3.txt", shared.color) != 0) exit(3);

    printf("%d\n",shared.graph->nodes[0].level);
    /*for (int i=0; i<3; i++) {
        printf("Friendly base %d \n", shared.graph->friendlyBases[i]);
        printf("Adversary base %d \n", shared.graph->adversaryBases[i]);
        printf("Friendly planter %d \n", shared.graph->friendlyPlanters[i]);
        printf("Adversary planter %d \n", shared.graph->adversaryPlanters[i]);
    }
    for (int i=0; i<6; i++) {
        printf("Plants at %d \n", shared.graph->plants[i]);
    }*/

    double kt = 0.5;
    double kn = 0.7; // 0 < kn <= 1
    double kz = 20.0;
    double delta = 80e-3; // delta is in meters
    double sigma = 2.;
    double epsilon = M_PI/8; // epsilon is in radians
    double wn = 0.2; // Command filter discrete cutoff frequency
    double kv_en = 0.;
    shared.teensy->set_path_following_gains(kt, kn, kz, sigma, epsilon, kv_en, delta, wn);
    
    if (pthread_create(&localizerID, NULL, localizer, NULL) != 0) exit(4);

    StartLidarBottom();

    shared.steppers->reset_all();
    shared.steppers->calibrate_all(CALL_BLOCKING, shared.valids);
    shared.servoFlaps->raise();
    
    shared.start_timer();
    // Generate random seed
    srand(time(NULL));
}

void finish_main() {

    localizerEnd = 1;
    pthread_join(localizerID, NULL);

    shared.~SharedVariables();

    dxl_close_port();

}

void *localizer(void* arg) {
    StartLidarTop();
    LidarData *lidarData = new LidarData();
    init_lidar(lidarData);
    // double xOdo, yOdo, thetaOdo;
    double x = 0, y = 0, theta = 0;
    // double odoWeight = 1.0;
    shared.get_robot_pos(&lidarData->x_odo, &lidarData->y_odo, &lidarData->theta_odo);
    while (!localizerEnd) {

        // teensy_mode_t teensyMode = shared.teensy->ask_mode();

        #ifdef TIME_MEAS
        clock_t odoClock;
        clock_t start = clock();
        #endif

        lidarGetRobotPosition(lidarData, 10, false, lidarData->readLidar_lost);

        #ifdef TIME_MEAS
        clock_t lidarClock = clock();
        #endif

        // if ((teensyMode == ModePositionControlOver || teensyMode == ModeIdle) && !lidarData->readLidar_lost) {
        //     shared.odo->get_pos(&xOdo, &yOdo, &thetaOdo);
        //     #ifdef TIME_MEAS
        //     odoClock = clock();
        //     #endif
        //     x = xOdo*odoWeight + lidarData->readLidar_x_robot*(1-odoWeight);
        //     y = yOdo*odoWeight + lidarData->readLidar_y_robot*(1-odoWeight);
        //     theta = thetaOdo*odoWeight + lidarData->readLidar_theta_robot*(1-odoWeight);
        // } else {
        shared.odo->get_pos(&x, &y, &theta);
        #ifdef TIME_MEAS
        odoClock = clock();
        #endif
        //}
        shared.teensy->set_position(x,y,theta);

        #ifdef TIME_MEAS
        clock_t teensyClock = clock();
        #endif

        shared.set_robot_pos(x,y,theta);
        //if (!lidarData->readLidar_lost) { 
            shared.set_adv_pos(
            lidarData->readLidar_x_opponent,
            lidarData->readLidar_y_opponent,
            lidarData->readLidar_d_opponent,
            lidarData->readLidar_a_opponent);
        // shared.set_adv_pos(
        // 400,
        // 400,
        // 400,
        // 0);
            shared.graph->update_adversary_pos(lidarData->readLidar_x_opponent, lidarData->readLidar_y_opponent);
        // shared.graph->update_adversary_pos(400, 400);
        //}
        lidarData->x_odo = x; lidarData->y_odo = y; lidarData->theta_odo = theta;

        #ifdef VERBOSE
        //TODO TODO
        #endif
        /*printf("                    %.3f %.3f %.3f   %.3f %.3f\n", lidarData->readLidar_x_robot, lidarData->readLidar_y_robot, lidarData->readLidar_theta_robot*180/M_PI, lidarData->readLidar_d_opponent,lidarData->readLidar_a_opponent*180.0/M_PI);
        printf("                    Adversary at %.3f %.3f\n", lidarData->readLidar_x_opponent,lidarData->readLidar_y_opponent);
        printf("odometry : %.3f %.3f %.3f\n", x, y, theta*180/M_PI);*/




        #ifdef TIME_MEAS
        printf("Timing results : \n");
        printf("Lidar took %ld clock cycles to update.\n\tCumulated time since iteration start : %ld\n", lidarClock - start, lidarClock - start);
        printf("Odometry took %ld clock cycles to update.\n\tCumulated time since iteration start : %ld\n", odoClock - lidarClock, odoClock - start);
        printf("Teensy took %ld clock cycles to update.\n\tCumulated time since iteration start : %ld\n", teensyClock - odoClock, teensyClock - start);
        #endif
        usleep(350000);
    }
    clear_lidar(lidarData);
    StopLidarTop();
    return NULL;
}

int main(int argc, char const *argv[])
{

    ask_user_input_params();

    // ------ INIT -----
   
    init_and_wait_for_start();

    shared.score +=16;
    oled_score_update(shared.score); 

    // ----- GAME -----
    uint8_t gameFinished = 0;
    do {

        Action* decided_action = make_decision();
        printf("Action type : %d\n", decided_action->action_type);
        double x = 0, y = 0, theta = 0;
        shared.get_robot_pos(&x,&y,&theta);
        printf("Current pos : (%.3f,%.3f,%.3f)\n",x,y,theta);

        gameFinished = (decided_action->action_type == GameFinished);
        decided_action->do_action();
        oled_score_update(shared.score); 
        free(decided_action->path);
        decided_action->path = NULL;
        delete decided_action;
    } while (!gameFinished);

    // ----- FINISH -----
    // shared.score +=15;
    // // shared.score *=0.9;
    // oled_score_update(shared.score); 
    
    shared.teensy->idle();
    shared.steppers->reset_all();
    shared.servoFlaps->idle(); shared.grpDeployer->idle(); shared.grpHolder->idle();
    dxl_idle(6, 1.0);
    dxl_idle(8, 1.0);
    StopLidarBottom();
    printf("Game finished with time %d\n", shared.update_and_get_timer());
    // TODO : show score
    getch();

    finish_main();

    exit(0);
}