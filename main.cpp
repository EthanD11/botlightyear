#include "shared_variables.h"
#include "actions.h"
#include "dynamixels.h"
#include "lidarTop.h"
#include "decision.h"
#include <unistd.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h> 
#include <chrono>
#include <unistd.h>  
#include <pthread.h> 
#include <termios.h>
#include <fcntl.h>
#include <cmath>
#include "oled.h"

#define VERBOSE
// #define TIME_MEAS

#define LIDAR_BOTTOM
// #define LIDAR_TOP
// #define DXL
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
    fprintf(stderr,"Which team do I play for ? Enter 'b' for team blue, 'y' for team yellow\n");
    string s; 
    do {
        std::cin >> s;
        char keyboard_input = s.at(0);
        if (keyboard_input == ASCII_b || keyboard_input == ASCII_B) { shared.color = TeamBlue; break; }
        else if (keyboard_input == ASCII_y || keyboard_input == ASCII_Y ) { shared.color = TeamYellow; break; }
        fprintf(stderr,"Invalid input color : %c\n", keyboard_input);
    } while (1);

    fprintf(stderr,"Please enter a starting base from the following : \n");
    if (shared.color == TeamBlue) {

        fprintf(stderr,"'bottomright' (reserved), 'topright', 'middleleft'\n");
        do {
            std::cin >> s;
            if (!s.compare("bottomright")) {
                shared.startingBaseID = shared.graph->friendlyBases[0];
                // #ifdef CLEAR_POTS
                // shared.odo->set_pos(0.035,0.165,0);
                // shared.set_robot_pos(0.035,0.165,0);
                // shared.teensy->set_position(0.035,0.165,0);
                // #else
                shared.odo->set_pos(0.035,0.155,0);
                shared.set_robot_pos(0.035,0.155,0);
                shared.teensy->set_position(0.035,0.155,0);
                // #endif
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
            fprintf(stderr,"Invalid input base : %s\n",s);
        } while (1);

    } else {
        fprintf(stderr,"'bottomleft' (reserved), 'topleft', 'middleright'\n");
        do {
            std::cin >> s;
            if (!s.compare("bottomleft")) {
                shared.startingBaseID = shared.graph->friendlyBases[0];
                // #ifdef CLEAR_POTS
                // shared.odo->set_pos(0.035,2.835,0);
                // shared.set_robot_pos(0.035,2.835,0);
                // shared.teensy->set_position(0.035,2.835,0);
                // #else
                shared.odo->set_pos(0.035,2.845,0);
                shared.set_robot_pos(0.035,2.845,0);
                shared.teensy->set_position(0.035,2.845,0);
                // #endif
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
            fprintf(stderr,"Invalid input base : %s\n",s);
        } while (1);
    }
}

void init_and_wait_for_start() {

    oled_init(); 
    #ifdef DXL
    dxl_init_port();
    dxl_ping(6,1.0);
    dxl_ping(8,1.0);
    shared.valids[3] = 1; //For now, if it passes this far, dynamixels are valid
    #endif

    #ifdef LIDAR_TOP
    StartLidarTop();
    #endif

    if (shared.graph->init_from_file("./graphs/BL_V3.txt", shared.color) != 0) exit(3);

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
    shared.teensy->set_position_controller_gains(0.9,2.5,-1.0,1.2);
    
    if (pthread_create(&localizerID, NULL, localizer, NULL) != 0) exit(4);
    #ifdef LIDAR_BOTTOM
    StartLidarBottom();
    shared.valids[4] = 1;
    #endif

    shared.steppers->reset_all();
    shared.steppers->calibrate_all(CALL_BLOCKING, shared.valids);
    shared.steppers->plate_move(0, CALL_BLOCKING); 
    shared.servoFlaps->raise();

    oled_ready_to_start(); 
    
    shared.start_timer();
    // Generate random seed
    srand(time(NULL));
}

void finish_main() {

    localizerEnd = 1;
    pthread_join(localizerID, NULL);

    shared.~SharedVariables();
    #ifdef DXL
    dxl_close_port();
    #endif
}

void *localizer(void* arg) {

    // double xOdo, yOdo, thetaOdo;
    double x = 0, y = 0, theta = 0;
    // double odoWeight = 1.0;

    #ifdef TIME_MEAS
    std::chrono::_V2::system_clock::time_point start, lidarClock, odoClock, teensyClock, sharedClock;
    double totalDur;
    std::chrono::duration<double> lidarDur, odoDur, teensyDur, sharedDur;
    #endif

    #ifdef LIDAR_TOP
    LidarData *lidarData = new LidarData();
    init_lidar(lidarData);
    lidarData->x_odo = 0, lidarData->y_odo = 0, lidarData->theta_odo = 0;
    shared.get_robot_pos(&lidarData->x_odo, &lidarData->y_odo, &(lidarData->theta_odo));
    #else
    shared.set_adv_pos(4,0,4,0);
    shared.graph->update_adversary_pos(4,0);
    #endif

    int8_t teensyI = 0;
    #ifdef LIDAR_TOP
    int8_t teensyN = 1;
    #else
    int8_t teensyN = 4;
    #endif

    while (!localizerEnd) {

        // teensy_mode_t teensyMode = shared.teensy->ask_mode();

        #ifdef TIME_MEAS
        start = std::chrono::high_resolution_clock::now();
        #endif

        #ifdef LIDAR_TOP
        lidarGetRobotPosition(lidarData, 10, false, lidarData->readLidar_lost);
        #endif

        #ifdef TIME_MEAS
        lidarClock = std::chrono::high_resolution_clock::now();
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
        odoClock = std::chrono::high_resolution_clock::now();
        #endif
        //}
        if (++teensyI == teensyN) {
            shared.teensy->set_position(x,y,theta);
            #ifdef TIME_MEAS
            teensyClock = std::chrono::high_resolution_clock::now();
            #endif
            teensyI = 0;
        }

        shared.set_robot_pos(x,y,theta);
        #ifdef LIDAR_TOP
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
        #endif
        #ifdef TIME_MEAS
        sharedClock = std::chrono::high_resolution_clock::now();
        #endif

        #ifdef VERBOSE
        //TODO TODO
        #endif
        /*printf("                    %.3f %.3f %.3f   %.3f %.3f\n", lidarData->readLidar_x_robot, lidarData->readLidar_y_robot, lidarData->readLidar_theta_robot*180/M_PI, lidarData->readLidar_d_opponent,lidarData->readLidar_a_opponent*180.0/M_PI);
        printf("                    Adversary at %.3f %.3f\n", lidarData->readLidar_x_opponent,lidarData->readLidar_y_opponent);
        printf("odometry : %.3f %.3f %.3f\n", x, y, theta*180/M_PI);*/




        #ifdef TIME_MEAS
        totalDur = 0;
        printf("Timing results : \n");

        lidarDur = lidarClock - start; totalDur += lidarDur.count();
        printf("Lidar took %.3f microseconds to update.\n\tCumulated time since iteration start : %.3f\n", 1e6*lidarDur.count(), 1e6*totalDur);

        odoDur = odoClock - lidarClock; totalDur += odoDur.count();
        printf("Odometry took %.3f microseconds to update.\n\tCumulated time since iteration start : %.3f\n", 1e6*odoDur.count(), 1e6*totalDur);

        if (!teensyI) {
            teensyDur = teensyClock - odoClock; totalDur += teensyDur.count();
            printf("Teensy took %.3f microseconds to update.\n\tCumulated time since iteration start : %.3f\n", 1e6*teensyDur.count(), 1e6*totalDur);

            sharedDur = sharedClock - teensyClock; totalDur += lidarDur.count();
            printf("Shared variables took %.3f microseconds to update.\n\tCumulated time since iteration start : %.3f\n", 1e6*sharedDur.count(), 1e6*totalDur);
        } else {
            sharedDur = sharedClock - odoClock; totalDur += lidarDur.count();
            printf("Shared variables took %.3f microseconds to update.\n\tCumulated time since iteration start : %.3f\n", 1e6*sharedDur.count(), 1e6*totalDur);
        }
        #endif
        usleep(50000);
    }
    
    #ifdef LIDAR_TOP
    clear_lidar(lidarData);
    StopLidarTop();
    #endif
    return NULL;
}

int main(int argc, char const *argv[])
{

    ask_user_input_params();

    // ------ INIT -----
   
    init_and_wait_for_start();

    shared.score += 21;
    oled_score_update(shared.score); 

    // ----- GAME -----

    uint8_t gameFinished = 0;
    do {
        #ifdef TIME_MEAS
        std::chrono::_V2::system_clock::time_point start, decisionClock;

        start = std::chrono::high_resolution_clock::now();
        Action *decided_action = make_decision();
        decisionClock = std::chrono::high_resolution_clock::now();

        std::chrono::duration<double> decisionDur = decisionClock - start;
        printf("Execution time of decision : %.1f\n", 1e9*decisionDur.count());

        #else
        Action* decided_action = make_decision();
        #endif
        printf("Action type : %d\n", decided_action->action_type);
        #ifndef CLEAR_POTS
        double x = 0, y = 0, theta = 0;
        #endif
        shared.get_robot_pos(&x,&y,&theta);
        #ifdef VERBOSE
        printf("Current pos : (%.3f,%.3f,%.3f)\n",x,y,theta);
        #endif

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
    #ifdef DXL
    dxl_idle(6, 1.0);
    dxl_idle(8, 1.0);
    #endif
    #ifdef LIDAR_BOTTOM
    StopLidarBottom();
    #endif
    printf("Game finished with time %d\n", shared.update_and_get_timer());
    // TODO : show score
    getch();

    finish_main();

    exit(0);
}