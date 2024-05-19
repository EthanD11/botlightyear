#include "teensy.h"
#include "odometry.h"
#include "servos.h"
#include <string>
#include <chrono>
#include <stdio.h>
#include <unistd.h>
#include <cmath>
#include <lgpio.h>

#define NLAPS 5
#define N1LAP 8

using namespace std::chrono;

SPIBus spiBus = SPIBus();
GPIOPins pins = GPIOPins(); 
Teensy teensy = Teensy(&spiBus, &pins);
Odometry odo = Odometry(&spiBus);
Flaps servoFlaps = Flaps(&spiBus);

const double deg_to_rads = M_PI/180;


int main() {
    usleep(500000);
    servoFlaps.raise();
    lguSleep(0.2);
    servoFlaps.idle();

    double kp = 0.8;
    double ka = 6.0;
    double kb = -2.0;
    double kw = 1.2;
    teensy.set_position_controller_gains(kp, ka, kb, kw);

    double kt = 0.1;
    double kn = 0.7; // 0 < kn <= 1
    double kz = 10.0;
    double delta = 80e-3; // delta is in meters
    double sigma = 0.;
    double epsilon = M_PI/8; // epsilon is in radians
    double wn = 0.2; // Command filter discrete cutoff frequency
    double kv_en = 0.;
    teensy.set_path_following_gains(kt, kn, kz, sigma, epsilon, kv_en, delta, wn);

    lguSleep(0.1);
    int ncheckpoints = 1+N1LAP*NLAPS;
    // double x1lap[6] = {1.0,0.4,0.4,1.2,1.6,1.6};
    // double y1lap[6] = {0.4,1.0,2.0,2.5,2.0,1.0};
    double x1lap[N1LAP] = {1.0, 0.5,  1.0, 1.5,  1.0, 0.5, 1.0, 1.5};
    double y1lap[N1LAP] = {0.4, 0.8, 1.28, 1.8, 2.2, 1.8, 1.28, 0.8};
    // double x1lap[N1LAP] = {1.0, 0.75, 0.4, 0.4, 0.75, 0.75, 0.4, 0.4, 0.75, 1.0,  0.8, 0.4, 0.75, 1.25, 1.6, 1.2,  1.0, 1.25, 1.6, 1.5};
    // double y1lap[N1LAP] = {0.4, 0.8, 0.85, 1.2, 1.25, 1.6, 1.65, 2.0, 2.05, 2.15, 2.4, 2.4, 2.05, 2.05, 2.4, 2.4, 2.15, 2.05, 1.5, 0.5};
    
    double x[NLAPS*N1LAP+1];
    double y[NLAPS*N1LAP+1];
    int j = 0;
    for (int i = 0; i < ncheckpoints; i++) {
        x[i] = x1lap[j];
        y[i] = y1lap[j];
        j = (j >= N1LAP-1) ? 0: j+1;
    }
    for (int i = 0; i < ncheckpoints; i++) {
        printf("%d: (%.3f, %.3f)\n", i, x[i], y[i]);
        if (i % N1LAP == N1LAP-1) printf("\n");
    }
    
    double theta_start = M_PI/2;
    double theta_end = M_PI/2;
    double vref = 0.4;
    double dist_goal_reached = 2.0;

    double xpos = 0, ypos = 0, thetapos = 0;
    odo.set_pos(x[0], y[0], theta_start);
    lguSleep(0.5);
    teensy.set_position(x[0], y[0], theta_start);
    lguSleep(1);

    teensy.pos_ctrl(x[0], y[0], atan2(y[1]-y[0], x[1]-x[0]));
    lguSleep(1);
    teensy.path_following(x, y, ncheckpoints, theta_start, theta_end, vref, dist_goal_reached);
    lguSleep(0.1);
    microseconds start_us = duration_cast< microseconds >(system_clock::now().time_since_epoch());
    microseconds current_us =  start_us; 
    microseconds last_save_us = start_us;
    microseconds last_pos_us = start_us;
    int64_t delta_save_us, delta_pos_us, time;
    char filename[256] = "mobility-data-6.csv";
    FILE* file = fopen(filename, "w");
    fprintf(file, "time,x,y,theta,mode\n");
    
    fclose(file);
    while (teensy.ask_mode() != ModePositionControlOver) {

        current_us =  duration_cast< microseconds >(system_clock::now().time_since_epoch());
        delta_pos_us = (int64_t) (current_us - last_pos_us).count();  
        if (delta_pos_us > 100000) { // Reset teensy pos every 500ms
            last_pos_us = duration_cast< microseconds >(system_clock::now().time_since_epoch());    
            odo.get_pos(&xpos, &ypos, &thetapos);
            teensy.set_position(xpos, ypos, thetapos);
            printf("teensy state : %d\tteensy pos: (%f,%f,%f)\n", teensy.ask_mode(), xpos, ypos, thetapos);
        }

        current_us =  duration_cast< microseconds >(system_clock::now().time_since_epoch());
        delta_save_us = (int64_t) (current_us - last_save_us).count();
        if (delta_save_us > 25e3) { // Save data every 25ms
            last_save_us =  duration_cast< microseconds >(system_clock::now().time_since_epoch());
            time = (current_us - start_us).count();
            odo.get_pos(&xpos, &ypos, &thetapos);
            file = fopen(filename, "a");
            
            fprintf(file, "%.5f,%.5f,%.5f,%.5f",1e-6*((double) time),xpos,ypos,thetapos);
            if (teensy.ask_mode() == ModePathFollowing) {
                fprintf(file,"ModePathFollowingn\n");
            }
            else if (teensy.ask_mode() == ModePositionControl) {
                fprintf(file, "ModePositionControl\n");
            } else {
                fprintf(file,"Other\n");
            }
            fclose(file);
        }
    }
    teensy.idle();
    servoFlaps.idle();
}