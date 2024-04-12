#include "teensy.h"
#include "odometry.h"
#include "dynamixels.h"
#include <stdio.h>
#include <unistd.h>
#include <cmath>
#include <lgpio.h>

SPIBus spiBus = SPIBus();
GPIOPins pins = GPIOPins(); 
Teensy teensy = Teensy(&spiBus, &pins);
Odometry odo = Odometry(&spiBus);

const double deg_to_rads = M_PI/180;

int main(int argc, char const *argv[]) {
    dxl_init_port();
    dxl_ping(6, 1.0);
    dxl_ping(8, 1.0);

    solar_panel(Blue, 0);

    usleep(500000);

    // Set position control gains
    double kp = 0.8;
    double ka = 2.5;
    double kb = -0.5;
    double kw = 4.0;
    teensy.set_position_controller_gains(kp, ka, kb, kw);

    // Set path follower gains
    double kt = 2.0;
    double kn = 0.3; // 0 < kn <= 1
    double kz = 25.0;
    double delta = 20e-3; // delta is in meters
    double sigma = 10;
    double epsilon = M_PI/8; // epsilon is in radians
    double wn = 0.2; // Command filter discrete cutoff frequency
    double kv_en = 0.;
    teensy.set_path_following_gains(kt, kn, kz, sigma, epsilon, kv_en, delta, wn);

    // Define trajectory
    int ncheckpoints = 2;
    double x[2] = {10e-2, 32.5e-2};
    double y[2] = {  1.5,     1.5};
    double theta_start = 0;
    double theta_end = 0;


    // Set teensy and odometry starting positions
    double xpos, ypos, thetapos;
    odo.set_pos(x[0], y[0], theta_start);
    teensy.set_position(x[0], y[0], theta_start);
    
    // Orientation with position control
    teensy.pos_ctrl(x[1], y[1], theta_end);
    lguSleep(0.1);

    // Reset teensy estimated position with odometry
    do {
        odo.get_pos(&xpos, &ypos, &thetapos);
        teensy.set_position(xpos, ypos, thetapos);
        printf("%.3f,%.3f,%.3f\n",xpos, ypos, thetapos);
        lguSleep(0.3);
    } while(abs(xpos-x[1]) > 0.01); 

    lguSleep(2.0);
    solar_panel(Blue, 0);

    dxl_close_port();

    return 0;
}
