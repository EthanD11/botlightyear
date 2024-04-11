#include "../headers/SPI_Modules.h"
#include <stdio.h>

const double deg_to_rads = M_PI/180;

int main(int argc, char const *argv[])
{
    init_spi(); 
    lguSleep(0.5);

    // Set position control gains
    double kp = 0.8;
    double ka = 2.5;
    double kb = -0.5;
    double kw = 4.0;
    teensy_set_position_controller_gains(kp, ka, kb, kw);

    // Set path follower gains
    double kt = 2.0;
    double kn = 0.3; // 0 < kn <= 1
    double kz = 25.0;
    double delta = 20e-3; // delta is in meters
    double sigma = 10;
    double epsilon = M_PI/8; // epsilon is in radians
    double wn = 0.2; // Command filter discrete cutoff frequency
    double kv_en = 0.;
    teensy_set_path_following_gains(kt, kn, kz, sigma, epsilon, kv_en, delta, wn);

    // Define trajectory
    int ncheckpoints = 2;
    double x[2] = {10e-2, 32.5e-2};
    double y[2] = {  1.5,     1.5};
    double theta_start = 0.;
    double theta_end = 0;


    // Set teensy and odometry starting positions
    double xpos, ypos, thetapos;
    odo_set_pos(x[0], y[0], theta_start);
    teensy_set_position(x[0], y[0], theta_start);
    
    // Orientation with position control
    teensy_pos_ctrl(x[1], y[1], theta_end);
    lguSleep(0.1);

    // Reset teensy estimated position with odometry
    while (true) {
        odo_get_pos(&xpos, &ypos, &thetapos);
        teensy_set_position(xpos, ypos, thetapos);
        lguSleep(0.3);
    }


    close_spi();
    return 0;
}
