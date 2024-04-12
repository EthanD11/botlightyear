#include "action_SP.h"

SPIBus spiBus = shared.spiBus;
GPIOPins pins = shared.pins;
Odometry odo = shared.odo; 
Teensy teensy = shared.teensy;

/* SOLAR_PANEL_PC: Position Control for Solar Panels */
void solar_panel_pc() {

    // Retrieve robot current position
    double x, y, theta; 
    shared.get_robot_pos(&x, &y, &theta); 

    // Set position control gains
    double kp = 0.8;
    double ka = 2.5;
    double kb = -0.5;
    double kw = 4.0;
    teensy.set_position_controller_gains(kp, ka, kb, kw);

    // Define trajectory to next SP
    int ncheckpoints = 2;
    double x[2] = {x, x+22.5e-2};
    double y[2] = {y,         y};
    double theta_start = theta;
    double theta_end = theta;

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

}

void turn_solar_panel(bool reserved) { 

    team_t team; 
    extern team_color_t color; 
    
    switch(color): 
        case TeamBlue: 
            team = Blue; 
        case TeamYellow:
            team = Yellow; 

    if (reserved) {
        
    }
    solar_panel_pc();


}