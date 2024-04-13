#include "action_SP.h"
#include <lgpio.h>

extern SharedVariables shared;

SPIBus spiBus = shared.spiBus;
GPIOPins pins = shared.pins;
Odometry odo = shared.odo; 
Teensy teensy = shared.teensy;

/* SOLAR_PANEL_PC: Position Control for Solar Panels 
   x, y, theta: robot position at the start of the displacement action
   xa, ya, ta : robot position actuated by thread TopLidar (odometry)
   Position Control from one solar panel to another, while checking current position from desired one
   While displacement, reset dxl 8 (wheel) to init position for left-right turn
*/
void solar_panel_pc() {

    //TO DO : Ask Ethan, define position control after idle

    // Retrieve robot current position
    double x, y, theta; 
    shared.get_robot_pos(&x, &y, &theta); 

    // Set position control gains (see with Ethan?)
    double kp = 0.8;
    double ka = 2.5;
    double kb = -0.5;
    double kw = 4.0;
    teensy.set_position_controller_gains(kp, ka, kb, kw);

    // Define trajectory to next SP
    int ncheckpoints = 2;
    double xc[2] = {x, x+22.5e-2};
    double yc[2] = {y,         y};
    double theta_start = theta;
    double theta_end = theta;
    
    // Orientation with position control
    teensy.pos_ctrl(xc[1], yc[1], theta_end);
    lguSleep(0.1);

    // Reset solar panel wheel (dxl 8)
    dxl_init_sp(); 

    // Read position with odometry : waiting end to start function turn_solar_panel
    do {
        double xa, ya, ta; 
        shared.get_robot_pos(&xa, &ya, &ta);
        lguSleep(0.3);
    } while(abs(xa-xc[1]) > 0.01); 
    
}

/* TURN_SOLAR_PANEL: Action sequence to turn a solar panel 
   bool reserved: if True, solar panels are private for the team, no need to read camera angle*/
void turn_solar_panel(bool reserved, uint8_t sp_counter) { 

    // Team management (rework to do)
    team_t team;
    
    switch(shared.color) {
        case TeamBlue:
            team = Blue;  
            break; 
        case TeamYellow:
            team = Yellow; 
            break;
    }

    // Case asking more than one solar panel
    while (sp_counter > 1) {
        // Action turn solar panel
        dxl_deploy(Down);
        if (reserved) {
            dxl_turn(team, 0); 
        }
        else {
            double angle = tagSolar(); 
            dxl_turn(team, angle); 
        }
        dxl_deploy(Mid);

        // Update dynamic score
        shared.score += 5;
        sp_counter--; 

        // Go to next
        solar_panel_pc();
    }

    if (sp_counter == 1) {
        //Action turn solar panel
        dxl_deploy(Down);
        if (reserved) {
            dxl_turn(team, 0); 
        }
        else {
            double angle = tagSolar(); 
            dxl_turn(team, angle); 
        }
        dxl_deploy(Up);

        // Update dynamic score
        shared.score += 5; 
        sp_counter--; 
    }

    // Reset solar panel wheel 
    dxl_init_sp();

    /* TO DO: If sp_counter = 0: return state action finish
              Return interrupt*/

}