#include "action_SP.h"
#include "action_displacement.h"
#include <lgpio.h>

extern SharedVariables shared;


void solar_panel_pc() {

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
    if (shared.color == TeamBlue) {
        double xc[2] = {x, x+22.5e-2};
    }
    else if (shared.color == TeamYellow) { //Backtracking
        double xc[2] = {x, x-22.5e-2};
    }
    double yc[2] = {y,         y};
    double theta_start = theta;
    double theta_end = theta;
    
    // Orientation with position control
    teensy.pos_ctrl(xc[1], yc[1], theta_end);
    lguSleep(0.1);

    // Reset solar panel wheel (dxl 8)
    dxl_init_sp(); 

    // Waiting end to start function turn_solar_panel
    // Check Teensy mode
    while ((teensy.ask_mode()) != ModePositionControlOver) { 
            uSleep(1000);
    } 

    return;   
}


void turn_solar_panel_reserved(uint8_t sp_counter) { 

    // TO DO: If sp_counter > 3: raise exception 


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

    // Path following to action : check interrupt
    if (path_following_to_action() != 0) {
        return;
    } 

    // Case asking more than one solar panel
    while (sp_counter > 1) {
        // Action turn solar panel
        dxl_deploy(Down);
        dxl_turn(team, 0); 
        dxl_deploy(Mid);

        // Update dynamic score
        shared.score += 5;
        sp_counter--; 

        // Go to next
        solar_panel_pc();
    }


    // Last solar panel
    if (sp_counter == 1) {
        //Action turn solar panel
        dxl_deploy(Down);
        dxl_turn(team, 0); 
        dxl_deploy(Up);

        // Update dynamic score
        shared.score += 5; 
        sp_counter--; 
    }

    // Reset solar panel wheel 
    dxl_init_sp();

    /* TO DO: If sp_counter = 0: return state action finish
              Return interrupt*/
    
    return; 
}

