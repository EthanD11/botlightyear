#include "action_SP.h"
#include "action_displacement.h"

#define VERBOSE
#ifdef VERBOSE
#include <stdio.h>
#endif

/* SOLAR_PANEL_PC: Position Control for Solar Panels 
   x, y, theta: robot position at the start of the displacement action
   xa, ya, ta : robot position actuated by thread TopLidar (odometry)
   Position Control from one solar panel to another, while checking current position from desired one
   While displacement, reset dxl 8 (wheel) to init position for left-right turn
*/
void solar_panel_pc() {

    Teensy *teensy = shared.teensy;
    Odometry *odo = shared.odo; 

    // Retrieve robot current position
    double x, y, theta; 
    shared.get_robot_pos(&x, &y, &theta); 
    #ifdef VERBOSE
    printf("Robot position from shared: %.3f, %.3f, %.3f \n", x, y, theta); 
    #endif

    // Set position control gains (see with Ethan?)
    double kp = 0.8;
    double ka = 2.5;
    double kb = -0.5;
    double kw = 4.0;
    teensy->set_position_controller_gains(kp, ka, kb, kw);

    // Define trajectory to next SP
    int ncheckpoints = 2;
    double xc[2] = {x, x};
    double yc[2] = {y, y-22.5e-2};
    double theta_start = theta;
    double theta_end = theta;
    
    #ifdef VERBOSE
    printf("Checkpoints relay: %.3f, %.3f\n", xc[0], yc[0]); 
    printf("Checkpoints target: %.3f, %.3f\n", xc[1], yc[1]); 
    #endif

    //double xpos, ypos, thetapos;
    //teensy.set_position(xc[0], yc[0], theta_start);
    
    // Orientation with position control
    teensy->pos_ctrl(xc[1], yc[1], theta_end);
    usleep(10000);

    // Reset solar panel wheel (dxl 8)
    dxl_init_sp(); 

   /* 
    do {
        odo.get_pos(&xpos, &ypos, &thetapos);
        teensy.set_position(xpos, ypos, thetapos);
        #ifdef VERBOSE
        printf("Odo position in while: %.3f,%.3f,%.3f\n",xpos, ypos, thetapos);
        #endif
        lguSleep(0.3);
    } while(abs(xpos-xc[1]) > 0.01); */

    //shared.set_robot_pos(xpos, ypos, thetapos); 

    // Waiting end to start function turn_solar_panel
    // Check Teensy mode
    while ((teensy->ask_mode()) != ModePositionControlOver) { 
        usleep(10000);
    } 

    //sleep(5); 
    
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

    /*// Path following to action : check interrupt
    if (path_following_to_action() != 0) {
        return;
    } */

    uint8_t counter = sp_counter; 

    // Case asking more than one solar panel
    while (counter > 1) {
        #ifdef VERBOSE
        printf("Counter: %d\n", counter); 
        #endif
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
        counter--; 

        // Go to next
        solar_panel_pc();
    }

    #ifdef VERBOSE
    printf("Check last sp counter: %d\n", counter); 
    #endif
    // Last solar panel
    if (counter == 1) {
        #ifdef VERBOSE
        printf("Last solar panel in sequence \n"); 
        #endif
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
        counter--; 
    }

    // Reset solar panel wheel 
    dxl_init_sp();

    /* TO DO: If sp_counter = 0: return state action finish
              Return interrupt*/

     return;         

}


void positionCtrlIterative() {

    Teensy *teensy = shared.teensy;
    Odometry *odo = shared.odo; 

    // Retrieve robot current position
    double x, y, theta; 
    shared.get_robot_pos(&x, &y, &theta); 
    teensy->set_position(x,y,theta); 
    sleep(5); 
    #ifdef VERBOSE
    printf("Robot position from shared: %.3f, %.3f, %.3f \n", x, y, theta); 
    #endif
    for (int i = 0; i<3; i++) {
        y-=22.5e-2; 
        teensy->pos_ctrl(x,y, theta);
        #ifdef VERBOSE
        printf("Position control number %d : y = %.3f \n", i, y); 
        #endif
        sleep(5); 
    }
        
}