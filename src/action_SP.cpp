#include "action_SP.h"

#include "dynamixels.h"
#include "cameraTag.h"

#include <pthread.h>

#define VERBOSE
#ifdef VERBOSE
#include <stdio.h>
#endif

typedef enum _state : int8_t {
    Path_Following, // Moving to destination
    Position_Control, // Moving to the next solar panel
    Solar_Panel, // Turning solar panel
    End, // End thread normally
    Abort // End thread with failure
} state_t;

static pthread_t KCID;
static volatile state_t state;
static volatile state_t stateKC;

volatile double camera_angle; 
volatile double dxl_angle;

static void leave() {
    state = Abort;
    pthread_join(KCID, NULL);
}

static void *kinematic_chain(void* argv) {

    uint8_t sp_counter = *(uint8_t*) argv;
    team_color_t team = shared.color; 

    // Reset wheel, just to be sure :)
    dxl_init_sp(); 

    while(1) {
        if (state == stateKC) {
            usleep(50000);
            continue; 
        }

        switch (state) {
            case Path_Following: // Moving to destination
                stateKC = Path_Following; 
                usleep(50000);
                break;

            case Position_Control: 
                stateKC = Position_Control; 
                dxl_init_sp(); // Reset wheel
                break; 

            case Solar_Panel: 
                stateKC = Solar_Panel; 
                dxl_deploy(Down);
                dxl_turn(team, dxl_angle); 

                if (sp_counter > 1) dxl_deploy(Mid);
                else dxl_deploy(Up);

                shared.score += 5; 
                break; 
            
            case End:
                return NULL;

            default: // Abort or End
                switch (stateKC) {
                    case Solar_Panel: 
                        dxl_deploy(Up); 
                        dxl_init_sp(); 
                        break; 
                
                    default:
                        break;
                }
                    
                return NULL;
            
        }

    }

    return NULL;
}

void ActionSP::do_action() {

    team_color_t team = shared.color; 

    state = Path_Following;
    stateKC = Path_Following;
    pthread_create(&KCID, NULL, kinematic_chain, &sp_counter);

    if (path_following_to_action(path)) return leave(); 

    while (sp_counter > 0) {
        state = Solar_Panel; 
        while (stateKC != Solar_Panel) usleep(50000);

        if (reserved) camera_angle = 0; 
        else camera_angle = tagSolar(); 

        ////////////////////////////////////////////////////

        state = Position_Control; 
        while (stateKC != Position_Control) usleep(50000); 

        double x, y, theta; 
        double yend; 
        shared.get_robot_pos(&x, &y, &theta); 

        if (team == TeamBlue) yend = y+22.5e-2; 
        else if (team == TeamYellow) yend = y-22.5e-2; 


        if (sp_counter > 1) {
            dxl_angle = camera_angle;
            if (action_position_control(x, yend, theta)) return leave();
        }
        
    }
    
    state = End;
    pthread_join(KCID, NULL);
}




/*void turn_solar_panel_reserved(uint8_t sp_counter) { 

    team_color_t team = shared.color; 

    // Case asking more than one solar panel
    while (sp_counter > 1) {
        #ifdef VERBOSE
        printf("SP Counter: %d\n", sp_counter); 
        #endif

        // Action turn solar panel
        dxl_deploy(Down);
        dxl_turn(team, 0);
        dxl_deploy(Mid);

        // Update dynamic score
        shared.score += 5;
        counter--; 

        // Go to next
        double x, y, theta, yend; 
        share.get_robot_pos(&x, &y, theta); 

        if (team == TeamYellow) {yend = y-22.5e-2}; // Reverse 
        else {yend = y+22.5e-2}; // Forward

        action_position_control(x, yend, theta); 
    }

    #ifdef VERBOSE
    printf("Check last SP Counter: %d\n", sp_counter); 
    #endif

    // Last solar panel
    if (sp_counter == 1) {
        #ifdef VERBOSE
        printf("Last solar panel in sequence \n"); 
        #endif

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

     return;         

}

void turn_solar_panel_public(uint8_t sp_counter) {

    team_color_t team = shared.color; 

    // Case asking more than one solar panel
    while (sp_counter > 1) {
        #ifdef VERBOSE
        printf("SP Counter: %d\n", sp_counter); 
        #endif

        // Action turn solar panel
        dxl_deploy(Down);
        double angle = tagSolar(); 
        dxl_turn(team, angle);
        dxl_deploy(Mid);

        // Update dynamic score
        shared.score += 5;
        counter--; 

        // Go to next
        double x, y, theta, yend; 
        share.get_robot_pos(&x, &y, theta); 

        if (team == TeamYellow) {yend = y-22.5e-2}; // Reverse 
        else {yend = y+22.5e-2}; // Forward

        action_position_control(x, yend, theta); 
    }

    #ifdef VERBOSE
    printf("Check last SP Counter: %d\n", sp_counter); 
    #endif

    // Last solar panel
    if (sp_counter == 1) {
        #ifdef VERBOSE
        printf("Last solar panel in sequence \n"); 
        #endif

        //Action turn solar panel
        dxl_deploy(Down);
        double angle = tagSolar(); 
        dxl_turn(team, angle);
        dxl_deploy(Mid);

        // Update dynamic score
        shared.score += 5; 
        sp_counter--; 
    }

    // Reset solar panel wheel 
    dxl_init_sp();

     return;   
}
*/