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
    #ifdef VERBOSE
    printf("SP Aborting\n"); 
    #endif
    state = Abort;
    pthread_join(KCID, NULL);
}

static void *kinematic_chain(void* argv) {

    #ifdef VERBOSE
    printf("Entering SP Kinematic Chain Thread\n"); 
    #endif

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
                #ifdef VERBOSE
                printf("SP Thread: Path_Following\n"); 
                #endif

                stateKC = Path_Following; 
                usleep(50000);
                break;

            case Position_Control: 
                #ifdef VERBOSE
                printf("SP Thread: Position_Control (Dxl reset)\n"); 
                #endif

                stateKC = Position_Control; 
                dxl_reset_sp(); // Reset wheel
                break; 

            case Solar_Panel: 
                #ifdef VERBOSE
                printf("SP Thread: Solar_Panel\n SP Thread sp_counter: %d\n SP Thread dxl_angle: %d\n", sp_counter, dxl_angle); 
                #endif

                stateKC = Solar_Panel; 

                dxl_deploy(Down);
                dxl_turn(team, dxl_angle); 

                if (sp_counter > 1) dxl_deploy(Mid);
                else dxl_deploy(Up);

                shared.score += 5; 
                sp_counter--; 

                #ifdef VERBOSE
                printf("SP Thread sp_counter: %d\n", sp_counter); 
                #endif:

                break; 
            
            case End:
                #ifdef VERBOSE
                printf("SP Thread: End\n"); 
                #endif

                return NULL;

            default: // Abort or End
                #ifdef VERBOSE
                printf("SP Thread: Abord, End\n"); 
                #endif

                switch (stateKC) {
                    case Solar_Panel: 
                        dxl_deploy(Up); 
                        dxl_reset_sp(); 
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

    #ifdef VERBOSE
    printf("Entering SP do_action\n"); 
    #endif

    team_color_t team = shared.color; 

    state = Path_Following;
    stateKC = Path_Following;
    pthread_create(&KCID, NULL, kinematic_chain, &sp_counter);

    if (path_following_to_action(path)) return leave(); 

    #ifdef VERBOSE
    printf("SP do_action: Successfull Path Following\n"); 
    #endif

    while (sp_counter > 0) {
        #ifdef VERBOSE
        printf("SP do_action: Solar_Panel\n SP do_action sp_counter : %d\n", sp_counter); 
        #endif

        state = Solar_Panel; 
        while (stateKC != Solar_Panel) usleep(50000);

        if (reserved) {
            camera_angle = 0; 
        }
        else {
            camera_angle = tagSolar(); 
        }

        #ifdef VERBOSE
        printf("SP do_action: camera_angle : %d\n", camera_angle); 
        #endif

        ////////////////////////////////////////////////////

        state = Position_Control; 
        while (stateKC != Position_Control) usleep(50000); 

        double x, y, theta; 
        double yend; 
        shared.get_robot_pos(&x, &y, &theta); 

        if (team == TeamBlue) { 
            yend = y-(22.5e-2); 
        }
        else if (team == TeamYellow) {
            yend = y+(22.5e-2); 
        }

        #ifdef VERBOSE
        printf("SP do_action: robot position = (%.3f, %.3f, %.3f)\n SP do_action: yend = %.3f\n", x, y, theta, yend); 
        #endif


        if (sp_counter > 1) {
            dxl_angle = camera_angle;
            if (action_position_control(x, yend, theta)) return leave();

            #ifdef VERBOSE
            printf("SP do_action: Successfull Postion Control\n"); 
            #endif
        }
        
    }
    
    #ifdef VERBOSE
    printf("SP do_action: End\n"); 
    #endif
    state = End;
    pthread_join(KCID, NULL);
}

    



