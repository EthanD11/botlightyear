#include "action_SP.h"

#include "dynamixels.h"
#include "cameraTag.h"

#include <pthread.h>
#include <cmath>

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

static volatile double camera_angle; 
static volatile double dxl_angle;
static volatile uint8_t sp_counter_glob;

static void leave() {
    #ifdef VERBOSE
    printf("SP Aborting\n"); 
    #endif
    state = Abort;
    pthread_join(KCID, NULL);
    shared.steppers->flaps_move(FlapsOpen);
}

static void *kinematic_chain(void* argv) {

    #ifdef VERBOSE
    printf("Entering SP Kinematic Chain Thread\n"); 
    #endif

    
    team_color_t team = shared.color; 

    // Reset wheel, just to be sure :)
    dxl_reset_sp(); 

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
                printf("SP Thread: Solar_Panel\n SP Thread sp_counter: %d\n SP Thread dxl_angle: %d\n", sp_counter_glob, dxl_angle); 
                #endif

                stateKC = Solar_Panel; 

                dxl_deploy(Down);
                dxl_turn(team, dxl_angle); 

                if (sp_counter_glob > 1) dxl_deploy(Mid);
                else dxl_deploy(Up);

                shared.score += 5; 
                sp_counter_glob--;

                #ifdef VERBOSE
                printf("SP Thread sp_counter: %d\n", sp_counter_glob); 
                #endif

                break; 
            
            case End:
                #ifdef VERBOSE
                printf("SP Thread: End\n"); 
                #endif

                return NULL;

            default: // Abort or End
                #ifdef VERBOSE
                printf("SP Thread: Abort, End\n"); 
                #endif

                switch (stateKC) {
                    case Solar_Panel: 
                        dxl_deploy(Up); 
                        dxl_reset_sp(); 
                        break; 
                    
                    case Position_Control: 
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

    Teensy *teensy = shared.teensy; 

    #ifdef VERBOSE
    printf("Entering SP do_action\n"); 
    #endif

    team_color_t team = shared.color; 

    state = Path_Following;
    stateKC = Path_Following;
    sp_counter_glob = sp_counter;
    if (pthread_create(&KCID, NULL, kinematic_chain, NULL) != 0) return;

    // Set path following from path planning (decision)
    int ncheckpoints = (int)path->nNodes;
    double *x = path->x;
    double *y = path->y;
    for (int i=0; i<ncheckpoints; i++) {
        printf("Node %d : x :%.3f and y: %.3f \n", i, x[i], y[i]); }
    double theta_start = path->thetaStart;
    double theta_end = path->thetaEnd;

    shared.steppers->flaps_move(FlapsPlant); 

    if (path_following_to_action(path)) return leave();
     
    // Step
    double step;
    if (sp_direction == Forward) {
        step = -22.5e-2; 
    } else if (sp_direction == Backward) {
        step = 22.5e-2; 
    }

    // Position of first shared solar panel
    double x16 = 1.790;
    double y16 = 1.725; 

    // Set position control gains (see with Ethan?)
    double kp = 0.8;
    double ka = 2.5;
    double kb = -1.75;
    double kw = 4.0;
    teensy->set_position_controller_gains(kp, ka, kb, kw);

    if (action_position_control(x16, y16, -M_PI_2)) return leave();  

    #ifdef VERBOSE
    printf("SP do_action: Successfull Path Following\n"); 
    #endif

    while (sp_counter_glob > 0) {
        #ifdef VERBOSE
        printf("SP do_action: Solar_Panel\n SP do_action sp_counter : %d\n", sp_counter_glob); 
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

        y16 += step; 

        #ifdef VERBOSE
        printf("SP do_action: robot position = (%.3f, %.3f, %.3f)\n SP do_action: yend = %.3f\n", x, y, theta, yend); 
        #endif


        if (sp_counter_glob >= 1) {
            dxl_angle = camera_angle;
            if (action_position_control(x16, y16, -M_PI_2)) return leave();

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
    shared.steppers->flaps_move(FlapsOpen);
}

    



