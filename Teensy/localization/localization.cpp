#include "localization.h"
#include "utils.h"
// #include <Encoder.h>

RobotPosition* init_robot_position(double x0, double y0, double theta_0) 
{
    RobotPosition* robot_position = (RobotPosition *) malloc(sizeof(RobotPosition));
    robot_position->x = x0;
    robot_position->y = y0;
    robot_position->theta = theta_0;
    robot_position->omega = 0;
    robot_position->vfwd = 0;
    #ifdef ODOMETERS_ENC
    robot_position->enc_l = new Encoder(26,25);
    robot_position->enc_r = new Encoder(31,30);
    #else
    robot_position->enc_l = new Encoder(25, 26);
    robot_position->enc_r = new Encoder(30, 31);
    #endif

    return robot_position;
}

void free_robot_position(RobotPosition *robot_position) {
    delete robot_position->enc_l;
    delete robot_position->enc_r;
    free(robot_position);
}

inline void reset_encoders(RobotPosition *robot_position) {
    robot_position->enc_l.write(0); 
    robot_position->enc_r.write(0); 
    robot_position->old_tick_left = 0; 
    robot_position->old_tick_right = 0;
}

inline void set_position(RobotPosition *robot_position, uint32_t dataBuf[3]) {
    robot_position->x      = ((double)(dataBuf[1]))*3/255;
    robot_position->y      = ((double)(dataBuf[2]))*2/255;
    robot_position->theta  = ((double)(dataBuf[3]))*2*M_PI/255 - M_PI;
}

void update_localization(RobotPosition *robot_position) 
{   
    double theta, dt;
    int tick_left, tick_right, old_tick_left, old_tick_right;
    double delta_left, delta_right, delta_fwd, delta_rot;
    Encoder enc_r, enc_l;

    enc_r = robot_position->enc_r;
    enc_l = robot_position->enc_l;
    old_tick_left = robot_position->old_tick_left;
    old_tick_right = robot_position->old_tick_right;
    theta = robot_position->theta;
    dt = robot_position->dt;
    
    // Each 20ms TODO : Restimate REG_DELAY
    if(robot_position->dt > REG_DELAY){
        // Updating values according to encoders
        tick_left  = enc_l.read(); 
        tick_right = enc_r.read();
        delta_left  = (tick_left  - old_tick_left )*TICKS_TO_M;
        delta_right = (tick_right - old_tick_right)*TICKS_TO_M;

        old_tick_left = tick_left; old_tick_right = tick_right;

        #ifdef VERBOSE
        printf("Ticks : %d, %d\n", old_tick_left, old_tick_right);
        #endif

        // Update position estimate from encoder data
        delta_fwd = (delta_left+delta_right)/2;
        delta_rot = (delta_right-delta_left)/(2*WHEEL_L); // Divided by two, divided by half the distance between the two wheels = 176.17mm

        robot_position->x           += delta_fwd*cos(theta+delta_rot/2);
        robot_position->y           += delta_fwd*sin(theta+delta_rot/2);
        robot_position->theta       += delta_rot;
        robot_position->vfwd        += delta_fwd / dt;
        robot_position->vrot        += delta_rot / dt;
        robot_position->speed_left  += delta_left / dt;
        robot_position->speed_right += delta_right / dt;

        #ifdef VERBOSE
        printf("Speed : %.4f\t%.4f\n", speed_left, speed_right);
        printf("Speed reference : %.4f\t%.4f\n", speed_refl, speed_refr);
        #endif
    }
}
