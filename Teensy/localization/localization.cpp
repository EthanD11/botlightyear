#include "localization.h"
#include "utils.h"
// #include <Encoder.h>

void init_robot_position(
    RobotPosition *robot_position,
    double x0, double y0, double theta_0) 
{
    robot_position = (RobotPosition *) malloc(sizeof(RobotPosition));
    robot_position->x = x0;
    robot_position->y = y0;
    robot_position->theta = theta_0;
    robot_position->omega = 0;
    robot_position->vfwd = 0;
}

void update_localization(RobotPosition *robot_position) 
{
    Encoder enc_r = robot_position->enc_r;
    Encoder enc_l = robot_position->enc_l;
    int tick_left, tick_right;

    // Get time
    robot_position->current_time = millis();
    robot_position->dt = robot_position->current_time - robot_position->control_time;

    // Get time
    int current_time = millis();

    // Each 20ms TODO : Restimate REG_DELAY
    if(robot_position->dt > REG_DELAY){

        // Updating values according to encoders
        tick_left = enc_l.read(); tick_right = enc_r.read();

        // Temporarily forget delta t to avoid remultiplying when computing x, y and theta
        speed_left  = (tick_left  - old_tick_left )*TICKS_TO_M;
        speed_right = (tick_right - old_tick_right)*TICKS_TO_M;

        old_tick_left = tick_left; old_tick_right = tick_right;

        #ifdef VERBOSE
        printf("Ticks : %d, %d\n", old_tick_left, old_tick_right);
        #endif

        switch (mode) {
        case ModeIdle:
            control_time = current_time;
            duty_cycle_update(0,0);
            return;

        case ModePositionControl:

            // Update position estimate from encoder data
            fwd = (speed_left+speed_right)/2;
            rot = (speed_right-speed_left)/(2*WHEEL_L); // Divided by two, divided by half the distance between the two wheels = 176.17mm

            x += fwd*cos(t+rot/2);
            y += fwd*sin(t+rot/2);
            t += rot;
            t3_position_ctrl(x,y,t,xr,yr,tr, &speed_refl, &speed_refr);
            break;

        case ModeSpeedControl:
            break;

        default: // ModeIdle
            control_time = current_time;
            duty_cycle_update(0,0);
            return;
        }

        speed_left  /= ((current_time - control_time)*1e-3);
        speed_right /= ((current_time - control_time)*1e-3);
        if ((std::abs(speed_refl) < SPD_TOL) && (std::abs(speed_refr) < SPD_TOL) && (std::abs(speed_left) < SPD_TOL) && (std::abs(speed_right) < SPD_TOL)) mode = ModeIdle;
        #ifdef VERBOSE
        printf("Speed : %.4f\t%.4f\n", speed_left, speed_right);
        printf("Speed reference : %.4f\t%.4f\n", speed_refl, speed_refr);
        #endif

        t1_speed_ctrl(speed_left, speed_right, speed_refl, speed_refr);

        // Update last control time
        control_time = current_time;

    }

}