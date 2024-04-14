#include "actions.h"

int8_t path_following_to_action(graph_path_t *path) {

    printf("Going to node %d\n", path->target);
    for (size_t i = 0; i < path->nNodes; i++)
    {
        printf("(%.3f,%.3f) ", path->x[i], path->y[i]);
    }
    printf("\n");

    Teensy *teensy = shared.teensy; 

    // Set path following from path planning (decision)
    int ncheckpoints = (int) path->nNodes; 
    double *x = path->x; 
    double *y = path->y; 

    double theta_start = path->thetaStart; 
    double theta_end = path->thetaEnd; 

    double kp = 0.8;
    double ka = 2.5;
    double kb = -0.8;
    double kw = 4.0;
    teensy->set_position_controller_gains(kp, ka, kb, kw);

    double kt = 2.0;
    double kn = 0.3; // 0 < kn <= 1
    double kz = 25.0;
    double delta = 20e-3; // delta is in meters
    double sigma = 10;
    double epsilon = M_PI/8; // epsilon is in radians
    double wn = 0.2; // Command filter discrete cutoff frequency
    double kv_en = 0.;
    teensy->set_path_following_gains(kt, kn, kz, sigma, epsilon, kv_en, delta, wn);

    teensy->path_following(x, y, ncheckpoints, theta_start, theta_end, vref, dist_goal_reached);

    // Check Teensy mode
    //usleep(100000);
    while ((teensy->ask_mode()) != ModePositionControlOver) {
        usleep(1000);
    } 
    return 0; 
}

int8_t action_position_control(double x_end, double y_end, double theta_end) {

    Teensy *teensy = shared.teensy;
    Odometry *odo = shared.odo; 

    // Retrieve robot current position
    double x, y, theta; 
    shared.get_robot_pos(&x, &y, &theta); 
    #ifdef VERBOSE
    printf("Robot position from shared: %.3f, %.3f, %.3f \n", x, y, theta); 
    #endif

    // Retrieve adversary current position
    double x_adv, y_adv; 
    shared.get_adv_pos(&x_adv , &y_adv); 
    #ifdef VERBOSE
    printf("Adversary position from shared: %.3f, %.3f, %.3f \n", x_adv, y_adv, theta_adv); 
    #endif

    // Check adversary
    


    // Set position control gains (see with Ethan?)
    double kp = 0.8;
    double ka = 2.5;
    double kb = -0.5;
    double kw = 4.0;
    teensy->set_position_controller_gains(kp, ka, kb, kw);

    // Define trajectory
    int ncheckpoints = 2;
    double xc[2] = {x, x_end};
    double yc[2] = {y, y_end};
    double theta_start = theta;
    double theta_end = theta_end;
    
    #ifdef VERBOSE
    printf("Checkpoints relay: %.3f, %.3f\n", xc[0], yc[0]); 
    printf("Checkpoints target: %.3f, %.3f\n", xc[1], yc[1]); 
    #endif
    
    // Orientation with position control
    teensy->pos_ctrl(xc[1], yc[1], theta_end);
    usleep(1);

    // Waiting end to start function turn_solar_panel
    // Check Teensy mode
    while ((teensy->ask_mode()) != ModePositionControlOver) { 
        usleep(10000);
    } 
    return 0; 

}