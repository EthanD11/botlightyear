/*! 
 * \file path_following_gr2.h
 * \brief main header of the controller
 * Usage example:
 * When in the StatePathInit:
 * ```
 * double x[3] = {0.0, 0.5, 1.O}
 * double y[3] = {0.0, 0.0, 0.0} // Straight line
 * double theta_start = robot_position->theta;
 * init_path_following(path_follower, x, y, 3, theta_start);
 * next_state = StatePathFollowing
 * ```
 * When in StatePathFollowing:
 * ```
 * int retval; // To know wether the end of the path is considered as reached
 * double vref = 40e-3; // Set constant vref at 40cm/s
 * double dist_goal_reached = 5e-2; // The algorithm returns 1 if the robot is at less than 5cm from the goal
 * retval = update_path_follower_ref_speed(path_follower, robot_position, vref, dist_goal_reached);
 * control_speed(speed_regulator, outputs, robot_position,
 *   path_follower->speed_refl, // give the reference speed from
 *   path_follower->speed_refr);
 * if (retval == 1) {
 *   position_controller->xref = path_follower->checkpoints_x[path_follower->n-1];
 *   position_controller->yref = path_follower->checkpoints_y[path_follower->n-1];
 *   position_controller->theta_ref = M_PI;
 *   mode = ModePositionControl;
 * }
 * ```
 */

#ifndef _PATH_FOLLOWING_GR2_H_
#define _PATH_FOLLOWING_GR2_H_

#include "../../utils.h"
#include "splines.h"
#include "../localization/localization.h"
#include <stdlib.h>

#define MAX_DS 5e-3

/*
    `double *checkpoints_x`: X coordinates of checkpoints
    `double *checkpoints_y`: Y coordinates of checkpoints
    `int n`: number of checkpoints

    `double *path`: Points along the path with `x[i] = path[2*i]` and `y[i] = path[2*i+1]`
    `int m`: number of points in the path
    
    `double Rspeed_ref`: Right reference speed
    `double Lspeed_ref`: Left reference speed
*/
typedef struct PathFollower {
    double *checkpoints_x;
    double *checkpoints_y;
    int n; // Number of checkpoints
    double last_x, last_y, last_q, last_theta;

    SplineSet *x_splines;
    SplineSet *y_splines;

    double *path; // Points along the path x[i] = path[2*i] and y[i] = path[2*(i+1)]
    int m; // Length of path

    // Control gains and control parameters
    // Reference: See https://link.springer.com/article/10.1007/s42405-021-00395-7
    double kt, kn, kz, sigma, epsilon, kv_en; // gains
    double delta; // Angular "smoother"
    double wn; // Command filter cutoff frequency
    double kif, kifdot; // Filtered command and its derivative
    double xsi_n; // Auxiliary signal that asymptotically reaches the cross-track error en in steady state
    double curvature_f, curvature_fdot;
    double vref_f;
    double vref_fdot;
    // State of the path_follower along trajectory
    int i_spline;
    double qref;
    double xref, yref;
    double s;

    // Angular position at last checkpoint (needed to switch to position control)
    // double theta_ref_last; // not implemented
    // outputs
    double speed_refr, speed_refl;
} PathFollower;


// Initializes path follower in CtrlStruct
PathFollower *init_path_follower();
void free_path_follower(PathFollower *path_follower);

/* 
 * Initializes a new path and reset all the internal variables
 * `double * x`: x coordinates of the checkpoints
 * `double * y`: y coordinates of the checkpoints
 * `int ncheckpoints`: the number of checkpoints
 * `double * theta_start`: the current orientation of the robot (just before the robot starts to follow the trajectory) 
 */
void init_path_following(PathFollower *path_follower, double *x, double *y, int ncheckpoints, double theta_start, double theta_stop);
void close_path_following(PathFollower *pf);

// Update the reference speed within the path follower
// Returns 1 if the end of the path is reached
// Returns 0 otherwise
int update_path_follower_ref_speed(
    PathFollower *path_follower,
    RobotPosition *robot_position, 
    double vref, 
    double dist_goal_reached);

inline double get_speed_refl(PathFollower *path_follower) {
    return path_follower->speed_refl;
}

inline double get_speed_refr(PathFollower *path_follower) {
    return path_follower->speed_refr;
}

/* Computes the path by interpolating cubic splines through checkpoints
 * the returned points are approximately equidistant of `dist` [m].
 * (the algorithm uses a first order approximation of the spline at each point
 * to determine the distance)
 * `PathFollower *path_follower`: The PathFollower (with checkpoints initialized)
 * `double dist`: the distance, in meters, between each points.
 */
void compute_entire_path(PathFollower *path_follower, double dist);

#endif
