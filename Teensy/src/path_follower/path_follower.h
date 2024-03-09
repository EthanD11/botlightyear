/*! 
 * \file path_following_gr2.h
 * \brief main header of the controller
 */

#ifndef _PATH_FOLLOWING_GR2_H_
#define _PATH_FOLLOWING_GR2_H_

#include "../../utils.h"
#include "splines.h"
#include "../localization/localization.h"
#include <stdlib.h>


#define MAX_DS 1e-3

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
    double last_x, last_y, last_q;

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

// Set the checkpoints of a PathFollower
void init_path_following(PathFollower *path_follower, double *x, double *y, int ncheckpoints, double theta_start);

/* Computes the path by interpolating cubic splines through checkpoints
 * the returned points are approximately equidistant of `dist` [m].
 * (the algorithm uses a first order approximation of the spline at each point
 * to determine the distance)
 * `PathFollower *path_follower`: The PathFollower (with checkpoints initialized)
 * `double dist`: the distance, in meters, between each points.
 */
void compute_entire_path(PathFollower *path_follower, double dist);

// Update the reference speed within the path follower
// Returns 1 if the end of the path is reached
// Returns 0 otherwise
int update_path_follower_ref_speed(
    PathFollower *path_follower,
    RobotPosition *robot_position, 
    double vref, 
    double dist_goal_reached);

#endif
