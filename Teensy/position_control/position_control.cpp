#include "position_control.h"

PositionController *init_position_controller(){
    PositionController* pc = (PositionController *) malloc(sizeof(PositionController));
    
    pc->speed_refl = 0.0;
    pc->speed_refr = 0.0;

    pc->kp =  0.1; // Proportional coefficient for distance error
    pc->ka =  0.5; // Proportional coefficient for direction error
    pc->kb = -0.1; // Proportional coefficient for orientation error
    pc->position_tol = 1e-1;      // Acceptable static error on position (m)
    pc->drift_tol    = 2e-1;      // Acceptable drift from reference position when reorienting (m)
    pc->angular_tol  = 2*8.73e-2; // Acceptable static error on orientation (rad, eq to 5 degrees)

    return pc;
}

inline void set_position_reference(
    PositionController *pc, uint32_t dataBuf[3]) 
{
    pc->xref      = ((double)(dataBuf[4]))*3/255;
    pc->yref      = ((double)(dataBuf[5]))*2/255;
    pc->theta_ref = ((double)(dataBuf[6]))*2*M_PI/255 - M_PI;
}

inline void set_position_reference(PositionController *pc,
    double xref, double yref, double theta_ref) 
{
    pc->xref      = xref;
    pc->yref      = yref;
    pc->theta_ref = theta_ref;
}

void control_position(
    PositionController *pc,
    RobotPosition *rp) 
{
    // Inspired by, but not identical to :
    // https://moodle.uclouvain.be/pluginfile.php/41211/mod_resource/content/1/Mobile_robots_control_2015.pdf?forcedownload=0
    // Slides 22-27

    #ifdef VERBOSE
    printf("Current coordinates : %.4f, %.4f, %.4f\n", x, y, theta);
    printf("Reference coordinates : %.4f, %.4f, %.4f\n", xr, yr, theta_r);
    #endif

    double xpos, ypos, theta;
    double xref, yref, theta_ref;
    double ex, ey; // Errors in cartesian coordinates
    double p, phi, alpha, beta; // Errors on position and orientation in "polar" coordinates
    double v_ref, rot_ref; // Reference velocity and rotation
    double kp, ka, kb, position_tol, drift_tol, angular_tol;
    
    xpos = rp->x; ypos = rp->y; theta = rp->theta;
    xref = pc->xref; yref = pc->yref; theta_ref = pc->theta_ref;
    kp = pc->kp; ka = pc->ka; kb = pc->kb;
    position_tol = pc->position_tol; drift_tol = pc->drift_tol; angular_tol = pc->angular_tol;

    // Comute the errors in standard coordinates
    ex = xref - xpos;
    ey = yref - ypos;

    // Compute the errors in "polar" coordinates
    p = hypot(ex, ey);
    if (p < position_tol) pc->flag_position_reached = 1;
    else if (p > drift_tol) pc->flag_position_reached = 0;

    if (pc->flag_position_reached) {
        // If the distance to the goal is acceptably small, assume goal is reached (p = alpha = 0)
        // Only the error on orientation remains
        alpha = 0;
        p = 0;
        beta = theta - theta_ref;
    } else {
        // Compute shortest path around the circle
        // Derived by @Kakoo :)
        phi = atan2(ey, ex);
        alpha = phi - theta;
        beta = theta_ref - phi;  // beta = theta_r - theta - alpha
        if (std::abs(alpha) > M_PI) alpha -= ((alpha > 0) ? 1 : -1) * M_PI * 2;
        if (std::abs(alpha) > M_PI_2) {
        p = -p;
        alpha += (alpha > 0) ? -M_PI : M_PI;
        beta  += (beta  > 0) ? -M_PI : M_PI;
        }
    }

    if (std::abs(beta) > PI) beta -= ((beta > 0) ? 1 : -1) * M_PI * 2;
    if (std::abs(beta) < angular_tol) beta = 0;

    #ifdef VERBOSE
    printf("Errors in polar coordinates : %.3f, %.3f, %.3f\n", p, alpha, beta);
    #endif

    // Compute reference velocity and rotation
    // Temp allows for a smoother transition, will be renamed if kept
    //temp = 0.557 * ((alpha + 1.22)/(0.14 + std::abs(alpha + (double) 1.22)) + (1.22 - alpha)/(0.14 + std::abs(alpha - (double) 1.22)));
    v_ref = kp * p;
    rot_ref = ka * alpha + kb * beta;

    // Translate into left and right wheel reference speed
    pc->speed_refl = v_ref - WHEEL_L * rot_ref;
    pc->speed_refr = v_ref + WHEEL_L * rot_ref;

}