#include "position_control.h"
#define VERBOSE
PositionController *init_position_controller() {
    PositionController* pc = (PositionController *) malloc(sizeof(PositionController));
    
    pc->speed_refl = 0.0;
    pc->speed_refr = 0.0;

    pc->kp =  0.5; // Proportional coefficient for distance error
    pc->ka =  3.0; // Proportional coefficient for direction error
    pc->kb = -0.5; // Proportional coefficient for orientation error
    pc->kw =  3.0; // Propoortional coefficient for orientation error when position is reached
    pc->position_tol = POSITION_TOL_IN;      // Acceptable static error on position (m)
    pc->drift_tol    = 2e-1;      // Acceptable drift from reference position when reorienting (m)
    pc->angular_tol  = 1*M_PI/180; // Acceptable static error on orientation (rad, eq to 5 degrees)

    return pc;
}

void set_position_controller_gains(PositionController *pc,
    double kp, double ka, double kb, double kw) 
{
    pc->kp =  kp; // Proportional coefficient for distance error
    pc->ka =  ka; // Proportional coefficient for direction error
    pc->kb =  kb; // Proportional coefficient for orientation error
    pc->kw =  kw; // Propoortional coefficient for orientation error when position is reached
}

void control_position(
    PositionController *pc,
    RobotPosition *rp) 
{
    // Inspired by, but not identical to :
    // https://moodle.uclouvain.be/pluginfile.php/41211/mod_resource/content/1/Mobile_robots_control_2015.pdf?forcedownload=0
    // Slides 22-27

    double x_ref, y_ref, theta_ref;
    double x, y, theta;
    double dx, dy, dxR, dyR;
    double p, a, b, kp, ka, kb, kw;
    double vref, omega_ref;

    x_ref = pc->xref;
    y_ref = pc->yref;
    theta_ref = pc->theta_ref;

    x = rp->x;
    y = rp->y;
    theta = rp->theta;

    kp = pc->kp;
    ka = pc->ka;
    kb = pc->kb;
    kw = pc->kw;

    dx = x_ref - x;
    dy = y_ref - y;
    p = hypot(dx, dy);
    int flag_goal_reached = (abs(p) < pc->position_tol);
    //int flag_too_far_away = (p > 1.0); // Stop if further than 50cm away from the target
    #ifdef VERBOSE
    // printf("xref = %f\n", x_ref);
    // printf("yref = %f\n", y_ref);
    // printf("theta_ref = %f\n", theta_ref);
    // printf("xpos = %f\n", x);
    // printf("ypos = %f\n", y);
    // printf("theta = %f\n", theta);
    // printf("p = %f\n");
    // printf("dx = %f\n", dx);
    // printf("dy = %f\n", dy);
    #endif
    
    switch (flag_goal_reached)
    {
        case FALSE:
        {   
            dxR =  cos(theta_ref)*dx + sin(theta_ref)*dy;
            dyR = -sin(theta_ref)*dx + cos(theta_ref)*dy;
            
            a = PIPERIODIC(-(theta-theta_ref) + atan2(dyR, dxR));
            b = PIPERIODIC(theta_ref - theta - a);
            // printf("a = %f\n", a);
            if (fabs(a) > M_PI_2) {
                // printf("a bigger than PI/2\n");
                p = -p;
                a += (a > 0) ? -PI : PI;
                b += (b > 0) ? -PI : PI;
            }
            #ifdef VERBOSE
            printf("p = %f\n", p);
            printf("a = %f\n", a);
            printf("b = %f\n", a);
            printf("Goal not reached\n");
            #endif
            vref = kp*p; //*SMOOTH_WINDOW(a, 0.8*M_PI/2, 5);
            omega_ref = ka*a + kb*b;
            pc->position_tol = POSITION_TOL_IN;
            break;
        }

        case TRUE:
        {   
            vref = 0.0;
            omega_ref = kw*PIPERIODIC(theta_ref - theta);
            #ifdef VERBOSE
            printf("Goal reached\n");
            #endif
            pc->position_tol = POSITION_TOL_OUT;
            break;
        }

        default:
        {
            vref = 0.0;
            omega_ref = 0.0;
            break;
        }
    }

    #ifdef VERBOSE
        printf("omega_ref = %f\n", omega_ref);
        printf("vref = %f\n", vref);
    #endif

    pc->speed_refr = (vref + WHEEL_L*omega_ref);
    pc->speed_refl = (vref - WHEEL_L*omega_ref);


}