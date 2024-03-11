#include "position_control.h"

PositionController *init_position_controller(){
    PositionController* pc = (PositionController *) malloc(sizeof(PositionController));
    
    pc->speed_refl = 0.0;
    pc->speed_refr = 0.0;

    pc->kp =  1.0; // Proportional coefficient for distance error
    pc->ka =  4.0; // Proportional coefficient for direction error
    pc->kb = -1.0; // Proportional coefficient for orientation error
    pc->kw = 10.0;
    pc->position_tol = 1e-2;      // Acceptable static error on position (m)
    pc->drift_tol    = 2e-1;      // Acceptable drift from reference position when reorienting (m)
    pc->angular_tol  = 1*M_PI/180; // Acceptable static error on orientation (rad, eq to 5 degrees)

    return pc;
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
    int flag_goal_reached = (p < pc->position_tol); // Stop at 1cm
    int flag_too_far_away = (p > 0.5); // Stop if further than 50cm away from the target
    
    switch (flag_goal_reached)
    {
        case FALSE:
        {
            dxR =  cos(theta_ref)*dx + sin(theta_ref)*dy;
            dyR = -sin(theta_ref)*dx + cos(theta_ref)*dy;
            
            a = PIPERIODIC(-(theta-theta_ref) + atan2(dyR, dxR));
            b = PIPERIODIC(theta_ref - theta - a);

            if (fabs(a) > M_PI_2) {
                p = -p;
                a += (a > 0) ? -PI : PI;
                b += (b  > 0) ? -PI : PI;
            }
            vref = kp*p*SMOOTH_WINDOW(a, 0.8*M_PI/2, 5);
            omega_ref = ka*a + kb*b;
            break;
        }

        case TRUE:
        {   
            vref = 0.0;
            omega_ref = kw*PIPERIODIC(theta_ref - theta);
            break;
        }

        default:
        {
            vref = 0.0;
            omega_ref = 0.0;
            break;
        }
    }

    if (!flag_too_far_away){
        pc->speed_refr = (vref + WHEEL_L*omega_ref);
        pc->speed_refl = (vref - WHEEL_L*omega_ref);
    } else {
        pc->speed_refr = 0;
        pc->speed_refl = 0;
    }


}