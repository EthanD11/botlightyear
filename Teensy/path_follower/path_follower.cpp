/*!
 * \file path_following_gr2.cc
 * \brief File description
 */
#include "math.h"
#include "path_following.h" // adapt it with your headers

NAMESPACE_INIT(ctrlGr2); // where X should be replaced by your group number

PathFollower* init_path_follower() {
    path_follower = (PathFollower *) malloc(sizeof(PathFollower));
    path_follower->Lspeed_ref = 0;
    path_follower->Rspeed_ref = 0;
    path_follower->kt = 25.0;
    path_follower->kn = 0.8; // 0 < kn <= 1
    path_follower->kz = 25.0;
    path_follower->delta = 30e-3; // delta is in meters
    path_follower->sigma = .01;
    path_follower->epsilon = 150-3; // epsilon is in meters
    path_follower->wn = 0.15; // Command filter discrete cutoff frequency
    path_follower->kv_en = 20; // vref = MIN(vref, kv_en/en)
    return path_follower;
}

void free_path_follower(PathFollower *path_follower) {
    free(path_follower->path);
    free(path_follower);
}

void init_path_following(PathFollower *path_follower, double *x, double *y, int n, double theta){
    path_follower->checkpoints_x = x;
    path_follower->checkpoints_y = y;
    path_follower->n = n;

    double *q_checkpoints = (double *) malloc(sizeof(double)*n);
    for (int i = 0; i < n; i++) q_checkpoints[i] = (double) i;
    store_vector(n, x, "../../resultsR/path_following_data_gr2/path_following__cx.csv");
    store_vector(n, y, "../../resultsR/path_following_data_gr2/path_following__cy.csv");
    path_follower->x_splines = compute_splines(q_checkpoints, path_follower->checkpoints_x, n);
    path_follower->y_splines = compute_splines(q_checkpoints, path_follower->checkpoints_y, n);


    path_follower->i_spline = 0;
    path_follower->qref = path_follower->x_splines->q[0];

    path_follower->xref = x[0];
    path_follower->yref = y[0];
    path_follower->s = 0;

    path_follower->xsi_n = 0;

    // Filtered states
    path_follower->kif = theta;
    path_follower->kifdot = 0;
    path_follower->curvature_f = 0.0;
    path_follower->curvature_fdot = 0.0;
    path_follower->vref_f = 0.0;
    path_follower->vref_fdot = 0.0;
    
    free(q_checkpoints);
}


/*
* Compute directly the entire path from checkpoints
* Was made for developing purposes, should not be used in final algorithm
*/
void compute_entire_path(PathFollower *path_follower, double ds) {
    int n_splines = path_follower->n;
    double *q_checkpoints = (double *) malloc(sizeof(double)*(n_splines+1));
    for (int i = 0; i <= n_splines; i++) q_checkpoints[i] = (double) i;
    SplineSet *x_splines = compute_splines(q_checkpoints, path_follower->checkpoints_x, n_splines);
    SplineSet *y_splines = compute_splines(q_checkpoints, path_follower->checkpoints_y, n_splines);

    int n_points = (int) (6.0 / ds);
    double q = q_checkpoints[0];
    double dq = 0;
    double dxdq, dydq;
    int i_spline = 0;
    int i_point = 0;

    // x[i] = path[2*i] and y[i] = path[2*i+1]
    double *path = (double *) malloc(sizeof(double)*(2*n_points));
    // FILE* file_q = fopen("../../resultsR/path_following_data_gr2/q_checkpoint.csv", "w");
    // FILE* file_dxdq = fopen("../../resultsR/path_following_data_gr2/dxdq.csv", "w");
    // FILE* file_dydq = fopen("../../resultsR/path_following_data_gr2/dydq.csv", "w");
  
    while (q < q_checkpoints[n_splines]) {
        if (i_point >= n_points) { 
            // If arrays are too small for number of points double array size
            double *new_mem = (double *) malloc(sizeof(double)*4*n_points);
            memcpy(new_mem, path, n_points*sizeof(double));
            free(path);
            path = new_mem;
            n_points = 2*n_points;
        }

        dq = q - q_checkpoints[i_spline];
        path[2*i_point]   = evaluate_spline(x_splines->a[i_spline], x_splines->b[i_spline], x_splines->c[i_spline], x_splines->d[i_spline], dq);
        path[2*i_point+1] = evaluate_spline(y_splines->a[i_spline], y_splines->b[i_spline], y_splines->c[i_spline], y_splines->d[i_spline], dq);

        dxdq = evaluate_spline_derivative(x_splines->b[i_spline], x_splines->c[i_spline], x_splines->d[i_spline], dq);
        dydq = evaluate_spline_derivative(y_splines->b[i_spline], y_splines->c[i_spline], y_splines->d[i_spline], dq);

        dq = ds / sqrt(dxdq*dxdq + dydq*dydq); // Estimation of the dq needed to travel of ds along the spline
        q += dq;

        i_point += 1;
        if (q >= q_checkpoints[i_spline+1]) i_spline += 1;

    }
    path_follower->path = path;
    path_follower->m = i_point;

    free_spline_set(x_splines);
    free_spline_set(y_splines);
    free(q_checkpoints);
}


/* 
* Computes the points which is at a distance `delta_s` of the current point
* along the path
* returns 1 if the next point is beyond the last checkpoint
* returns 0 otherwise
*/
int compute_next_point(PathFollower *pf, double delta_s, double dist_goal_reached) {
    double xnext, ynext;
    double delta_q, dq, dxdq, dydq, s, ds;

    SplineSet *x_splines = pf->x_splines;
    SplineSet *y_splines = pf->y_splines;
    
    double *q_checkpoints = x_splines->q;
    int i_spline = pf->i_spline;
    
    s = 0;
    
    while (s < delta_s) {
        // Compute dq corresponding to ds (1st order approx)
        delta_q = pf->qref - q_checkpoints[i_spline];
        dxdq = evaluate_spline_derivative(x_splines->b[i_spline], x_splines->c[i_spline], x_splines->d[i_spline], delta_q);
        dydq = evaluate_spline_derivative(y_splines->b[i_spline], y_splines->c[i_spline], y_splines->d[i_spline], delta_q);

        dq = ds / sqrt(dxdq*dxdq + dydq*dydq); // Estimation of the dq needed to travel of ds along the spline
        pf->qref = MAX(pf->qref+dq, 0); // Update the qref

        // update spline index if we go pass a checkpoint
        if (pf->qref >= q_checkpoints[i_spline+1]) i_spline += 1;
        pf->i_spline = i_spline;
        double xend = pf->x_splines->x[pf->n];
        double yend = pf->y_splines->x[pf->n];
        if (hypot(pf->xref - xend, pf->yref - yend) <= dist_goal_reached) {
            printf("Switch mode\n");
            return 1;
        }
        ds = MIN(MAX_DS, delta_s - s);
        s += ds;
    }
    return 0;
}

// Reference paper:
// See https://link.springer.com/article/10.1007/s42405-021-00395-7
int update_path_follower_ref_speed(
    PathFollower *pf, 
    double vref, 
    double xpos, double ypos, double theta,
    double dist_goal_reached, 
    double dt) 
{

    SplineSet *x_splines = pf->x_splines;
    SplineSet *y_splines = pf->y_splines;
    double *q_checkpoints = x_splines->q;
    double ex, ey, et, en, delta_q, dq, dxdq, dydq, d2xdq2, d2ydq2, dt, omega_ref, theta, v;
    double delta_s, kid, kir, curvature, kif, kifdot, kitilde, kidtilde, xsi_n, epsilon_n;
    double kt, kn, kz, delta, wn, z, sigma; // Controller parameters
    int i_spline;

    double vctrl = v;

    // use local variables for ease of read
    dt = cvs->dt;
    kt = pf->kt;  kn = pf->kn; kz = pf->kz; sigma = pf->sigma; // Control gains
    kif = pf->kif; kifdot = pf->kifdot; wn = pf->wn; xsi_n = xsi_n;
    delta = pf->delta;

    theta = cvs->rob_pos->theta;

    // Compute error in the fixed reference frame
    ex = cvs->rob_pos->x - pf->xref;
    ey = cvs->rob_pos->y - pf->yref;

    // Compute derivatives at reference point on the curve
    i_spline = pf->i_spline;
    delta_q = pf->qref - q_checkpoints[i_spline];
    dxdq = evaluate_spline_derivative(x_splines->b[i_spline], x_splines->c[i_spline], x_splines->d[i_spline], delta_q);
    dydq = evaluate_spline_derivative(y_splines->b[i_spline], y_splines->c[i_spline], y_splines->d[i_spline], delta_q);
    d2xdq2 = evaluate_spline_second_derivative(x_splines->c[i_spline], x_splines->d[i_spline], delta_q);
    d2ydq2 = evaluate_spline_second_derivative(y_splines->c[i_spline], y_splines->d[i_spline], delta_q);
    
    // https://en.wikipedia.org/wiki/Curvature#In_terms_of_a_general_parametrization
    // Compute curvature
    double tmp = sqrt(dxdq*dxdq+dydq*dydq);
    curvature = (dxdq*d2ydq2 - dydq*d2xdq2) / (tmp*tmp*tmp);


    // Compute angle of the reference frame with x-axis tangent to trajectory (Serret-Frenet frame)
    kir = atan2(dydq, dxdq);

    // Compute error in the Serret-Frenet frame
    et =  cos(kir) * ex + sin(kir) * ey;
    en = -sin(kir) * ex + cos(kir) * ey;
    kid = kir - asin(kn*en / (delta + fabs(en)));
    kidtilde = kid - kir;

    // Angular error
    z = theta - kif;

    // Extended cross-track error
    epsilon_n = en - xsi_n;

    // Course angle error
    kitilde = theta - kir;

    // Reference speed correction
    vref = MAX(vref + SIGMOID(-(et-5e-2))*(25e-2-vref) - fabs(pf->kv_en*en),//fabs(pf->curvature_f)*pf->kv_curv, 
        15e-2);// + (fabs(en)/pf->kv_en) + 1e-15));

    // Filters
    pf->curvature_f += pf->curvature_fdot;
    pf->curvature_fdot = wn*wn*(curvature - pf->curvature_f) - SQRT2 * wn * pf->curvature_fdot;
    pf->kif += kifdot;
    pf->kifdot = wn*wn*(kid - kif) - SQRT2 * wn * kifdot;
    pf->vref_f += pf->vref_fdot;
    pf->vref_fdot = wn*wn*(vref - pf->vref_f) - SQRT2 * wn * pf->vref_fdot;
    

    // Auxiliary signal (xsi_n = en in steady state)
    double sinckitilde = SINC(kitilde);
    double h = vctrl*sinckitilde*(kn/(delta+en));
    double dxsindt= -h*xsi_n + vctrl*sinckitilde*((kif-kir) - sin(kidtilde));
    pf->xsi_n += dxsindt*dt;

    // Compute arc-length evolution of serret-frenet frame along the path
    delta_s = (kt*(et) + vctrl*cos(kir - theta))*dt/(1 - pf->curvature_f*xsi_n);
    double z_term = -kz*z;
    double dkifdt_term = kifdot;
    double sinc_term = vctrl*sinckitilde*epsilon_n;
    double tanh_term = -sigma*tanh(z/pf->epsilon);
    omega_ref = -kz*z + kifdot - vctrl*sinckitilde*epsilon_n -sigma*tanh(z/pf->epsilon);

    pf->s += delta_s;

    // update reference speeds
    
    pf->Rspeed_ref = (pf->vref_f + WHEEL_L*omega_ref) / WHEEL_R;
    pf->Lspeed_ref = (pf->vref_f - WHEEL_L*omega_ref) / WHEEL_R;

    // Compute the point on the path which is at a distance delta_s from the current point
    double step = MIN(delta_s, MAX_DS); // Take step no bigger than MAX_DS

    dq = step / sqrt(dxdq*dxdq + dydq*dydq); // Estimation of the dq needed to travel of ds along the spline
    pf->qref = MAX(pf->qref+dq, 0); // Update the qref
    if (pf->qref >= q_checkpoints[i_spline+1]){
        i_spline += 1;
    }

    double xend = pf->x_splines->x[pf->n-1];
    double yend = pf->y_splines->x[pf->n-1];
    double dist = hypot(pf->xref - xend, pf->yref - yend);
    if (dist <= dist_goal_reached) {
        printf("Switch mode\n");
        return 1;
    }

    if (step < delta_s) { // if delta_s is bigger than MAX_DS, then use the function compute_next_point
        printf("compute next point\n");
        if (compute_next_point(pf, delta_s - step, dist_goal_reached) == 1) return 1;
    }
    
    delta_q = pf->qref - q_checkpoints[i_spline];
    pf->xref = evaluate_spline(x_splines->a[i_spline], x_splines->b[i_spline], x_splines->c[i_spline], x_splines->d[i_spline], delta_q);
    pf->yref = evaluate_spline(y_splines->a[i_spline], y_splines->b[i_spline], y_splines->c[i_spline], y_splines->d[i_spline], delta_q);
    pf->i_spline = i_spline;
    return 0;
}