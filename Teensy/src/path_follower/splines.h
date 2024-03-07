/*! 
 * \file splines.h
 * \brief Structure and functions to handle splines
 */

#ifndef _SPLINES_H_
#define _SPLINES_H_

#include <stdlib.h>
#include "cmath"
#include <string.h>

/* Stores a set of splines as lists of coefficients `a`, `b`, `c` and `d`
 * Si(q) = ai + bi * (q - qi) + ci * (q - qi)^2 + di * (q - qi)^3
 * `int n`: Number of splines
 * `double *q`: length `n+1` - stores the checkpoints 
 * `double *a`: length `n` 
 * `double *b`: length `n` 
 * `double *c`: length `n` 
 * `double *d`: length `n` 
 */
typedef struct SplineSet {
    int n; // number of splines
    double *a, *b, *c, *d; // Spline coefficients
    double *q; // Spline i spans from q[i] to q[i+1]
    double *x; // x checkpoints
    double *y; // y checkpoints
} SplineSet;
void init_spline_set(SplineSet *splines, int n);
void free_spline_set(SplineSet *splines);

/* Computes splines coefficients from a list of points
Compute the spline S(q) with Si(qi) = xi and Si(qi+1) = xi+1
q: parameter values at points
x: spline value at point
npoints: number of points (hence npoints - 1 splines)
splines: a non allocated pointer to a spline structure
*/
SplineSet* compute_splines(double *q, double *x, int npoints);

/*
    Interpolates equidistant points within each splines 
*/
double* interpolate_splines(SplineSet *splines, int m);

inline double evaluate_spline(double a, double b, double c, double d, double dq) {
    return ((d*dq + c)*dq + b)*dq + a;
}
inline double evaluate_spline_derivative(double b, double c, double d, double dq) {
    return (3*d*dq + 2*c)*dq + b;
}
inline double evaluate_spline_second_derivative(double c, double d, double dq) {
    return 6*dq + 2*c;
}

// void test_splines();


#endif
