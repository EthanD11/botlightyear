/*!
 * \file utils_gr2.h
 * \brief Define variables and function useful throughout the project
 * and that do not have any dependencies to other files 
 */
#ifndef _UTILS_H_
#define _UTILS_H_

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <time.h>
#include <stdint.h>

#define PI 3.141592653589793238462643383279502
#define SATURATE(a,lb,ub) ((a) > (ub) ? (ub) : ((a) < (lb) ? (lb) : (a)))
#define ABS(a) ((a) >= 0 ? (a) : (-(a)))
#define PERIODIC(a,lb,ub) ((a)>ub ? ((lb)+(a)-(ub)) : ((a)<(lb) ? (ub)+(a)-(lb) : (a)))
#define SIGMOID(a) (1/(1+exp(-(a))))
#define SMOOTH_WINDOW(a,T,steepness) (SIGMOID(steepness*(a+T))-SIGMOID(steepness*(a-T)))
#define SMOOTH_CUT(a,T,steepness) (SIGMOID(-steepness*(a+T))+SIGMOID(steepness*(a-T)))
#define MAX(a,b) ((a) >= (b) ? (a) : (b))
#define MIN(a,b) ((a) <= (b) ? (a) : (b))
#define SQRT2 1.4142135623730950488
#define SINC(x) (((x) == 0) ? 1 : (sin(x)/(x)));

#define TRUE 1
#define FALSE 0


// ------ Program parameters -------
#define REF_SPEED_LIMIT 0.7 // Saturation level of the reference angular velocity (rad_mot/s, 113*100*2pi/64, 0.9/30e-3*30)
#define REG_DELAY 20 // Delay between two updates (ms)
#define SPD_TOL 1e-2 // Max speed at which motors can be turned off (rad_mot/s)

#ifdef ODOMETERS_ENC
#define WHEEL_L 126e-3 // Half the distance between the two wheels (m)
#define TICKS_TO_M = 1.7257e-5; // Multiply to get meters from tick count. pi*45e-3/8192
#else
#define WHEEL_L 88.085e-3 // Half the distance between the two wheels (m)
#define TICKS_TO_M = 1.3806e-6; // Multiply to get meters from tick count. pi*72e-3/20/8192
#endif

// ------- Useful function -------

void store_vector(int n, double *x, char *filename);
void erase_file(char *filename);
void append_to_file(char *filename, double data);
inline int SAT(int x, int limit);
inline double SAT(double x, double limit);

#endif