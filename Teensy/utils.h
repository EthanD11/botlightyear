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

#define WHEEL_R 30e-3   
#define WHEEL_L 90e-3

void store_vector(int n, double *x, char *filename);
void erase_file(char *filename);
void append_to_file(char *filename, double data);

#endif