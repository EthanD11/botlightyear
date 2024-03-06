#include "utils.h"
#include "localization.h"

#ifndef _REGULATOR_H_
#define _REGULATOR_H_

#define MOTOR_DUTY_RANGE 256  // Duty cycle range

#define ADZ_ENABLE
#ifdef ADZ_ENABLE
const int adz = 35;
#endif

typedef struct Regulator {
    double duty_cycle_l, duty_cycle_r;
    double isl, isr;
    double kp, ki;
    double imax;
} Regulator;

Regulator *init_regulator();
void free_regulator(Regulator *regulator);

void speed_ctrl(
    Regulator *regulator, 
    double speed_l,
    double speed_r,
    double speed_refr,
    double speed_refl);

#endif