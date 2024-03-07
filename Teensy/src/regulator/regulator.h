#include "../../utils.h"
#include "../localization/localization.h"

#ifndef _REGULATOR_H_
#define _REGULATOR_H_

#define MOTOR_DUTY_RANGE 256  // Duty cycle range

#define ADZ_ENABLE
#ifdef ADZ_ENABLE
const int adz = 35;
#endif

typedef struct Regulator {
    int duty_cycle_refl, duty_cycle_refr;
    double isl, isr;
    double kp, ki;
    double imax;
} Regulator;

Regulator *init_regulator();
void free_regulator(Regulator *regulator);

void control_speed(
    Regulator *regulator, 
    double speed_l,
    double speed_r,
    double speed_refr,
    double speed_refl);

#endif