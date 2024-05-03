#include "../../utils.h"
#include "../localization/localization.h"
#include "../inout_interface/output_interface.h"
#include "../localization/localization.h"

#ifndef _REGULATOR_H_
#define _REGULATOR_H_

#define MOTOR_DUTY_RANGE 256  // Duty cycle range

#define ADZ_ENABLE
#ifdef ADZ_ENABLE
#define ADZ_L 60
#define ADZ_R 45
#endif

typedef struct Regulator {
    int duty_cycle_refl, duty_cycle_refr;
    double isl, isr;
    double kp_r, ki_r, kp_l, ki_l;
    // double el_filtered, er_filtered;
    double wc;
    double imax;
} Regulator;

Regulator *init_regulator();
void free_regulator(Regulator *regulator);


/* Updates the duty cycle references of the speed controller
 * in order to reach the given reference speed
 */
void control_speed(
    Regulator *reg,
    RobotPosition *rob_pos,
    double speed_refl,
    double speed_refr);

inline int get_duty_cycle_refl(Regulator *speed_regulator) {
    return speed_regulator->duty_cycle_refl;
}
inline int get_duty_cycle_refr(Regulator *speed_regulator) {
    return speed_regulator->duty_cycle_refr;
}

void reset_regulator(Regulator *speed_regulator);

#endif