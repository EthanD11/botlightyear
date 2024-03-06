#include "regulator.h"


Regulator *init_regulator() {
    Regulator *reg = (Regulator *) malloc(sizeof(Regulator));
    
    reg->kp = 7.795856e-01; // Former t1_kp
    reg->ki = 8.398411e-01 * REG_DELAY * 1e-3; // Former t1_ki
    
    reg->imax = 30; // Anti windup

    reg->isl = 0.0; // left integral state
    reg->isr = 0.0; // right integral state
    reg->duty_cycle_l = 0.0; // left duty cycle command
    reg->duty_cycle_r = 0.0; // right duty cycle command
}

void free_regulator(Regulator *regulator) {
    free(regulator);
}

inline void control_speed(
    Regulator *reg, 
    double speed_l,
    double speed_r,
    double speed_refl,
    double speed_refr) {

    speed_l = SAT(speed_l, REF_SPEED_LIMIT);
    speed_r = SAT(speed_r, REF_SPEED_LIMIT);

    // PI controller
    double esl, esr;        // Errors on speed, left and right
    double vl, vr;          // Voltage output commands, left and right
    int duty_cycle_left, duty_cycle_right;  // Duty cycles

    // Compute error
    esl = speed_refl - speed_l;
    esr = speed_refr - speed_r;

    // Compute integral
    reg->isl = SAT(reg->isl + esl, reg->imax);
    reg->isr = SAT(reg->isr + esr, reg->imax);

    #ifdef VERBOSE
    printf("Left Integral  : %.4f\n", isl);
    printf("Right Integral : %.4f\n", isr);
    #endif

    // Compute voltages
    vl = reg->kp * esl + reg->ki * reg->isl;
    vr = reg->kp * esr + reg->ki * reg->isr;

    // update duty cycle, assuming duty cycle changes average voltage linearly
    #ifdef ADZ_ENABLE
    // add an anti-deadzone term to compensate the deadzone around 0
    int adz_l = adz * (fabs(speed_refl) > 0.02) * (fabs(speed_l) < 0.01) * (1-2*(vl < 0));
    int adz_r = adz * (fabs(speed_refr) > 0.02) * (fabs(speed_r) < 0.01) * (1-2*(vr < 0));
    reg->duty_cycle_l = SAT(((int)(vl * MOTOR_DUTY_RANGE)) + adz_l, MOTOR_DUTY_RANGE);
    reg->duty_cycle_r = SAT(((int)(vr * MOTOR_DUTY_RANGE)) + adz_r, MOTOR_DUTY_RANGE);
    #else
    reg->duty_cycle_l = SAT((int)(vl * MOTOR_DUTY_RANGE), MOTOR_DUTY_RANGE);
    reg->duty_cycle_r = SAT((int)(vr * MOTOR_DUTY_RANGE), MOTOR_DUTY_RANGE);
    #endif

    #ifdef VERBOSE
    printf("Voltages : %.4f, %.4f\n", vl*24, vr*24);
    printf("Updating duty cycles to : %d, %d\n\n", reg->duty_cycle_l, reg->duty_cycle_l);
    #endif

}