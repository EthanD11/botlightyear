#include "regulator.h"

Regulator *init_regulator() {
    Regulator *reg = (Regulator *) malloc(sizeof(Regulator));
    
    // reg->kp = 7.795856e-01; // Former t1_kp
    // reg->ki = 8.398411e-01 * REG_DELAY * 1e-3; // Former t1_ki
    
    reg->kp = 13; // Former t1_kp
    reg->ki = 44; // Former t1_ki
    

    reg->imax = 1; // Anti windup [V/V]

    reg->isl = 0.0; // left integral state
    reg->isr = 0.0; // right integral state
    reg->duty_cycle_refl = 0.0; // left duty cycle command
    reg->duty_cycle_refr = 0.0; // right duty cycle command

    return reg;
}

void free_regulator(Regulator *regulator) {
    free(regulator);
}
void control_speed(
    Regulator *reg,
    RobotPosition *rob_pos,
    double speed_refl,
    double speed_refr) {

    // PI controller
    double esl, esr;        // Errors on speed, left and right
    double vl, vr;          // Voltage output commands, left and right
    double speed_r, speed_l;
    double dt;

    speed_l = rob_pos->speed_left;
    speed_r = rob_pos->speed_right;
    speed_refl = SAT(speed_refl, REF_SPEED_LIMIT);
    speed_refr = SAT(speed_refr, REF_SPEED_LIMIT);
    dt = rob_pos->dt;

    // Compute error
    esl = speed_refl - rob_pos->speed_left;
    esr = speed_refr - rob_pos->speed_right;

    // Compute integral
    reg->isl = SAT(reg->isl + reg->ki*esl*dt, reg->imax);
    reg->isr = SAT(reg->isr + reg->ki*esr*dt, reg->imax);

    #ifdef VERBOSE
    printf("Left Integral  : %.4f\n", isl);
    printf("Right Integral : %.4f\n", isr);
    #endif

    // Compute voltages
    vl = reg->kp * esl + reg->isl;
    vr = reg->kp * esr + reg->isr;

    // update duty cycle, assuming duty cycle changes average voltage linearly
    #ifdef ADZ_ENABLE
    // add an anti-deadzone term to compensate the deadzone around 0
    int adz_l = adz * (fabs(speed_refl) > 0.02) * (fabs(speed_l) < 0.01) * (1-2*(vl < 0));
    int adz_r = adz * (fabs(speed_refr) > 0.02) * (fabs(speed_r) < 0.01) * (1-2*(vr < 0));
    reg->duty_cycle_refl = SAT(((int)(vl * MOTOR_DUTY_RANGE)) + adz_l, MOTOR_DUTY_RANGE);
    reg->duty_cycle_refr = SAT(((int)(vr * MOTOR_DUTY_RANGE)) + adz_r, MOTOR_DUTY_RANGE);
    #else
    reg->duty_cycle_refl = SAT((int)(vl * MOTOR_DUTY_RANGE), MOTOR_DUTY_RANGE);
    reg->duty_cycle_refr = SAT((int)(vr * MOTOR_DUTY_RANGE), MOTOR_DUTY_RANGE);
    #endif

    #ifdef VERBOSE
    printf("Voltages : %.4f, %.4f\n", vl*24, vr*24);
    printf("Updating duty cycles to : %d, %d\n\n", reg->duty_cycle_refl, reg->duty_cycle_refl);
    #endif

}