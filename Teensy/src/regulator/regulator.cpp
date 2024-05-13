#include "regulator.h"

Regulator *init_regulator() {
    Regulator *reg = (Regulator *) malloc(sizeof(Regulator));
    
    reg->kp_l = 10.204081632653060/3; // Former t1_kp
    reg->ki_l = 24.295432458697764/3; // Former t1_ki

    reg->kp_r = 6.666666666666665/3; // Former t1_kp
    reg->ki_r = 23.809523809523803/3; // Former t1_ki

    reg->imax = 1; // Anti windup [V/V]

    reg->isl = 0.0; // left integral state
    reg->isr = 0.0; // right integral state
    reg->el_filtered = 0.0; // left filtered error
    reg->er_filtered = 0.0; // left filtered
    reg->duty_cycle_refl = 0.0; // left duty cycle command
    reg->duty_cycle_refr = 0.0; // right duty cycle command

    reg->wc = 50*2*M_PI; // [rad/s]

    return reg;
}

void free_regulator(Regulator *regulator) {
    free(regulator);
}

void reset_regulator(Regulator *speed_regulator) {
    speed_regulator->isl = 0.0;
    speed_regulator->isr = 0.0;
    speed_regulator->el_filtered = 0.0;
    speed_regulator->er_filtered = 0.0;
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
    double alpha;
    double speed;
    double gain_factor;

    speed_l = rob_pos->speed_left;
    speed_r = rob_pos->speed_right;
    speed = hypot(speed_l, speed_r);
    speed_refl = SAT(speed_refl, REF_SPEED_LIMIT);
    speed_refr = SAT(speed_refr, REF_SPEED_LIMIT);
    dt = rob_pos->dt;

    if (speed < 0.05) gain_factor = 5;
    else if (speed < 0.1) gain_factor = 3;
    else gain_factor = 1;

    double kp_l = gain_factor*reg->kp_l;
    double kp_r = gain_factor*reg->kp_r;
    double ki_l = gain_factor*reg->ki_l;
    double ki_r = gain_factor*reg->ki_r;

    // Compute error
    esl = speed_refl - speed_l;
    esr = speed_refr - speed_r;

    alpha = exp(-reg->wc*dt);
    reg->el_filtered = alpha * reg->el_filtered + (1 - alpha) * esl;
    reg->er_filtered = alpha * reg->er_filtered + (1 - alpha) * esr;

    // Compute integral
    reg->isl = SAT(reg->isl + ki_l*reg->el_filtered*dt, reg->imax);
    reg->isr = SAT(reg->isr + ki_r*reg->er_filtered*dt, reg->imax);

    #ifdef VERBOSE
    printf("Left Integral  : %.4f\n", isl);
    printf("Right Integral : %.4f\n", isr);
    #endif

    // Compute voltages
    
    vl = kp_l * reg->el_filtered + reg->isl;
    vr = kp_r * reg->er_filtered + reg->isr;

    // update duty cycle, assuming duty cycle changes average voltage linearly
    #ifdef ADZ_ENABLE
    // add an anti-deadzone term to compensate the deadzone around 0
    int adz_l = ADZ_L * (fabs(speed_refl) > 0.01) * (fabs(speed_l) < 0.01) * (1-2*(vl < 0));
    int adz_r = ADZ_R * (fabs(speed_refr) > 0.01) * (fabs(speed_r) < 0.01) * (1-2*(vr < 0));
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