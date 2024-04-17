#include <Arduino.h>
#include "../../utils.h"

#ifndef _OUTPUT_INTERFACE_H_
#define _OUTPUT_INTERFACE_H_

// Pins
// IN1
#define D_L 4
#define D_R 15
// IN2
#define C_L 3
#define C_R 14
// Enable
#define PWM_R 22
#define PWM_L 23
// Current sensors
#define CURRENT_L 41
#define CURRENT_R 40
// Encoders
#define LEVEL_SHIFTER 2
// Test points
#define A1 29
#define A2 37
#define A3 36

#define MOTOR_DUTY_RANGE 256  // Duty cycle range
#define BUF_STEP 3           // Max step to avoid brutal speed changes

typedef struct OutputInterface {
    int duty_cycle_refl, duty_cycle_refr; // Reference duty cycles
    int duty_cycle_l, duty_cycle_r; // Current duty cycle, avoid sharp changes
    int digital_write_a1pin, digital_write_a2pin, digital_write_a3pin;
} OutputInterface;

OutputInterface *init_outputs();

/*
 * Updates the duty cycle reference of the motors
 * The duty cycle references are numbers between -256 and 256
 * The duty cycle is thus DC= dcref / 255
 */
inline void set_motors_duty_cycle(OutputInterface *outputs, int dcref_left, int dcref_right) {
    outputs->duty_cycle_refl = dcref_left;
    outputs->duty_cycle_refr = dcref_right;
}

/*
 * Updates the duty cycle reference of the motors
 * The duty cycle references are numbers between -256 and 256
 * The duty cycle is thus DC= dcref / 255
 */ 
inline void set_a1pin(OutputInterface *outputs, int value) {
    outputs->digital_write_a1pin = value;
} 
inline void set_a2pin(OutputInterface *outputs, int value) {
    outputs->digital_write_a2pin = value;
}
inline void set_a3pin(OutputInterface *outputs, int value) {
    outputs->digital_write_a3pin = value;
} 
inline void set_apins(OutputInterface *outputs, int8_t value) {
    outputs->digital_write_a1pin = (value & 0x01);
    outputs->digital_write_a2pin = (value & 0x02) >> 1;
    outputs->digital_write_a3pin = (value & 0x04) >> 2;
    // printf("value = %d\n", value);
    // printf("a1 = %d\n", outputs->digital_write_a1pin);
    // printf("a2 = %d\n", outputs->digital_write_a2pin);
    // printf("a3 = %d\n", outputs->digital_write_a3pin);
}

inline void duty_cycle_update(OutputInterface *outputs)
{
    int duty_cycle_l, duty_cycle_r;
    int duty_cycle_refl, duty_cycle_refr;

    duty_cycle_l = outputs->duty_cycle_l;
    duty_cycle_r = outputs->duty_cycle_r;

    duty_cycle_refl = outputs->duty_cycle_refl;
    duty_cycle_refr = outputs->duty_cycle_refr;

    // Left buffered control
    duty_cycle_l += SAT(duty_cycle_refl - duty_cycle_l, BUF_STEP);
    analogWrite(PWM_L, std::abs(duty_cycle_l));

    // Right buffered control
    duty_cycle_r += SAT(duty_cycle_refr - duty_cycle_r, BUF_STEP);
    analogWrite(PWM_R, std::abs(duty_cycle_r));

    outputs->duty_cycle_l = duty_cycle_l;
    outputs->duty_cycle_r = duty_cycle_r;

    // Left Direction (forward vs backward)
    if (duty_cycle_l < 0) {
        digitalWrite(C_L, HIGH);
        digitalWrite(D_L, LOW);
    } else {
        digitalWrite(C_L, LOW);
        digitalWrite(D_L, HIGH);
    }

    // Right Direction (forward vs backward)
    if (duty_cycle_r < 0) {
        digitalWrite(C_R, LOW);
        digitalWrite(D_R, HIGH);
    } else {
        digitalWrite(C_R, HIGH);
        digitalWrite(D_R, LOW);
    }
}

inline void write_outputs(OutputInterface *outputs) {
    analogWrite(A1, outputs->digital_write_a1pin*256);
    analogWrite(A2, outputs->digital_write_a2pin*256);
    analogWrite(A3, outputs->digital_write_a3pin*256);
    duty_cycle_update(outputs);
}

#endif