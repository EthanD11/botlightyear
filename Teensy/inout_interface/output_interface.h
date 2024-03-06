#include <Arduino.h>
#include "utils.h"

#ifndef _OUTPUT_INTERFACE_H_
#define _OUTPUT_INTERFACE_H_

// Pins
// IN1
#define C_L = 14
#define C_R = 3;
// IN2
#define D_L = 15
#define D_R = 4;
// Enable
#define PWM_L = 22
#define PWM_R = 23;
// Current sensors
#define CURRENT_L = 41
#define CURRENT_R = 40;
// Encoders
#define LEVEL_SHIFTER = 2;
// Test points
#define A1 = 29;
#define A2 = 37;
#define A3 = 36;

typedef struct OutputInterface {
    int duty_cycle_refl, duty_cycle_refr; // Reference duty cycles
    int duty_cycle_l, duty_cycle_r; // Current duty cycle, avoid sharp changes
    int analog_write_a3pin;
} OutputInterface;

OutputInterface *init_outputs();
inline void write_outputs(OutputInterface *outputs);
inline void duty_cycle_update(OutputInterface *outputs);


#endif