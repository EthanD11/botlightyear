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
//const uint8_t CURRENT_L = 41, CURRENT_R = 40;
// Test points
#define a1 = 29;
#define a2 = 37;

void init_motors();
void duty_cycle_update();

#endif