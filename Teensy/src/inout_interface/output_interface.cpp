#include "output_interface.h"

OutputInterface *init_outputs() {
    OutputInterface *outputs = (OutputInterface *) malloc(sizeof(OutputInterface));
    outputs->duty_cycle_l = 0.0;
    outputs->duty_cycle_r = 0.0;
    outputs->duty_cycle_refl = 0.0;
    outputs->duty_cycle_refr = 0.0;

    // Output pins
    pinMode(C_L, OUTPUT);
    pinMode(D_L, OUTPUT);
    pinMode(PWM_L, OUTPUT);
    pinMode(C_R, OUTPUT);
    pinMode(D_R, OUTPUT);
    pinMode(PWM_R, OUTPUT);

    // ----- ENCODERS -----
    // Enable the level shifter
    pinMode(LEVEL_SHIFTER, OUTPUT);
    digitalWrite(LEVEL_SHIFTER, HIGH);

    // ---- MOTORS -----
    digitalWrite(C_L, LOW);
    digitalWrite(D_L, HIGH);
    digitalWrite(C_R, HIGH);
    digitalWrite(D_R, LOW);
    
    analogWriteFrequency(PWM_L, 20e3);
    analogWriteFrequency(PWM_R, 20e3);

    analogWrite(PWM_L, 0);
    analogWrite(PWM_R, 0);

    // ----- TEST PINS -----
    pinMode(A1, OUTPUT);
    pinMode(A2, OUTPUT);
    pinMode(A3, OUTPUT);

    analogWriteFrequency(A1, 20e3);
    analogWriteFrequency(A2, 20e3);
    analogWriteFrequency(A3, 20e3);
        
    analogWrite(A1, 0);
    analogWrite(A2, 0);
    analogWrite(A3, 0);

    return outputs;
}