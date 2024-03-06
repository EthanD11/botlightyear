#include "output_interface.h"

void init_outputs() {
    OutputInterface *outputs = (OutputInterface *) malloc(sizeof(OutputInterface));
    outputs->duty_cycle_curl = 0.0;
    outputs->duty_cycle_curr = 0.0;

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
    analogWriteFrequency(A3,20e3);

    analogWrite(A1, 0);
    analogWrite(A2, 0);
    analogWrite(A3,0);
}

inline void write_outputs(OutputInterface *outputs) {
    analogWrite(A3, outputs->analog_write_a3pin);
    duty_cycle_update(outputs);
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
    analogWrite(A1, std::abs(duty_cycle_l));

    // Right buffered control
    duty_cycle_r += SAT(duty_cycle_refr - duty_cycle_r, BUF_STEP);
    analogWrite(PWM_R, std::abs(duty_cycle_r));
    analogWrite(A2, std::abs(duty_cycle_r));

    outputs->duty_cycle_l = duty_cycle_l;
    outputs->duty_cycle_r = duty_cycle_r;

    // Left Direction (forward vs backward)
    if (duty_cycle_curl < 0) {
        digitalWrite(C_L, HIGH);
        digitalWrite(D_L, LOW);
    } else {
        digitalWrite(C_L, LOW);
        digitalWrite(D_L, HIGH);
    }

    // Right Direction (forward vs backward)
    if (duty_cycle_curr < 0) {
        digitalWrite(C_R, LOW);
        digitalWrite(D_R, HIGH);
    } else {
        digitalWrite(C_R, HIGH);
        digitalWrite(D_R, LOW);
    }
}