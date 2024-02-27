#if !defined(_controller_H_)
#define _controller_H_

// Uncomment to enable debug messages (! do not use on teensy (not as is, needs adaptation, coming soon I guess) !):
#define VERBOSE
#ifdef VERBOSE
#include <stdio.h>
#endif

//#define ODOMETERS_ENC

// Parameters definiton
#define REF_SPEED_LIMIT 0.7 // Saturation level of the reference angular velocity (rad_mot/s, 113*100*2pi/64, 0.9/30e-3*30)
#define REG_DELAY 20 // Delay between two updates (ms)
#define SPD_TOL 1e-2 // Max speed at which motors can be turned off (rad_mot/s)
#ifdef ODOMETERS_ENC
#define WHEEL_L 126e-3 // Half the distance between the two wheels (m)
#else
#define WHEEL_L 88.085e-3 // Half the distance between the two wheels (m)
#endif

/**
 * @brief Type 1 PI control of the speed based on angular velocities (left and right, actual and reference)
 * Computes the error, controls it with a PI controller, updates the duty cycle of the motor input voltage.
 * Arguments are respectively : omega-left, omega-right, omega-left-reference, omega-right-reference.
 */
void t1_speed_ctrl(double speed_l, double speed_r, double ref_l, double ref_r);

/**
 * @brief Type 3 State-feedback control of the position and orientation of the robot.
 * Computes the error in polar coordinates, controls with a state-feedback vector, and transforms into a reference input
 * for type 1 control.
 * Arguments are respectively : 
 * current x, y and theta, reference x, y and theta, pointers to contain left and right output reference speeds
 */
void t3_position_ctrl(double x, double y, double theta, double xr, double yr, double theta_r, double *ref_l, double *ref_r);

/**
 * @brief Initializes motor pins. Should be called during setup.
 */
void init_motors();

/**
 * @brief Updates duty cycles on left and right motor PWM signals. Includes a buffering check to avoid brutal changes in voltage.
 */
void duty_cycle_update(int left, int right);

#endif