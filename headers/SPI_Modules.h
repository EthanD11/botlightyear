#ifndef BLY_SPI_MODULES_H
#define BLY_SPI_MODULES_H

#include <lgpio.h>
#include <stdint.h>
#include <unistd.h>
#include "math.h"

#define VERBOSE
#include <stdio.h>

// SPI
#define SPI_DE0 1
#define SPI_TEENSY 0
#define SPI_MODE_DEFAULT 0
#define SPI_SPEED_HZ_DEFAULT 100000 // Arbitrary, but 500000 is reasonable

// Odometers conversion factors
#define ODO_TICKS_TO_M 1.7257283863713464e-05 // Conversion factor ticks to meters. (theoretical : pi*45e-3/8192; practical : 610e-3/(2**3+2**4+2**6+4*2**11))

// Sonars


// Steppers
#define PLATEAU_REDUCTION 8.5
#define PLATEAU_ANGLE_OUVERTURE 103.33
#define PLATEAU_TIC_STEPPER 1600


// ----- SPI -----

/**
 * @brief Opens the SPI channels with default settings
 */
int init_spi();

/**
 * @brief Closes the SPI channels
*/
void close_spi();

// ----- ODOMETERS -----

/**
 * @brief Get current ticks from left and right odometers and update them to given pointers. gpioInitialise() must have been called before.
 * 
 * @param tick_left pointer that contains number of ticks of left wheel after call
 * @param tick_right pointer that contains number of ticks of right wheel after call
 */
void odo_get_tick(int32_t *tick_left, int32_t *tick_right);

/**
 * @brief Resets internal values of DE0-Nano
 */
void odo_reset();

/**
 * @brief Initializes sonar pins

void init_sonar();

**
 * @brief Trigger sonar pulse and return output distance. Must be called after init_sonar

double sonar_ask();*/

// ----- TEENSY -----
typedef enum {
	QueryIdle, // Idle, reset motor voltages to 0V
	QueryTestRead, // SPI test, answer with [1,2,3,4]
	QueryTestWrite, // SPI test, answer with data received
	QueryDoPositionControl, // Position update, data received = [flag,x,y,t,xr,yr,tr]
	QueryDoPathFollowing,
	QueryAskGoalReached,
	QueryAskState,
	QuerySetPosition
} query_t;

/**
 * @brief Send constant speed query to Teensy. Must be called after init_spi.
*/
void teensy_spd_ctrl(double speed_left, double speed_right);

/**
 * @brief Ask the teensy to enter the position control mode with the specified reference position.
 * Must be called after init_spi. 
 */
void teensy_pos_ctrl(double xr, double yr, double tr);

void teensy_pos_ctrl(double x, double y, double t, double xr, double yr, double tr); 

void teensy_spd_ctrl(double speed_left, double speed_right);

/**
 * @brief Ask the teensy to enter the path following mode with the specified checkpoints. 
 * Must be called after init_spi. 
 */
void teensy_path_following(double *x, double *y, int ncheckpoints, double theta_start, double theta_end);

/**
 * @brief Send query to idle Teensy. It will stop any control effort over the motors. Must be called after init_spi.
 */
void teensy_idle();

/**
 * @brief Set the position of the teensy
*/
void teensy_set_position(double x, double y, double theta);

/**
 * @brief Return in which mode the teensy is currently
*/
void teensy_ask_mode();

// ------ SERVOS -----

typedef enum {
    ServoIdle,
    ServoDeploy,
    ServoRaise,
} servo_cmd_t; // Commands for flaps servomotor

/**
 * @brief Raises (retracts) flaps. Maintains a constant torque until servo_idle is called
 */
void servo_cmd(servo_cmd_t command);

// ------ STEPPERS -------

typedef enum {
    StprPlate, 
    StprSlider, 
    StprFlaps
} steppers_t;

typedef enum {
    FlapsOpen, 
    FlapsPlant, 
    FlapsPot
} flaps_pos_t;

typedef enum {
    SliderLow,
    SliderHigh,
    SliderPlate, 
    SliderTake
} slider_pos_t;

/**
 * @brief Move the stepper 'stepperName' of 'steps' steps in the 'neg' direction (0 for positive, 1 for negative)
*/
void stpr_move(steppers_t stepperName, uint32_t steps, int neg);

/**
 * @brief Move flaps to 'pos' (FlapsOpen, FlapsPlant or FlapsPot)
*/
void flaps_move(flaps_pos_t pos);

/**
 * @brief Move slider to 'pos' (SliderLow, SliderHigh, SliderPlate or SliderTake)
*/
void slider_move(slider_pos_t pos);

/**
 * @brief Move plate to slot number 'slot' ([-3 ; 3], 0 is neutral)
*/
void plate_move(int slot);

/**
 * @brief Sets the nominal and calibration speed of the stepper
 */
void stpr_setup_speed(int nominalSpeed, int calibrationSpeed, steppers_t stepper); 

/**
 * @brief Activates the calibration of given stepper
 */
void stpr_calibrate(steppers_t stepper); 

/**
 * @brief Resets stepper module to be ready for another calibration
 */
void stpr_reset(steppers_t stepper); 

/**
 * @brief Sets up the acceleration of a stepper (lower 'acc' -> higher acceleration)
*/
void stpr_setup_acc(steppers_t stepper, uint8_t acc);

#endif