#ifndef MECATROMINIBOT_SPI_MODULES_H
#define MECATROMINIBOT_SPI_MODULES_H

#include <lgpio.h>
#include <stdint.h>
#include <unistd.h>

#define VERBOSE
#ifdef VERBOSE
#include <stdio.h>
#endif

// SPI
#define SPI_DE0 1
#define SPI_TEENSY 0
#define SPI_MODE_DEFAULT 0
#define SPI_SPEED_HZ_DEFAULT 1000000 // Arbitrary, but 500000 is reasonable

// Odometers and encoders conversion factors
#define TICKS_TO_METERS 1.7257283863713464e-05 // Conversion factor ticks to meters. (theoretical : pi*45e-3/8192; practical : 610e-3/(2**3+2**4+2**6+4*2**11))
#define TICKS_TO_RADS 9.817477042468104 // Conversion factor ticks to rad_motor/s (theoretical : 100/64*2*pi)

// Sonars


// Steppers


#define REDUCTION_PLATEAU 8.5
#define ANGLE_OUVERTURE_PLATEAU 103.33
#define TIC_STEPPER_PLATEAU 1600

typedef enum {
    Plate, 
    Slider, 
    Flaps
} steppers_t;

typedef enum {
    Open, 
    Plant, 
    Pot
} positions_flaps_t;

typedef enum {
    Bas,
    Haut,
    Plateau, 
    Take
} position_slider_t;


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
void get_odo_tick(int32_t *tick_left, int32_t *tick_right);

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

/**
 * @brief Send constant speed query to Teensy. Must be called after init_spi.
*/
void teensy_spd_ctrl(double speed_left, double speed_right);

/**
 * @brief Send position update (current and reference) to Teensy. Must be called after init_spi. 
 */
void teensy_pos_ctrl(double x, double y, double t, double xr, double yr, double tr);

/**
 * @brief Send query to idle Teensy. It will stop any control effort over the motors. Must be called after init_spi.
 */
void teensy_idle();

// ------ SERVOS -----

/**
 * @brief Raises (retracts) flaps. Maintains a constant torque until servo_idle is called
 */
void servo_raise();

/**
 * @brief Deploys flaps. Maintains a constant torque until servo_idle is called
 */
void servo_deploy();

/**
 * @brief Releases torque command from the servomotors
 */
void servo_idle();


// ------ STEPPERS -------


void moveStepperSteps(steppers_t stepperName, int steps, int neg); 
void moveFlaps (positions_flaps_t pos); 
void moveSlider(position_slider_t pos); 
void PositionPlateau(int pot);

/**
 * @brief Sets the nominal and calibration speed of the stepper
 */
void setupStepperSpeed (int nominalSpeed, int calibrationSpeed, steppers_t stepper); 

/**
 * @brief Activtes the calibration of given stepper
 */
void calibrateStepper(steppers_t stepper); 

/**
 * @brief Resets stepper module to be ready for another calibration
 */
void resetStepperModule (steppers_t stepper); 

#endif