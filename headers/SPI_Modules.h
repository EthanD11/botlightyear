#ifndef BLY_SPI_MODULES_H
#define BLY_SPI_MODULES_H

#include <lgpio.h>
#include <stdint.h>
#include <unistd.h>
#include "math.h"

// SPI
#define SPI_DE0 1
#define SPI_TEENSY 0
#define SPI_MODE_DEFAULT 0
#define SPI_SPEED_HZ_DEFAULT 100000 // Arbitrary, but 500000 is reasonable

// SPI2 

#define CALL_BLOCKING 1
#define CALL_NON_BLOCKING 0


// Odometers conversion factors
#define ODO_TICKS_TO_M 1.7257283863713464e-05 // Conversion factor ticks to meters. (theoretical : pi*45e-3/8192; practical : 610e-3/(2**3+2**4+2**6+4*2**11))
#define ODO_WHEEL_L 24.48e-2

// Sonars


// Steppers
#define FALSE 0

#define PLATEAU_REDUCTION 8.5
#define PLATEAU_ANGLE_OUVERTURE 103.33
#define PLATEAU_TIC_STEPPER 1600

// Other
#define SATURATE(a,lb,ub) ((a) > (ub) ? (ub) : ((a) < (lb) ? (lb) : (a)))


// ----- SPI -----

/**
 * @brief Opens the SPI channels with default settings
 */
int init_spi();

/**
 * @brief Closes the SPI channels
*/
void close_spi();

/**
 * Test SPI communication, returns 0 on success
*/
int test_spi();

// ----- SPI2 -----

typedef enum {
    StprSliderGPIO = 0, 
    StprPlateGPIO = 5, 
    StprFlapsGPIO = 6, 
    BpSwitchFlapsLeftGPIO = 13, 
    BpSwitchFlapsRightGPIO = 19
} spi2_gpio_t;

/**
 * @brief Claims the SPI2 GPIOs and opens the chip
 * @return 0 if no error, 1 if chip open fail, 2 if set user fail
*/
uint8_t init_spi2(); 

/**
 * @brief Frees the SPI2 GPIOs and closes the chip
*/
void close_spi2();

/**
 * @brief Claims a GPIO
 * @return 0 if no error, 1 if claim input fail
* @param gpio_pin the pin number of the GPIO to be claimed
*/
uint8_t claim_gpio(uint8_t gpio_pin); 

/**
 * @brief Frees a GPIO pin number
 * @param gpio_pin the pin number of the GPIO to be freed
*/
void free_gpio(uint8_t gpio_pin); 

// ----- ODOMETERS -----

/**
 * DEPRECATED Get current ticks from left and right odometers and update them to given pointers
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
 * @brief Set the DE0's internal position to (x,y,theta). Resets internal odometer values
*/
void odo_set_pos(double x, double y, double theta);

/**
 * @brief Get the DE0's internal position and 
*/
void odo_get_pos(double *x, double *y, double *theta);

/**
 * @brief Initializes sonar pins

void init_sonar();

**
 * @brief Trigger sonar pulse and return output distance. Must be called after init_sonar

double sonar_ask();*/

// ----- TEENSY -----

typedef enum {
	QueryIdle,
	QueryDoPositionControl,
	QueryDoSpeedControl,
	QueryDoSetDutyCycle,
	QueryDoPathFollowing,
	QueryDoConstantDutyCycle,
	QueryAskState,
	QuerySetPosition,
	QuerySetPositionControlGains,
	QuerySetPathFollowerGains
} query_t;

typedef enum {
  ModeIdle,
  ModePositionControl,
  ModePathFollowingInit,
  ModePathFollowing,
  ModeSpeedControl,
  ModeConstantDC
} controlmode_t; 

/**
 * @brief Send constant speed query to Teensy. Must be called after init_spi.

void teensy_spd_ctrl(double speed_left, double speed_right);*/

/**
 * @brief Ask the teensy to enter the position control mode with the specified reference position.
 * Must be called after init_spi. 
 */
void teensy_pos_ctrl(double xr, double yr, double tr); 

/**
 * @brief Apply a constant duty cycle to motors via Teensy
 */
void teensy_constant_dc(int dc_refl, int dc_refr);

/**
 * @brief Ask the teensy to enter the path following mode with the specified checkpoints. 
 * Must be called after init_spi. 
 */
void teensy_path_following(double *x, double *y, int ncheckpoints, double theta_start, double theta_end, double vref, double dist_goal_reached);

/**
 * @brief Send query to idle Teensy. It will stop any control effort over the motors. Must be called after init_spi.
 */
void teensy_idle();

/**
 * @brief Set the position of the teensy
*/
void teensy_set_position(double x, double y, double theta);

/**
 * @brief Set the gains of the position controller
*/
void teensy_set_position_controller_gains(double kp, double ka, double kb, double kw);

/**
 * @brief Set the gains of the path following controller
*/
void teensy_set_path_following_gains(double kt, double kn, double kz, double sigma, double epsilon, double kv_en, double delta, double wn);

/**
 * @brief Return in which mode the teensy is currently
*/
int teensy_ask_mode();

/**
 * @brief Ask current position to Teensy, based on odometry.
 * 
 
void teensy_ask_pos(double *x, double *y, double *theta);*/

// ------ SERVOS -----

typedef enum {
    FlapsIdle,
    FlapsDeploy,
    FlapsRaise,
} flaps_servo_cmd_t; // Commands for flaps servomotor


typedef enum {
    HolderIdle, 
    HolderClosed,
    HolderOpen, 
    HolderPot,
    HolderPlant
} gripper_holder_cmd_t; // Commands for gripper holder servomotor

typedef enum {
    DeployerIdle, 
    DeployerRaise,
    DeployerHalf, 
    DeployerDeploy
} gripper_deployer_cmd_t; // Commands for griper deployer servomotor


/**
 * @brief Raises (retracts) flaps. Maintains a constant torque until FlapsIdle is called
 * @param command a flaps position (FlapsIdle, FlapsDeploy or FlapsRaise)
 */
void flaps_servo_cmd(flaps_servo_cmd_t command);

/**
 * @brief Opens and closes the gripper. Maintains a constant torque until HolderIdle is called
 * @param command a holder position (HolderIdle, HolderClosed, HolderPot or HolderPlant)
 */
void gripper_holder_cmd(gripper_holder_cmd_t command);

/**
 * @brief Raises and deploys the gripper. Maintains a constant torque until DeployerIdle is called
 * @param command a deployer position (DeployerIdle, DeployerRaise or DeployerDeploy)
 */
void gripper_deployer_cmd(gripper_deployer_cmd_t command); 

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
    SliderTake, 
    SliderDeposit
} slider_pos_t;

/**
 * @brief Move the stepper 'stepperName' of 'steps' steps in the 'neg' direction (0 for positive, 1 for negative)
 * @param stepperName the stepper to calibrate (StprPlate, StprSlider or StprFlaps)
 * @param steps the step count to go to
 * @param neg the direction of the step count (0 for positive, 1 for negative)
 * @param blocking CALL_BLOCKING if the command is blocking (waits until finished), CALL_NON_BLOCKING if it's non-blocking
 * (non-blocking by default)
*/
void stpr_move(steppers_t stepperName, uint32_t steps, uint8_t neg, uint8_t blocking = CALL_NON_BLOCKING);

/**
 * @brief Activates the calibration of given stepper
 * @param stepperName the stepper to calibrate (StprPlate, StprSlider or StprFlaps)
 * @param blocking CALL_BLOCKING if the command is blocking (waits until finished), CALL_NON_BLOCKING if it's non-blocking
 * (non-blocking by default)
 */
void stpr_calibrate(steppers_t stepperName, uint8_t blocking = CALL_NON_BLOCKING); 

/**
 * @brief Move flaps stepper to 'pos' (FlapsOpen, FlapsPlant or FlapsPot)
 * @param pos the position of the flaps stepper (FlapsOpen, FlapsPlant or FlapsPot)
 * @param blocking CALL_BLOCKING if the command is blocking (waits until finished), CALL_NON_BLOCKING if it's non-blocking
 * (non-blocking by default)
*/
void flaps_move(flaps_pos_t pos, uint8_t blocking = CALL_NON_BLOCKING);

/**
 * @brief Move slider stepper to 'pos' (SliderLow, SliderHigh, SliderPlate or SliderTake)
 * @param pos the position of the slider stepper (SliderLow, SliderHigh, SliderPlate or SliderTake)
 * @param blocking CALL_BLOCKING if the command is blocking (waits until finished), CALL_NON_BLOCKING if it's non-blocking
 * (non-blocking by default)
*/
void slider_move(slider_pos_t pos, uint8_t blocking = CALL_NON_BLOCKING);


/**
 * @brief Move plate to slot number 'slot' ([-3 ; 3], 0 is neutral)
 * @param blocking CALL_BLOCKING if the command is blocking (waits until finished), CALL_NON_BLOCKING if it's non-blocking 
 * (non-blocking by default)
*/
void plate_move(int8_t slot, uint8_t blocking = CALL_NON_BLOCKING);

/**
 * @brief Sets the nominal and initial speed of the stepper
 * @param stepperName the stepper to setup the speed(StprPlate, StprSlider or StprFlaps)
 * @param nominalSpeed is the upper bound of the speed limit for the stepper's speed
 * @param initialSpeed is the initial speed that the stepper begins with at each movement
 */
void stpr_setup_speed(steppers_t stepperName, int nominalSpeed, int initialSpeed); 

/**
 * @brief Sets the calibration speed and the small calibration speed of the stepper
 * @param stepperName the stepper to setup the calibration speed (StprPlate, StprSlider or StprFlaps)
 * @param calibrationSpeed is the nominal speed in calibration mode for the first calibration step
 * @param smallCalibrationSpeed is the smaller speed used in the second calibration step
 */
void stpr_setup_calib_speed(int calibrationSpeed, int smallCalibrationSpeed, steppers_t stepperName);


/**
 * @brief Resets stepper module to be ready for another calibration
 * @param stepperName the stepper to reset (StprPlate, StprSlider or StprFlaps)
 */
void stpr_reset(steppers_t stepperName); 

/**
 * @brief Sets up the acceleration of a stepper (lower 'acc' -> higher acceleration)
 * @param stepperName the stepper to setup the acceleration (StprPlate, StprSlider or StprFlaps)
 * @param acc_steps the acceleration counter of the stepper. At each "acc" steps, the stepper will increase its speed from its initial speed towards the nominal speed
*/
void stpr_setup_acc(steppers_t stepperName, uint8_t accSteps);

/**
 * @brief Calibrates all the steppers (non-blocking assignment)
*/
void stpr_calibrate_all(uint8_t blocking = CALL_NON_BLOCKING);

/**
 * @brief Resets all the steppers
*/
void stpr_reset_all();

#endif