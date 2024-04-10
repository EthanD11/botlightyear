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




// Odometers conversion factors
#define ODO_TICKS_TO_M 1.7257283863713464e-05 // Conversion factor ticks to meters. (theoretical : pi*45e-3/8192; practical : 610e-3/(2**3+2**4+2**6+4*2**11))
#define ODO_WHEEL_L 24.48e-2

// Sonars




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
void spi_close();

/**
 * Test SPI communication, returns 0 on success
*/
int test_spi();

// ----- SPI2 -----


/**
 * @brief Claims the SPI2 GPIOs and opens the chip
 * @return 0 if no error, 1 if chip open fail, 2 if set user fail
*/
uint8_t init_spi2(); 

/**
 * @brief Frees the SPI2 GPIOs and closes the chip
*/
void spi_close2();

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

#endif