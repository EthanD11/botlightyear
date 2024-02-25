#ifndef MECATROMINIBOT_SPI_MODULES_H
#define MECATROMINIBOT_SPI_MODULES_H

#include <lgpio.h>
#include <stdint.h>

// SPI
#define SPI_DE0 1
#define SPI_TEENSY 0
#define SPI_MODE_DEFAULT 0
#define SPI_SPEED_HZ_DEFAULT 500000 // Arbitrary, but 500000 is reasonable

// Odometers and encoders conversion factors
#define TICKS_TO_METERS 1.7257283863713464e-05 // Conversion factor ticks to meters. (theoretical : pi*45e-3/8192; practical : 610e-3/(2**3+2**4+2**6+4*2**11))
#define TICKS_TO_RADS 9.817477042468104 // Conversion factor ticks to rad_motor/s (theoretical : 100/64*2*pi)

// Sonars


/**
 * @brief Opens the SPI channels with default settings
 */
void init_spi();

/**
 * @brief Closes the SPI channels
*/
void close_spi();

/**
 * @brief Get current ticks from left and right odometers and update them to given pointers. gpioInitialise() must have been called before.
 * 
 * @param tick_left pointer that contains number of ticks of left wheel after call
 * @param tick_right pointer that contains number of ticks of right wheel after call
 */
void get_odo_tick(int32_t *tick_left, int32_t *tick_right);

/**
 * @brief Get fast the current ticks form left and right odometers and update them to given pointers. gpioInitialise() must have been called before.
 * Executes in only one SPI query, at the cost of considering only the 16 MSBs of the counter. Handy for large movements.
 * 
 * @param tick_left pointer that contains number of ticks of left wheel after call
 * @param tick_right pointer that contains number of ticks of right wheel after call
 */
void get_odo_tick_fast(int32_t *tick_left, int32_t *tick_right);

/**
 * @brief Get the current speed from left and right encoders and update them to given pointers. gpioInitialise() must have been called before.
 * 
 * @param spd_left pointer that contains speed of left wheel after call
 * @param spd_right pointer that contains speed of right wheel after call
 */
void get_enc_spd(int32_t *spd_left, int32_t*spd_right);

/**
 * @brief Get fast the current speed from left and right encoders and update them to given pointers. gpioInitialise() must have been called before.
 * Executes in only one SPI query, at the cost of considering only the 16 LSBs of the counter. Handy for fast speed control.
 * 
 * @param spd_left pointer that contains speed of left wheel after call
 * @param spd_right pointer that contains speed of right wheel after call
 */
void get_enc_spd_fast(int32_t *spd_left, int32_t *spd_right);

/**
 * @brief Resets internal values of DE0-Nano
 */
void odo_enc_reset();

/**
 * @brief Initializes sonar pins

void init_sonar();

**
 * @brief Trigger sonar pulse and return output distance. Must be called after init_sonar

double sonar_ask();*/

#endif