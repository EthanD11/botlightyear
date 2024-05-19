#ifndef _BLY_ODOMETRY_H_
#define _BLY_ODOMETRY_H_

#include "SPI_bus.h"
#include <stdint.h>

//#define VERBOSE
#ifdef VERBOSE
#include <stdio.h>
#endif

#define ODO_TICKS_TO_M 1.7257283863713464e-05 // Conversion factor ticks to meters. (theoretical : pi*45e-3/8192; practical : 610e-3/(2**3+2**4+2**6+4*2**11))


class Odometry : private SPIUser
{
public:
    /** @brief Odometry-related object.
    * Uses SPI to communicate the odometers' position information
    */
    Odometry(SPIBus *bus) : SPIUser(bus) {
        #ifdef VERBOSE
        printf("Constructor Odometry\n");
        #endif
    }
    /**
     * @brief Resets the odometer module. Sets the ticks and the position to the sent value from Odometry::set_pos().
    */
    void reset();
    /**
     * @brief Get the current robot position
     * @param x The pointer to fill the x position of the robot
     * @param y The pointer to fill the y position of the robot
     * @param theta The pointer to fill the orientation of the robot
     * If one of the pointers is NULL, it will not be filled
    */
    void get_pos(double *x, double *y, double *theta);

    /**
     * @brief Send the current robot position. The data sent is only applied when the Odometry::reset() is called
     * @param x The x position of the robot
     * @param y The y position of the robot
     * @param theta The orientation of the robot
    */
    void set_pos(double x, double y, double theta);

    /**
     * @brief Get odometers ticks
     * @param ticksLeft A pointer to fill the ticks of the left odometer
     * @param ticksRight A pointer to fill the ticks of the right odometer
     * If one of the pointers is NULL, it will not be filled
    */
    void get_ticks(int32_t *ticksLeft, int32_t *ticksRight);
};

#endif