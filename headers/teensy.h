#ifndef _BLY_TEENSY_H_
#define _BLY_TEENSY_H_

#include "SPI_bus.h"
#include "GPIO.h"

//#define VERBOSE
#ifdef VERBOSE
#include <stdio.h>
#endif

enum __teensy_mode_t : int8_t {
    ModeIdle,
    ModePositionControl,
    ModePathFollowingInit,
    ModePathFollowing,
    ModeSpeedControl,
    ModeConstantDC,
    ModePositionControlOver,
    ModeUnknown = -1
};
typedef __teensy_mode_t teensy_mode_t;

class Teensy : public SPIUser, public GPIOUser
{
public:
    /**
     * @brief Initialises the Teensy object
     * @param bus a pointer to the SPIBus object
     * @param pins a pointer to the GPIOPins object
     */
    Teensy(SPIBus *bus, GPIOPins *pins) : SPIUser(bus), GPIOUser(pins) {
        #ifdef VERBOSE
        printf("Constructor Teensy\n");
        #endif
    }
    /**
     * @brief Sends a path following request to the teensy 
     * @param x,y The coordinates of the checkpoints to follow
     * @param ncheckpoints The number of checkpoints to follow
     * @param theta_start The initial orientation of the robot when starting the path following
     * @param theta_end The final orientation of the robot when finishing the path following
     * @param vref The reference speed to follow during the displacement
     * @param dist_goal_reached The distance (in meters) from where the controller switches to a position control at the end of the trajectory
     */
    void path_following(double *x, double *y, int ncheckpoints,
                        double theta_start, double theta_end,
                        double vref, double dist_goal_reached);
     /**
     * @brief Sets up the robot's position on the teensy
     * @param x,y,theta The robot's position to setup
     */                   
    void set_position(double x, double y, double theta);
    /**
     * @brief Sends a position control request to the teensy
     * @param xr,yr,theta_r The robot's desired end position 
     */  
    void pos_ctrl(double xr, double yr, double theta_r);
    /**
     * @brief Sends a constant dutycycle request to the teensy
     * @param dc_refl, dc_refr The reference duty cycles of the left and right wheels (from 0 up to 255)
     */  
    void constant_dc(int dc_refl, int dc_refr);
    /**
     * @brief Sends a speed control request to the teensy
     * @param speed_left, speed_right The reference speeds of the left and right wheels (in m/s)
     */  
    void spd_ctrl(double speed_left, double speed_right);
    /**
     * @brief Idles the teensy : no control is applied on the motors
     */  
    void idle();
    /**
     * @brief Gets the teensy current FSM state
     * @returns The mode associated with teensy's internal FSM, or ModeUnknown if SPI communication error
     */  
    teensy_mode_t ask_mode();
    /**
     * @brief Sets up the controller gains for position control
     */  
    void set_position_controller_gains(double kp, double ka, double kb, double kw);
    /**
     * @brief Sets up the controller gains for path following
     */  
    void set_path_following_gains(double kt, double kn, double kz,
                                    double sigma, double epsilon, double kv_en, double delta, double wn);
};

#endif