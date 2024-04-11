#ifndef _BLY_TEENSY_H_
#define _BLY_TEENSY_H_

#include "SPI_bus.h"
#include "GPIO.h"

typedef enum int8_t {
    ModeIdle,
    ModePositionControl,
    ModePathFollowingInit,
    ModePathFollowing,
    ModeSpeedControl,
    ModeConstantDC,
    ModeUnknown = -1
} teensy_mode_t;

class Teensy : SPIUser, GPIOUser
{
public:
    /**
     * @brief Initialises the Teensy object
     * @param bus a pointer to the SPIBus object
     * @param pins a pointer to the GPIOPins object
     */
    Teensy(SPIBus *bus, GPIOPins *pins) : SPIUser(bus), GPIOUser(pins) {}
    void path_following(double *x, double *y, int ncheckpoints,
                        double theta_start, double theta_end,
                        double vref, double dist_goal_reached);
    void set_position(double x, double y, double theta);
    void pos_ctrl(double xr, double yr, double theta_r);
    void constant_dc(int dc_refl, int dc_refr);
    void spd_ctrl(double speed_left, double speed_right);
    void idle();
    teensy_mode_t ask_mode();
    void set_position_controller_gains(double kp, double ka, double kb, double kw);
    void set_path_following_gains(double kt, double kn, double kz,
                                    double sigma, double epsilon, double kv_en, double delta, double wn);
};

#endif