#ifndef _BLY_TEENSY_H_
#define _BLY_TEENSY_H_

#include "SPI_bus.h"

class Teensy
{
private:
    SPI_bus *bus;
public:
    Teensy(SPI_bus *bus);
    ~Teensy();
    void path_following(double *x, double *y, int ncheckpoints,
                        double theta_start, double theta_end,
                        double vref, double dist_goal_reached);
    void set_position(double x, double y, double theta);
    void pos_ctrl(double xr, double yr, double theta_r);
    void constant_dc(int dc_refl, int dc_refr);
    void idle();
    int ask_mode();
    void set_position_controller_gains(double kp, double ka, double kb, double kw);
    void set_path_following_gains(double kt, double kn, double kz,
                                    double sigma, double epsilon, double kv_en, double delta, double wn);
};

#endif