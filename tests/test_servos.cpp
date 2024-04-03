#include "SPI_Modules.h"

int main(int argc, char const *argv[])
{
    if (init_spi() != 0) return -1;
    flaps_servo_cmd(FlapsDeploy);
    lguSleep(3);
    flaps_servo_cmd(FlapsRaise);
    lguSleep(1);
    //flaps_servo_cmd(FlapsIdle);
    close_spi();
    return 0;
}
