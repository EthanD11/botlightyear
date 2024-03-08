#include "SPI_Modules.h"

int main(int argc, char const *argv[])
{
    if (init_spi() != 0) return -1;
    servo_deploy();
    lguSleep(3);
    servo_raise();
    lguSleep(1);
    servo_idle();
    close_spi();
    return 0;
}
