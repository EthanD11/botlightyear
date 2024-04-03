#include "SPI_Modules.h"

int main(int argc, char const *argv[])
{
    if (init_spi() != 0) return -1;
    servo_cmd(ServoDeploy);
    lguSleep(3);
    servo_cmd(ServoRaise);
    lguSleep(1);
    servo_cmd(ServoIdle);
    close_spi();
    return 0;
}
