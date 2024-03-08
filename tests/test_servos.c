#include "SPI_Modules.h"

int main(int argc, char const *argv[])
{
    if (init_spi() != 0) return -1;
    printf("%d\n", servo_toggle());
    lguSleep(3);
    printf("%d\n", servo_toggle());
    lguSleep(1);
    servo_idle();
    close_spi();
    return 0;
}
