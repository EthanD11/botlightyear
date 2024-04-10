#include "SPI_bus.h"
#include <stdio.h>

SPIBus bus = SPIBus();

int main(int argc, char const *argv[])
{
    /*int Teensy_Handle = lgSpiOpen(0, SPI_TEENSY, SPI_SPEED_HZ_DEFAULT, 0);
    if (Teensy_Handle < 0) return -1;
    char send[] = {6,0,0,0,0};
    char receive[5];
    lgSpiXfer(Teensy_Handle, send, receive, 1);
    printf("%d\n", receive[0]);
    send[0] = 7;
    lgSpiXfer(Teensy_Handle, send, receive, 1);
    printf("%d\n", receive[0]);
    send[0] = 7;
    lgSpiXfer(Teensy_Handle, send, receive, 1);
    printf("%d\n", receive[0]);
    send[0] = 8;
    lgSpiXfer(Teensy_Handle, send, receive, 1);
    printf("%d\n", receive[0]);
    send[0] = 0;
    lgSpiXfer(Teensy_Handle, send, receive, 1);
    printf("%d\n", receive[0]);
    lgSpiClose(Teensy_Handle);*/

    return bus.test();

}
