#include "SPI_bus.h"
#include <stdio.h>

SPIBus spiBus = SPIBus();

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

    char send[] = {0x7E,0,0,0,0};
    for (int i = 0; i < 1000; i++) {
        //char receive[5];
        spiBus.DE0_write(send);
        //send[0] = 0x1F;
        //spiBus.DE0_xfer(send, receive);
        //printf("%x,%x,%x,%x,%x\n", receive[0], receive[1], receive[2], receive[3], receive[4]);
        spiBus.test();
        //if (spiBus.test() != 0) return 1;
    }

    return 0;

}
