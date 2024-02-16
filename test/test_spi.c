#include "SPI_Modules.h"
#include <stdio.h>

int main(int argc, char const *argv[])
{
    if (gpioInitialise() < 0) return -1;
    int handle = spiOpen(SPI_DE0, SPI_SPEED_HZ_DEFAULT, SPI_MODE_DEFAULT);
    char send[] = {0,0,0,0,0};
    char receive[5];

    spiXfer(handle, send, receive, 5);
    for (int i = 0; i < 5; i++)
    {
        if (receive[i] != i) {
            printf("spi_test failed : receive[%d] == %d != %d\n",i,receive[i],i);
            return -1;
        }
    }
    
    // store in register
    send[0] = 0x8F; send[1] = 0x05; send[2] = 0x04; send[3] = 0x03; send[4] = 0x02;
    spiXfer(handle, send, receive, 5);

    // read register
    send[0] = 0x0F; send[1] = 0x00; send[2] = 0x00; send[3] = 0x00; send[4] = 0x00;
    spiXfer(handle, send, receive, 5);
    if (receive[0] != 0x00) {printf("spi_test failed : receive[%d] == %d != %d\n",0,receive[0],0x00); return -1;}
    if (receive[1] != 0x05) {printf("spi_test failed : receive[%d] == %d != %d\n",1,receive[1],0x05); return -1;}
    if (receive[2] != 0x04) {printf("spi_test failed : receive[%d] == %d != %d\n",2,receive[2],0x04); return -1;}
    if (receive[3] != 0x03) {printf("spi_test failed : receive[%d] == %d != %d\n",3,receive[3],0x03); return -1;}
    if (receive[4] != 0x02) {printf("spi_test failed : receive[%d] == %d != %d\n",4,receive[4],0x02); return -1;}

    spiClose(handle);
    gpioTerminate();
    return 0;
}
