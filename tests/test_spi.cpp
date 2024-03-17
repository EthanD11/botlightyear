#include <SPI_Modules.h>

int main(int argc, char const *argv[])
{
    int Teensy_Handle = lgSpiOpen(0, SPI_TEENSY, SPI_SPEED_HZ_DEFAULT, 0);
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
    lgSpiClose(Teensy_Handle);

    int DE0_Handle = lgSpiOpen(0, SPI_DE0, SPI_SPEED_HZ_DEFAULT, 0);
    if (DE0_Handle < 0) return -1;
    send[0] = 0x00; send[1] = 0x00; send[2] = 0x00; send[3] = 0x00; send[4] = 0x00;

    lgSpiXfer(DE0_Handle, send, receive, 5);
    for (int i = 0; i < 5; i++)
    {
        if (receive[i] != i) {
            printf("SPI test 1 failed : receive[%d] == %d != %d\n",i,receive[i],i);
        }
    }
    
    // store in register
    send[0] = 0x8F; send[1] = 0x05; send[2] = 0x04; send[3] = 0x03; send[4] = 0x02;
    lgSpiXfer(DE0_Handle, send, receive, 5);

    // read register
    send[0] = 0x0F; send[1] = 0x00; send[2] = 0x00; send[3] = 0x00; send[4] = 0x00;
    lgSpiXfer(DE0_Handle, send, receive, 5);
    if (receive[0] != 0x00) {printf("SPI test 2 failed : receive[%d] == %d != %d\n",0,receive[0],0x00);}
    if (receive[1] != 0x05) {printf("SPI test 2 failed : receive[%d] == %d != %d\n",1,receive[1],0x05); }
    if (receive[2] != 0x04) {printf("SPI test 2 failed : receive[%d] == %d != %d\n",2,receive[2],0x04); }
    if (receive[3] != 0x03) {printf("SPI test 2 failed : receive[%d] == %d != %d\n",3,receive[3],0x03); }
    if (receive[4] != 0x02) {printf("SPI test 2 failed : receive[%d] == %d != %d\n",4,receive[4],0x02); }

    lgSpiClose(DE0_Handle);
    return 0;
}
