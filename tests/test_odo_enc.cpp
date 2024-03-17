#include "SPI_Modules.h"
#include <stdio.h>
#include <unistd.h>

int main(int argc, char const *argv[])
{
    
    init_spi();
    odo_enc_reset();
    int left, right;
    printf("Place left and right odos between 10 000 and 100 000\n");
    while(1){
        get_odo_tick(&left,&right);
        printf("left : %d right : %d\r\n", left, right);
        lguSleep(5e-3);
        fflush(stdout);
        lguSleep(5e-3);
        
        if (left > 10000 && left < 100000 && right > 10000 && right < 100000) break;

    }

    /*while(1){
        printf("Place left and right speed between 100 and 1 000\r\n");
        get_enc_spd(&left,&right);
        printf("left : %d right : %d\r\n", left, right);
        usleep(50000);
        fflush(stdout);
        usleep(50000);
        
        if (left > 100 && left < 1000 && right > 100 && right < 1000) break;

    }*/

    return 0;

}
