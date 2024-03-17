#include "SPI_Modules.h"
#include <stdio.h>
#include <unistd.h>

int main(int argc, char const *argv[])
{
    
    init_spi();
    odo_reset();
    int left, right;
    printf("Place left and right odos between 10 000 and 100 000\n");
    while(1){
        odo_get_tick(&left,&right);
        printf("left : %d right : %d\r\n", left, right);
        lguSleep(5e-3);
        fflush(stdout);
        lguSleep(5e-3);
        
        if (left > 10000 && left < 100000 && right > 10000 && right < 100000) break;

    }

    return 0;

}
