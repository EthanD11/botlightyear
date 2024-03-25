#include "SPI_Modules.h"

int main(int argc, char const *argv[])
{
    //Init Starting Switch
    int handle = lgGpiochipOpen(0);
    if (handle < 0) exit(1);

    if (lgGpioClaimInput(handle, 0, 25) != 0) {
        printf("Cannot set GPIO to input \n");
        exit(1);
    }
    if (lgGpioClaimOutput(handle, 0, 16) != 0) {
        printf("Cannot set GPIO to input \n");
        exit(1);
    }

    lgGpioWrite(handle, 16, 1);
    lguSleep(1);

    int start = lgGpioRead(handle,25);
    printf("Initial value of GPIO: %d \n", start); 
    status = lgGpioGetMode(handle, 25); 


    printf("Waiting start of the game... \n");

    //Wait for starting switch
    int i = 0;
    while (start == 1) {
        start = lgGpioRead(handle,25);
        printf("Waiting %d, GPIO %d \n", i, start); 
        sleep(1);
        i++;
    }

    printf("Game started! \n");

    exit(0);
}