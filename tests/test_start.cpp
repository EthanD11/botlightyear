#include "SPI_Modules.h"

int main(int argc, char const *argv[])
{
    //Init Starting Switch
    int handle = lgGpiochipOpen(0);
    if (handle < 0) exit(1);
    if (lgGpioClaimInput(handle, 0, 4) != 0) {
        printf("Cannot set GPIO to input \n");
        exit(1);
    }
    int start = lgGpioRead(handle, 4);
    printf("Initial value of GPIO: %d \n", start);

    printf("Waiting start of the game... \n");
    int i = 0;
    //Wait for starting switch
    while (start == 0) {
        start = lgGpioRead(handle, 4);
        printf("Waiting %d, GPIO %d \n", i, start); 
        sleep(1);
        i++;
    }

    printf("Game started! \n");

    exit(0);
}