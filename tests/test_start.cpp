#include "SPI_Modules.h"
#include <stdio.h>
int main(int argc, char const *argv[])
{
    //Init Starting Switch

    /*lgChipInfo_t chipinfo;
    lgLineInfo_t lInfo;    
    for (int i = 0; i < 6; i++)
    {
        int handle = lgGpiochipOpen(i);
        if (handle < 0) exit(1);
        if (lgGpioGetChipInfo(handle,&chipinfo) < 0) exit(1);
        printf("lines=%d name=%s label=%s\n", chipinfo.lines, chipinfo.name, chipinfo.label);

        for (uint32_t j = 0; j < chipinfo.lines; j++)
        {
            if (lgGpioGetLineInfo(handle, j, &lInfo) < 0) exit(1);
            printf("lFlags=%d name=%s user=%s offset=%d\n", lInfo.lFlags, lInfo.name, lInfo.user, lInfo.offset);
        }
        lgGpiochipClose(handle);
    }*/

    int handle = lgGpiochipOpen(4);
    if (handle < 0) exit(1);
    lgChipInfo_t chipinfo;
    if (lgGpioGetChipInfo(handle,&chipinfo) < 0) exit(1);
    printf("lines=%d name=%s label=%s\n", chipinfo.lines, chipinfo.name, chipinfo.label);

    if (lgGpioSetUser(handle, "Bot Lightyear") < 0) exit(2);

    if (lgGpioClaimInput(handle, LG_SET_PULL_NONE, 4) != 0) exit(3);

    lgLineInfo_t lInfo;
    if (lgGpioGetLineInfo(handle, 4, &lInfo) < 0) exit(4);
    printf("lFlags=%d name=%s user=%s offset=%d\n", lInfo.lFlags, lInfo.name, lInfo.user, lInfo.offset);

    printf("Waiting for setup... \n");

    int start;
    do {
        start = lgGpioRead(handle,4);
        if (start < 0) exit(5);
    } while (start);
    printf("Starting cord setup\n");
    
    lguSleep(1);

    printf("Waiting start of the game... \n");
    do {
        start = lgGpioRead(handle,4);
        if (start < 0) exit(5);
    } while (!start);

    lgGpioFree(handle, 4);
    lgGpiochipClose(handle);

    printf("Game started! \n");

    exit(0);
}