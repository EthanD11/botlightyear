#include "SPI_Modules.h"
#include <termios.h>
#define ESC_ASCII_VALUE                 0x1b

int getch()
{
#if defined(__linux__) || defined(__APPLE__)
  struct termios oldt, newt;
  int ch;
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  ch = getchar();
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  return ch;
#elif defined(_WIN32) || defined(_WIN64)
  return _getch();
#endif
}

int main(int argc, char const *argv[])
{

    int pin = 23;

    int handle = lgGpiochipOpen(4);
    if (handle < 0) exit(1);
    lgChipInfo_t chipinfo;
    if (lgGpioGetChipInfo(handle,&chipinfo) < 0) exit(1);
    printf("lines=%d name=%s label=%s\n", chipinfo.lines, chipinfo.name, chipinfo.label);

    if (lgGpioSetUser(handle, "Bot Lightyear") < 0) exit(2);

    if (lgGpioClaimInput(handle, LG_SET_PULL_NONE, pin) != 0) exit(3);

    lgLineInfo_t lInfo;
    if (lgGpioGetLineInfo(handle, pin, &lInfo) < 0) exit(4);
    printf("lFlags=%d name=%s user=%s offset=%d\n", lInfo.lFlags, lInfo.name, lInfo.user, lInfo.offset);

    int start;
    do {
        start = lgGpioRead(handle,pin);
        printf("%d\n",start);
        if (start < 0) exit(5);
        printf("Press any key for next read, or ESC to stop\n");
        if (getch() == ESC_ASCII_VALUE)
            break;
    } while (1);
    
    lgGpioFree(handle, pin);
    lgGpiochipClose(handle);

    exit(0);
}