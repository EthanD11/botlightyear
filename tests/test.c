#include <lgpio.h>

int main(int argc, char const *argv[])
{
    int handle = lgGpiochipOpen(0);
    if (handle < 0) return 1;
    int err = lgGpioClaimOutput(handle, 0, 18, LG_HIGH);
    if (err < 0) return 1;
    lguSleep(1);
    // GPIO LOW
    if (lgGpioWrite(handle, 18, LG_LOW) < 0) return 1;
    lgGpioFree(handle, 18);
    lgGpiochipClose(handle);
    return 0;
}
