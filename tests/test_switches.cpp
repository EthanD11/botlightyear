#include "SPI_Modules.h"

int main(int argc, char const *argv[])
{
    int DE0_handle = lgSpiOpen(0, SPI_DE0, SPI_SPEED_HZ_DEFAULT, SPI_MODE_DEFAULT);
    char send[] = {0x84,0,0,0,1};
    lgSpiWrite(DE0_handle, send, 5);
    lgSpiClose(DE0_handle);
    return 0;
}
