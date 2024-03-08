#include "SPI_Modules.h"

int main(int argc, char const *argv[])
{
    int DE0_handle = lgSpiOpen(0, SPI_DE0, SPI_SPEED_HZ_DEFAULT, SPI_MODE_DEFAULT);
    //char send[] = {0x83,0x40,0,0,0};
    //char send[] = {0x83,0x80,0,0,0};
    char send[] = {0x83,0xC0,0,0,0}; // Reset position to 0
    //char send[] = {0x83,0xC0,0,0b00000110,0b01000000}; // FULL TURN = 1600 steps pour stepper d'Arnaud (17hs16-2004s1)
    //char send[] = {0x83,0xC0,0,0b00011001,0}; // 4 full turns to test precision
    //char send[] = {0x83,0xC0,0,0b00110010,0}; // 8 full turns to test precision
    //char send[] = {0x83,0xC0,0,0b01100100,0}; // 16 full turns to test precision
    //char send[] = {0x83,0xDF,0xFF,0xF9,0xC0 }; // -1 full turn = -1600

    lgSpiWrite(DE0_handle, send, 5);
    lgSpiClose(DE0_handle);
    return 0;
}
