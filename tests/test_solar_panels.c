#include "solar_panels.h"
#include <unistd.h>

int main(int argc, char const *argv[])
{
    init_port();
    deployP(); 
    sleep(0.5);
    multi_turnP();
    sleep(0.5);
    raiseP();
    close_port();
    return 0;
}