#include "solar_panels.h"
#include <unistd.h>

int main(int argc, char const *argv[])
{
    init_port();
    sleep(1);
    deploy(); 
    sleep(1);
    multi_turn(); 
    sleep(10);
    idle(); 
    sleep(1);
    raise();
    sleep(1);
    close_port();
    return 0;
}