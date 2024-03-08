#include "solar_panels.h"
#include<unistd.h>

int main(int argc, char const *argv[])
{
    deploy(); 
    sleep(1);
    multi_turn(); 
    sleep(10);
    idle(); 
    sleep(1);
    raise();
    return 0;
}