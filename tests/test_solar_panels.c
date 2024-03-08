#include "solar_panels.h"
#include <unistd.h>

int main(int argc, char const *argv[])
{
    init_port();
    /*deployP(); 
    sleep(0.5);
    multi_turnP();
    sleep(0.5);
    raiseP();
    //ping_dxl(8, 1.0);*/

    /*
    openG_withoutreturn(); 
    sleep(1); 
    closeG_withoutreturn();*/

    //raiseG_withoutreturn(); 
    //sleep(1); 
    deployG_withoutreturn();

    close_port();
    return 0;
}