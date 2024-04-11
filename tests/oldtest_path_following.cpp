#include "teensy.h"

SPIBus spiBus = SPIBus();
GPIOPins pins = GPIOPins(); 
Teensy teensy = Teensy(&spiBus, &pins);

int main(int argc, char const *argv[]){
    int ncheckpoints = 4;
    double x[4] = {1,2,3,4};
    double y[4] = {4,3,2,1};
    double theta = 0.0;

    teensy.path_following(x, y, ncheckpoints, theta);
}