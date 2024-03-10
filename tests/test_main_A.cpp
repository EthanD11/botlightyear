#include "../Dynamixel-XL_320-Lib/XL_320.h"

using namespace std;

int main(int argc, char **argv)
{

    XL_320 Servo;
    Servo.verbose = true;

    // --- Timing optimization

    /*
     * Do this only once per Servo, the result is stored in the EEPROM
     */

    // Servo.setTorqueEnable(0);
    // Servo.setReturnDelayTime(100);
    // Servo.setStatusReturnLevel(1);
    // Servo.setTorqueEnable(0);

    // --- Commands

    // Test ping
    Servo.ping();

    // Test LED colors
    for (int i=0; i<8; i++) {
        Servo.setLED(i);
        usleep(1000000);
    }

    return 0;
}
