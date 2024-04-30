#ifndef BLY_GPIO_H
#define BLY_GPIO_H

#include <stdint.h>


typedef enum {
    StartingCordGPIO = 4,
    StprSliderGPIO = 25, 
    StprPlateGPIO = 1, 
    StprFlapsGPIO = 12, 
    BpSwitchFlapsLeftGPIO = 16, 
    BpSwitchFlapsRightGPIO = 20,
    TeensyA1 = 13, // A1 = J13
    TeensyA2 = 19, // A2 = J14
    TeensyA3 = 26  // A3 = J15
} GPIO_t;

class GPIOPins
{
private:
    int handle;
    uint8_t claim_gpio(GPIO_t gpio_pin); 
    void free_gpio(GPIO_t gpio_pin);
public:
    GPIOPins();
    ~GPIOPins();
    /**
     * @brief Waits until the gpio has the value val
     * @param gpio the gpio to read
     * @param val the value to wait (0 for LOW, 1 for HIGH)
     * @param msMaxWait maximum delay time to wait for the value in ms (by default 5000ms)
     * @return 0 for completion, -1 if time fail
    */
    int8_t wait_for_gpio_value(GPIO_t gpio, uint8_t val, uint32_t msMaxWait = 5000);
    /**
     * @brief Waits until the gpio has the value val
     * @param gpio the gpio to read
     * @return the read value (0 for LOW, 1 for HIGH)
    */
    int8_t read(GPIO_t gpio);
};

/*
Class defining all the GPIO users' methods
When using a GPIO, a class needs to extend this one
*/
class GPIOUser
{
public:
    GPIOPins *pins;
    GPIOUser(GPIOPins *pins);
    ~GPIOUser();
};

#endif