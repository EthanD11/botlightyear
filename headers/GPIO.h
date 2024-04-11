#ifndef BLY_GPIO_H
#define BLY_GPIO_H

#include <stdint.h>


typedef enum {
    StprSliderGPIO = 25, 
    StprPlateGPIO = 1, 
    StprFlapsGPIO = 12, 
    BpSwitchFlapsLeftGPIO = 16, 
    BpSwitchFlapsRightGPIO = 20,
    TeensyA1 = 26, // A1 = J13
    TeensyA2 = 19, // A2 = J14
    TeensyA3 = 13  // A3 = J15
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
    void wait_for_gpio_value(GPIO_t gpio, uint8_t val, uint32_t msMaxWait = 5000);
    int8_t read(GPIO_t gpio);
};

class GPIOUser
{
public:
    GPIOPins *pins;
    GPIOUser(GPIOPins *pins);
    ~GPIOUser();
};

#endif