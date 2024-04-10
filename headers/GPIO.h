#include <stdint.h>


typedef enum {
    StprSliderGPIO = 0, 
    StprPlateGPIO = 5, 
    StprFlapsGPIO = 6, 
    BpSwitchFlapsLeftGPIO = 13, 
    BpSwitchFlapsRightGPIO = 19
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
    void wait_for_gpio_value(GPIO_t gpio, uint8_t val); 
};

class GPIOUser
{
public:
    GPIOPins *pins;
    GPIOUser(GPIOPins *pins);
    ~GPIOUser();
};
