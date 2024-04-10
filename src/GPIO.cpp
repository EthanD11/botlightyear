#include "GPIO.h"
#include <lgpio.h>
#include <stdlib.h>

//#define PI5

GPIOPins::GPIOPins() {

    #ifdef PI5
    int handle = lgGpiochipOpen(4);
    #else
    int handle = lgGpiochipOpen(0);
    #endif
    if (handle < 0) exit(1);
    if (lgGpioSetUser(handle, "Bot Lightyear") < 0) exit(2);

    this->claim_gpio(StprSliderGPIO); 
    this->claim_gpio(StprPlateGPIO); 
    this->claim_gpio(StprFlapsGPIO); 
    this->claim_gpio(BpSwitchFlapsLeftGPIO); 
    this->claim_gpio(BpSwitchFlapsRightGPIO); 
}

GPIOPins::~GPIOPins() {
    this->free_gpio(StprSliderGPIO); 
    this->free_gpio(StprPlateGPIO); 
    this->free_gpio(StprFlapsGPIO); 
    this->free_gpio(BpSwitchFlapsLeftGPIO); 
    this->free_gpio(BpSwitchFlapsRightGPIO); 
    lgGpiochipClose(handle);
}

uint8_t GPIOPins::claim_gpio(GPIO_t gpio_pin) {
    if (lgGpioClaimInput(handle, LG_SET_PULL_NONE, gpio_pin) != 0) return 1;
    return 0;
}

void GPIOPins::free_gpio(GPIO_t gpio_pin) {
    lgGpioFree(handle, gpio_pin);
}

void GPIOPins::wait_for_gpio_value(GPIO_t gpio, uint8_t val) {
    lguSleep(0.001); // Sleep to avoid timing conflicts with spi message
    int readValue;
    do {
        readValue = lgGpioRead(handle,4);
        if (readValue < 0) {
            printf("Error : gpio %d not readable \n", gpio); 
            return;
        }
        lguSleep(0.001);
    } while (readValue != val);
}

GPIOUser::GPIOUser(GPIOPins *pins)
{
    this->pins = pins;
}

GPIOUser::~GPIOUser()
{
    if (pins == NULL) return;
    pins->~GPIOPins();
    pins = NULL;
}