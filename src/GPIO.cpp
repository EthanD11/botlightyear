#include "GPIO.h"
#include <lgpio.h>
#include <stdlib.h>

#define PI5

DE0_feedback::DE0_feedback() {

    #ifdef PI5
    int SPI2_handle = lgGpiochipOpen(4);
    #else
    int SPI2_handle = lgGpiochipOpen(1);
    #endif
    if (SPI2_handle < 0) exit(1);
    if (lgGpioSetUser(SPI2_handle, "Bot Lightyear") < 0) exit(2);

    this->claim_gpio(StprSliderGPIO); 
    this->claim_gpio(StprPlateGPIO); 
    this->claim_gpio(StprFlapsGPIO); 
    this->claim_gpio(BpSwitchFlapsLeftGPIO); 
    this->claim_gpio(BpSwitchFlapsRightGPIO); 
    return 0;
}

void DE0_feedback::~DE0_feedback() {
    this->free_gpio(StprSliderGPIO); 
    this->free_gpio(StprPlateGPIO); 
    this->free_gpio(StprFlapsGPIO); 
    this->free_gpio(BpSwitchFlapsLeftGPIO); 
    this->free_gpio(BpSwitchFlapsRightGPIO); 
    lgGpiochipClose(SPI2_handle);
}

uint8_t DE0_feedback::claim_gpio(feedback_GPIO_DE0_t gpio_pin) {
    if (lgGpioClaimInput(SPI2_handle, LG_SET_PULL_NONE, gpio_pin) != 0) return 1;
    return 0;
}

void DE0_feedback::free_gpio(feedback_GPIO_DE0_t gpio_pin) {
    lgGpioFree(SPI2_handle, gpio_pin);
}

void DE0_feedback::wait_for_gpio_value(feedback_GPIO_DE0_t gpio, uint8_t val) {
    lguSleep(0.001); // Sleep to avoid timing conflicts with spi message
    int readValue;
    do {
        readValue = lgGpioRead(SPI2_handle,4);
        if (readValue < 0) {
            printf("Error : gpio %d not readable \n", stepper_gpio); 
            return;
        }
        lguSleep(0.001);
    } while (readValue != val);
}