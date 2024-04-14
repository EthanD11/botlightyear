#include "GPIO.h"
#include <lgpio.h>
#include <stdlib.h>
#include <time.h>
#include <stdio.h>

#define PI5

GPIOPins::GPIOPins() {

    #ifdef PI5
    handle = lgGpiochipOpen(4);
    #else
    handle = lgGpiochipOpen(0);
    #endif
    if (handle < 0) exit(1);
    if (lgGpioSetUser(handle, "Bot Lightyear") < 0) exit(2);

    if (this->claim_gpio(StartingCordGPIO) != 0) exit(3); 
    if (this->claim_gpio(StprSliderGPIO) != 0) exit(3); 
    if (this->claim_gpio(StprPlateGPIO) != 0) exit(3); 
    if (this->claim_gpio(StprFlapsGPIO) != 0) exit(3); 
    if (this->claim_gpio(BpSwitchFlapsLeftGPIO) != 0) exit(3); 
    if (this->claim_gpio(BpSwitchFlapsRightGPIO) != 0) exit(3);
    if (this->claim_gpio(TeensyA1) != 0) exit(3);
    if (this->claim_gpio(TeensyA2) != 0) exit(3);
    if (this->claim_gpio(TeensyA3) != 0) exit(3);
}

GPIOPins::~GPIOPins() {
    this->free_gpio(StartingCordGPIO); 
    this->free_gpio(StprSliderGPIO); 
    this->free_gpio(StprPlateGPIO); 
    this->free_gpio(StprFlapsGPIO); 
    this->free_gpio(BpSwitchFlapsLeftGPIO); 
    this->free_gpio(BpSwitchFlapsRightGPIO);
    this->free_gpio(TeensyA1);
    this->free_gpio(TeensyA2);
    this->free_gpio(TeensyA3);
    lgGpiochipClose(handle);
}

uint8_t GPIOPins::claim_gpio(GPIO_t gpio_pin) {
    if (lgGpioClaimInput(handle, LG_SET_PULL_NONE, gpio_pin) != 0) return 1;
    return 0;
}

void GPIOPins::free_gpio(GPIO_t gpio_pin) {
    lgGpioFree(handle, gpio_pin);
}

void GPIOPins::wait_for_gpio_value(GPIO_t gpio, uint8_t val, uint32_t msMaxWait) {
    int readValue;
    uint32_t msElapsed;
    struct timespec start_ts;
    struct timespec now_ts;
    clock_gettime(CLOCK_BOOTTIME, &start_ts);
    lguSleep(0.001); // Sleep to avoid timing conflicts with spi message
    do {
        readValue = lgGpioRead(handle,gpio);
        if (readValue < 0) printf("Error : gpio %d not readable \n", gpio); 
        lguSleep(0.001);
        clock_gettime(CLOCK_BOOTTIME, &now_ts);
        msElapsed = (now_ts.tv_sec - start_ts.tv_sec) * 1000 + (uint32_t) ((now_ts.tv_nsec-start_ts.tv_nsec)/1000000);
    } while (readValue != val && msElapsed < msMaxWait);
}

int8_t GPIOPins::read(GPIO_t gpio) {
    for (int8_t i = 0; i < 5; i++) {
        //printf("Handle %d \n", handle); 
        int res = lgGpioRead(handle, gpio);
        if (res >= 0) return (int8_t) res;
        printf("Error : gpio %d not readable \n", gpio);
        lguSleep(0.001);
    }
    return -1;
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