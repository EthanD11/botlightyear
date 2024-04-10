#include <stdint.h>

class DE0_feedback
{
private:
    /* data */
    uint8_t claim_gpio(feedback_GPIO_DE0_t gpio_pin); 
    void free_gpio(feedback_GPIO_DE0_t gpio_pin);
public:
    DE0_feedback(/* args */);
    ~DE0_feedback();
    uint8_t init();
    void close();
    void wait_for_gpio_value(feedback_GPIO_DE0_t gpio, uint8_t val); 
};

DE0_feedback::DE0_feedback(/* args */)
{
}

DE0_feedback::~DE0_feedback()
{
}

typedef enum {
    StprSliderGPIO = 0, 
    StprPlateGPIO = 5, 
    StprFlapsGPIO = 6, 
    BpSwitchFlapsLeftGPIO = 13, 
    BpSwitchFlapsRightGPIO = 19
} feedback_GPIO_DE0_t;
