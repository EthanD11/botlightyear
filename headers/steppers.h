#ifndef BLY_STEPPERS_H
#define BLY_STEPPERS_H

#include "SPI_bus.h"
#include "GPIO.h"
#include <stdint.h>

#define CALL_BLOCKING 1
#define CALL_NON_BLOCKING 0

/*The name of a stepper*/
typedef enum {
    StprPlate, 
    StprSlider, 
    StprFlaps
} steppers_t;

/*The poisition of a flaps*/
typedef enum {
    FlapsOpen, 
    FlapsPlant, 
    FlapsPot
} flaps_pos_t;

typedef enum {
    SliderLow,
    SliderHigh,
    SliderStorage,
    //SliderPlate, 
    //SliderTake, 
    SliderDepositPot, 
    SliderIntermediateLow
} slider_pos_t;

class Steppers : public SPIUser, public GPIOUser
{
public:

    Steppers(SPIBus *bus, GPIOPins *pins) : SPIUser(bus), GPIOUser(pins) {}

    /**
     * @brief Move the stepper 'stepperName' of 'steps' steps in the 'neg' direction (0 for positive, 1 for negative)
     * @param stepperName the stepper to calibrate (StprPlate, StprSlider or StprFlaps)
     * @param steps the step count to go to
     * @param neg the direction of the step count (0 for positive, 1 for negative)
     * @param blocking CALL_BLOCKING if the command is blocking (waits until finished), CALL_NON_BLOCKING if it's non-blocking
     * (non-blocking by default)
    */
    void move(steppers_t stepperName, uint32_t steps, uint8_t neg, uint8_t blocking = CALL_NON_BLOCKING);

    /**
     * @brief Move flaps stepper to 'pos' (FlapsOpen, FlapsPlant or FlapsPot)
     * @param pos the position of the flaps stepper (FlapsOpen, FlapsPlant or FlapsPot)
     * @param blocking CALL_BLOCKING if the command is blocking (waits until finished), CALL_NON_BLOCKING if it's non-blocking
     * (non-blocking by default)
    */
    void flaps_move(flaps_pos_t pos, uint8_t blocking = CALL_NON_BLOCKING);

    /**
     * @brief Move slider stepper to 'pos' (SliderLow, SliderHigh, SliderPlate or SliderTake)
     * @param pos the position of the slider stepper (SliderLow, SliderHigh, SliderPlate or SliderTake)
     * @param blocking CALL_BLOCKING if the command is blocking (waits until finished), CALL_NON_BLOCKING if it's non-blocking
     * (non-blocking by default)
    */
    void slider_move(slider_pos_t pos, uint8_t blocking = CALL_NON_BLOCKING);


    /**
     * @brief Move plate to slot number 'slot' ([-3 ; 3], 0 is neutral)
     * @param blocking CALL_BLOCKING if the command is blocking (waits until finished), CALL_NON_BLOCKING if it's non-blocking 
     * (non-blocking by default)
    */
    void plate_move(int8_t slot, uint8_t blocking = CALL_NON_BLOCKING);

    /**
     * @brief Activates the calibration of given stepper
     * @param stepperName the stepper to calibrate (StprPlate, StprSlider or StprFlaps)
     * @param blocking CALL_BLOCKING if the command is blocking (waits until finished), CALL_NON_BLOCKING if it's non-blocking
     * (non-blocking by default)
     */
    void calibrate(steppers_t stepperName, uint8_t blocking = CALL_NON_BLOCKING); 



    /**
     * @brief Sets the nominal and initial speed of the stepper
     * @param stepperName the stepper to setup the speed(StprPlate, StprSlider or StprFlaps)
     * @param nominalSpeed is the upper bound of the speed limit for the stepper's speed
     * @param initialSpeed is the initial speed that the stepper begins with at each movement
     */
    void setup_speed(steppers_t stepperName, int nominalSpeed, int initialSpeed); 

    /**
     * @brief Sets the calibration speed and the small calibration speed of the stepper
     * @param stepperName the stepper to setup the calibration speed (StprPlate, StprSlider or StprFlaps)
     * @param calibrationSpeed is the nominal speed in calibration mode for the first calibration step
     * @param smallCalibrationSpeed is the smaller speed used in the second calibration step
     */
    void setup_calib_speed(steppers_t stepperName, int calibrationSpeed, int smallCalibrationSpeed);

    /**
     * @brief Resets stepper module to be ready for another calibration
     * @param stepperName the stepper to reset (StprPlate, StprSlider or StprFlaps)
     */
    void reset(steppers_t stepperName); 

    /**
     * @brief Sets up the acceleration of a stepper (lower 'acc' -> higher acceleration)
     * @param stepperName the stepper to setup the acceleration (StprPlate, StprSlider or StprFlaps)
     * @param acc_steps the acceleration counter of the stepper. At each "acc" steps, the stepper will increase its speed from its initial speed towards the nominal speed
    */
    void setup_acc(steppers_t stepperName, uint8_t accSteps);

    /**
     * @brief Calibrates all the steppers
    */
    void calibrate_all(uint8_t blocking = CALL_NON_BLOCKING);

    /**
     * @brief Resets all the steppers
    */
    void reset_all();
};

#endif