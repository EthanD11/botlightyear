#ifndef _BLY_SERVOS_H_
#define _BLY_SERVOS_H_

#include "SPI_bus.h"
#include <stdint.h>

class Flaps : public SPIUser
{
private:

void send_flaps_dutyCycle(uint16_t servo_flaps1_duty_cycle, uint16_t servo_flaps2_duty_cycle); 
public:
    /** @brief Servomotors flaps object.
    * Uses SPI to control the two flaps servomo together
    */
    Flaps(SPIBus *bus) : SPIUser(bus) {}
    /** @brief Deploys and holds the flaps down
    */
    void deploy();
    /** @brief Raises and holds the flaps up at 90 degrees
    */
    void raise();
    /** @brief Idles the flaps, no dutycycle is sent
    */
    void idle();
};

class GripperDeployer : public SPIUser
{
private:
void send_dutyCycle(uint16_t duty_cycle); 
public:
    /** @brief Servomotor gripper deployer object.
    * Uses SPI to control the servomotor related to the rotational degree of freedom of the gripper
    */
    GripperDeployer(SPIBus *bus) : SPIUser(bus) {}
    /** @brief Idles the gripper deployer servo, no dutycycle is sent
    */
    void idle();
    /** @brief Raises the gripper at a 45 degrees position and holds it
    */
    void half();
    /** @brief Raises the gripper slightly for pot deposit and holds its position
    */
    void pot_deposit();
    /** @brief Deploys the gripper down and holds its position
    */
    void deploy();
    /** @brief Raises the gripper at a 70 degrees position and holds it
    */
    void raise();
    /** @brief Raises the gripper slightly for plant lift and holds its position
    */
    void plantLift();
};

class GripperHolder : public SPIUser
{
private:
void send_dutyCycle(uint16_t duty_cycle); 
public:
    /** @brief Servomotor gripper holder object.
    * Uses SPI to control the servomotor related to gripper opening/closing
    */
    GripperHolder(SPIBus *bus) : SPIUser(bus) {}
    /** @brief Idles the gripper holder, no dutycycle is sent
    */
    void idle();
    /** @brief Closes the gripper and holds that position
    */
    void close(); 
    /** @brief Opens the gripper to let go plants/pots and holds that position
    */
    void open(); 
    /** @brief Opens the gripper fully up to mechanical stop and holds that position
    */
    void open_full(); 
    /** @brief Closes the gripper enough to hold a pot and holds that position
    */
    void hold_pot(); 
    /** @brief Closes the gripper enough to hold a plant and holds that position
    */
    void hold_plant(); 
};

#endif