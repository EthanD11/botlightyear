#ifndef BLY_SHARED_H
#define BLY_SHARED_H

#define TOTAL_GAME_TIME 600

#include "colors.h"
#include "SPI_bus.h"
#include "GPIO.h"
#include "steppers.h"
#include "servos.h"
#include "graph.h"
#include "odometry.h"
#include "teensy.h"
#include <time.h>
#include <stdint.h>

typedef enum _storage_slot : int8_t
{
    SlotM3,
    SlotM2,
    SlotM1,
    Slot3,
    Slot2,
    Slot1,
    SlotGripper,
    SlotFlaps, 
    SlotInvalid = -1
} storage_slot_t;

/**
 * A storage_content_t is a three bit binary word 0bCBA
 * A is true if the slot contains a pot (empty or not), false otherwise
 * B is true if the slot contains a plant (in a pot or not), false otherwise
 * C is true if the slot contains a weak plant, false otherwise
*/
typedef enum _storage_content : uint8_t
{
    ContainsNothing = 0b000,
    ContainsPot = 0b001,
    ContainsStrongPlant = 0b010,
    ContainsWeakPlant = 0b110,
    ContainsStrongPlantInPot = 0b011,
    ContainsWeakPlantInPot = 0b111
} storage_content_t;

class SharedVariables
{
private:
    double x, y, theta; // Current robot position
    double xAdv, yAdv, dAdv, aAdv; // Current adversary position
    time_t tStart; // Game start time
public:
    /** @brief The shared variables object. Contains common variables, objects and metchods usable by files
    */
    SharedVariables();
    ~SharedVariables();

    /** @brief Get the current robot position
     * Usage : `double x, y, theta; get_robot_pos(&x,&y,&theta);`
     * @param x The pointer to fill the x position of the robot
     * @param y The pointer to fill the y position of the robot
     * @param theta The pointer to fill the orientation of the robot
     * If any of the three pointer is NULL, it will not be filled
    */
    void get_robot_pos(double *x, double *y, double *theta);
    /** @brief Set the current robot position.
     * @param x The x position of the robot
     * @param y The y position of the robot
     * @param theta The orientation of the robot
    */
    void set_robot_pos(double x, double y, double theta);  
    /** @brief Get the current adversary's position.
     * Usage : `double xAdv, yAdv, thetaAdv; get_adv_pos(&xAdv,&yAdv,&thetaAdv);`
     * @param x The pointer to fill the x position of the adversary
     * @param y The pointer to fill the y position of the adversary
     * @param theta The pointer to fill the orientation of the adversary
     * If any of the three pointer is NULL, it will not be filled
    */
    void get_adv_pos(double *xAdv, double *yAdv, double *dAdv, double *aAdv);
    /** @brief Set the current adversary position.
     * @param x The x position of the adversary
     * @param y The y position of the adversary
     * @param theta The orientation of the adversary
    */
    void set_adv_pos(double xAdv, double yAdv, double dAdv, double aAdv);

    /** @brief Get position from odos and reset teensy with it
     */
    void teensy_reset_pos();

    /** @brief Starts game timer on starting cord pull detection
     */
    void start_timer();

    /** @brief Updates and returns the remaining game time
     * @return The remaining game time of the game, in seconds (rounded UP)
     */
    int16_t update_and_get_timer();

    team_color_t color; // Team color (blue vs yellow)
    uint8_t score; // Current score
    uint8_t startingBaseID; // Graph node ID of the base where the robot started
    storage_content_t storage[8]; // Storage of the robot, indices respect the order defined by storage_slot_t
    uint8_t nFreeSlots; // Number of free storage slots
    uint8_t plantersDone[3]; // Indicates planters that are already filled with some plants. Order : 0 = reserved, 1 = next to reserved, 2 = adversary's side of the map
    uint8_t zonesDone[3]; // Indicates zones that are already filled with some plants. Order : 0 = reserved, 1 = friendly side of the map (not reserved), 2 = adversary's side of the map
    uint8_t plantCounts[6]; // Plant counts in each of the plant zones. Indexes follow graph convention
    uint8_t SPsDone[2]; // Indicates solar panels already done (by groups of 3). Order : 0= common, 1 = reserved
    uint8_t spBlockDone; // Idfk
    uint8_t backToBaseDone; // Signals the robot has returned to base, finished all prepared actions
    uint8_t goingToBase; // Signals if the robot is heading to base (at the end of the game)


    uint8_t valids[5]; //Validities of each major actuator/sensor. Order : SptrPlate, StprSlider, StprFlaps, dxls, lidarBottom

    SPIBus *spiBus;
    GPIOPins *pins;
    Steppers *steppers;
    Odometry *odo;
    Teensy *teensy;
    Flaps *servoFlaps;
    GripperDeployer *grpDeployer;
    GripperHolder *grpHolder;
    Graph *graph;
};

extern SharedVariables shared;  

#endif