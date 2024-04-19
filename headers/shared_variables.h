#ifndef BLY_SHARED_H
#define BLY_SHARED_H

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
    ConstainsWeakPlantInPot = 0b111
} storage_content_t;

class SharedVariables
{
private:
    double x, y, theta; // Current robot position
    double xAdv, yAdv, dAdv, aAdv; // Current adversary position
    time_t tStart; // Game start time
public:
    SharedVariables();
    ~SharedVariables();

    // Usage : `double x, y, theta; get_robot_pos(&x,&y,&theta);`
    // If any of the three pointer is NULL, it will not be filled
    void get_robot_pos(double *x, double *y, double *theta);
    void set_robot_pos(double x, double y, double theta);    
    // Usage : `double xAdv, yAdv, thetaAdv; get_adv_pos(&xAdv,&yAdv,&thetaAdv);`
    // If any of the three pointer is NULL, it will not be filled
    void get_adv_pos(double *xAdv, double *yAdv, double *dAdv, double *aAdv);
    void set_adv_pos(double xAdv, double yAdv, double dAdv, double aAdv);

    // Starts game timer on starting cord pull detection
    void start_timer();
    // Updates and returns the remaining game time
    int8_t update_and_get_timer();

    team_color_t color; // Team color (blue vs yellow)
    uint8_t score; // Current score
    uint8_t startingBaseID; // Graph node ID of the base where the robot started
    storage_content_t storage[8]; // Storage of the robot, indices respect the order defined by storage_slot_t
    uint8_t nFreeSlots; // Number of free storage slots
    uint8_t plantersDone[3]; 
    uint8_t zonesDone[3]; 
    uint8_t SPsDone[2]; // Common, Reserved


    uint8_t valids[6]; // SptrPlate, StprSlider, StprFlaps, dxl1, dxl2, lidarBottom

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