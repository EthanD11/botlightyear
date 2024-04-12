#include <time.h>
#include <stdint.h>

typedef enum _storage_slot : uint8_t
{
    SlotM3,
    SlotM2,
    SlotM1,
    Slot3,
    Slot2,
    Slot1,
    SlotGripper,
    SlotFlaps
}storage_slot_t;

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
}storage_content_t;

class ActionVariables
{
public:
    ActionVariables();
    ~ActionVariables();
    void pos_rd_lock(); // Lock from other threads, low priority
    void pos_wr_lock(); // Lock from other threads, high priority
    void pos_unlock(); // Unlock for other treads
    double x, y, theta; // Current robot position
    double xadv, yadv, thetaadv; // Current adversary position
    storage_content_t storage[7]; // Storage of the robot, indices respect the order defined by storage_slot_t
    storage_slot_t nFreeSlots; // Number of free storage slots
};

