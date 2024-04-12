#include "action_planter.h"
#include <cmath>

void planter_place_objects(uint8_t nObjects) {

    servo_flaps.raise();
    steppers.flaps_move(FlapsOpen);
    steppers.plate_move(0, CALL_BLOCKING);

    uint8_t nDropped = 0;

    // Retrieve current position
    double x, y, theta;
    actVar.pos_rd_lock();
    x = actVar.x; y = actVar.y; theta = actVar.theta;
    actVar.pos_unlock();

    // Empty gripper first
    if (actVar.storage[SlotGripper] != ContainsNothing) {

        teensy.pos_ctrl(x+0.2*cos(theta)+0.32/rint(nObjects/2), y+0.2*sin(theta), theta);
        nDropped++;
    }

    // Empty storage
    steppers.slider_move(SliderHigh, CALL_BLOCKING);
    for (uint8_t i = Slot1; i <= SlotM3; i--)
    {
        if (actVar.storage[i] == ContainsNothing) continue;
        uint8_t plateSlotID;
        switch (i)
        {
        case Slot1:
            plateSlotID = 1;
            break;
        case Slot2:
            plateSlotID = 2;
            break;
        case Slot3:
            plateSlotID = 3;
            break;
        case SlotM1:
            plateSlotID = -1;
            break;
        case SlotM2:
            plateSlotID = -2;
            break;
        case SlotM3:
            plateSlotID = -3;
            break;
        }
    }
    
}