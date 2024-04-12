#include "action_planter.h"
#include <cmath>

extern SharedVariables shared;

void planter_place_objects(uint8_t nObjects) {

    Flaps servoFlaps = shared.servoFlaps;
    Steppers steppers = shared.steppers;
    Teensy teensy = shared.teensy;

    servoFlaps.raise();
    steppers.flaps_move(FlapsOpen);
    steppers.plate_move(0, CALL_BLOCKING);

    uint8_t nDropped = 0;

    // Retrieve current position
    double x, y, theta;
    shared.get_robot_pos(&x, &y, &theta);

    // Empty gripper first
    if (shared.storage[SlotGripper] != ContainsNothing) {

        teensy.pos_ctrl(x+0.2*cos(theta)+0.32/rint(nObjects/2), y+0.2*sin(theta), theta);
        nDropped++;
    }

    // Empty storage
    steppers.slider_move(SliderHigh, CALL_BLOCKING);
    for (uint8_t i = Slot1; i <= SlotM3; i--)
    {
        if (shared.storage[i] == ContainsNothing) continue;
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