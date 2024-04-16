#include "action_planter.h"
#include "shared_variables.h"
#include <pthread.h>
#include <cmath>
#include <algorithm>
#include <stdio.h>
#include <unistd.h>

typedef enum _state : int8_t
{
    PF, // Moving to destination
    Clear, // Clearing pots
    Get, // Get next plant in plate
    Drop, // Drop plant
    End, // End thread normally
    Abort // End thread with failure
} state_t;

pthread_t KCID;
volatile state_t state;
volatile state_t stateKC;

void leave() {
    state = Abort;
    pthread_join(KCID, NULL);
}

void *kinematic_chain(void *args) {
    Steppers *steppers = shared.steppers;
    Flaps *servoFlaps = shared.servoFlaps;
    GripperDeployer *grpDeployer = shared.grpDeployer;
    GripperHolder *grpHolder = shared.grpHolder;

    // Reset actuators
    steppers->flaps_move(FlapsOpen);
    servoFlaps->raise();
    steppers->plate_move(0,CALL_BLOCKING);
    grpDeployer->half();
    steppers->slider_move(SliderHigh, CALL_BLOCKING);

    storage_slot_t slotID;
    storage_content_t toDrop;
    while (1) {
        if (state == stateKC) {
            usleep(50000);
            continue;
        }
        switch (state)
        {
        case PF: // Moving to destination
            usleep(50000);
            break;

        case Clear: // Clearing pots away

            break;

        case Get: // Get next plant in plate
            stateKC = Get;
            storage_slot_t slotID = get_next_unloaded_slot_ID(ContainsWeakPlant);
            if (slotID == SlotInvalid) slotID = get_next_unloaded_slot_ID(ContainsStrongPlant);
            toDrop = shared.storage[slotID];
            steppers->plate_move(get_plate_slot(slotID),CALL_BLOCKING);
            grpDeployer->deploy();

            if (slotID == SlotGripper) { // If plant already in gripper
                update_plate_content(slotID, ContainsNothing);
                break;
            }

            // Grab plant
            grpHolder->open_full();
            steppers->slider_move(SliderStorage, CALL_BLOCKING);
            if (toDrop & ContainsPot) grpHolder->hold_pot();
            else grpHolder->hold_plant();
            update_plate_content(slotID, ContainsNothing);

            // Ready to drop
            steppers->slider_move(SliderHigh, CALL_BLOCKING);
            grpDeployer->half();
            steppers->plate_move(0, CALL_BLOCKING);
            grpDeployer->deploy();
            break;

        case Drop:
            steppers->slider_move(SliderIntermediateLow, CALL_BLOCKING);
            grpHolder->open_full();
            stateKC = Drop;
            shared.score += 3 + ((toDrop & ContainsPot) == ContainsPot) + ((toDrop & ContainsWeakPlant) == ContainsWeakPlant);
            grpDeployer->half();
            steppers->slider_move(SliderHigh);
            break;
        
        case End:
            return NULL;

        default: // Abort or End
            switch (stateKC)
            {
            case Get:
                slotID = get_next_free_slot_ID(ContainsStrongPlant);
                if (slotID == SlotInvalid) slotID = SlotGripper;
                steppers->plate_move(slotID, CALL_BLOCKING);
                if (toDrop & ContainsPot) {
                    printf("Pot kinematic chain not implemented\n");
                } else {
                    steppers->slider_move(SliderStorage, CALL_BLOCKING);
                    grpHolder->open_full();
                }
                update_plate_content(slotID, toDrop);
                steppers->slider_move(SliderHigh,CALL_BLOCKING);
                steppers->plate_move(0);
                break;

            case Clear:
                steppers->flaps_move(FlapsOpen, CALL_BLOCKING);
                servoFlaps->raise();
                break;
            
            default:
                break;
            }
                
            return NULL;
        }
    }

    return NULL;
}

void ActionPlanter::do_action() {
    if (nbPlants > 3) nbPlants = 3;
    state = PF;
    stateKC = PF;
    pthread_create(&KCID, NULL, kinematic_chain, NULL);

    if (path_following_to_action(path)) return leave();
    double xPlanter, yPlanter, thetaPlanter;
    double x, y, theta;
    xPlanter = path->x[path->nNodes];
    yPlanter = path->y[path->nNodes];
    thetaPlanter = path->thetaEnd;

    if (needsPotClear) {
        state = Clear;
        while (stateKC != Clear) usleep(50000);

        if (action_position_control(xPlanter+0.1*cos(thetaPlanter)-0.5*((int8_t)needsPotClear)*sin(thetaPlanter),
                                    yPlanter+0.1*sin(thetaPlanter)+0.5*((int8_t)needsPotClear)*cos(thetaPlanter),
                                    thetaPlanter-M_PI_2*((int8_t)needsPotClear))) return leave();

        if (action_position_control(xPlanter+0.2*cos(thetaPlanter)+0.3*((int8_t)needsPotClear)*sin(thetaPlanter),
                                    yPlanter+0.2*sin(thetaPlanter)-0.3*((int8_t)needsPotClear)*cos(thetaPlanter),
                                    thetaPlanter-M_PI_2*((int8_t)needsPotClear))) return leave();

        if (action_position_control(xPlanter, yPlanter, thetaPlanter)) return leave();
        
    }

    planter_side_t nextSpot = preference;
    for (uint8_t i = 0; i < nbPlants; i++) {

        state = Get;
        while (stateKC != Get) usleep(50000);

        if (action_position_control(xPlanter+0.15*cos(thetaPlanter)-0.08*nextSpot*sin(thetaPlanter),
                                    yPlanter+0.15*sin(thetaPlanter)+0.08*nextSpot*cos(thetaPlanter),
                                    thetaPlanter)) return leave();

        shared.teensy->constant_dc(65,65);
        while (shared.pins->read(BpSwitchFlapsLeftGPIO) != 1 && shared.pins->read(BpSwitchFlapsRightGPIO) != 1) 
            usleep(10000);
        shared.teensy->idle();

        state = Drop;
        while (stateKC != Drop) 
            usleep(50000);

        if (action_position_control(xPlanter,yPlanter,thetaPlanter)) return leave();

        switch (preference)
        {
        case SideLeft:
            if (i == 1) nextSpot = SideMiddle;
            else nextSpot = SideRight;
            break;
        case SideMiddle:
            if (i == 1) nextSpot = SideLeft;
            else nextSpot = SideRight;
            break;
        case SideRight:
            if (i == 1) nextSpot = SideMiddle;
            else nextSpot = SideLeft;
            break;
        }

    }
       
    state = End;
    pthread_join(KCID, NULL);

}