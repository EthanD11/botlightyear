#include "action_return.h"
#include <pthread.h>
#include <stdio.h>

typedef enum _state : int8_t
{
    PF, // Moving to destination
    Get, // Get next plant in plate
    Drop, // Drop plant
    End, // End thread normally
    Abort // End thread with failure
} state_t;

static pthread_t KCID;
static volatile state_t state;
static volatile state_t stateKC;

static void leave() {
    state = Abort;
    pthread_join(KCID, NULL);
}

static void *kinematic_chain(void *args) {
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

        case Get: // Get next plant in plate
            stateKC = Get;
            slotID = get_next_unloaded_slot_ID(ConstainsWeakPlantInPot);
            if (slotID == SlotInvalid) slotID = get_next_unloaded_slot_ID(ContainsStrongPlant);
            toDrop = shared.storage[slotID];
            grpDeployer->half();
            steppers->plate_move(get_plate_slot(slotID),CALL_BLOCKING);
            grpDeployer->deploy();

            if (slotID == SlotGripper) { // If plant already in gripper
                update_plate_content(slotID, ContainsNothing);
                break;
            }

            // Grab plant
            grpHolder->open_full();
            steppers->slider_move(SliderStorage, CALL_BLOCKING);
            if ((toDrop & ContainsPot) == ContainsPot) grpHolder->hold_pot();
            else grpHolder->hold_plant();
            update_plate_content(slotID, ContainsNothing);
            usleep(400000);

            // Ready to drop
            steppers->slider_move(SliderHigh);
            usleep(700000);
            steppers->plate_move(0, CALL_BLOCKING);
            grpDeployer->idle();
            break;

        case Drop:
            steppers->slider_move(SliderLow, CALL_BLOCKING);
            grpHolder->open();
            stateKC = Drop;
            if ((toDrop == ConstainsWeakPlantInPot) || (toDrop & ContainsStrongPlant == ContainsStrongPlant))
                shared.score += 3 + ((toDrop & ContainsPot) == ContainsPot);
            steppers->slider_move(SliderHigh);
            usleep(200000);
            grpHolder->idle();
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
            
            default:
                break;
            }
                
            return NULL;
        }
    }

    return NULL;
}


void ActionBackToBase::do_action() {

    #ifdef EMPTY_PLANTS_ON_RETURN

    if (abs(path->x[path->nNodes-1]) > 0.5)
    path->x[path->nNodes-1] += 0.1*(1-2*(path->x[path->nNodes-1] < 1.0));

    if (nbPlants > 0) {

        state = Get;
        stateKC = PF;
        pthread_create(&KCID, NULL, kinematic_chain, NULL);

        if (nbPlants > 2) nbPlants = 2; 

        double xZone, yZone, thetaZone;
        xZone = path->x[path->nNodes-1]+0.1;
        yZone = path->y[path->nNodes-1];
        thetaZone = path->thetaEnd;
        path->nNodes--;


        if (shared.color == TeamBlue) {
            double x = 0, y = 0;
            shared.get_robot_pos(&x, &y, NULL);
            
            shared.teensy->pos_ctrl(x,y,-M_PI);
            sleep(1);

            shared.get_robot_pos(&x, &y, NULL);
            shared.teensy->pos_ctrl(x,y,M_PI_2);
        }

        for (uint8_t i = 0; i < nbPlants; i++) {

            state = Get;
            while (stateKC != Get) usleep(50000);

            if (action_position_control(xZone+(0.1-i*0.1)*cos(thetaZone),
                                        yZone+(0.1-i*0.1)*sin(thetaZone),
                                        thetaZone)) return leave();

            state = Drop;
            while (stateKC != Drop) 
                usleep(50000);

        }
        
        state = End;
        pthread_join(KCID, NULL);
    
    if (path_following_to_action(path) == -1) return;
    // Idle
    shared.teensy->idle(); 
    // Score update
    shared.score += 10;

    #else

    if (abs(path->x[path->nNodes-1]) > 0.5)
        path->x[path->nNodes-1] += 0.1*(1-2*(path->x[path->nNodes-1] < 1.0));

    if (path_following_to_action(path) == -1) return;
    // Idle
    shared.teensy->idle(); 
    // Score update
    shared.score += 10;
    
    #endif
}