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


static pthread_t KCID;
static volatile state_t state;
static volatile state_t stateKC;
static volatile bool lastPlantInPlanter = false;
static uint8_t planterId = 0;

void gainNormalPlanter(){
    // shared.teensy->set_position_controller_gains(0.9,2.5,-1.0,1.0);
    shared.teensy->set_position_controller_gains(0.7,3.0,-1.0,4.0);
}


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
    steppers->slider_move(SliderHigh, CALL_BLOCKING);
    grpDeployer->idle();

    storage_slot_t slotID = SlotInvalid;
    storage_content_t toDrop = ContainsNothing;
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
            stateKC = Clear;
            steppers->flaps_move(FlapsPlant);
            servoFlaps->deploy();
            break;

        case Get: // Get next plant in plate
            if (stateKC == Clear) {
                steppers->flaps_move(FlapsOpen, CALL_BLOCKING);
                servoFlaps->raise();
            }
            stateKC = Get;
            slotID = get_next_unloaded_slot_ID(ContainsWeakPlant);
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
            if (toDrop & ContainsPot) {
                grpHolder->hold_pot();
                //printf("Warning : Pot not implemented  !! \n");
            }
            else grpHolder->hold_plant();
            update_plate_content(slotID, ContainsNothing);
            usleep(400000);

            // Ready to drop
            steppers->slider_move(SliderHigh);
            usleep(700000);
            grpDeployer->half();
            steppers->plate_move(0, CALL_BLOCKING);
            // grpDeployer->idle();
            break;

        case Drop:
            grpDeployer->deploy();
            steppers->slider_move(SliderIntermediateLow, CALL_BLOCKING);
            grpHolder->open();
            stateKC = Drop;
            shared.plantersDone[planterId] +=1; // Updates the plant count on the given planter
            shared.score += 4 + ((toDrop & ContainsPot) == ContainsPot);
            steppers->slider_move(SliderHigh,CALL_BLOCKING);
            printf("Drop Finish\n");
            //usleep(200000);
            // grpHolder->idle();
            // grpDeployer->idle();
            if(lastPlantInPlanter == false){state = Get;}
            break;

        case End:
            grpDeployer->idle();
            grpDeployer->idle();
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
    printf("----- Start of action planter ----- \n\n");
    planterId = this->planterIdx; 
    gainNormalPlanter();
    if (nbPlants > 3) nbPlants = 3;
    state = PF;
    stateKC = PF;
    if (pthread_create(&KCID, NULL, kinematic_chain, NULL) != 0) return;

    double xPlanter, yPlanter, thetaPlanter;
    xPlanter = path->x[path->nNodes-1];
    yPlanter = path->y[path->nNodes-1];
    thetaPlanter = path->thetaEnd;
    printf("Xplanter : %f, Yplanter : %f, thetaPlanter : %f \n",xPlanter,yPlanter,thetaPlanter);
    if (path_following_to_action(path)) return leave();

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
        printf("nextSpot : ,%d\n",nextSpot);
        // /!\ planter est pas la jardiniere mais le point de PF de la jardiniere!!!
        if (action_position_control(xPlanter+0.25*cos(thetaPlanter)-0.1*nextSpot*sin(thetaPlanter),
                                    yPlanter+0.25*sin(thetaPlanter)+0.1*nextSpot*cos(thetaPlanter),
                                    thetaPlanter)) return leave();

        //shared.teensy->constant_dc(65,65);
        // double valueConstantDC = 80;
        // shared.teensy->constant_dc(valueConstantDC,valueConstantDC);
        double speedValue = 0.1;
        printf("speed Control to planter\n");
        shared.teensy->spd_ctrl(speedValue,speedValue);
        int8_t pins_state = 0;
        while (shared.pins->read(BpSwitchFlapsLeftGPIO) != 1 || shared.pins->read(BpSwitchFlapsRightGPIO) != 1) {
            usleep(200000);
            //printf("pin_state : %d \n",pins_state);
            if ( shared.pins->read(BpSwitchFlapsRightGPIO) ==1 && pins_state !=2) {
                pins_state = 2; 
                shared.teensy->spd_ctrl(speedValue*1.8,0);
                //shared.teensy->constant_dc(0,valueConstantDC*3/2);
            } else if (shared.pins->read(BpSwitchFlapsLeftGPIO) ==1 && pins_state!=1){
                pins_state = 1;
                shared.teensy->spd_ctrl(0,speedValue*1.8);
                // shared.teensy->constant_dc(valueConstantDC*3/2,0);
            } else if (pins_state != 0) {
                pins_state = 0;
                shared.teensy->spd_ctrl(speedValue,speedValue);
                // shared.teensy->constant_dc(valueConstantDC,valueConstantDC);
            }
        }
        shared.teensy->idle();

        state = Drop;
        while (stateKC != Drop) 
            usleep(50000);
        printf("Plant dropped \n");
        //si pas derniere plante, vas deja la cherché apres avoir fini drop
        if (i != nbPlants - 1){
            lastPlantInPlanter = true;
            if (action_position_control(xPlanter,yPlanter,periodic_angle( thetaPlanter+M_PI) ,0.02,30)) return leave();
        } 
        else  {
            lastPlantInPlanter =false;
             if (action_position_control(xPlanter,yPlanter,thetaPlanter,0.02,30)) return leave();
        }
       

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
    printf("End of action planter\n\n");

}