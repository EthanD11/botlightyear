#include "action_pots.h"
#include <stdio.h>
/*
Takes pot and puts it to the plate storage specified
*/
void take_pot_kinematicChain(int8_t slotNumber, int8_t numeroPot) {
    GripperHolder* holder = shared.grpHolder; 
    GripperDeployer* deployer = shared.grpDeployer; 
    Steppers* steppers = shared.steppers; 
    Teensy* teensy = shared.teensy; 
    Flaps* servoFlaps = shared.servoFlaps; 



    //numeroPot :
    //   _____
    //  / 1 2 \ 
    // | 3 4 5 |
    //position des pot de la premiere zone bleu
    double posPotX = 0.6125;
    double posPotY = 0.035;
    double posPotTheta = M_PI_2;
    //teensy->set_position(posPotX,posPotY,mappingOnPi(posPotTheta-M_PI));
    //printf("Starting angle : %f \n", mappingOnPi(posPotTheta-M_PI)); 
    //parametre approach/prise pot 
    double betaPot1 = M_PI_2/3;
    double distanceRoue = 0.28;
    double deltaApproach = 0.15;
    double rayonPot = 0.08;

    double posPotXApproach;
    double posPotYApproach;
    double posPotThetaApproach;
    double posPotXPrise;
    double posPotYPrise;
    double posPotThetaPrise;

    deployer->deploy();
    holder->open();
    //steppers->setup_speed(StprSlider, 300,600);
    //steppers->reset_all(); 
    //steppers->calibrate(StprPlate, CALL_BLOCKING);
    //steppers->calibrate(StprSlider, CALL_BLOCKING); 
    switch (numeroPot)
    {
    case 1 :
        posPotXApproach = posPotX+(distanceRoue+deltaApproach+rayonPot)*cos(betaPot1+posPotTheta);
        posPotYApproach = posPotY+(distanceRoue+deltaApproach+rayonPot)*sin(betaPot1+posPotTheta);
        posPotThetaApproach = posPotTheta+betaPot1-M_PI;
        posPotXPrise = posPotX+(distanceRoue+rayonPot)*cos(betaPot1+posPotTheta);
        posPotYPrise = posPotY+(distanceRoue+rayonPot)*sin(betaPot1+posPotTheta);
        posPotThetaPrise = posPotTheta+betaPot1-M_PI;
        break;
    case 2 :
        posPotXApproach = posPotX+(distanceRoue+deltaApproach+rayonPot)*cos(-betaPot1+posPotTheta);
        posPotYApproach = posPotY+(distanceRoue+deltaApproach+rayonPot)*sin(-betaPot1+posPotTheta);
        posPotThetaApproach = posPotTheta-betaPot1-M_PI;
        posPotXPrise = posPotX+(distanceRoue+rayonPot)*cos(-betaPot1+posPotTheta);
        posPotYPrise = posPotY+(distanceRoue+rayonPot)*sin(-betaPot1+posPotTheta);
        posPotThetaPrise = posPotTheta-betaPot1-M_PI;
        break;
    default:
        break;
    }
    //remapping pour -PI a +PI
    posPotThetaApproach = periodic_angle(posPotThetaApproach);
    posPotThetaPrise = periodic_angle(posPotThetaPrise);
    //-----approche grossiere-----
    printf("approche grossiere : %f, %f, %f ,distance :%f\n", posPotXApproach, posPotYApproach, posPotThetaApproach,distanceRoue+deltaApproach+rayonPot);
    //teensy->pos_ctrl(posPotXApproach,posPotYApproach,posPotThetaApproach); 
    if (action_position_control(posPotXApproach,posPotYApproach,posPotThetaApproach)==-1) return; 
    // sleep(10);
    servoFlaps->deploy();
    steppers->flaps_move(FlapsIntermediatePot); 
    steppers->slider_move(SliderPreparePot);
    //-----approche précise-----
    printf("approche précise : %f, %f, %f, distance :%f\n", posPotXPrise, posPotYPrise, posPotThetaPrise,distanceRoue+rayonPot);
    //teensy->pos_ctrl(posPotX+(distanceRoue)*(betaPot1),posPotY+(distanceRoue)*(betaPot1),posPotTheta+betaPot1-M_PI);
    // teensy->pos_ctrl(posPotXPrise,posPotYPrise,posPotThetaPrise);
    if (action_position_control(posPotXPrise,posPotYPrise,posPotThetaPrise)==-1) return; 
    // sleep(10);    
    //-----prise pot-----
    printf("debut chaine cinématique\n");
    steppers->flaps_move(FlapsPot,CALL_BLOCKING);
    steppers->flaps_move(FlapsIntermediatePot,CALL_BLOCKING);
    holder->open_full();
    steppers->slider_move(SliderLow,CALL_BLOCKING);
    holder->hold_pot();//fermeture pot
    if (action_position_control(posPotXApproach,posPotYApproach,posPotThetaApproach)==-1) return; 
    steppers->flaps_move(FlapsOpen);
    //remonte
    steppers->slider_move(SliderHigh, CALL_BLOCKING);
    steppers->plate_move(slotNumber, CALL_BLOCKING); 

    deployer->pot_deposit();
    steppers->slider_move(SliderDepositPot, CALL_BLOCKING);
    deployer->deploy();
    steppers->slider_move(SliderStorage, CALL_BLOCKING);
    holder->open_full();
    deployer->half();
    steppers->slider_move(SliderHigh,CALL_BLOCKING);
    steppers->plate_move(0, CALL_BLOCKING);
    holder->idle();
    deployer->idle();
    sleep(1000);

    // deployer->deploy();
    // holder->open();

    // servoFlaps->deploy();
    // steppers->flaps_move(FlapsIntermediatePot); 
    // steppers->slider_move(SliderPreparePot);
    // //approche TEENSY
    // sleep(2);
    // steppers->flaps_move(FlapsPot,CALL_BLOCKING);
    // steppers->flaps_move(FlapsIntermediatePot,CALL_BLOCKING);
    // steppers->flaps_move(FlapsOpen);
    // holder->open_full();
    // steppers->slider_move(SliderLow,CALL_BLOCKING);
    // holder->hold_pot();//fermeture pot

    // //remonte
    // steppers->slider_move(SliderHigh, CALL_BLOCKING);
    // steppers->plate_move(slotNumber, CALL_BLOCKING); 

    // deployer->pot_deposit();
    // steppers->slider_move(SliderDepositPot, CALL_BLOCKING);
    // deployer->deploy();
    // steppers->slider_move(SliderStorage, CALL_BLOCKING);
    // holder->open_full();
    // deployer->half();
    // steppers->slider_move(SliderHigh,CALL_BLOCKING);
    // steppers->plate_move(0, CALL_BLOCKING);
    // holder->idle();
    // deployer->idle();
}

void ActionPots::do_action() {
    double xpos, ypos, theta_pos; 

    if (path_following_to_action(path) == -1) return; 

    for (uint8_t i=0; i<this->potCounter; i++) {
        shared.get_robot_pos(&xpos, &ypos, &theta_pos);

        storage_slot_t nextSlot = get_next_free_slot_ID(ContainsStrongPlantInPot); // Completely empty slot (no pot, no plants)
        int8_t plate_pos = get_plate_slot(nextSlot); 


        take_pot_kinematicChain(plate_pos,2); // Launches the kinematic chain
    }

}