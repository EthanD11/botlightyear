#include "action_pots.h"
#include <stdio.h>
#include <pthread.h>




static pthread_t KCID;
static volatile bool ThreadKinematicOccuped = false;
static volatile bool stopBeforeMovePot = false;


static double posPotX ; 
static double posPotY ;
static double posPotTheta;
static double clearanceAngle;

int8_t sign(double a) {
    return (a>=0) ? 1 : -1; 
}
 
/*
Takes pot and puts it to the plate storage specified
*/

void *take_pot_kinematicChain_SecondPart(void *args ){
    // !!! ne peut pas utiliser flaps !!!
    //int slotNumber
    printf("start thread kinematic\n");
    int slotNumber = *((int*) args);
    GripperHolder* holder = shared.grpHolder; 
    GripperDeployer* deployer = shared.grpDeployer; 
    Steppers* steppers = shared.steppers; 
    // Flaps* servoFlaps = shared.servoFlaps; 
    stopBeforeMovePot = true;
    ThreadKinematicOccuped = true;
    //remonte
    steppers->slider_move(SliderHigh, CALL_BLOCKING);
    //depose pot en plateau
    steppers->plate_move(slotNumber, CALL_BLOCKING); 
    deployer->pot_deposit();
    steppers->slider_move(SliderDepositPot, CALL_BLOCKING);
    deployer->deploy();
    steppers->slider_move(SliderStorage, CALL_BLOCKING);
    //reouvre et remonte
    holder->open_full();
    usleep(100000); //wait 100ms
    deployer->half();
    stopBeforeMovePot = false;
    steppers->slider_move(SliderHigh,CALL_BLOCKING);
    steppers->plate_move(0, CALL_BLOCKING);
    deployer->deploy();
    ThreadKinematicOccuped = false;
    printf("end thread kinematic\n");
    
    return NULL;
}

void get_posPot(int8_t pathTarget, bool freeTheGarden) {
    switch (pathTarget)
    {
    case 44:
        //cote droit en bas
        posPotX = 0.6125;
        posPotY = 0.035;
        posPotTheta = M_PI_2;
        clearanceAngle = -M_PI_2;
        break;
    case 42:
        //cote droit a gauche
        posPotX = 1.3875;
        posPotY = 0.035;
        posPotTheta = M_PI_2;
        clearanceAngle = -M_PI_2;
        break;
    case 37:
        //dessus cote droit
        posPotX = 1.965;
        posPotY = 1.0;
        posPotTheta = M_PI;
        freeTheGarden = false;
        break;
    case 15:
        //dessus cote gauche
        posPotX = 1.965;
        posPotY = 2.0;
        posPotTheta = M_PI;
        freeTheGarden = false;
        break;
    case 3:
        //cote gauche en haut
        posPotX = 1.3875;
        posPotY = 2.965;
        posPotTheta = -M_PI_2;
        clearanceAngle = M_PI_2;
        break;
    case 1:
        //cote gauche en bas
        posPotX = 0.6125;
        posPotY = 2.965;
        posPotTheta = -M_PI_2;
        clearanceAngle = M_PI_2;
        break;
    default:
        printf("pathTarget non reconnu comme une prise de pot\n");
        return;
        break;
    }
}

void take_pot_kinematicChain(int8_t slotNumber, int numeroPot, int8_t pathTarget,bool removePot4 = false,bool freeTheGarden = false) {
    GripperHolder* holder = shared.grpHolder; 
    GripperDeployer* deployer = shared.grpDeployer; 
    Steppers* steppers = shared.steppers; 
    Teensy* teensy = shared.teensy; 
    Flaps* servoFlaps = shared.servoFlaps; 

    //numeroPot :
    //   _____
    //  / 1 2 \ 
    // | 3 4 5 |
    // double posPotX ; 
    // double posPotY ;
    // double posPotTheta;
    // double clearanceAngle;
    get_posPot(pathTarget, freeTheGarden); 


    //parametre approach/prise pot 
    double betaPot1 = M_PI_2/3;
    double distanceRoue = 0.27;
    double deltaApproach = 0.1;
    double rayonPot = 0.08;
    double deltaRayonPot35 = 0.015;

    double posPotXApproach, posPotYApproach,posPotThetaApproach;
    double posPotXPrise, posPotYPrise ,posPotThetaPrise;
    double posPotIntermediateX,posPotIntermediateY;


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
    case 3:
        posPotIntermediateX = posPotX+(rayonPot+deltaRayonPot35)*cos(posPotTheta+M_PI_2);
        posPotIntermediateY = posPotY+(rayonPot+deltaRayonPot35)*sin(posPotTheta+M_PI_2);
        posPotXApproach = posPotIntermediateX+(distanceRoue+deltaApproach)*cos(posPotTheta);
        posPotYApproach = posPotIntermediateY+(distanceRoue+deltaApproach)*sin(posPotTheta);
        posPotThetaApproach = posPotTheta-M_PI;
        posPotXPrise = posPotIntermediateX+(distanceRoue)*cos(posPotTheta);
        posPotYPrise = posPotIntermediateY+(distanceRoue)*sin(posPotTheta);
        posPotThetaPrise = posPotTheta-M_PI;
        break;
    case 5:
        posPotIntermediateX = posPotX+(rayonPot+deltaRayonPot35)*cos(posPotTheta-M_PI_2);
        posPotIntermediateY = posPotY+(rayonPot+deltaRayonPot35)*sin(posPotTheta-M_PI_2);
        posPotXApproach = posPotIntermediateX+(distanceRoue+deltaApproach)*cos(posPotTheta);
        posPotYApproach = posPotIntermediateY+(distanceRoue+deltaApproach)*sin(posPotTheta);
        posPotThetaApproach = posPotTheta+M_PI;
        posPotXPrise = posPotIntermediateX+(distanceRoue)*cos(posPotTheta);
        posPotYPrise = posPotIntermediateY+(distanceRoue)*sin(posPotTheta);
        posPotThetaPrise = posPotTheta+M_PI;
        break;
    default:
        printf("numeroPot %d non reconnu comme un pot valide\n",numeroPot);
        return;
        break;
    }
    printf("here we go for the pot %d\n",numeroPot);
    //remapping pour -PI a +PI
    posPotThetaApproach = periodic_angle(posPotThetaApproach);
    posPotThetaPrise = periodic_angle(posPotThetaPrise);
    //-----approche grossiere-----
    printf("approche grossiere : %f, %f, %f ,distance :%f\n", posPotXApproach, posPotYApproach, posPotThetaApproach,distanceRoue+deltaApproach+rayonPot);
    //teensy->pos_ctrl(posPotXApproach,posPotYApproach,posPotThetaApproach); 
    if (action_position_control(posPotXApproach,posPotYApproach,posPotThetaApproach)==-1) return; 
    // sleep(10);


    //attend variable global de chaine cinematique fini 
    while (ThreadKinematicOccuped == true) {usleep(1000);}
    deployer->deploy();
    holder->open_full();
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

    // lancement de la fin de la cinematique sur un thread
    if (pthread_create(&KCID, NULL, take_pot_kinematicChain_SecondPart, (void *)&slotNumber) != 0) return;
    //take_pot_kinematicChain_SecondPart(slotNumber);

    // recule, réouvre flaps et vas au prochain pot si doit pas dabord degager l'autre
    switch (numeroPot){ // recule plus pour pot 1 et 2
    case 1:
        posPotXApproach = posPotX+(distanceRoue+2*deltaApproach+rayonPot)*cos(betaPot1+posPotTheta);
        posPotYApproach = posPotY+(distanceRoue+2*deltaApproach+rayonPot)*sin(betaPot1+posPotTheta);
        break;
    case 2:
        posPotXApproach = posPotX+(distanceRoue+2*deltaApproach+rayonPot)*cos(-betaPot1+posPotTheta);
        posPotYApproach = posPotY+(distanceRoue+2*deltaApproach+rayonPot)*sin(-betaPot1+posPotTheta);
        break;  
    default:
        break;
    }
    if (removePot4 == false){
        if (action_position_control(posPotXApproach,posPotYApproach,posPotThetaApproach)==-1) return; 
        steppers->flaps_move(FlapsOpen);
        servoFlaps->raise();
    }
    if (removePot4) {
         // se repositionne face au pot centraux
        posPotXApproach = posPotX+(distanceRoue+deltaApproach)*cos(posPotTheta);
        posPotYApproach = posPotY+(distanceRoue+deltaApproach)*sin(posPotTheta);
        posPotThetaApproach = posPotTheta-M_PI;
        posPotThetaPrise = periodic_angle(posPotThetaPrise);
        if (action_position_control(posPotXApproach,posPotYApproach,posPotThetaApproach)==-1) return; 
        // remove pot 4
        //servoFlaps->raise();
        printf("starting remove pot 4\n");
        posPotXPrise = posPotX+(distanceRoue)*cos(posPotTheta);
        posPotYPrise = posPotY+(distanceRoue)*sin(posPotTheta);
        posPotThetaApproach = posPotTheta-M_PI;
        posPotThetaPrise = periodic_angle(posPotThetaPrise);
        printf("target for remove 4 : %f,%f,%f\n",posPotXPrise,posPotYPrise,posPotThetaPrise);
        if (action_position_control(posPotXPrise,posPotYPrise,posPotThetaPrise)==-1) return; 
        steppers->flaps_move(FlapsPot,CALL_BLOCKING);
        if (action_position_control(posPotXApproach,posPotYApproach,posPotThetaApproach)==-1) return; 
        steppers->flaps_move(FlapsOpen);
        usleep(400000);
        servoFlaps->raise();
    }
    // si doit degager TOUT les autres pots
    if (freeTheGarden){
        printf("come back Aproach and then starting remove all pots\n");
        if (action_position_control(posPotXApproach,posPotYApproach,posPotThetaApproach)==-1) return; 
        steppers->flaps_move(FlapsOpen);
        servoFlaps->raise();
        // se repositionne face au pot centraux
        double posPotXThrow1 = posPotX+(0.30)*cos(posPotTheta +(M_PI/6)*(-sign(clearanceAngle)));
        double posPotYThrow1 = posPotY+(0.30)*sin(posPotTheta +(M_PI/6)*(-sign(clearanceAngle)));
        double posPotThetaThrow1 = posPotTheta+ ((M_PI/6)+M_PI_2)*(sign(clearanceAngle));
        double posPotXThrow2 = posPotX;
        double posPotYThrow2 = posPotY+(0.18)*sin(posPotTheta);
        double posPotThetaThrow2 = posPotTheta+clearanceAngle;
        posPotThetaThrow1 = periodic_angle(posPotThetaThrow1);
        posPotThetaThrow2 = periodic_angle(posPotThetaThrow2);
        //printf("posPotTheta : %f, clearanceAngle : %f\n",posPotTheta,clearanceAngle);
        printf("target 1 for remove all pots : %f,%f,%f\n",posPotXThrow1,posPotYThrow1,posPotThetaThrow1);
        if (action_position_control(posPotXThrow1,posPotYThrow1,posPotThetaThrow1)==-1) return;
        printf("target 2 for remove all pots : %f,%f,%f\n",posPotXThrow2,posPotYThrow2,posPotThetaThrow2);
        if (action_position_control(posPotXThrow2,posPotYThrow2,posPotThetaThrow2)==-1) return;
    }   
    while (stopBeforeMovePot == true){usleep(1000);}
}




void initial_pos_stepper(){
    Steppers* steppers = shared.steppers; 
    GripperHolder* holder = shared.grpHolder; 
    GripperDeployer* deployer = shared.grpDeployer; 
    Flaps* servoFlaps = shared.servoFlaps; 

    steppers->flaps_move(FlapsOpen);
    servoFlaps->raise();
    steppers->plate_move(0,CALL_BLOCKING);
    steppers->slider_move(SliderHigh,CALL_BLOCKING);
    holder->idle();
    deployer->idle();
}


int8_t get_numeroPot(int8_t i) {
    //numeroPot :
    //   _____
    //  / 1 2 \ 
    // | 3 4 5 |
    int orderPot[5] = {1, 3, 2, 5, 4};
    if (i >= 5) return -1;
    return orderPot[i];
}

void ActionPots::do_action() {
    double xpos, ypos, theta_pos; 
    double xposInitiale, yposInitiale, theta_posInitiale;
    int pathTarget;
    bool removePot4KinematicChaine = false;
    bool freeTheGardenLastTurn = false;
    pathTarget = path->target;

    shared.teensy->set_position_controller_gains(0.4,1.5,-0.5,0.5);
    get_posPot(pathTarget, freeTheGardenLastTurn); 
    shared.get_robot_pos(&xpos, &ypos, &theta_pos);
    if (hypot(xpos-posPotX, ypos-posPotY) > 0.7) {
        if (path_following_to_action(path) == -1) return; 
    }
    
    shared.get_robot_pos(&xposInitiale, &yposInitiale, &theta_posInitiale);
    initial_pos_stepper();
    for (uint8_t i=0; i<this->potCounter; i++) {
        shared.get_robot_pos(&xpos, &ypos, &theta_pos);
        storage_slot_t nextSlot = get_next_free_slot_ID(ContainsStrongPlantInPot); // Completely empty slot (no pot, no plants)
        int8_t plate_pos = get_plate_slot(nextSlot); 
        int8_t numeroPot = get_numeroPot(i);
        // If last pot is 3 or 5, remove pot 4
        if (this->removePot4){
            if ((numeroPot == 3 || numeroPot == 5)&& i == this->potCounter-1) {removePot4KinematicChaine = true;} 
            else {removePot4KinematicChaine = false;}
        }
        if (this->freeTheGarden && i == this->potCounter-1) {freeTheGardenLastTurn = true;} 
        else {freeTheGardenLastTurn = false;}
        
        take_pot_kinematicChain(plate_pos,numeroPot,pathTarget, removePot4KinematicChaine,freeTheGardenLastTurn); // Launches the kinematic chain
        update_plate_content(nextSlot, ContainsPot); 

    }
    //printf("come back pos initial: %f, %f, %f\n", xposInitiale, yposInitiale, theta_posInitiale);
    //if (action_position_control(xposInitiale,yposInitiale,periodic_angle(theta_posInitiale+M_PI))==-1) return; 

    while(ThreadKinematicOccuped == true) {usleep(1000);};
    initial_pos_stepper();
    sleep(1000);
}