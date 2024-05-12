#include "action_plants.h"
#include "lidarBottom.h"
#include <unistd.h>
#include <cmath>
#include <pthread.h>
#include <stdio.h>


static pthread_t KCID;
static volatile bool ThreadKinematicOccuped = false;
static volatile bool hasPot = false;

static uint8_t plantZoneId = 0; 

double plant_approach_dist = 0.4; //0.35 MAX
double plant_grab_dist = 0.25; // 0.22 MIN
double plant_approach_angle = M_PI/6; 
double away_distance = 0.1; 
/*
Takes plant and puts it to the plate storage specified
*/


void gainDeBasePlante(){
    double gainDesGain = 1.0;
    // shared.teensy->set_position_controller_gains(1.0*gainDesGain,2.5*gainDesGain,-1.0*gainDesGain,1.8*gainDesGain);
    shared.teensy->set_position_controller_gains(0.7,3.0,-1.0,4.0);
        
}

void gainPrecisPlante(){
    // shared.teensy->set_position_controller_gains(0.9,2.5,0.0,1);
    shared.teensy->set_position_controller_gains(0.9,2.5,0.0,4.0);


}

void gainReculePlante(){
    shared.teensy->set_position_controller_gains(3.0,2.5,0.0,1); 
}

void teensyIdle(){
    Teensy* teensy = shared.teensy; 
    teensy->idle();
}



void PrepareApproachTakePlant(){
    GripperHolder* holder = shared.grpHolder; 
    GripperDeployer* deployer = shared.grpDeployer; 
    Steppers* steppers = shared.steppers; 
    Teensy* teensy = shared.teensy; 
    Flaps* servoFlaps = shared.servoFlaps; 

    //attend thread kinematic termine
    while(ThreadKinematicOccuped == true){usleep(1000);}
    printf("preparation pince et flaps pour approche\n");
    deployer->deploy(); 
    holder->open_full();
    //approach
    steppers->slider_move(SliderPreparePlant);
    steppers->flaps_move(FlapsApproachPlant);
    servoFlaps->deploy();
}

void *take_plant_kinematicChain_SecondPart(void *args ){
    printf("Start thread kinematic plant\n");
    // /!\ NO FLAPS HERE
    GripperHolder* holder = shared.grpHolder; 
    GripperDeployer* deployer = shared.grpDeployer; 
    Steppers* steppers = shared.steppers; 
    Teensy* teensy = shared.teensy; 
    Flaps* servoFlaps = shared.servoFlaps; 
    int8_t slotNumber = *((int8_t*) args);
    ThreadKinematicOccuped = true;
    //remonte
    steppers->slider_move(SliderHigh);//tricks pour replier servo au milieu
    usleep(300000);
    deployer->plantLift();
    shared.pins->wait_for_gpio_value(StprSliderGPIO, 1, 10000);

    //mise dans plateau
    // printf("slotNumber : %d\n",slotNumber);
    steppers->plate_move(slotNumber, CALL_BLOCKING); 
    if (hasPot == true){
        //steppers->slider_move(SliderIntermediatePlantInPot,CALL_BLOCKING);
        deployer->deploy();  
        steppers->slider_move(SliderStorageInPot, CALL_BLOCKING);  
    }else {
        steppers->slider_move(SliderIntermediatePlant,CALL_BLOCKING);
        deployer->deploy(); 
        steppers->slider_move(SliderStorage, CALL_BLOCKING);    
    }
    holder->open_full();
    usleep(100000);

    //remonte et remise en place
    deployer->half();
    steppers->slider_move(SliderHigh, CALL_BLOCKING); 
    steppers->plate_move(0, CALL_BLOCKING); 
    holder->idle();
    deployer->idle();
    ThreadKinematicOccuped = false;
    printf("End thread kinematic plant\n");
    return NULL;    
}


void take_plant_kinematicChain(int8_t slotNumber) {
    GripperHolder* holder = shared.grpHolder; 
    GripperDeployer* deployer = shared.grpDeployer; 
    Steppers* steppers = shared.steppers; 
    Teensy* teensy = shared.teensy; 
    Flaps* servoFlaps = shared.servoFlaps; 

    double x_pos_init = 0, y_pos_init = 0, theta_pos_init = 0; 
    shared.get_robot_pos(&x_pos_init, &y_pos_init, &theta_pos_init);

    deployer->deploy();
    holder->open_full();

    // Align plant
    steppers->flaps_move(FlapsPlant, CALL_BLOCKING); 
    steppers->flaps_move(FlapsApproachPlant);

    // take de plant
    // Move backward
    // /!\ a verifier si zone jaune
    //shared.teensy->set_position_controller_gains(2.0,2.5,-1.0,1.8); 
    gainReculePlante();
    printf("Recule pour prendre : %f, %f, %f\n", x_pos_init-0.03*cos(theta_pos_init), y_pos_init-0.03*sin(theta_pos_init), theta_pos_init,0.01,10);
    if (action_position_control(x_pos_init-0.03*cos(theta_pos_init), y_pos_init-0.03*sin(theta_pos_init), theta_pos_init) == -1) return; 
    //teensy->pos_ctrl(x_pos_init-0.02*cos(theta_pos_init), y_pos_init-0.02*sin(theta_pos_init), theta_pos_init);
    // descent
    // steppers->slider_move(SliderIntermediateLow,CALL_BLOCKING);
    // deployer->half();
    steppers->slider_move(SliderLow, CALL_BLOCKING);
    deployer->deploy();
    // Move forward
    printf("Avance pour prendre %f, %f, %f\n",x_pos_init, y_pos_init, theta_pos_init);
    if (action_position_control(x_pos_init, y_pos_init, theta_pos_init) == -1) return; 
    gainDeBasePlante();
    //action_position_control(x_pos_init, y_pos_init, theta_pos_init);
    usleep(100000);
    holder->hold_plant();
    int8_t args = slotNumber;  
    // printf("slotNumber in kine1 %d\n",args);
    if (pthread_create(&KCID, NULL, take_plant_kinematicChain_SecondPart, (void *)&args) != 0) return;
    usleep(300000);//wait avant de repartir direct
    //teensy->set_position_controller_gains(0.4,2.5,-1.5,1.0);
}


void initial_pos_stepper_forPlant(){
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



/**
 * @brief Uses position control to 
 * @return 0 if completion success, -1 otherwise
*/ 
// VERSION OF KAKOO
// int8_t position_to_plant(double x_plant, double y_plant, double x_plant_center, double y_plant_center, bool isLeft, bool isFirst = false) {
//     double plant_to_center_dist = hypot(y_plant_center-y_plant, x_plant_center-x_plant); //donne l'hypothenuse
//     double alpha; 
//     if (isFirst) {
//         alpha = 0; 
//     } else {
//         if (isLeft) {
//             alpha = -plant_approach_angle;
//         } else {
//             alpha = plant_approach_angle;
//         }
//     }
//     printf("Plant approach distance = %.3f, isFirst = %d and alpha = %.3f \n", plant_approach_dist, isFirst, alpha); 
//     // First, position control to the approach point
//     double dx = (x_plant - x_plant_center)/plant_to_center_dist; 
//     double dy= (y_plant - y_plant_center)/plant_to_center_dist; 
//     double x_approach = x_plant + (dx*cos(alpha) + dy*sin(alpha))* plant_approach_dist; 
//     double y_approach = y_plant + (-dx*sin(alpha) + dy*cos(alpha))* plant_approach_dist; 
//     double theta_approach = periodic_angle(atan2(y_plant_center-y_plant, x_plant_center-x_plant)-alpha);
//     printf("Position control to approach to x = %.3f, y = %.3f and theta = %.3f \n", x_approach, y_approach, theta_approach);
//     shared.teensy->set_position_controller_gains(0.5,2.5,-1.5,1.0);
//     if (action_position_control(x_approach, y_approach, theta_approach) == -1) return -1; 
//
//     // Then, position control to the plant distance
//
//     double x_grab = x_plant + (dx*cos(alpha) + dy*sin(alpha)) * plant_grab_dist; 
//     double y_grab = y_plant + (-dx*sin(alpha) + dy*cos(alpha)) * plant_grab_dist; 
//     shared.teensy->set_position_controller_gains(0.4,2.5,-1.5,1.0);
//     printf("Position control to grab to x = %.3f, y = %.3f and theta = %.3f \n", x_grab, y_grab, theta_approach);
//     // shared.servoFlaps->deploy();
//     // shared.steppers->flaps_move(FlapsApproachPlant);
//     PrepareApproachTakePlant();
//     if (action_position_control(x_grab, y_grab, theta_approach) == -1) return -1; 
//     return 0; 
// }


int8_t position_to_plant(double x_plant, double y_plant, double x_plant_center, double y_plant_center, bool isFirst = false) {
    //hypot donne l'hypoténuse
    double plant_to_center_dist = hypot(y_plant_center-y_plant, x_plant_center-x_plant); 
    double x_approach=0, y_approach=0, theta_approach=0;
    shared.get_robot_pos(&x_approach, &y_approach, &theta_approach);
    double dx = (x_plant - x_plant_center)/plant_to_center_dist;
    double dy= (y_plant - y_plant_center)/plant_to_center_dist;
    if (isFirst) {
        x_approach = x_plant + dx * plant_approach_dist;
        y_approach = y_plant + dy * plant_approach_dist;
        theta_approach = atan2(y_plant_center-y_plant, x_plant_center-x_plant);
    } else {
        //si pas premier, garde la meme position et change juste l angle 
        theta_approach = atan2(y_plant-y_approach, x_plant-x_approach);
    }
    printf("Plant approach distance = %.3f, isFirst = %d\n", plant_approach_dist, isFirst); 
    printf("Position control to approach to x = %.3f, y = %.3f and theta = %.3f \n", x_approach, y_approach, theta_approach);
    // shared.teensy->set_position_controller_gains(0.5,2.5,-1.5,1.0);
    gainDeBasePlante();
    if (action_position_control(x_approach, y_approach, periodic_angle(theta_approach),0.01,5) == -1) return -1; 

    // Then, position control to the plant distance
    double x_grab = x_plant - cos(theta_approach) * plant_grab_dist; 
    double y_grab = y_plant - sin(theta_approach) * plant_grab_dist; 
    // shared.teensy->set_position_controller_gains(0.4,2.5,-1.5,1.0);
    PrepareApproachTakePlant();
    printf("Position control to grab to x = %.3f, y = %.3f and theta = %.3f \n", x_grab, y_grab, theta_approach);
    gainPrecisPlante();
    if (action_position_control(x_grab, y_grab, theta_approach) == -1) return -1; 
    gainDeBasePlante();
    printf("Ready to grab\n");
    return 0; 
}

// VERSION DE KAKOO
// int8_t move_back(double x_plant, double y_plant) {
//     Steppers* steppers = shared.steppers; 
//     Flaps* servoFlaps = shared.servoFlaps;
//     double x_pos = 0, y_pos = 0, theta_pos = 0; 
//     shared.get_robot_pos(&x_pos, &y_pos, &theta_pos);
//     double plant_to_bot_dist = hypot(x_plant-x_pos, y_plant-y_pos); 
//     double dx = (x_pos-x_plant)/plant_to_bot_dist; 
//     double dy = (y_pos-y_plant)/plant_to_bot_dist; 
//     double x_away = x_pos+dx*away_distance; 
//     double y_away = y_pos+dy*away_distance; 
//     printf("Position control to go back to x = %.3f, y = %.3f and theta = %.3f \n", x_away, y_away, theta_pos);
//     // shared.teensy->set_position_controller_gains(2.0,5.0,-0.8,4.0);
//     if (action_position_control(x_away, y_away, theta_pos) == -1) return -1; 
//     //une fois reculé, remet flaps a position initiale
//     steppers->flaps_move(FlapsOpen);
//     servoFlaps->raise();
//     return 0; 
// }


int8_t move_back(double x_plant, double y_plant) {
    Steppers* steppers = shared.steppers; 
    Flaps* servoFlaps = shared.servoFlaps;

    //hypot donne l'hypoténuse
    double x_approach=0, y_approach=0, theta_approach=0;
    shared.get_robot_pos(&x_approach, &y_approach, &theta_approach);
    double dx = plant_approach_dist * cos(theta_approach);
    double dy = plant_approach_dist * sin(theta_approach);
    
    x_approach = x_plant - dx ;
    y_approach = y_plant - dy ;
 
    printf("Position control to go back to x = %.3f, y = %.3f and theta = %.3f \n", x_approach, y_approach, theta_approach);
    // shared.teensy->set_position_controller_gains(2.0,5.0,-0.8,4.0);

    if (action_position_control(x_approach, y_approach, theta_approach) == -1) return -1; 

    //une fois reculé, remet flaps a position initiale
    steppers->flaps_move(FlapsOpen);
    servoFlaps->raise();
    return 0; 
}


void get_closest_plant_from_kakoo(double x_pos, double y_pos, uint8_t plantNode, double* x_plant, double* y_plant, uint8_t* plants_collected, int8_t isFirst = 0) {
    double x_center = shared.graph->nodes[plantNode].x;
    double y_center = shared.graph->nodes[plantNode].y;
    double x_plant_min = 2; 
    double y_plant_min = 3; 
    double dist = 3; 
    uint8_t idx = 0;
    for (uint8_t i=0; i<6; i++) {
        if (!plants_collected[i]){
            double x_new_plant = x_center + cos(-M_PI_2+i*M_PI/3)*0.1;
            double y_new_plant = y_center + sin(-M_PI_2+i*M_PI/3)*0.1;
            // double dist_plant = isFirst ? ((shared.color == TeamBlue) ? y_new_plant : (3-y_new_plant)) : (hypot(x_pos-x_new_plant, y_pos-y_new_plant));
            double dist_plant = hypot(x_pos-x_new_plant, y_pos-y_new_plant);
            if (dist_plant < dist) {
                dist = dist_plant;
                y_plant_min = y_new_plant; 
                x_plant_min = x_new_plant; 
                idx = i; 
            }
        } 
    }
    printf("Closest plant to wall at distance = %.3f (index %d)\n", dist, idx);
    *x_plant = x_plant_min; 
    *y_plant = y_plant_min; 
    plants_collected[idx] = 1; 
}


int8_t get_closest_plant_from_lidar(double x_pos, double y_pos, double theta_pos, uint8_t plantNode, double* x_plant, double* y_plant) {

    Steppers* steppers = shared.steppers; 
    Flaps* servoFlaps = shared.servoFlaps;

    steppers->flaps_move(FlapsOpen,CALL_BLOCKING);
    servoFlaps->raise();
    PlantZone** plantZone = new PlantZone *[6]; 
    initBottomLidar(plantZone);
    printf("Plant zone init\n");
    int zones[6] = {1,1,1,1,1,1}; 
    getNumberOfPlantInAllZone(x_pos, y_pos, theta_pos, zones, plantZone);
    printf("Number of plant in all zones\n");
    uint8_t zoneIdx; 
    switch (plantNode) {
        case 21: 
            zoneIdx = 0; 
            break;
        case 12 :
            zoneIdx = 1; 
            break;
        case 14 :
            zoneIdx = 2; 
            break;
        case 25 :
            zoneIdx = 3; 
            break;
        case 29 :
            zoneIdx = 4; 
            break;
        case 31 :
            zoneIdx = 5; 
            break;
        default : 
            zoneIdx = 0; 
            printf("Invalid plant zone !\n");
            return -1; 
    }
    uint8_t plantCount = plantZone[zoneIdx]->numberPlant; 
    shared.plantCounts[plantZoneId] = plantCount; 
    printf("plantcounts : %d \n", plantCount);
    if (plantCount==0) {
        deleteBottomLidar(plantZone); 
        printf("Lidar sees no plant in zone ! \n");
        return -1; 
    } 
    printf("avant avant free\n");

    double dist_min_robot = 5; 
    double plant_dist_robot; 
    uint8_t closest_plant_idx = 0; 
    for (uint8_t i=0; i<plantCount; i++){
        printf("Lidar sees plant %d at x = %.3f and y = %.3f \n", i, plantZone[zoneIdx]->xPlant[i], plantZone[zoneIdx]->yPlant[i]); 
        // plant_dist_wall = (shared.color == TeamBlue) ? plantZone[zoneIdx]->yPlant[i] : (3-plantZone[zoneIdx]->yPlant[i]);
        plant_dist_robot = hypot(x_pos - plantZone[zoneIdx]->xPlant[i], y_pos - plantZone[zoneIdx]->yPlant[i]);
        if (plant_dist_robot < dist_min_robot) {
            closest_plant_idx = i; 
            dist_min_robot = plant_dist_robot;
        }
    }
    printf("Closest plant to robot at distance = %.3f (index %d)\n", dist_min_robot, closest_plant_idx);


    *x_plant = plantZone[zoneIdx]->xPlant[closest_plant_idx];
    *y_plant = plantZone[zoneIdx]->yPlant[closest_plant_idx];
    printf("avant free\n");
    deleteBottomLidar(plantZone); 
    printf("Lidar scan end\n");
    return 0; 
}


void ActionPlants::do_action() {
    plantZoneId = this->plantZoneIdx;
    uint8_t plants_taken[6] = {0,0,0,0,0,0};
    double xpos = 0, ypos = 0, theta_pos = 0; 
    double xpos_initial = 0, ypos_initial = 0, theta_pos_initial = 0; 
    double x_plant, y_plant, theta_plant; 

    printf("Start action plant\n");
    initial_pos_stepper_forPlant();
    shared.get_robot_pos(&xpos_initial, &ypos_initial, &theta_pos_initial);
    
    // Positions itself in front of the plant node, without going in
    uint8_t plantsNode = path->target; 
    double *x = path->x;
    double *y = path->y;

    if (path->nNodes >= 3) {
        printf("More than 3 nodes: do path following\n");
        path->thetaEnd = atan2(y[path->nNodes-1]-y[path->nNodes-3], x[path->nNodes-1]-x[path->nNodes-3]);
        path->nNodes -= 2;
        if (path_following_to_action(path) == -1) return; 
    } else {
        printf("Less than 3 nodes: do position control\n");
        path->thetaEnd = atan2(y[path->nNodes-1]-ypos_initial, x[path->nNodes-1]-xpos_initial);
    if (action_position_control(x[path->nNodes-1], y[path->nNodes-1], path->thetaEnd,0.01,20)) return;
    }
    // if (action_position_control(0.7, 0.45, M_PI/2)) return;
    

    shared.get_robot_pos(&xpos_initial, &ypos_initial, &theta_pos_initial);
    printf("Path following done with success \n");
    // while ((shared.teensy->ask_mode() != ModePositionControlOver) && (shared.teensy->ask_mode() != ModeIdle)){
    //     usleep(50000);
    // }
    

    for (uint8_t plant_i = 0; plant_i < plantCounter; plant_i++) {
        printf("\n-----Here we go for plant %d-----\n",plant_i);
        // Get closest plant from lidar pov
        printf("Scanning with lidar...\n");
        teensyIdle();
        usleep(250000);
        shared.get_robot_pos(&xpos, &ypos, &theta_pos);
        if (get_closest_plant_from_lidar(xpos, ypos, theta_pos, plantsNode, &x_plant, &y_plant) == -1) return;
        //get_closest_plant_from_kakoo(xpos, ypos, plantsNode, &x_plant, &y_plant, plants_taken); 
        // usleep(200000);
        printf("Scan lidar over\n");

        theta_plant = atan2(y_plant-ypos, x_plant-xpos); 
        printf("Got plant at %f, %f, %f, beginning the approach\n", x_plant, y_plant, theta_plant,0.01,10);
        
        // Position robot in front of plant
        // if (position_to_plant(x_plant, y_plant, shared.graph->nodes[plantsNode].x, shared.graph->nodes[plantsNode].y, trigo_diff(theta_plant, theta_pos)>0, plant_i==0)) return; 
        // shared.teensy->set_position_controller_gains(0.9,2.5,-1.0,1.2);
        // shared.teensy->set_position_controller_gains(0.9,2.5,-1.0,1.8);
        gainDeBasePlante();
        if (position_to_plant(x_plant, y_plant, shared.graph->nodes[plantsNode].x, shared.graph->nodes[plantsNode].y, plant_i==0)) return; 

        // Get next storage slot and put the plant
        storage_slot_t nextSlot = get_next_free_slot_ID(ContainsStrongPlant); 
        int8_t plate_pos = get_plate_slot(nextSlot); 
        hasPot = shared.storage[nextSlot] && ContainsPot;
        // printf("nextSlot %d plate_pos %d\n",nextSlot, plate_pos);
        printf("Activating the kinematic chain\n");//, x_plant, y_plant, theta_plant);
        take_plant_kinematicChain(plate_pos); 
        update_plate_content(nextSlot, ContainsWeakPlant); 
        gainDeBasePlante();
        move_back(x_plant, y_plant); 
    }

    // Position control to the initial location with opposed theta for departure
    //if (action_position_control(xpos_initial, ypos_initial, periodic_angle(theta_pos_initial-M_PI)) == -1) return; 
    while(ThreadKinematicOccuped == true){usleep(1000);}
    printf("End of plant action \n\n"); 
    
}
