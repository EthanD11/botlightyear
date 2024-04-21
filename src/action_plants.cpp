#include "action_plants.h"
#include "lidarBottom.h"
#include <unistd.h>
#include <cmath>


double plant_approach_dist = 0.4; //0.35 MAX
double plant_grab_dist = 0.24; // 0.22 MIN
double plant_approach_angle = M_PI/6; 
double away_distance = 0.1; 
/*
Takes plant and puts it to the plate storage specified
*/
void take_plant_kinematicChain(int8_t slotNumber) {
    GripperHolder* holder = shared.grpHolder; 
    GripperDeployer* deployer = shared.grpDeployer; 
    Steppers* steppers = shared.steppers; 
    Teensy* teensy = shared.teensy; 
    Flaps* servoFlaps = shared.servoFlaps; 

    double x_pos_init = 0, y_pos_init = 0, theta_pos_init = 0; 
    shared.get_robot_pos(&x_pos_init, &y_pos_init, &theta_pos_init);

    //teensy->set_position(1.0,1.0,0);
    //teensy->set_position_controller_gains(2.0,6.0,-0.2,4.0);
    teensy->set_position_controller_gains(3.0,5.0,-0.8,4.0);


    servoFlaps->deploy();
    steppers->flaps_move(FlapsPlant, CALL_BLOCKING); 
    steppers->flaps_move(FlapsOpen);
    usleep(200000);
    teensy->pos_ctrl(x_pos_init-0.06*cos(theta_pos_init), y_pos_init-0.06*sin(theta_pos_init), theta_pos_init);
    usleep(300000);
    servoFlaps->raise();

    steppers->slider_move(SliderHigh, CALL_BLOCKING);
    deployer->half(); 
    steppers->plate_move(0, CALL_BLOCKING); 

    deployer->deploy();

    holder->open();

    steppers->slider_move(SliderLow, CALL_BLOCKING);
    teensy->pos_ctrl(x_pos_init, y_pos_init, theta_pos_init);
    usleep(600000);
    holder->hold_plant();
    usleep(250000);

    deployer->half(); 
    steppers->slider_move(SliderHigh, CALL_BLOCKING);
    steppers->plate_move(slotNumber, CALL_BLOCKING); 

    
    steppers->slider_move(SliderStorage);
    usleep(450000);
    deployer->deploy(); 
    usleep(300000);
    holder->open();
    usleep(200000);

    deployer->half();
    holder->idle();
    steppers->slider_move(SliderHigh, CALL_BLOCKING); 
    steppers->plate_move(0, CALL_BLOCKING); 

    deployer->idle();
    teensy->set_position_controller_gains(0.8,2.5,-0.8,4.0);
}

/**
 * @brief Uses position control to 
 * @return 0 if completion success, -1 otherwise
*/
int8_t position_to_plant(double x_plant, double y_plant, double x_plant_center, double y_plant_center, bool isLeft, bool isFirst = false) {
    double plant_to_center_dist = hypot(y_plant_center-y_plant, x_plant_center-x_plant); 
    double alpha; 
    if (isFirst) {
        alpha = 0; 
    } else {
        if (isLeft) {
            alpha = -plant_approach_angle;
        } else {
            alpha = plant_approach_angle;
        }
    }
    printf("Plant approach distance = %.3f, isFirst = %d and alpha = %.3f \n", plant_approach_dist, isFirst, alpha); 
    // First, position control to the approach point
    double dx = (x_plant - x_plant_center)/plant_to_center_dist; 
    double dy= (y_plant - y_plant_center)/plant_to_center_dist; 
    double x_approach = x_plant + (dx*cos(alpha) + dy*sin(alpha))* plant_approach_dist; 
    double y_approach = y_plant + (-dx*sin(alpha) + dy*cos(alpha))* plant_approach_dist; 
    double theta_approach = periodic_angle(atan2(y_plant_center-y_plant, x_plant_center-x_plant)-alpha);
    printf("Position control to approach to x = %.3f, y = %.3f and theta = %.3f \n", x_approach, y_approach, theta_approach);
    shared.teensy->set_position_controller_gains(0.8,2.5,-1.75,3.0);

    if (action_position_control(x_approach, y_approach, theta_approach) == -1) return -1; 

    // Then, position control to the plant distance

    double x_grab = x_plant + (dx*cos(alpha) + dy*sin(alpha)) * plant_grab_dist; 
    double y_grab = y_plant + (-dx*sin(alpha) + dy*cos(alpha)) * plant_grab_dist; 
    // shared.teensy->set_position_controller_gains(2.0,5.0,-0.8,4.0);
    shared.teensy->set_position_controller_gains(0.8,2.5,-1.75,4.0);


    printf("Position control to grab to x = %.3f, y = %.3f and theta = %.3f \n", x_grab, y_grab, theta_approach);
    
    if (action_position_control(x_grab, y_grab, theta_approach) == -1) return -1; 
    return 0; 
}

int8_t move_back(double x_plant, double y_plant) {
    double x_pos = 0, y_pos = 0, theta_pos = 0; 
    shared.get_robot_pos(&x_pos, &y_pos, &theta_pos);
    double plant_to_bot_dist = hypot(x_plant-x_pos, y_plant-y_pos); 

    double dx = (x_pos-x_plant)/plant_to_bot_dist; 
    double dy = (y_pos-y_plant)/plant_to_bot_dist; 

    double x_away = x_pos+dx*away_distance; 
    double y_away = y_pos+dy*away_distance; 

    printf("Position control to go back to x = %.3f, y = %.3f and theta = %.3f \n", x_away, y_away, theta_pos);
    // shared.teensy->set_position_controller_gains(2.0,5.0,-0.8,4.0);

    if (action_position_control(x_away, y_away, theta_pos) == -1) return -1; 

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
            double dist_plant = isFirst ? ((shared.color == TeamBlue) ? y_new_plant : (3-y_new_plant)) : (hypot(x_pos-x_new_plant, y_pos-y_new_plant));
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
    printf("plantcounts : %d \n", plantCount);
    if (plantCount==0) {
        deleteBottomLidar(plantZone); 
        printf("Lidar sees no plant in zone ! \n");
        return -1; 
    } 
    printf("avant avant free\n");

    double dist_min_wall = 5; 
    double plant_dist_wall; 
    uint8_t closest_plant_idx = 0; 
    for (uint8_t i=0; i<plantCount; i++){
        printf("Lidar sees plant %d at x = %.3f and y = %.3f \n", i, plantZone[zoneIdx]->xPlant[i], plantZone[zoneIdx]->yPlant[i]); 
        plant_dist_wall = (shared.color == TeamBlue) ? plantZone[zoneIdx]->yPlant[i] : (3-plantZone[zoneIdx]->yPlant[i]);
        if (plant_dist_wall < dist_min_wall) {
            closest_plant_idx = i; 
            dist_min_wall = plant_dist_wall;
        }
    }
    printf("Closest plant to wall at distance = %.3f (index %d)\n", dist_min_wall, closest_plant_idx);


    *x_plant = plantZone[zoneIdx]->xPlant[closest_plant_idx];
    *y_plant = plantZone[zoneIdx]->yPlant[closest_plant_idx];
    printf("avant free\n");
    deleteBottomLidar(plantZone); 
    printf("Lidar scan end\n");
    return 0; 
}


void ActionPlants::do_action() {
    uint8_t plants_taken[6] = {0,0,0,0,0,0};
    double xpos = 0, ypos = 0, theta_pos = 0; 
    double xpos_initial = 0, ypos_initial = 0, theta_pos_initial = 0; 
    shared.get_robot_pos(&xpos_initial, &ypos_initial, &theta_pos_initial);
    
    // Positions itself in front of the plant node, without going in
    uint8_t plantsNode = path->target; 
    double *x = path->x;
    double *y = path->y;

    if (path->nNodes >= 3) {
        path->thetaEnd = atan2(y[path->nNodes-1]-y[path->nNodes-3], x[path->nNodes-1]-x[path->nNodes-3]);
        path->nNodes -= 2;
        if (path_following_to_action(path) == -1) return; 
    } else {
        path->thetaEnd = atan2(y[path->nNodes-1]-ypos_initial, x[path->nNodes-1]-xpos_initial);
        if (action_position_control(x[path->nNodes-1], y[path->nNodes-1], path->thetaEnd)) return;
    }

    shared.get_robot_pos(&xpos_initial, &ypos_initial, &theta_pos_initial);
    printf("Path following done with success \n");
    
    
    double x_plant, y_plant, theta_plant; 
    for (uint8_t plant_i = 0; plant_i < plantCounter; plant_i++) {
        // Get closest plant from lidar pov
        printf("Scanning with lidar...\n");

        shared.get_robot_pos(&xpos, &ypos, &theta_pos);
        if (get_closest_plant_from_lidar(xpos, ypos, theta_pos, plantsNode, &x_plant, &y_plant) == -1) return;
        //get_closest_plant_from_kakoo(xpos, ypos, plantsNode, &x_plant, &y_plant, plants_taken); 

        theta_plant = atan2(y_plant-ypos, x_plant-xpos); 
        printf("Got plant at %f, %f, %f, beginning the approach\n", x_plant, y_plant, theta_plant);
        if (position_to_plant(x_plant, y_plant, shared.graph->nodes[plantsNode].x, shared.graph->nodes[plantsNode].y, trigo_diff(theta_plant, theta_pos)>0, plant_i==0)) return; 
        // Get next storage slot and put the plant
        storage_slot_t nextSlot = get_next_free_slot_ID(ContainsStrongPlant); 
        int8_t plate_pos = get_plate_slot(nextSlot); 
        printf("Activating the kinematic chain\n");//, x_plant, y_plant, theta_plant);
        take_plant_kinematicChain(plate_pos); 
        update_plate_content(nextSlot, ContainsWeakPlant); 
        move_back(x_plant, y_plant); 
    }

    // Position control to the initial location with opposed theta for departure
    if (action_position_control(xpos_initial, ypos_initial, periodic_angle(theta_pos_initial-M_PI)) == -1) return; 
    
}
