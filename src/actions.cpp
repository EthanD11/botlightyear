#include "actions.h"
#include <cmath>
#include <algorithm>

//#define VERBOSE
#include <stdio.h>

uint8_t closer_in_path(graph_path_t *path, double xr, double yr)
{
    double distance, min_distance = 3.6;
    uint8_t found_node_id;

    for (uint8_t i = 0; i < path->nNodes; i++)
    {
        double x_node = shared.graph->nodes[path->idNodes[i]].x;
        double y_node = shared.graph->nodes[path->idNodes[i]].y;

        distance = sqrt((x_node - xr) * (x_node - xr) + (y_node - yr) * (y_node - yr));
        if (distance < min_distance)
        {
            found_node_id = i;
            min_distance = distance;
        }
    }
    return found_node_id;
}

uint8_t adversary_in_path(graph_path_t *path, uint8_t closer_node_id)
{
    shared.graph->level_rdlock();
    for (uint8_t i = closer_node_id; i < std::min(closer_node_id + 2, (int)path->nNodes); i++)
    {
        if ((shared.graph->nodes[path->idNodes[i]].level & NODE_ADV_PRESENT) != 0)
            return -1;
    }
    shared.graph->level_unlock();
    return 0;
}

int8_t path_following_to_action(graph_path_t *path)
{

    Graph::print_path(path);

    Teensy *teensy = shared.teensy;

    // Set path following from path planning (decision)
    int ncheckpoints = (int)path->nNodes;
    double *x = path->x;
    double *y = path->y;
    // for (int i=0; i<ncheckpoints; i++) {
    //     printf("Node %d : x :%.3f and y: %.3f \n", i, x[i], y[i]);
    // }
    double theta_start = path->thetaStart;
    double theta_end = path->thetaEnd;

    double kp = 0.8;
    double ka = 2.5;
    double kb = -0.8;
    double kw = 4.0;
    teensy->set_position_controller_gains(kp, ka, kb, kw);

    /*double kt = 2.0;
    double kn = 0.3; // 0 < kn <= 1
    double kz = 25.0;
    double delta = 20e-3; // delta is in meters
    double sigma = 10;
    double epsilon = M_PI / 8; // epsilon is in radians
    double wn = 0.2;           // Command filter discrete cutoff frequency
    double kv_en = 0.;
    teensy->set_path_following_gains(kt, kn, kz, sigma, epsilon, kv_en, delta, wn);*/

    double xCurrent, yCurrent;
    shared.get_robot_pos(&xCurrent, &yCurrent, NULL);

    double first_node_theta = atan2(y[1] - yCurrent, x[1] - xCurrent);
    #ifdef VERBOSE
    printf("First node theta : %.3f and current theta : %.3f \n", first_node_theta, theta_start);
    #endif

    if (abs(first_node_theta - theta_start) > M_PI_4)
    { //&& abs(first_node_theta - theta_start) < 3*M_PI_4
        teensy->pos_ctrl(xCurrent, yCurrent, first_node_theta);
        #ifdef VERBOSE
        printf("Position control before PF to %.3f, %.3f, %.3f\n", xCurrent, yCurrent, first_node_theta);
        #endif
        while ((teensy->ask_mode()) != ModePositionControlOver)
        {
            usleep(10000);
        }
    }
    #ifdef VERBOSE
    printf("PF\n");
    #endif
    // for (int i=0; i<ncheckpoints; i++) {
    //     printf("Node %d : x :%.3f and y: %.3f \n", i, x[i], y[i]);
    // }

    teensy->path_following(x, y, ncheckpoints, theta_start, theta_end, vref, dist_goal_reached);

    // Check Teensy mode
    usleep(100000);
    while ((teensy->ask_mode()) != ModePositionControlOver)
    {

        // Get update on robot and adversary position
        double xr, yr, tr;
        shared.get_robot_pos(&xr, &yr, &tr);
        double xa, ya, da, ta;
        shared.get_adv_pos(&xa, &ya, &da, &ta);

        // Get closer in path
        uint8_t closer_node_id = closer_in_path(path, xr, yr);

        // Check advsersary in path
        if (adversary_in_path(path, closer_node_id) == -1)
        {
            teensy->idle();
            #ifdef VERBOSE
            printf("Adversary in path !!\n");
            #endif
            free(path);
            sleep(1);
            return -1;
        }

        // Security check: adversary too close
        double tolerance = 0.6;
        if ((da < tolerance) && (abs(ta) < M_PI / 2))
        {
            #ifdef VERBOSE
            printf("Adversary too close !!\n");
            #endif
            teensy->idle();
            free(path);
            sleep(1);
            return -1;
        }

        usleep(1000);
    }

    free(path);
    usleep(3000000);
    return 0; // Adversary not found and successfull path following
}

int8_t action_position_control(double x_end, double y_end, double theta_end)
{

    Teensy *teensy = shared.teensy;

    // Set position control gains (see with Ethan?)
    double kp = 0.8;
    double ka = 2.5;
    double kb = -0.5;
    double kw = 4.0;
    teensy->set_position_controller_gains(kp, ka, kb, kw);

    // Retrieve robot current position
    double x, y, theta;
    shared.get_robot_pos(&x, &y, &theta);
    #ifdef VERBOSE
    printf("Robot position from shared: %.3f, %.3f, %.3f \n", x, y, theta);
    #endif
    double alpha = std::abs(atan2(y_end - y, x_end - x) - theta);
    uint8_t reverse = (alpha > M_PI_2 && alpha < 3*M_PI_2);
    uint8_t stopped = 0;

    // Orientation with position control
    teensy->pos_ctrl(x_end, y_end, theta_end);
    usleep(1000);

    // Waiting end to start function turn_solar_panel
    // Check Teensy mode
    while ((teensy->ask_mode()) != ModePositionControlOver)
    {

        // Retrieve adversary current position
        double d_adv, a_adv;
        shared.get_adv_pos(NULL, NULL, &d_adv, &a_adv);
        #ifdef VERBOSE
        printf("Adversary position from shared: %.3f, %.3f\n", x_adv, y_adv);
        #endif

        if ((!reverse && d_adv < 0.4 && std::abs(a_adv) < M_PI/3) || 
            (reverse && d_adv < 0.2 && std::abs(a_adv) > 2*M_PI/3)) {
            stopped++;
            if (stopped == 1) {
                teensy->idle();
                printf("Adversary too close for position control !!\n");
            }
            if (stopped >= 300) return -1;
        } else if (stopped) {
            stopped = 0;
            teensy->pos_ctrl(x_end, y_end, theta_end);
        }

        usleep(10000);
    }
    return 0;
}

int8_t get_plate_slot_ID(storage_slot_t slotID) {
    int8_t plateSlotID; 
    switch (slotID)
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
    default: 
        printf("Invalid plate slot ! {%d}\n", slot);
        return 0;
        break;
    }
    return plateSlotID; 
}

storage_slot_t get_next_unloaded_slot_ID (storage_content_t content) {
    storage_slot_t unloaded_slot_order[6] = {Slot1, SlotM1, Slot2, SlotM2, Slot3, SlotM3}; 
    storage_content_t storage = shared.storage;
    for (uint8_t i=0; i<6; i++) {
        if (storage[unloaded_slot_order[i]]&content) {
            return unloaded_slot_order[i]; 
        }
    }
    return SlotInvalid; 
}


storage_slot_t get_next_free_slot_ID (storage_content_t content) {
    storage_slot_t loading_slot_order[6] = {SlotM3, Slot3, SlotM2, Slot2, SlotM1, Slot1}; 
    storage_content_t storage = shared.storage;
    for (uint8_t i=0; i<6; i++) {
        if (!(storage[loading_slot_order[i]]&content)) {
            return loading_slot_order[i]; 
        }
    }
    return SlotInvalid; 
}

void update_plate_content(storage_slot_t slotID, storage_content_t content) {
    if(content == ContainsNothing) {
        shared.storage[slotID] = ContainsNothing; 
    } else {
        shared.storage[slotID] |= content; 
    }
}
