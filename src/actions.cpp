#include "actions.h"
#include <cmath>
#include <algorithm>

#define VERBOSE
#include <stdio.h>

uint8_t closer_in_path(graph_path_t *path, double xr, double yr)
{
    double distance, min_distance = 3.6;
    uint8_t found_node_id = 0;

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
    for (uint8_t i = closer_node_id; i < std::min(closer_node_id + 4, (int)path->nNodes); i++)
    {
        if ((shared.graph->nodes[path->idNodes[i]].level & NODE_ADV_PRESENT) != 0) {
            shared.graph->level_unlock();
            return -1;
        }
    }
    shared.graph->level_unlock();
    return 0;
}

const double safety_distance_to_walls = 0.2;
const double safety_distance_to_pami_side_wall = 0.3;
const double dist_useless = 0.05;
uint8_t back_manoeuvre(double backward_dist) {
    //do {
    // 1) Initiate pos control gains
    shared.teensy->set_position_controller_gains(
        0.8, // kp
        8.0, // ka
        -2.0, // kb
        6.0 // kw
    );

    // 2) Get position
    double xpos = 0, ypos = 0, thetapos = 0;
    shared.get_robot_pos(&xpos, &ypos, &thetapos);

    // 3) Compute new position 
    double xnew, ynew, thetanew;
    xnew = xpos - cos(thetapos)*backward_dist;
    ynew = ypos - sin(thetapos)*backward_dist;
    thetanew = thetapos;

    // 4) While new position is too close to wall, reduce backward_dist and recompute new position
    while ((
            (xnew       < safety_distance_to_pami_side_wall) ||
            (ynew       < safety_distance_to_walls)          ||
            (2.0 - xnew < safety_distance_to_walls)          ||
            (3.0 - ynew < safety_distance_to_walls)
           )  && 
           (backward_dist > dist_useless)) 
    {
        backward_dist = backward_dist - 0.01;
        xnew = xpos - cos(thetapos)*backward_dist;
        ynew = ypos - sin(thetapos)*backward_dist;
    }

    // 5) If robot is far away enough from a wall, do back manoeuver
    bool manoeuver_useless = (backward_dist <= dist_useless);
    if (!manoeuver_useless) {
        // 5.1) Do position control to computed position
        shared.teensy->pos_ctrl(xnew, ynew, thetanew);
        
        // 5.2) Wait for position control to be over
        while ((shared.teensy->ask_mode()) != ModePositionControlOver) usleep(10000);
    }
    // } while (shared.graph->identify_pos(xpos,ypos).level & NODE_ADV_PRESENT != 0)

    return 1;
}

int8_t path_following_to_action(graph_path_t *path)
{

    Graph::print_path(path);

    printf("Entering path following to action\n"); 

    Teensy *teensy = shared.teensy;
    teensy->idle();
    usleep(50000);

    // Set path following from path planning (decision)
    int ncheckpoints = (int)path->nNodes;
    double *x = path->x;
    double *y = path->y;
    /*for (int i=0; i<ncheckpoints; i++) {
        printf("Node %d : x :%.3f and y: %.3f \n", i, x[i], y[i]);
    }*/
    double theta_start = path->thetaStart;
    double theta_end = path->thetaEnd;

    double kp = 1.0;
    double ka = 6.0;
    double kb = -1.0;
    double kw = 4.0;
    teensy->set_position_controller_gains(kp, ka, kb, kw);

    double xCurrent = 0, yCurrent = 0;
    shared.get_robot_pos(&xCurrent, &yCurrent, NULL);

    double first_node_theta = atan2(y[1] - yCurrent, x[1] - xCurrent);
    #ifdef VERBOSE
    //printf("First node theta : %.3f and current theta : %.3f \n", first_node_theta, theta_start);
    #endif

    uint8_t teensyStart = 0;

    if (trigo_diff(first_node_theta, theta_start) > M_PI_4)
    {
        // double towardsCenter = atan2(1.5 - yCurrent, 1 - xCurrent);
        // teensy->pos_ctrl(xCurrent, yCurrent, towardsCenter);
        // sleep(1);
        shared.teensy_reset_pos();
        shared.get_robot_pos(&xCurrent, &yCurrent, NULL);
        teensy->pos_ctrl(xCurrent, yCurrent, first_node_theta);
        #ifdef VERBOSE
        printf("Position control before PF to %.3f, %.3f, %.3f\n", xCurrent, yCurrent, first_node_theta);
        #endif
        while (teensy->ask_mode() == ModePositionControlOver || teensy->ask_mode() == ModeIdle) {
            teensyStart++;
            if (teensyStart > 60) {
                printf("Relaunching pos control to theta first node\n");
                teensy->idle();
                usleep(50000);
                shared.get_robot_pos(&xCurrent, &yCurrent, NULL);
                teensy->pos_ctrl(xCurrent, yCurrent, first_node_theta);
            }
            usleep(50000);
        }
        while ((teensy->ask_mode()) != ModePositionControlOver)
        {
            usleep(10000);
        }
    }
    #ifdef VERBOSE
    printf("PF\n");
    #endif
    /*for (int i=0; i<ncheckpoints; i++) {
        printf("Node %d : x :%.3f and y: %.3f \n", i, x[i], y[i]);
    }*/
    shared.teensy_reset_pos();
    teensy->path_following(x, y, ncheckpoints, first_node_theta, theta_end, vref, dist_goal_reached);

    // Check Teensy mode
    #ifdef VERBOSE
    //printf("Wait for Teensy ok\n");
    #endif
    while (teensy->ask_mode() == ModePositionControlOver || teensy->ask_mode() == ModeIdle) { 
        teensyStart++;
        if (teensyStart > 60) {
            printf("Relaunching path following due to mode unchanged\n");
            teensy->idle();
            usleep(50000);
            shared.get_robot_pos(&xCurrent, &yCurrent, NULL);
            teensy->pos_ctrl(xCurrent, yCurrent, first_node_theta);
            printf("Starting position control to (%f,%f,%f)\n", xCurrent, yCurrent, first_node_theta);
            sleep(1);
            teensy->path_following(x, y, ncheckpoints, theta_start, theta_end, vref, dist_goal_reached);
            for (int i = 0; i < ncheckpoints; i++) {
                printf("Node %d; (%f,%f)\n", i, x[i], y[i]);
            }
            teensyStart = 0;
        } 
        usleep(300000);
    }
    teensyStart = 0;
    int16_t teensyReset = 0;
    while ((teensy->ask_mode()) != ModePositionControlOver)
    {
        // if (!shared.goingToBase && shared.update_and_get_timer() < 35) { // TODO Rework timer aborts
        //     printf("Path follwing aborted because of timer\n");
        //     return -1;
        // }
        
        // Get update on robot and adversary position
        double xr = 0, yr = 0, tr = 0;
        shared.get_robot_pos(&xr, &yr, &tr);
        double xa = 0, ya = 0, da = 0, ta = 0;
        shared.get_adv_pos(&xa, &ya, &da, &ta);

        // Get closer in path
        uint8_t closer_node_id = closer_in_path(path, xr, yr);

        // Check advsersary in path
        if (adversary_in_path(path, closer_node_id) == -1)
        {
            teensy->idle();
            #ifdef VERBOSE
            printf("Adversary in path !! %d %d\n", xa, ya, da, ta);
            #endif
            usleep(200000);
            shared.teensy_reset_pos();
            return -1;
        }

        // Security check: adversary too close
        double tolerance = 0.75;
        //printf("da =%f, ta : %f \n", da, ta);
        if ((da < tolerance) && (abs(ta) < M_PI/3))
        {
            #ifdef VERBOSE
            printf("Adversary too close %f %f !!\n", da, ta);
            #endif
            teensy->idle();
            usleep(300000);
            shared.teensy_reset_pos();
            back_manoeuvre(0.6);
            return -1;
        }

        if (hypot(xr-xCurrent,yr-yCurrent) < 0.03) {
            teensyStart++;
            if (teensyStart > 100) {
                printf("Relaunching path following due to absence of movement\n");
                teensy->idle();
                usleep(50000);
                teensy->path_following(x,y,ncheckpoints,theta_start,theta_end,vref, dist_goal_reached);
                teensyStart = 0;
            }
        }

        if (teensy->ask_mode() == ModeIdle) {

            #ifdef VERBOSE
            printf("Teensy unstable, restarting\n");
            #endif
            shared.teensy_reset_pos();

            if (closer_node_id >= path->nNodes-2) {
                teensy->pos_ctrl(x[path->nNodes-1], y[path->nNodes-1], theta_end);
            } else {
                closer_node_id++;

                double theta_recover = atan2(shared.graph->nodes[path->idNodes[closer_node_id]].y - yr,
                                             shared.graph->nodes[path->idNodes[closer_node_id]].x - xr );

                teensy->pos_ctrl(xr, yr, theta_recover);
                sleep(2);
                ncheckpoints -= closer_node_id;
                x += closer_node_id;
                y += closer_node_id;
                teensy->path_following(x,y,ncheckpoints,theta_recover, theta_end, vref, dist_goal_reached);
            }
        }


        if (++teensyReset >= 20) {
            shared.teensy_reset_pos();
            teensyReset = 0;
        }
        usleep(50000);
    }

    shared.teensy_reset_pos();
    return 0; // Adversary not found and successfull path following
}

int8_t action_position_control(double x_end, double y_end, double theta_end, double pos_tol, double angle_tol)
{
    shared.teensy_reset_pos();
    Teensy *teensy = shared.teensy;

    // Set position control gains (see with Ethan?)
    // double kp = 0.8;
    // double ka = 2.5;
    // double kb = -0.5;
    // double kw = 4.0;
    // teensy->set_position_controller_gains(kp, ka, kb, kw);

    // Retrieve robot current position
    double x = 0, y = 0, theta = 0;
    shared.get_robot_pos(&x, &y, &theta);
    #ifdef VERBOSE
    printf("Robot position from shared: %.3f, %.3f, %.3f \n", x, y, theta);
    #endif
    double alpha = std::abs(atan2(y_end - y, x_end - x) - theta);
    uint8_t reverse = (alpha > M_PI_2 && alpha < 3*M_PI_2);
    uint8_t stopped = 0;

    // Orientation with position control

    uint8_t teensyStart = 0;
    teensy->pos_ctrl(x_end, y_end, theta_end);
    while (teensy->ask_mode() == ModePositionControlOver || teensy->ask_mode() == ModeIdle) {
        teensyStart++;
        if (teensyStart > 60) {
            printf("Relaunching pos control to theta first node\n");
            teensy->idle();
            usleep(50000);
            shared.get_robot_pos(&x, &y, NULL);
            teensy->pos_ctrl(x_end, y_end, theta_end);
        }
        usleep(50000);
    }

    // Waiting end to start function turn_solar_panel
    // Check Teensy mode
    while ((teensy->ask_mode()) != ModePositionControlOver)
    {

        // if (!shared.goingToBase && shared.update_and_get_timer() < 30) { // TODO Rework timer aborts
        //     printf("Position control aborted because of timer\n");
        //     return -1;
        // }

        // Retrieve adversary current position
        double d_adv = 0, a_adv = 0;
        shared.get_adv_pos(NULL, NULL, &d_adv, &a_adv);
        #ifdef VERBOSE
        //printf("Adversary position from shared: %.3f, %.3f\n", d_adv, a_adv);
        #endif

        if ((!reverse && d_adv < 0.4 && std::abs(a_adv) < M_PI/3) || 
            (reverse && d_adv < 0.2 && std::abs(a_adv) > 2*M_PI/3)) {
            stopped++;
            if (stopped == 1) {
                teensy->idle();
                printf("Adversary too close for position control !!\n");
            }
            if (stopped >= 300) {
                // teensy->set_position_controller_gains(kp, ka, kb, kw);
                shared.teensy_reset_pos();
                return -1;
            } 
        } else if (stopped) {
            stopped = 0;
            teensy->pos_ctrl(x_end, y_end, theta_end);
        }
        if (pos_tol != DEFAULT_DIST_TOL || angle_tol != DEFAULT_ANGLE_TOL) {
            shared.get_robot_pos(&x, &y, &theta);
            if (hypot(x_end-x, y_end-y)< pos_tol && fabs(trigo_diff(theta_end, theta))<angle_tol*M_PI/180.0) {
                printf("Checking non-defult tolerances succeded with robot in %.3f, %.3f, %.3f\n",x,y,theta); 
                shared.teensy->idle(); 
                shared.teensy_reset_pos();
                return 0; 
            }
        }

        usleep(10000);
    }
    // teensy->set_position_controller_gains(kp, ka, kb, kw);
    shared.teensy_reset_pos();
    return 0;
}

int8_t get_plate_slot(storage_slot_t slotID) {
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
        printf("Invalid plate slot ! {%d}\n", slotID);
        return 0;
        break;
    }
    return plateSlotID; 
}

storage_slot_t get_next_unloaded_slot_ID (storage_content_t content) {
    storage_slot_t unloaded_slot_order[6] = {Slot1, SlotM1, Slot2, SlotM2, Slot3, SlotM3}; 
    storage_content_t *storage = shared.storage;
    for (uint8_t i = 0; i < 6; i++) {
        if ((storage[unloaded_slot_order[i]] & content) == content) {
            return unloaded_slot_order[i]; 
        }
    }
    return SlotInvalid; 
}


storage_slot_t get_next_free_slot_ID (storage_content_t content) {
    storage_slot_t loading_slot_order[6] = {SlotM3, Slot3, SlotM2, Slot2, SlotM1, Slot1}; 
    storage_content_t *storage = shared.storage;
    for (uint8_t i = 0; i < 6; i++) {
        if (!(storage[loading_slot_order[i]] & content)) {
            return loading_slot_order[i]; 
        }
    }
    return SlotInvalid; 
}

void update_plate_content(storage_slot_t slotID, storage_content_t content) {
    if(content == ContainsNothing) {
        shared.storage[slotID] = ContainsNothing; 
    } else {
        shared.storage[slotID] = (storage_content_t) (((uint8_t) content) | ((uint8_t)shared.storage[slotID] )); 
    }
}
int8_t get_content_count(storage_content_t content) {
    int8_t count = 0; 
    for (int8_t i=SlotM3; i<SlotFlaps; i++) {
        count += (((uint8_t)shared.storage[i]) & content) !=0; 
    }
    return count; 
}

double periodic_angle(double angle) {
    if (angle > M_PI) return angle-2*M_PI; 
    if (angle < -M_PI) return angle+2*M_PI; 
    return angle;
}

double trigo_diff(double theta_a, double theta_b) {
    double diff = theta_a-theta_b;
    return periodic_angle(diff);
}