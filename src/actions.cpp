#include "actions.h"
#include <cmath>
#include <algorithm>

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

    for (uint8_t i = closer_node_id; i < std::min(closer_node_id + 2, (int)path->nNodes); i++)
    {
        if ((shared.graph->nodes[path->idNodes[i]].level & NODE_ADV_PRESENT) != 0)
            return -1;
    }

    return 0;
}

int8_t path_following_to_action(graph_path_t *path)
{

    Graph::print_path(path);

    Teensy *teensy = shared.teensy;

    // Set path following from path planning (decision)
    int ncheckpoints = (int)path->nPoints;
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

    double kt = 2.0;
    double kn = 0.3; // 0 < kn <= 1
    double kz = 25.0;
    double delta = 20e-3; // delta is in meters
    double sigma = 10;
    double epsilon = M_PI / 8; // epsilon is in radians
    double wn = 0.2;           // Command filter discrete cutoff frequency
    double kv_en = 0.;
    teensy->set_path_following_gains(kt, kn, kz, sigma, epsilon, kv_en, delta, wn);

    double xCurrent, yCurrent;
    double new_x[path->nPoints + 1]; double new_y[path->nPoints + 1];
    shared.get_robot_pos(&xCurrent, &yCurrent, NULL);
    double dist_tol = 0.15;
    if (hypot(xCurrent - x[0], yCurrent - y[0]) < dist_tol)
    {
        x[0] = xCurrent;
        y[0] = yCurrent;
    }
    else
    {
        new_x[0] = xCurrent;
        new_y[0] = yCurrent;
        for (uint8_t i = 0; i < path->nPoints; i++)
        {
            new_x[i + 1] = x[i];
            new_y[i + 1] = y[i];
        }
        x = new_x;
        y = new_y;
        ncheckpoints++;
    }

    double first_node_theta = atan2(y[1] - yCurrent, x[1] - xCurrent);
    printf("First node theta : %.3f and current theta : %.3f \n", first_node_theta, theta_start);

    if (abs(first_node_theta - theta_start) > M_PI_4)
    { //&& abs(first_node_theta - theta_start) < 3*M_PI_4
        teensy->pos_ctrl(xCurrent, yCurrent, first_node_theta);
        printf("Position control before PF to %.3f, %.3f, %.3f\n", xCurrent, yCurrent, first_node_theta);
        while ((teensy->ask_mode()) != ModePositionControlOver)
        {
            usleep(10000);
        }
    }
    printf("PF\n");
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
            printf("Adversary in path !!\n");
            free(path);
            sleep(1);
            return -1;
        }

        // Security check: adversary too close
        double tolerance = 0.6;
        if ((da < tolerance) && (abs(ta) < M_PI / 2))
        {
            printf("Adversary too close !!\n");
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
    Odometry *odo = shared.odo;

    // Retrieve robot current position
    double x, y, theta;
    shared.get_robot_pos(&x, &y, &theta);
#ifdef VERBOSE
    printf("Robot position from shared: %.3f, %.3f, %.3f \n", x, y, theta);
#endif

    // Retrieve adversary current position
    double x_adv, y_adv;
    shared.get_adv_pos(&x_adv, &y_adv, NULL, NULL);
#ifdef VERBOSE
    printf("Adversary position from shared: %.3f, %.3f\n", x_adv, y_adv);
#endif

    // Set position control gains (see with Ethan?)
    double kp = 0.8;
    double ka = 2.5;
    double kb = -0.5;
    double kw = 4.0;
    teensy->set_position_controller_gains(kp, ka, kb, kw);

    // Define trajectory
    int ncheckpoints = 2;
    double xc[2] = {x, x_end};
    double yc[2] = {y, y_end};
    double theta_start = theta;

#ifdef VERBOSE
    printf("Checkpoints relay: %.3f, %.3f\n", xc[0], yc[0]);
    printf("Checkpoints target: %.3f, %.3f\n", xc[1], yc[1]);
#endif

    // Orientation with position control
    teensy->pos_ctrl(xc[1], yc[1], theta_end);
    usleep(1);

    // Waiting end to start function turn_solar_panel
    // Check Teensy mode
    while ((teensy->ask_mode()) != ModePositionControlOver)
    {
        usleep(10000);
    }
    return 0;
}