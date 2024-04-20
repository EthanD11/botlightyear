#include "graph.h"
#include <stdio.h>
#include <time.h>
#include <cmath>
#include <algorithm>

Graph graph = Graph();


uint8_t closer_in_path(graph_path_t *path, double xr, double yr)
{
    double distance, min_distance = 3.6;
    uint8_t found_node_id;

    for (uint8_t i = 0; i < path->nNodes; i++)
    {
        double x_node = graph.nodes[path->idNodes[i]].x;
        double y_node = graph.nodes[path->idNodes[i]].y;

        distance = sqrt((x_node - xr) * (x_node - xr) + (y_node - yr) * (y_node - yr));
        if (distance < min_distance)
        {
            found_node_id = i;
            min_distance = distance;
        }
    }
    printf("found node id : %d\n", found_node_id);
    return found_node_id;
}

uint8_t adversary_in_path(graph_path_t *path, uint8_t closer_node_id)
{
    graph.level_rdlock();
    for (uint8_t i = closer_node_id; i < std::min(closer_node_id + 3, (int)path->nNodes); i++)
    {
        printf("%d, %d\n", graph.nodes[path->idNodes[i]].level, graph.nodes[path->idNodes[i]].level & NODE_ADV_PRESENT);
        if ((graph.nodes[path->idNodes[i]].level & NODE_ADV_PRESENT) != 0) {
            graph.level_unlock();
            return -1;
        }
    }
    graph.level_unlock();
    return 0;
}

int main(int argc, char const *argv[])
{
    char filename[] = "./graphs/BL_V3.txt";
    if (graph.init_from_file(filename, TeamBlue)) return -1;
    printf("Graph initialised from file %s\n", filename);
    graph.print();
    /*for (size_t i = 0; i < graphNbNodes; i++)
    {
        // Prints all info on each node of the graph
        printf("%d\n",graphNodes[i].id); 
        printf("%.3f,%.3f\n",graphNodes[i].x, graphNodes[i].y);
        printf("%d\n",graphNodes[i].clearance_lvl); 
        printf("%d\n",graphNodes[i].nNeighbors);
        for (size_t j = 0; j < graphNodes[i].nNeighbors-1; j++)
        {
            printf("%d,", graphNodes[i].neighbors[j]->id);
        }
        printf("%d\n\n", graphNodes[i].neighbors[graphNodes[i].nNeighbors-1]->id);
    }*/
    
    double xFrom = 0.435, yFrom = 1.860; // 20
    uint8_t tos[] = {41,45};
    uint8_t len_tos = 2;
    graph.level = 0;
    /*graph.update_obstacle(10,1);
    graph.update_obstacle(15,1);
    graph.update_obstacle(20,1);
    graph.update_obstacle(25,1);
    graph.update_adversary_pos(1.0,1.0);
    graph.update_adversary_pos(0.5,2.0);*/

    printf("Searching for a path between (%.3f,%.3f) and {", xFrom, yFrom);
    for (uint8_t i = 0; i < len_tos-1; i++) { printf("%d,", tos[i]); }
    printf("%d} with graph level %d\n",tos[len_tos-1], graph.level);

    clock_t start = clock();
    graph_path_t* result = graph.compute_path(xFrom, yFrom, tos, len_tos);
    clock_t stop = clock();

    if (result == NULL) printf("No path was found\n");
    else {
        Graph::print_path(result);
        printf("\nTime taken : %ld clock cycles\n", stop-start);
    }

    graph.update_adversary_pos(0.500, 0.7);
    printf("AAA\n");


    uint8_t node = closer_in_path(result, 0.500,1.550);
    printf("Current node : %d\n", node);

    uint8_t advInPath = adversary_in_path(result, node);
    if (advInPath) printf("Adv in path\n");

    

    /*double x = 1.0;
    double y = 1.0;
    double dist;
    uint8_t id = graph.identify_pos(x,y,&dist);
    printf("Position (%.3f,%.3f) identifies with node %d with an error of %.3f\n", x, y, id, dist);*/

    free(result);

    return 0;
}
