#include "graph.h"
#include <stdio.h>
#include <time.h>

Graph graph = Graph();

int main(int argc, char const *argv[])
{
    char filename[] = "./graphs/BL_V3.txt";
    if (graph.init_from_file(filename, TeamBlue)) return -1;
    printf("Graph initialised from file %s\n", filename);
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

    double x = 1.0;
    double y = 1.0;
    double dist;
    uint8_t id = graph.identify_pos(x,y,&dist);
    printf("Position (%.3f,%.3f) identifies with node %d with an error of %.3f\n", x, y, id, dist);

    free(result);

    return 0;
}
