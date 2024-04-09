#include "graph.h"
#include <stdio.h>
#include <time.h>

int main(int argc, char const *argv[])
{
    char filename[] = "./graphs/BL_V2.txt";
    if (init_graph_from_file(filename, TeamBlue)) return -1;
    printf("Graph initialised from file %s\n", filename);
    /*for (size_t i = 0; i < graphNbNodes; i++)
    {
        // Prints all info on each node of the graph
        printf("%d\n",graphNodes[i].id); 
        printf("%.3f,%.3f\n",graphNodes[i].x, graphNodes[i].y);
        printf("%d\n",graphNodes[i].clearance_lvl); 
        printf("%d\n",graphNodes[i].nb_neighbors);
        for (size_t j = 0; j < graphNodes[i].nb_neighbors-1; j++)
        {
            printf("%d,", graphNodes[i].neighbors[j]->id);
        }
        printf("%d\n\n", graphNodes[i].neighbors[graphNodes[i].nb_neighbors-1]->id);
    }*/
    
    uint8_t from = 20;
    uint8_t tos[] = {41,45};
    uint8_t len_tos = 2;
    graphLevel = 0;
    graph_level_update(10,3,ENABLE_PROPAGATION);

    printf("Searching for a path between %d and {", from);
    for (uint8_t i = 0; i < len_tos-1; i++) { printf("%d,", tos[i]); }
    printf("%d} with graph level %d\n",tos[len_tos-1], graphLevel);

    clock_t start = clock();
    graph_path_t* result = graph_compute_path(from, tos, len_tos, 0);
    clock_t stop = clock();

    if (result == NULL) {
        printf("No path was found\n");
        return 0;
    }
    print_path(result);
    printf("\nTime taken : %ld clock cycles\n", stop-start);

    double x = 0.0;
    double y = -1.0;
    double dist;
    uint8_t id = graph_identify_pos(x,y,&dist);
    printf("Position (%.3f,%.3f) identifies with node %d with an error of %.3f\n", x, y, id, dist);

    free(result);
    graph_free();

    return 0;
}
