#include "graph.h"
#include <stdio.h>
#include <time.h>

int main(int argc, char const *argv[])
{
    char filename[] = "./graphs/MB_V1.txt";
    if (init_graph_from_file(filename)) return -1;
    printf("Graph initialised from file %s\n", filename);
    /*for (size_t i = 0; i < graph_nb_nodes; i++)
    {
        // Prints all info on each node of the graph
        printf("%d\n",graph_nodes[i].id); 
        printf("%.3f,%.3f\n",graph_nodes[i].x, graph_nodes[i].y);
        printf("%d\n",graph_nodes[i].clearance_lvl); 
        printf("%d\n",graph_nodes[i].nb_neighbors);
        for (size_t j = 0; j < graph_nodes[i].nb_neighbors-1; j++)
        {
            printf("%d,", graph_nodes[i].neighbors[j]->id);
        }
        printf("%d\n\n", graph_nodes[i].neighbors[graph_nodes[i].nb_neighbors-1]->id);
    }*/
    
    uint8_t from = 35;
    uint8_t tos[] = {44,5};
    uint8_t len_tos = 2;
    graph_level = 1;
    graph_level_update(33,3,ENABLE_PROPAGATION);
    graph_level_update(3,NODE_DANGER,DISABLE_PROPAGATION);
    graph_level_update(42,NODE_FREE,DISABLE_PROPAGATION);
    graph_level_update(44, NODE_FREE, DISABLE_PROPAGATION);

    printf("Searching for a path between %d and {", from);
    for (uint8_t i = 0; i < len_tos-1; i++) { printf("%d,", tos[i]); }
    printf("%d} with graph level %d\n",tos[len_tos-1], graph_level);

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
    free_graph();

    return 0;
}
