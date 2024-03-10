#include "graph.h"
#include <stdio.h>
#include <time.h>

int main(int argc, char const *argv[])
{
    char *filename = "./graphs/MB_V1.txt";
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
    
    int from = 0;
    int to = 44;
    graph_level = 0;
    graph_level_update(24,NODE_BLOCKED,ENABLE_PROPAGATION);
    graph_level_update(3,NODE_DANGER,DISABLE_PROPAGATION);
    graph_level_update(42,NODE_FREE,DISABLE_PROPAGATION);
    graph_level_update(44, NODE_FREE, DISABLE_PROPAGATION);

    printf("Searching for a path between %d and %d with graph level %d\n", from, to, graph_level);

    clock_t start = clock();
    graph_path_t* result = graph_compute_path(from,to);
    clock_t stop = clock();

    if (result == NULL) {
        printf("No path was found\n");
        return 0;
    }
    printf("Found a path in %d points\n", result->nb_nodes);
    for (size_t i = 0; i < result->nb_nodes; i++)
    {
        printf("(%.3f,%.3f) ", result->x[i], result->y[i]);
    }
    printf("\nTime taken : %ld clock cycles\n", stop-start);
    free(result);
    free_graph();

    return 0;
}
