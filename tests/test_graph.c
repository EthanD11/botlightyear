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
    graph_level_update(24,2,1);
    graph_level_update(3,1,0);
    graph_level_update(42,0,0);
    //graph_nodes[graph_bases[0]].level = 0; // equivalent
    graph_nodes[44].level = 0;

    printf("Searching for a path between %d and %d with graph level %d\n", from, to, graph_level);

    clock_t start = clock();
    ASPath result = graph_compute_path(from,to);
    clock_t stop = clock();

    if (result == NULL) {
        printf("No path was found\n");
        return 0;
    }
    size_t count = ASPathGetCount(result);
    float cost = ASPathGetCost(result);
    printf("Found a path in %ld points, total length = %.3f\n", count, cost);
    for (size_t i = 0; i < count; i++)
    {
        printf("%d ", ((graph_node_t*)ASPathGetNode(result, i))->id);
    }
    printf("\nTime taken : %ld clock cycles\n", stop-start);
    ASPathDestroy(result);
    free_graph();

    return 0;
}
