#include "graph.h"
#include <stdio.h>
#include <time.h>

int main(int argc, char const *argv[])
{
    int from = 0;
    int to = 40;
    if (init_graph_from_file("./graphs/V1.txt")) return -1;
    /*for (size_t i = 0; i < graph_nb_nodes; i++)
    {
        printf("%d\n",graph_nodes[i].id); 
        printf("%.3f,%.3f\n",graph_nodes[i].x, graph_nodes[i].y);
        printf("%d\n",graph_nodes[i].penalty); 
        printf("%d\n",graph_nodes[i].nb_neighbors);
        for (size_t j = 0; j < graph_nodes[i].nb_neighbors-1; j++)
        {
            printf("%d,", graph_nodes[i].neighbors[j]->id);
        }
        printf("%d\n\n", graph_nodes[i].neighbors[graph_nodes[i].nb_neighbors-1]->id);
    }*/
    
    printf("Graph initialised\n");
    clock_t start = clock();
    ASPath result = compute_path(from,to);
    clock_t stop = clock();
    if (result == NULL) return -1;
    size_t count = ASPathGetCount(result);
    float cost = ASPathGetCost(result);
    printf("Found a path in %ld points, total length = %.3f\n", count, cost);
    for (size_t i = 0; i < count; i++)
    {
        printf("%d ", ((graph_node_t*)ASPathGetNode(result, i))->id);
    }
    printf("\n");
    printf("Time taken : %ld\n", stop-start);
    free_graph();

    return 0;
}
