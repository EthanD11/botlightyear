#include "graph.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

//#define VERBOSE

void node_neighbors(ASNeighborList neighbors, void* node, void* context) {
    graph_node_t* cur_node = (graph_node_t*) node;
    for (int i = 0; i < cur_node->nb_neighbors; i++)
    {
        graph_node_t* neighbor = cur_node->neighbors[i];
        if (neighbor->penalty == NODE_BLOCKED) continue;
        float dx = neighbor->x-cur_node->x;
        float dy = neighbor->y-cur_node->y;
        float cost = hypot(dx, dy) + (neighbor->penalty == NODE_DANGER) ? NODE_DANGER_PENALTY : 0.0;
        ASNeighborListAdd(neighbors, neighbor, cost);
    }
}

float path_cost_heuristic(void* fromNode, void* toNode, void* context) {
    graph_node_t* from = (graph_node_t*) fromNode;
    graph_node_t* to = (graph_node_t*) fromNode;
    float dx = to->x-from->x;
    float dy = to->y-from->y;
    return hypot(dx, dy) + (to->penalty == NODE_DANGER) ? NODE_DANGER_PENALTY : 0.0;
}

int node_comparator(void *node1, void *node2, void *context) {
    return ((graph_node_t*) node1)->id - ((graph_node_t*) node2)->id;
}

int init_graph_from_file(const char *filename) {
    FILE *input_file = fopen(filename, "r");
    if (input_file == NULL) return -1;

    // Scan number of nodes
    if (fscanf(input_file, "Number of nodes : %hhd\n", &graph_nb_nodes) != 1) return -1;
    #ifdef VERBOSE
    printf("Number of nodes : %d\n", graph_nb_nodes);
    #endif
    
    // Malloc graph
    graph_nodes = malloc(graph_nb_nodes*sizeof(graph_node_t));
    if (graph_nodes == NULL) return -1;

    for (int8_t i = 0; i < graph_nb_nodes; i++) {

        // Set id
        graph_nodes[i].id = i;

        // Scan x and y coordinates
        if (fscanf(input_file, "%f,%f\n", &(graph_nodes[i].x), &(graph_nodes[i].y)) != 2) return -1;
        #ifdef VERBOSE
        printf("%.3f,%.3f\n", graph_nodes[i].x, graph_nodes[i].y);
        #endif
    }

    // Scan neighbors lists
    // One line = the list of comma-separated neighbors of one node
    char list[64];
    for (size_t i = 0; i < 64; i++){ list[i] = 0; }
    char *token;
    int8_t node_id;
    for (int8_t i = 0; i < graph_nb_nodes; i++)
    {
        graph_nodes[i].nb_neighbors = 0;
        for (size_t i = 0; i < 64; i++){ list[i] = 0; }
        if (fscanf(input_file, "%s\n", list) != 1) return -1; // Get next line

        token = strtok(list, ","); // Split line based on commas, get 1st token
        while (token != NULL) {

            if (sscanf(token,"%hhd", &node_id) != 1) return -1; // Interpret token as a neighbor id

            graph_nodes[i].neighbors[graph_nodes[i].nb_neighbors] = &(graph_nodes[node_id]); // Add to list of neighbors
            graph_nodes[i].nb_neighbors++;
            #ifdef VERBOSE
            printf("%d", node_id);
            #endif
            token = strtok(NULL,","); // Get next token, returns NULL if no more token left
            #ifdef VERBOSE
            if (token != NULL) printf(",");
            #endif
        }
        #ifdef VERBOSE
        printf("\n");
        #endif
    }

    // Scan bases nodes, assign danger level penalty
    if (fscanf(input_file, "Bases : %s\n", list) != 1) return -1;
    #ifdef VERBOSE
    printf("Bases : ");
    #endif
    token = strtok(list, ",");
    while (token != NULL) {

        if (sscanf(token, "%hhd", &node_id) != 1) return -1;
        graph_nodes[node_id].penalty = NODE_DANGER; 
        #ifdef VERBOSE
        printf("%d", node_id);
        #endif
        token = strtok(NULL,",");
        #ifdef VERBOSE
        if (token != NULL) printf(",");
        #endif
    }
    #ifdef VERBOSE
    printf("\n");
    #endif

    // Scan plant nodes, assign block level penalty
    if (fscanf(input_file, "Plants : %s", list) != 1) return -1;
    #ifdef VERBOSE
    printf("Plants : ");
    #endif
    token = strtok(list, ",");
    while (token != NULL) {

        if (sscanf(token, "%hhd", &node_id) != 1) return -1;
        graph_nodes[node_id].penalty = NODE_BLOCKED;
        #ifdef VERBOSE
        printf("%d", node_id);
        #endif
        token = strtok(NULL,",");
        #ifdef VERBOSE
        if (token != NULL) printf(",");
        #endif
    }    
    #ifdef VERBOSE
    printf("\n");
    #endif

    return 0;
}

void free_graph() { free(graph_nodes); }

ASPath compute_path(const int from, const int to) {
    if (graph_nodes[to].penalty == NODE_BLOCKED) return NULL;
    ASPathNodeSource source;
    source.nodeSize = sizeof(graph_node_t);
    source.nodeNeighbors = node_neighbors;
    source.pathCostHeuristic = path_cost_heuristic;
    source.earlyExit = NULL;
    source.nodeComparator = node_comparator;
    return ASPathCreate(&source, NULL, &(graph_nodes[from]), &(graph_nodes[to]));
}
