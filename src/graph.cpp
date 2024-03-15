#include "graph.h"
#include "AStar.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

// Uncomment to enable
// When enabled, init_graph_from_file should print out the exact file it has read
//#define VERBOSE

void node_neighbors(ASNeighborList neighbors, void* node, void* context) {
    graph_node_t* cur_node = (graph_node_t*) node;
    for (int i = 0; i < cur_node->nb_neighbors; i++)
    {
        graph_node_t* neighbor = cur_node->neighbors[i];
        if (neighbor->level > graph_level) continue;
        float dx = neighbor->x - cur_node->x;
        float dy = neighbor->y - cur_node->y;
        float cost = hypot(dx, dy);
        ASNeighborListAdd(neighbors, neighbor, cost);
    }
}

float path_cost_heuristic(void* fromNode, void* toNode, void* context) {
    graph_node_t* from = (graph_node_t*) fromNode;
    graph_node_t* to = (graph_node_t*) fromNode;
    float dx = to->x - from->x;
    float dy = to->y - from->y;
    return hypot(dx, dy);
}

int node_comparator(void *node1, void *node2, void *context) {
    return ((graph_node_t*) node1)->id - ((graph_node_t*) node2)->id;
}

graph_path_t *graph_compute_path(const int from, const int to) {
    if (graph_nodes[to].level > graph_level) return NULL;

    ASPathNodeSource source;
    source.nodeSize = sizeof(graph_node_t);
    source.nodeNeighbors = node_neighbors;
    source.pathCostHeuristic = path_cost_heuristic;
    source.earlyExit = NULL;
    source.nodeComparator = node_comparator;
    ASPath path = ASPathCreate(&source, NULL, &(graph_nodes[from]), &(graph_nodes[to]));
    if (ASPathGetCount(path) == 0) {
        ASPathDestroy(path);
        return NULL;
    }

    int8_t count = ASPathGetCount(path);
    void *temp = malloc(sizeof(graph_path_t) + 2*count*sizeof(double));
    graph_path_t *result = (graph_path_t*) temp;
    result->x = (double *) (temp + sizeof(graph_path_t));
    result->y = (double *) (temp + sizeof(graph_path_t) + count*sizeof(double));

    result->nb_nodes = count;
    for (int8_t i = 0; i < count; i++)
    {
        graph_node_t *node = (graph_node_t*) ASPathGetNode(path, i);
        result->x[i] = node->x;
        result->y[i] = node->y;
    }
    
    return result;
}

void graph_level_update(const int node, const int level, const int propagation) {
    graph_node_t *_node = &(graph_nodes[node]);
    _node->level = level;
    if (propagation) {
        for (int8_t i = 0; i < _node->nb_neighbors; i++) {

            int8_t node_affected = 1; // Is this neighbor a plant or a base ?
            for (int8_t j = 0; j < 6; j++){
                if (graph_bases[j] == _node->neighbors[i]->id || graph_plants[j] == _node->neighbors[i]->id) {
                    node_affected = 0; // If yes, its level should not be affected by the propagation
                    break;
                }
            }
            
            // If affected, update neighbor to max(0, level-1)
            if (node_affected) _node->neighbors[i]->level = (level <= 1) ? 0 : level-1;
        }   
    }
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
    graph_nodes = (graph_node_t*) malloc(graph_nb_nodes*sizeof(graph_node_t));
    if (graph_nodes == NULL) return -1;

    for (int8_t i = 0; i < graph_nb_nodes; i++) {

        // Set id & level
        graph_nodes[i].id = i;
        graph_nodes[i].level = 0;

        // Scan x and y coordinates
        if (fscanf(input_file, "%f,%f\n", &(graph_nodes[i].x), &(graph_nodes[i].y)) != 2) return -1;
        #ifdef VERBOSE
        printf("%.3f,%.3f\n", graph_nodes[i].x, graph_nodes[i].y);
        // Rotation for recentering based on robot modelling's convention
        //printf("%.3f,%.3f\n", (graph_nodes[i].y-1), -(graph_nodes[i].x-1.5));
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

    // Scan bases nodes, assign level
    if (fscanf(input_file, "Bases : %s\n", list) != 1) return -1;
    #ifdef VERBOSE
    printf("Bases : ");
    #endif
    token = strtok(list, ",");
    int i = 0;
    while (token != NULL) {

        if (sscanf(token, "%hhd", &node_id) != 1) return -1;
        graph_nodes[node_id].level = 1; 
        graph_bases[i] = node_id;
        #ifdef VERBOSE
        printf("%d", node_id);
        #endif
        token = strtok(NULL,",");
        #ifdef VERBOSE
        if (token != NULL) printf(",");
        #endif
        i++;
    }
    #ifdef VERBOSE
    printf("\n");
    #endif

    // Scan plant nodes, assign level
    if (fscanf(input_file, "Plants : %s", list) != 1) return -1;
    #ifdef VERBOSE
    printf("Plants : ");
    #endif
    token = strtok(list, ",");
    i = 0;
    while (token != NULL) {

        if (sscanf(token, "%hhd", &node_id) != 1) return -1;
        graph_nodes[node_id].level = 1;
        graph_plants[i] = node_id;
        #ifdef VERBOSE
        printf("%d", node_id);
        #endif
        token = strtok(NULL,",");
        #ifdef VERBOSE
        if (token != NULL) printf(",");
        #endif
        i++;
    }    
    #ifdef VERBOSE
    printf("\n");
    #endif

    graph_level = 0;
    return 0;
}

void free_graph() { free(graph_nodes); }
