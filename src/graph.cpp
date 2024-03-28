#include "graph.h"
#include "AStar.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

// Uncomment to enable
// When enabled, init_graph_from_file should print out the exact file it has read
//#define VERBOSE

typedef struct __graph_targets
{
    uint8_t len_targets;
    uint8_t *targets;
} graph_targets_t;


void node_neighbors(ASNeighborList neighbors, void* node, void* context) {
    graph_node_t* cur_node = (graph_node_t*) node;
    for (uint8_t i = 0; i < cur_node->nb_neighbors; i++)
    {
        graph_node_t* neighbor = cur_node->neighbors[i];
        if (neighbor->level > graphLevel) continue;
        float cost = hypot(neighbor->x - cur_node->x, neighbor->y - cur_node->y);
        ASNeighborListAdd(neighbors, neighbor, cost);
    }
}

float path_cost_heuristic(void* fromNode, void* toNode, void* context) {
    graph_node_t* from = (graph_node_t*) fromNode;
    graph_targets_t* targets = (graph_targets_t*) context;
    float min = 1e10;
    for (uint8_t i = 0; i < targets->len_targets; i++)
    {
        graph_node_t* to = graphNodes + targets->targets[i];
        if (to->level > graphLevel) continue;
        float cost = hypot(to->x - from->x, to->y - from->y);
        if (min > cost) min = cost;
    }
    return min;
}

int node_comparator(void *node1, void *node2, void *context) {
    return ((graph_node_t*) node1)->id - ((graph_node_t*) node2)->id;
}

int early_exit(size_t visitedCount, void *visitingNode, void *goalNode, void *context) {
    graph_node_t *current = (graph_node_t*) visitingNode;
    graph_targets_t *targets = (graph_targets_t*) context;
    for (size_t i = 0; i < targets->len_targets; i++)
    {
        graph_node_t *target = graphNodes + targets->targets[i];
        if (target->level <= graphLevel && node_comparator(current, target, NULL) == 0) return 1; 
    }
    return 0;
}

graph_path_t *graph_compute_path(const uint8_t from, uint8_t *targets, const uint8_t len_targets, const uint8_t oversampling) {
    if (len_targets == 0) return NULL;

    // Initiate search arguments
    ASPathNodeSource source;
    source.nodeSize = sizeof(graph_node_t);
    source.nodeNeighbors = node_neighbors;
    source.pathCostHeuristic = path_cost_heuristic;
    source.earlyExit = early_exit;
    source.nodeComparator = node_comparator;
    graph_targets_t context;
    context.len_targets = len_targets;
    context.targets = targets;

    // Start the search
    ASPath path = ASPathCreate(&source, &context, graphNodes + from, graphNodes + targets[0]);
    if (ASPathGetCount(path) == 0) {
        // Failure, returning
        ASPathDestroy(path);
        return NULL;
    }

    // Success, create arrays of coordinates for path following and cost
    uint8_t n_keypoints = ASPathGetCount(path);
    uint8_t n_nodes = n_keypoints + oversampling * (n_keypoints-1);

    void *temp = malloc(sizeof(graph_path_t) + 2*n_nodes*sizeof(double));
    graph_path_t *result = (graph_path_t*) temp;
    result->x = (double *) (((uint8_t*)temp) + sizeof(graph_path_t));
    result->y = (double *) (((uint8_t*)temp) + sizeof(graph_path_t) + n_nodes*sizeof(double));
    
    result->nb_nodes = n_nodes;
    result->total_cost = ASPathGetCost(path);

    graph_node_t *curr_node = (graph_node_t *) ASPathGetNode(path, 0);
    graph_node_t *next_node;
    uint8_t n_points = oversampling + 1; // Number of points between current node (included) and next node (excluded)
    for (uint8_t i = 0; i < n_keypoints-1; i++)
    {
        next_node = (graph_node_t *) ASPathGetNode(path, i+1);
        for (uint8_t j = 0; j < n_points; j++)
        {
            result->x[i*n_points+j] = (j*next_node->x + (n_points-j) * curr_node->x) / n_points;
            result->y[i*n_points+j] = (j*next_node->y + (n_points-j) * curr_node->y) / n_points;
        }
        curr_node = next_node;
    }
    result->x[n_nodes-1] = curr_node->x;
    result->y[n_nodes-1] = curr_node->y;
    result->target = curr_node->id;

    ASPathDestroy(path);
    return result;    
}

void graph_level_update(const int node, const int level, const int propagation) {
    graph_node_t *_node = &(graphNodes[node]);
    _node->level = level;
    if (propagation) {
        for (uint8_t i = 0; i < _node->nb_neighbors; i++) {

            uint8_t node_affected = 1; // Is this neighbor a plant or a base ?
            for (uint8_t j = 0; j < 6; j++){
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

uint8_t graph_identify_pos(double x, double y, double *dist) {
    uint8_t id = 0;
    *dist = hypot(graphNodes[0].x - x, graphNodes[0].y - y);
    for (uint8_t i = 1; i < graphNbNodes; i++)
    {
        double temp = hypot(graphNodes[i].x - x, graphNodes[i].y - y);
        if (*dist > temp) {
            *dist = temp;
            id = i;
        }
    }
    return id;
}

int init_graph_from_file(const char *filename, team_color_t color) {
    if (color == NoTeam) return -1;
    FILE *input_file = fopen(filename, "r");
    if (input_file == NULL) return -1;

    // Scan number of nodes
    if (fscanf(input_file, "Number of nodes : %hhd\n", &graphNbNodes) != 1) return -1;
    #ifdef VERBOSE
    printf("Number of nodes : %d\n", graphNbNodes);
    #endif
    
    // Malloc graph
    graphNodes = (graph_node_t*) malloc(graphNbNodes*sizeof(graph_node_t));
    if (graphNodes == NULL) return -1;

    uint8_t trash;
    for (int8_t i = 0; i < graphNbNodes; i++) {

        // Set id & level
        graphNodes[i].id = i;
        graphNodes[i].level = 0;
        graphNodes[i].score = 0;

        // Scan x and y coordinates
        if (fscanf(input_file, "%hhd:%f,%f\n", &(graphNodes[i].x), &(graphNodes[i].y), &trash) != 3) return -1;
        if (trash != i) return -1;
        #ifdef VERBOSE
        printf("%d:%.3f,%.3f\n", i, graphNodes[i].x, graphNodes[i].y);
        // Rotation for recentering based on robot modelling's convention
        //printf("%.3f,%.3f\n", (graphNodes[i].y-1), -(graphNodes[i].x-1.5));
        #endif
    }

    // Scan neighbors lists
    // One line = the list of comma-separated neighbors of one node
    char list[64];
    for (size_t i = 0; i < 64; i++){ list[i] = 0; }
    char *token;
    uint8_t node_id;
    for (uint8_t i = 0; i < graphNbNodes; i++)
    {
        graphNodes[i].nb_neighbors = 0;
        for (size_t i = 0; i < 64; i++){ list[i] = 0; }
        if (fscanf(input_file, "%hhd:%s\n", list, trash) != 2) return -1; // Get next line
        if (trash != i) return -1;
        #ifdef VERBOSE
        printf("%d:", i);
        #endif

        token = strtok(list, ","); // Split line based on commas, get 1st token
        while (token != NULL) {

            if (graphNodes[i].nb_neighbors == 7) return -1;
            if (sscanf(token,"%hhd", &node_id) != 1) return -1; // Interpret token as a neighbor id

            graphNodes[i].neighbors[graphNodes[i].nb_neighbors] = &(graphNodes[node_id]); // Add to list of neighbors
            graphNodes[i].nb_neighbors++;
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

    // Scan blue bases nodes, assign level
    if (fscanf(input_file, "Blue bases, reserved first : %s\n", list) != 1) return -1;
    #ifdef VERBOSE
    printf("Blue bases, reserved first : ");
    #endif
    token = strtok(list, ",");
    
    for (uint8_t i = 0; i < 3; i++) {
        if (token == NULL) return -1;
        if (sscanf(token, "%hhd", &node_id) != 1) return -1;
        if (color == TeamBlue) {
            graphFriendlyBases[i] = node_id;
        } else {
            graphNodes[node_id].level = 1; 
            graphAdversaryBases[i] = node_id;
        }
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

    // Scan yellow bases nodes, assign level
    if (fscanf(input_file, "Yellow bases, reserved first : %s\n", list) != 1) return -1;
    #ifdef VERBOSE
    printf("Yellow bases, reserved first : ");
    #endif
    token = strtok(list, ",");
    
    for (uint8_t i = 0; i < 3; i++) {
        if (token == NULL) return -1;
        if (sscanf(token, "%hhd", &node_id) != 1) return -1;
        if (color == TeamYellow) {
            graphFriendlyBases[i] = node_id;
        } else {
            graphNodes[node_id].level = 1; 
            graphAdversaryBases[i] = node_id;
        }
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

    // Scan plant nodes, assign level
    if (fscanf(input_file, "Plants : %s\n", list) != 1) return -1;
    #ifdef VERBOSE
    printf("Plants : ");
    #endif
    token = strtok(list, ",");
    i = 0;
    while (token != NULL) {

        if (sscanf(token, "%hhd", &node_id) != 1) return -1;
        graphNodes[node_id].level = 1;
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

    // Scan pot nodes, assign level
    if (fscanf(input_file, "Pots : %s", list) != 1) return -1;
    #ifdef VERBOSE
    printf("Pots : ");
    #endif
    token = strtok(list, ",");
    i = 0;
    while (token != NULL) {

        if (sscanf(token, "%hhd", &node_id) != 1) return -1;
        //graphNodes[node_id].level = 0;
        graph_pots[i] = node_id;
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

    graphLevel = 0;
    return 0;
}

void free_graph() { free(graphNodes); }

void print_path(graph_path_t* path) {
    printf("Path towards %d of length %.3fm in %d points\n", path->target, path->total_cost, path->nb_nodes);
    for (uint8_t i = 0; i < path->nb_nodes; i++)
    {
        printf("(%.3f,%.3f) ", path->x[i], path->y[i]);
    }
}
