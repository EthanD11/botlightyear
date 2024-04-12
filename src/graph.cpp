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
    Graph *graph;
} context_t;

Graph::Graph() {
    nNodes = -1;
    nodes = NULL;
}

Graph::~Graph() {
    if (nNodes != -1 && nodes != NULL) free(nodes);
}

void node_neighbors(ASNeighborList neighbors, void* node, void* context) {
    graph_node_t* curNode = (graph_node_t*) node;
    Graph *graph = ((context_t *) context)->graph;
    for (uint8_t i = 0; i < curNode->nNeighbors; i++)
    {
        graph_node_t* neighbor = curNode->neighbors[i];
        if (neighbor->level > graph->level) continue;
        float cost = hypot(neighbor->x - curNode->x, neighbor->y - curNode->y);
        ASNeighborListAdd(neighbors, neighbor, cost);
    }
}

float path_cost_heuristic(void* fromNode, void* toNode, void* context) {
    graph_node_t* from = (graph_node_t*) fromNode;
    context_t* ctx = (context_t*) context;
    float min = 1e10;
    for (uint8_t i = 0; i < ctx->len_targets; i++)
    {
        graph_node_t* to = ctx->graph->nodes + ctx->targets[i];
        if (to->level > ctx->graph->level) continue;
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
    context_t *ctx = (context_t*) context;
    for (size_t i = 0; i < ctx->len_targets; i++)
    {
        graph_node_t *target = ctx->graph->nodes + ctx->targets[i];
        if (node_comparator(current, target, NULL) == 0 && target->level <= ctx->graph->level) return 1; 
    }
    return 0;
}

graph_path_t *Graph::compute_path(uint8_t from, uint8_t *targets, uint8_t len_targets, uint8_t oversampling) {
    if (len_targets == 0) return NULL;

    // Initiate search arguments
    ASPathNodeSource source;
    source.nodeSize = sizeof(graph_node_t);
    source.nodeNeighbors = node_neighbors;
    source.pathCostHeuristic = path_cost_heuristic;
    source.earlyExit = early_exit;
    source.nodeComparator = node_comparator;
    context_t context;
    context.len_targets = len_targets;
    context.targets = targets;
    context.graph = this;

    // Start the search
    ASPath path = ASPathCreate(&source, &context, this->nodes + from, this->nodes + targets[0]);
    if (ASPathGetCount(path) == 0) {
        // Failure, returning
        ASPathDestroy(path);
        return NULL;
    }

    // Success, create arrays of coordinates for path following and cost
    uint8_t nKeyPoints = ASPathGetCount(path);
    nNodes = nKeyPoints + oversampling * (nKeyPoints-1);

    void *temp = malloc(sizeof(graph_path_t) + 2*nNodes*sizeof(double));
    graph_path_t *result = (graph_path_t*) temp;
    result->x = (double *) (((uint8_t*)temp) + sizeof(graph_path_t));
    result->y = (double *) (((uint8_t*)temp) + sizeof(graph_path_t) + nNodes*sizeof(double));
    
    result->nNodes = nNodes;
    result->totalCost = ASPathGetCost(path);

    graph_node_t *curNode = (graph_node_t *) ASPathGetNode(path, 0);
    graph_node_t *nextNode;
    uint8_t nPoints = oversampling + 1; // Number of points between current node (included) and next node (excluded)
    for (uint8_t i = 0; i < nKeyPoints-1; i++)
    {
        nextNode = (graph_node_t *) ASPathGetNode(path, i+1);
        for (uint8_t j = 0; j < nPoints; j++)
        {
            result->x[i*nPoints+j] = (j*nextNode->x + (nPoints-j) * curNode->x) / nPoints;
            result->y[i*nPoints+j] = (j*nextNode->y + (nPoints-j) * curNode->y) / nPoints;
        }
        curNode = nextNode;
    }
    result->x[nNodes-1] = curNode->x;
    result->y[nNodes-1] = curNode->y;
    result->target = curNode->id;

    ASPathDestroy(path);
    return result;    
}

void Graph::node_level_update(int node, int level, int propagation) {
    graph_node_t *_node = &(nodes[node]);
    _node->level = level;
    if (propagation) {
        for (uint8_t i = 0; i < _node->nNeighbors; i++) {

            uint8_t nodeAffected = 1; // Is this neighbor a plant or a base ?
            for (uint8_t j = 0; j < 6; j++){
                if (friendlyBases[j] == _node->neighbors[i]->id 
                    || adversaryBases[j] == _node->neighbors[i]->id
                    || plants[j] == _node->neighbors[i]->id) {
                    nodeAffected = 0; // If yes, its level should not be affected by the propagation
                    break;
                }
            }
            
            // If affected, update neighbor to max(0, level-1)
            if (nodeAffected) _node->neighbors[i]->level = (level <= 1) ? 0 : level-1;
        }   
    }
}

uint8_t Graph::identify_pos(double x, double y, double *dist) {
    uint8_t id = 0;
    *dist = hypot(nodes[0].x - x, nodes[0].y - y);
    for (uint8_t i = 1; i < nNodes; i++)
    {
        double temp = hypot(nodes[i].x - x, nodes[i].y - y);
        if (*dist > temp) {
            *dist = temp;
            id = i;
        }
    }
    return id;
}

int Graph::init_from_file(const char *filename, team_color_t color) {
    if (color == NoTeam) return -1;
    FILE *input_file = fopen(filename, "r");
    if (input_file == NULL) return -1;

    // Scan number of nodes
    if (fscanf(input_file, "Number of nodes : %hhd\n", &nNodes) != 1) return -1;
    #ifdef VERBOSE
    printf("Number of nodes : %d\n", nNodes);
    #endif
    
    // Malloc graph
    nodes = (graph_node_t*) malloc(nNodes*sizeof(graph_node_t));
    if (nodes == NULL) return -1;

    uint8_t trash;
    for (int8_t i = 0; i < nNodes; i++) {

        // Set id & level
        nodes[i].id = i;
        nodes[i].level = 0;

        // Scan x and y coordinates
        if (fscanf(input_file, "%hhd:%f,%f\n", &trash, &(nodes[i].x), &(nodes[i].y)) != 3) return -1;
        if (trash != i) return -1;
        #ifdef VERBOSE
        printf("%d:%.3f,%.3f\n", i, nodes[i].x, nodes[i].y);
        // Rotation for recentering based on robot modelling's convention
        //printf("%.3f,%.3f\n", (nodes[i].y-1), -(nodes[i].x-1.5));
        #endif
    }

    // Scan neighbors lists
    // One line = the list of comma-separated neighbors of one node
    char list[64];
    char *token;
    uint8_t node_id;
    for (uint8_t i = 0; i < nNodes; i++)
    {
        nodes[i].nNeighbors = 0;
        for (size_t i = 0; i < 64; i++){ list[i] = 0; }
        if (fscanf(input_file, "%hhd:%s\n", &trash, list) != 2) return -1; // Get next line
        if (trash != i) return -1;
        #ifdef VERBOSE
        printf("%d:", i);
        #endif

        token = strtok(list, ","); // Split line based on commas, get 1st token
        while (token != NULL) {

            if (nodes[i].nNeighbors == 7) return -1;
            if (sscanf(token,"%hhd", &node_id) != 1) return -1; // Interpret token as a neighbor id

            nodes[i].neighbors[nodes[i].nNeighbors] = nodes + node_id; // Add to list of neighbors
            nodes[i].nNeighbors++;
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
    for (size_t i = 0; i < 64; i++){ list[i] = 0; }
    if (fscanf(input_file, "Blue bases, reserved first : %s\n", list) != 1) return -1;
    #ifdef VERBOSE
    printf("Blue bases, reserved first : ");
    #endif
    token = strtok(list, ",");
    for (uint8_t i = 0; i < 3; i++) {
        if (token == NULL) return -1;
        if (sscanf(token, "%hhd", &node_id) != 1) return -1;
        if (color == TeamBlue) {
            friendlyBases[i] = node_id;
        } else {
            nodes[node_id].level = 1; 
            adversaryBases[i] = node_id;
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
    for (size_t i = 0; i < 64; i++){ list[i] = 0; }
    if (fscanf(input_file, "Yellow bases, reserved first : %s\n", list) != 1) return -1;
    #ifdef VERBOSE
    printf("Yellow bases, reserved first : ");
    #endif
    token = strtok(list, ",");
    for (uint8_t i = 0; i < 3; i++) {
        if (token == NULL) return -1;
        if (sscanf(token, "%hhd", &node_id) != 1) return -1;
        if (color == TeamYellow) {
            friendlyBases[i] = node_id;
        } else {
            nodes[node_id].level = 1; 
            adversaryBases[i] = node_id;
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

    // Scan blue planters nodes
    for (size_t i = 0; i < 64; i++){ list[i] = 0; }
    if (fscanf(input_file, "Blue planters, reserved first : %s\n", list) != 1) return -1;
    #ifdef VERBOSE
    printf("Blue planter, reserved first : ");
    #endif
    token = strtok(list, ",");
    for (uint8_t i = 0; i < 3; i++) {
        if (token == NULL) return -1;
        if (sscanf(token, "%hhd", &node_id) != 1) return -1;
        if (color == TeamBlue) {
            friendlyPlanters[i] = node_id;
        } else {
            adversaryPlanters[i] = node_id;
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

    // Scan yellow planters nodes
    for (size_t i = 0; i < 64; i++){ list[i] = 0; }
    if (fscanf(input_file, "Yellow planters, reserved first : %s\n", list) != 1) return -1;
    #ifdef VERBOSE
    printf("Yellow planters, reserved first : ");
    #endif
    token = strtok(list, ",");
    for (uint8_t i = 0; i < 3; i++) {
        if (token == NULL) return -1;
        if (sscanf(token, "%hhd", &node_id) != 1) return -1;
        if (color == TeamYellow) {
            friendlyBases[i] = node_id;
        } else {
            nodes[node_id].level = 1; 
            adversaryBases[i] = node_id;
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
    for (size_t i = 0; i < 64; i++){ list[i] = 0; }
    if (fscanf(input_file, "Plants : %s\n", list) != 1) return -1;
    #ifdef VERBOSE
    printf("Plants : ");
    #endif
    token = strtok(list, ",");
    for (uint8_t i = 0; i < 6; i++) {
        if (token == NULL) return -1;
        if (sscanf(token, "%hhd", &node_id) != 1) return -1;
        nodes[node_id].level = 1;
        plants[i] = node_id;
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

    // Scan pot nodes, assign level
    for (size_t i = 0; i < 64; i++){ list[i] = 0; }
    if (fscanf(input_file, "Pots : %s\n", list) != 1) return -1;
    #ifdef VERBOSE
    printf("Pots : ");
    #endif
    token = strtok(list, ",");
    for (uint8_t i = 0; i < 6; i++) {
        if (token == NULL) return -1;

        if (sscanf(token, "%hhd", &node_id) != 1) return -1;
        //nodes[node_id].level = 0;
        pots[i] = node_id;
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

    // Scan blue solar panel nodes, assign level
    for (size_t i = 0; i < 64; i++){ list[i] = 0; }
    if (fscanf(input_file, "Blue solar panels : %s\n", list) != 1) return -1;
    #ifdef VERBOSE
    printf("Blue solar panels : ");
    #endif
    token = strtok(list, ",");
    for (uint8_t i = 0; i < 3; i++) {
        if (token == NULL) return -1;
        if (sscanf(token, "%hhd", &node_id) != 1) return -1;
        if (color == TeamBlue) {
            friendlySPs[i] = node_id;
        } else {
            nodes[node_id].level = 1; 
            adversarySPs[i] = node_id;
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

    // Scan yellow solar panel nodes, assign level
    for (size_t i = 0; i < 64; i++){ list[i] = 0; }
    if (fscanf(input_file, "Yellow solar panels : %s\n", list) != 1) return -1;
    #ifdef VERBOSE
    printf("Yellow solar panels : ");
    #endif
    token = strtok(list, ",");
    for (uint8_t i = 0; i < 3; i++) {
        if (token == NULL) return -1;
        if (sscanf(token, "%hhd", &node_id) != 1) return -1;
        if (color == TeamYellow) {
            friendlySPs[i] = node_id;
        } else {
            nodes[node_id].level = 1; 
            adversarySPs[i] = node_id;
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

    // Scan common solar panel nodes, assign level
    for (size_t i = 0; i < 64; i++){ list[i] = 0; }
    if (fscanf(input_file, "Common solar panels : %s", list) != 1) return -1;
    #ifdef VERBOSE
    printf("Common solar panels : ");
    #endif
    token = strtok(list, ",");
    for (uint8_t i = 0; i < 3; i++) {
        if (token == NULL) return -1;
        if (sscanf(token, "%hhd", &node_id) != 1) return -1;
        nodes[node_id].level = 1;
        commonSPs[i] = node_id;
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

    level = 0;
    return 0;
}

void Graph::print_path(graph_path_t* path) {
    printf("Path towards %d of length %.3fm in %d points\n", path->target, path->totalCost, path->nNodes);
    for (uint8_t i = 0; i < path->nNodes; i++)
    {
        printf("(%.3f,%.3f) ", path->x[i], path->y[i]);
    }
}
