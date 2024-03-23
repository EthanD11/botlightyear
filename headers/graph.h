#ifndef BLY_GRAPH_H_
#define BLY_GRAPH_H_
#include <stdint.h>
#include <stdlib.h>

#define NODE_FREE 0
#define NODE_DANGER 1
#define NODE_BLOCKED 2

#define DISABLE_PROPAGATION 0
#define ENABLE_PROPAGATION 1

typedef struct graph_node
{
    uint8_t id; // Index of the node in the 'graph_nodes' array & identifier for the visual graph
    float x, y; // X & Y Coordinates (refer to the visual graph)
    int8_t level; // Level of availability of this node (see 'graph_level')
    uint8_t nb_neighbors; // Number of neighbors
    struct graph_node* neighbors[8]; // List of neighbors
} graph_node_t;

typedef struct graph_path
{
    uint8_t nb_nodes; // Number of nodes in the path
    double *x; // Array of x coordinates
    double *y; // Array of y coordinates
    double total_cost; // Total distance of travel for this path
} graph_path_t;

inline uint8_t graph_nb_nodes; // Number of nodes in the graph
inline graph_node_t* graph_nodes; // Array of size 'graph_nb_nodes' (after initialization !) containing each node

/**
 * Any node such that node.level > graph_level will be ignored by the graph search
 * A higher graph_level therefore means a higher tolerance of obstacles near the path (more risky but faster)
 *  General idea :
 * 0 = Free, the node is not obstructed
 * 1 = Danger, the node is obstructed by game items or close to a large object (typically another robot)
 * 2 = Blocked, the node is obstructed by a large object (typically another robot) and/or cannot be reached at this time
*/
inline int8_t graph_level;

inline uint8_t graph_bases[6]; // Array of bases ids, first three are blue, last three are yellow, first one of each group is reserved (ie 0 and 3)
inline uint8_t graph_plants[6]; // Array of plant spots ids
inline uint8_t graph_pots[6]; // Array of pot spots ids

/**
 * @brief Initializes graph_nodes from file 'filename'. Returns 0 on success, -1 otherwise.
*/
int init_graph_from_file(const char *filename);

/**
 * @brief Frees the graph_nodes array and its nodes
*/
void free_graph();

/**
 * Computes a path sequence from node 'from' to the closest node in the 'targets' array (refer to the visual graph)
 * The level of the 'targets' and/or the graph_level must be set accordingly before the call
 * 'oversampling' intermediate points are added between each key point for better path following (recommended values : 0, 1 or 2)
 * Returns NULL if no path is found
 * Note that the arrays of x and y coordinates are directly next to the structure in the memory
 * Only the pointer returned must be passed to free() after the call, not the x and y arrays of the structure
*/
graph_path_t *graph_compute_path(const uint8_t from, uint8_t *targets, const uint8_t len_targets, const uint8_t oversampling);

/**
 * Updates the level of 'node' to 'level'
 * Propagates 'level'-1 to the neighbors of 'node' if 'propagation' != 0
 * Plants and bases are not affected by propagation
*/
void graph_level_update(const int node, const int level, const int propagation);

/**
 * @brief Prints cost, number of nodes, and coordinates in this path
*/
void print_path(graph_path_t* path); 

/**
 * Identify the given (x,y) coordinates to a node of the graph.
 * Returns identified node id.
 * Stores the distance to this node into 'dist'
 */
uint8_t graph_identify_pos(double x, double y, double *dist);


/*
// Total number of nodes
const int graph_nb_nodes = 41;

// Nodes x coordinates [m]
// xi = graph_xy[2*i]
// yi = graph_xy[2*i+1]
const float graph_xy[] = {
    0.400, 0.400, // 0
    0.460, 0.612, // 1
    0.400, 1.000, // 2
    0.460, 1.388, // 3
    0.400, 2.600, // 4

    0.612, 1.700, // 5
    0.725, 1.388, // 6
    0.725, 1.000, // 7
    0.725, 0.612, // 8
    0.762, 0.300, // 9
    
    1.000, 0.700, // 10
    1.138, 1.000, // 11
    1.000, 1.300, // 12
    1.000, 1.540, // 13
    1.388, 1.700, // 14
    
    1.235, 1.450, // 15
    1.275, 1.250, // 16
    1.500, 1.000, // 17
    1.275, 0.750, // 18
    1.235, 0.550, // 19
    
    1.500, 0.375, // 20
    1.725, 0.750, // 21
    1.725, 1.250, // 22
    1.500, 1.500, // 23
    1.612, 1.700, // 24
    
    1.765, 1.450, // 25
    1.862, 1.000, // 26
    1.765, 0.550, // 27
    2.000, 0.700, // 28
    2.000, 1.300, // 29
    
    2.000, 1.540, // 30
    2.388, 1.700, // 31
    2.275, 1.388, // 32
    2.275, 1.000, // 33
    2.275, 0.612, // 34
    
    2.238, 0.300, // 35
    2.600, 0.400, // 36
    2.540, 0.612, // 37
    2.600, 1.000, // 38
    2.540, 1.388, // 39
    
    2.600, 1.600  // 40

};

// List of neighbors for each node (the list of neighbors of node i is graph_neighbors[i])
// -1 marks the end of the list
const int8_t graph_neighbors[41][8] = {
    { 8, 1, 9,-1}, // 0
    { 8, 2, 0,-1}, // 1
    { 7, 6, 3, 1, 8,-1}, // 2
    { 5, 4, 2,-1}, // 3
    { 5, 3, 6,-1}, // 4

    { 4, 6,13,-1}, // 5
    {13, 5, 4, 3, 2, 7,12,-1}, // 6
    {11,12, 6, 2, 8,10,-1}, // 7
    {10, 7, 2, 1, 0, 9,-1}, // 8
    {20,19,10, 8, 0,-1}, // 9
    
    {18,11, 7, 8, 9,-1}, // 10
    {17,16,12, 7,10,18,-1}, // 11
    {15,13, 6, 7,11,16,-1}, // 12
    {14, 5, 6,12,15,-1}, // 13
    {24,13,15,23,-1}, // 14
    
    {23,14,13,12,16,-1}, // 15
    {22,23,15,12,11,17-1}, // 16
    {26,22,16,11,18,21,-1}, // 17
    {21,17,11,10,19,20,-1}, // 18
    {18,10, 9,20,-1}, // 19
    
    {27,21,18,19, 9,35,-1}, // 20
    {26,17,18,20,27,28,-1}, // 21
    {29,25,23,16,17,26,-1}, // 22
    {24,14,15,16,22,25,-1}, // 23
    {14,23,25,30,-1}, // 24
    
    {30,24,23,22,29,-1}, // 25
    {33,29,22,21,28,-1}, // 26
    {28,21,20,35,-1}, // 27
    {33,26,21,27,35,34,-1}, // 28
    {32,30,25,22,26,33,-1}, // 29
    
    {31,24,25,29,32,-1}, // 30
    {30,32,40,-1}, // 31
    {39,40,31,30,29,33,38,-1}, // 32
    {38,32,29,26,28,34,-1}, // 33
    {37,33,28,35,-1}, // 34
    
    {36,34,28,27,20,-1}, // 35
    {37,34,35,-1}, // 36
    {38,34,36,-1}, // 37
    {39,32,33,34,37,-1}, // 38
    {40,32,38,-1}, // 39
    
    {31,32,39,-1}, // 40

};

// Nodes validity (i can be used for a path if graph_nodes[i] == i, -1 otherwise)
// Plant spots are set to -1 by default at initial state
// Must be maintained throughout the match for accurate path planning
int8_t graph_nodes[] = {
    0,
    1,
    2,
    3,
    4,

    5,
    6,
    7,
    8,
    9,
    
    -1,
    11,
    -1,
    13,
    14,
    
    15,
    16,
    17,
    18,
    19,
    
    -1,
    21,
    22,
    -1,
    24,
    
    25,
    26,
    27,
    -1,
    -1,
    
    30,
    31,
    32,
    33,
    34,
    
    35,
    36,
    37,
    38,
    39,
    
    40
    
};

 Reserved nodes depending on color (blue = 0, yellow = 1)
const int8_t graph_nodes_reserved[2][4] = {
    {44,5,4,43}, // Blue
    {40,0,1,41}  // Yellow
};


    0.400, 1.600, // 0
    0.612, 1.700, // 1
    1.388, 1.700, // 2
    1.612, 1.700, // 3
    2.388, 1.700, // 4

    2.600, 1.600, // 5
    1.000, 1.540, // 6
    1.500, 1.500, // 7
    2.000, 1.540, // 8
    0.460, 1.388, // 9
    
    0.725, 1.388, // 10
    1.000, 1.300, // 11
    1.250, 1.400, // 12
    1.750, 1.400, // 13
    2.000, 1.300, // 14
    
    2.388, 1.388, // 15
    2.665, 1.388, // 16
    1.250, 1.200, // 17
    1.500, 1.188, // 18
    1.750, 1.200, // 19
    
    0.400, 1.000, // 20
    0.725, 1.000, // 21
    1.000, 1.000, // 22
    1.250, 1.000, // 23
    1.500, 1.000, // 24
    
    1.750, 1.000, // 25
    2.000, 1.000, // 26
    2.275, 1.000, // 27
    2.600, 1.000, // 28
    1.250, 0.800, // 29
    
    1.500, 0.812, // 30
    1.750, 0.800, // 31
    0.460, 0.612, // 32
    0.612, 0.612, // 33
    1.000, 0.700, // 34
    
    1.250, 0.600, // 35
    1.750, 0.600, // 36
    2.000, 0.700, // 37
    2.388, 0.612, // 38
    2.665, 0.612, // 39
    
    0.400, 0.400, // 40
    0.762, 0.300, // 41
    1.500, 0.335, // 42
    2.238, 0.300, // 43
    2.600, 0.400  // 44

    { 1, 9,10,-1}, // 0
    { 0,10, 6,-1}, // 1
    { 3, 6,12, 7,-1}, // 2
    { 2, 7,13, 8,-1}, // 3
    { 8,15, 5,-1,-1}, // 4

    { 4,15,16,-1}, // 5
    { 2, 1,10,11,12,-1}, // 6
    { 3, 2,12,18,13,-1}, // 7
    { 4, 3,13,14,15,-1}, // 8
    {10, 0,20,-1}, // 9
    
    { 6, 1, 0, 9,20,21,11,-1}, // 10
    {12, 6,10,21,22,17,-1}, // 11
    { 7, 2, 6,11,17,18,-1}, // 12
    { 8, 3, 7,18,19,14,-1}, // 13
    {15, 8,13,19,26,27,-1}, // 14
    
    { 5, 4, 8,14,27,28,16,-1}, // 15
    { 5,15,28,-1}, // 16
    {18,12,11,22,23,24,-1}, // 17
    {13, 7,12,17,23,24,25,19,-1}, // 18
    {14,13,18,24,25,26,-1}, // 19
    
    {21,10, 9,32,33,-1}, // 20
    {22,11,10,20,33,34,-1}, // 21
    {23,17,11,21,34,29,-1}, // 22
    {24,18,17,22,29,30,-1}, // 23
    {25,19,18,17,23,29,30,31,-1}, // 24
    
    {26,19,18,24,30,31,-1}, // 25
    {27,14,19,25,31,37,-1}, // 26
    {28,15,14,26,37,38,-1}, // 27
    {16,15,27,38,39,-1}, // 28
    {30,24,23,22,34,35,-1}, // 29
    
    {31,25,24,23,29,35,42,36,-1}, // 30
    {26,25,24,30,36,37,-1}, // 31
    {33,20,40,-1}, // 32
    {34,21,20,32,40,41,-1}, // 33
    {29,22,21,33,41,35,-1}, // 34
    
    {30,29,34,41,42,-1}, // 35
    {37,31,30,42,43,-1}, // 36
    {27,26,31,36,43,38,-1}, // 37
    {39,28,27,37,43,44,-1}, // 38
    {28,38,44,-1}, // 39
    
    {33,32,41,-1}, // 40
    {42,35,34,33,40,-1}, // 41
    {36,30,35,41,43,-1}, // 42
    {44,38,37,36,42,-1}, // 43
    {39,38,43,-1}  // 44
 */

#endif