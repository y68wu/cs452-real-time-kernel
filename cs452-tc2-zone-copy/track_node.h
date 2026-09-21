#ifndef _track_node_h_
#define _track_node_h_ 1

typedef enum {
        NODE_NONE,
        NODE_SENSOR,
        NODE_BRANCH,
        NODE_MERGE,
        NODE_ENTER,
        NODE_EXIT
} node_type;

#define DIR_AHEAD 0
#define DIR_STRAIGHT 0
#define DIR_CURVED 1

struct track_node;
typedef struct track_node track_node;
typedef struct track_edge track_edge;

struct track_edge {
        track_edge *reverse;
        track_node *src;
        track_node *dest;
        int dist;
};

struct track_node {
        const char *name;
        node_type type;
        int num;
        track_node *reverse;
        track_edge edge[2];
};

// 这个graph model保存directed nodes和millimetre edge distances，供Track B/D route finding使用。

#endif
