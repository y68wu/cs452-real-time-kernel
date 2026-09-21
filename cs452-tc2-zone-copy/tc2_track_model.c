#include "tc2_track_model.h"

static int strings_equal(const char *left, const char *right) {
        if (!left || !right) return 0;
        while (*left && *right && *left == *right) {
                ++left;
                ++right;
        }
        return *left == '\0' && *right == '\0';
}

static int find_node_by_name(
        const track_node track[TRACK_MAX], const char *name) {
        if (!track || !name) return -1;
        for (int node = 0; node < TRACK_MAX; ++node) {
                if (track[node].name &&
                    strings_equal(track[node].name, name)) {
                        return node;
                }
        }
        return -1;
}

static const tc2_track_start_definition start_definitions[
        TC2_TRACK_START_COUNT] = {
        {"A", "EN5"},
        {"B", "EN4"},
        {"C", "EN7"},
        {"D", "EN10"},
        {"E", "EN9"},
        {"F", "EN3"}
};

/*
 * d1-d8 are physical operator endpoints, not sensor aliases.  Lowercase is
 * intentional: the physical directed sensors remain uppercase D1-D16.
 * Each side names the last directed sensor seen when approaching the
 * physical point from that side.  Distances are pickup-centre-to-endpoint
 * measurements rounded to the nearest millimetre from the operator's inch
 * readings (1 in = 25.4 mm).  The pickup is at the geometric centre of the
 * 8 in train, so no nose/tail correction belongs in these endpoint values.
 *
 * Graph edges continue to use the nominal Marklin part accumulation.  These
 * endpoint offsets are independent physical measurements and therefore need
 * not add up byte-for-byte to the nominal sensor interval.
 */
static const tc2_track_destination_definition destination_definitions[
        TC2_TRACK_DESTINATION_COUNT] = {
        {
                "d1",
                {
                        {"A12", 381, TC2_TRACK_GEOMETRY_TRIAL,
                         0, 0, 0},
                        {"A15", 381, TC2_TRACK_GEOMETRY_TRIAL,
                         0, 0, 0}
                }
        },
        {
                "d2",
                {
                        {"A4", 226, TC2_TRACK_GEOMETRY_TRIAL,
                         0, 0, 0},
                        {"B15", 226, TC2_TRACK_GEOMETRY_TRIAL,
                         0, 0, 0}
                }
        },
        {
                "d3",
                {
                        {"C11", 89, TC2_TRACK_GEOMETRY_TRIAL,
                         0, 0, 0},
                        {"B6", 254, TC2_TRACK_GEOMETRY_TRIAL,
                         0, 0, 0}
                }
        },
        {
                "d4",
                {
                        {"C10", 76, TC2_TRACK_GEOMETRY_TRIAL,
                         0, 0, 0},
                        {"B2", 279, TC2_TRACK_GEOMETRY_TRIAL,
                         0, 0, 0}
                }
        },
        {
                "d5",
                {
                        {"E5", 0, TC2_TRACK_GEOMETRY_TRIAL,
                         0, 0, 0},
                        {"E6", 0, TC2_TRACK_GEOMETRY_TRIAL,
                         0, 0, 0}
                }
        },
        {
                "d6",
                {
                        {"E13", 0, TC2_TRACK_GEOMETRY_TRIAL,
                         0, 0, 0},
                        {"E14", 0, TC2_TRACK_GEOMETRY_TRIAL,
                         0, 0, 0}
                }
        },
        {
                "d7",
                {
                        {"E9", 305, TC2_TRACK_GEOMETRY_TRIAL,
                         0, 0, 0},
                        {"D6", 305, TC2_TRACK_GEOMETRY_TRIAL,
                         0, 0, 0}
                }
        },
        {
                "d8",
                {
                        {"C15", 203, TC2_TRACK_GEOMETRY_TRIAL,
                         0, 0, 0},
                        {"D11", 203, TC2_TRACK_GEOMETRY_TRIAL,
                         0, 0, 0}
                }
        }
};

const tc2_track_start_definition *Tc2TrackStartDefinition(int index) {
        if (index < 0 || index >= TC2_TRACK_START_COUNT) return 0;
        return &start_definitions[index];
}

const tc2_track_destination_definition *
Tc2TrackDestinationDefinition(int index) {
        if (index < 0 || index >= TC2_TRACK_DESTINATION_COUNT) return 0;
        return &destination_definitions[index];
}

const tc2_track_destination_side *
Tc2TrackDestinationSide(int destination_index, int side) {
        const tc2_track_destination_definition *definition =
                Tc2TrackDestinationDefinition(destination_index);
        if (!definition || side < 0 ||
            side >= TC2_TRACK_DESTINATION_SIDE_COUNT) {
                return 0;
        }
        return &definition->side[side];
}

char Tc2TrackPhysicalTurnoutDirection(int graph_direction) {
        if (graph_direction == DIR_STRAIGHT) return 'S';
        if (graph_direction == DIR_CURVED) return 'C';
        return '\0';
}

static int validate_sensor_pair(
        const track_node track[TRACK_MAX], int node_index) {
        const track_node *node = &track[node_index];
        if (node->type != NODE_SENSOR || !node->reverse) return -1;
        int reverse_index = (int)(node->reverse - track);
        if (reverse_index < 0 || reverse_index >= TRACK_MAX ||
            track[reverse_index].type != NODE_SENSOR ||
            track[reverse_index].reverse != node) {
                return -1;
        }
        return 0;
}

/*
 * Return the distance to target only when it is the next sensor reachable
 * from node.  Turnout branches are explored, but encountering any different
 * sensor closes that path.  This lets the catalog validate a physical point
 * between two direction-specific sensors without pretending they are a
 * reverse pair at the same detector.
 */
static int next_sensor_distance_search(
        const track_node track[TRACK_MAX], int node_index, int target_index,
        int distance_mm, unsigned char visiting[TRACK_MAX]) {
        if (node_index < 0 || node_index >= TRACK_MAX ||
            target_index < 0 || target_index >= TRACK_MAX ||
            visiting[node_index]) {
                return -1;
        }
        if (track[node_index].type == NODE_SENSOR) {
                return node_index == target_index ? distance_mm : -1;
        }

        visiting[node_index] = 1;
        int edge_count = track[node_index].type == NODE_BRANCH ? 2 : 1;
        int found = -1;
        for (int edge = 0; edge < edge_count; ++edge) {
                const track_edge *next = &track[node_index].edge[edge];
                if (!next->dest || next->dist < 0) continue;
                int next_index = (int)(next->dest - track);
                int candidate = next_sensor_distance_search(
                        track, next_index, target_index,
                        distance_mm + next->dist, visiting);
                if (candidate >= 0 &&
                    (found < 0 || candidate < found)) {
                        found = candidate;
                }
        }
        visiting[node_index] = 0;
        return found;
}

static int next_sensor_distance(
        const track_node track[TRACK_MAX],
        int source_index, int target_index) {
        if (source_index < 0 || source_index >= TRACK_MAX ||
            target_index < 0 || target_index >= TRACK_MAX) {
                return -1;
        }
        const track_edge *edge = &track[source_index].edge[DIR_AHEAD];
        if (!edge->dest || edge->dist < 0) return -1;
        unsigned char visiting[TRACK_MAX] = {0};
        int next_index = (int)(edge->dest - track);
        return next_sensor_distance_search(
                track, next_index, target_index, edge->dist, visiting);
}

int Tc2TrackCatalogValidate(const track_node track[TRACK_MAX]) {
        if (!track) return -1;

        for (int start = 0; start < TC2_TRACK_START_COUNT; ++start) {
                const tc2_track_start_definition *definition =
                        Tc2TrackStartDefinition(start);
                int node = definition ?
                        find_node_by_name(
                                track, definition->enter_node) : -1;
                if (!definition || !definition->label ||
                    !definition->enter_node || node < 0 ||
                    track[node].type != NODE_ENTER) {
                        return -1;
                }
        }

        for (int destination = 0;
             destination < TC2_TRACK_DESTINATION_COUNT; ++destination) {
                const tc2_track_destination_definition *definition =
                        Tc2TrackDestinationDefinition(destination);
                if (!definition || !definition->label) return -1;
                int anchor[TC2_TRACK_DESTINATION_SIDE_COUNT];
                for (int side = 0;
                     side < TC2_TRACK_DESTINATION_SIDE_COUNT; ++side) {
                        const tc2_track_destination_side *entry =
                                &definition->side[side];
                        anchor[side] = entry->anchor_sensor ?
                                find_node_by_name(
                                        track, entry->anchor_sensor) : -1;
                        if (anchor[side] < 0 ||
                            track[anchor[side]].type != NODE_SENSOR ||
                            entry->base_offset_mm < 0 ||
                            entry->geometry_evidence <
                                    TC2_TRACK_GEOMETRY_PROVISIONAL ||
                            entry->geometry_evidence >
                                    TC2_TRACK_GEOMETRY_CONFIRMED) {
                                return -1;
                        }
                        if (entry->straight_guide &&
                            find_node_by_name(
                                    track, entry->straight_guide) < 0) {
                                return -1;
                        }
                        if ((entry->previous_anchor_sensor &&
                             entry->previous_anchor_distance_mm <= 0) ||
                            (!entry->previous_anchor_sensor &&
                             entry->previous_anchor_distance_mm != 0) ||
                            (entry->previous_anchor_sensor &&
                             find_node_by_name(
                                     track,
                                     entry->previous_anchor_sensor) < 0)) {
                                return -1;
                        }
                }
                int legacy_reverse_pair =
                        track[anchor[0]].reverse == &track[anchor[1]] &&
                        track[anchor[1]].reverse == &track[anchor[0]];
                if (!legacy_reverse_pair) {
                        int target_from_side_0 =
                                (int)(track[anchor[1]].reverse - track);
                        int target_from_side_1 =
                                (int)(track[anchor[0]].reverse - track);
                        int interval_0 = next_sensor_distance(
                                track, anchor[0], target_from_side_0);
                        int interval_1 = next_sensor_distance(
                                track, anchor[1], target_from_side_1);
                        int offset_sum =
                                definition->side[0].base_offset_mm +
                                definition->side[1].base_offset_mm;
                        int residual = offset_sum - interval_0;
                        if (residual < 0) residual = -residual;
                        if (interval_0 <= 0 || interval_1 <= 0 ||
                            interval_0 != interval_1 ||
                            residual > 100) {
                                return -1;
                        }
                } else if (definition->side[0].base_offset_mm != 0 ||
                           definition->side[1].base_offset_mm != 0) {
                        return -1;
                }
        }

        int sensor_nodes = 0;
        int sensor_pairs = 0;
        int branch_nodes = 0;
        int merge_nodes = 0;
        for (int node = 0; node < TRACK_MAX; ++node) {
                if (track[node].type == NODE_SENSOR) {
                        if (validate_sensor_pair(track, node) < 0) return -1;
                        ++sensor_nodes;
                        if (&track[node] < track[node].reverse) {
                                ++sensor_pairs;
                        }
                } else if (track[node].type == NODE_BRANCH) {
                        ++branch_nodes;
                        if (!track[node].reverse ||
                            track[node].reverse->type != NODE_MERGE ||
                            track[node].reverse->reverse != &track[node] ||
                            track[node].reverse->num != track[node].num) {
                                return -1;
                        }
                } else if (track[node].type == NODE_MERGE) {
                        ++merge_nodes;
                        if (!track[node].reverse ||
                            track[node].reverse->type != NODE_BRANCH ||
                            track[node].reverse->reverse != &track[node] ||
                            track[node].reverse->num != track[node].num) {
                                return -1;
                        }
                }
        }

        return sensor_nodes == TC2_TRACK_SENSOR_NODE_COUNT &&
               sensor_pairs == TC2_TRACK_SENSOR_PAIR_COUNT &&
               branch_nodes == TC2_TRACK_SWITCH_COUNT &&
               merge_nodes == TC2_TRACK_SWITCH_COUNT ? 0 : -1;
}
