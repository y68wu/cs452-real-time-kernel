#include "track_route.h"

#include <limits.h>
#include <stdint.h>

typedef enum {
        ROUTE_REVERSAL_NONE = 0,
        ROUTE_REVERSAL_ANY_NODE,
        ROUTE_REVERSAL_SENSOR_ONLY
} route_reversal_policy;

static int route_streq(const char *left, const char *right) {
        if (!left || !right) return 0;
        while (*left && *right && *left == *right) {
                ++left;
                ++right;
        }
        return *left == *right;
}

static int edge_count(const track_node *node) {
        if (node->type == NODE_BRANCH) return 2;
        if (node->type == NODE_EXIT || node->type == NODE_NONE) return 0;
        return 1;
}

static int node_pointer_index(track_node *track, const track_node *node) {
        uintptr_t base;
        uintptr_t limit;
        uintptr_t address;
        uintptr_t span = (uintptr_t)sizeof(track_node) * TRACK_MAX;

        if (!track || !node) return -1;
        base = (uintptr_t)(void *)track;
        if (base > UINTPTR_MAX - span) return -1;
        limit = base + span;
        address = (uintptr_t)(const void *)node;
        if (address < base || address >= limit) return -1;
        uintptr_t offset = address - base;
        if (offset % sizeof(track_node) != 0) return -1;
        return (int)(offset / sizeof(track_node));
}

static int graph_is_valid(track_node *track) {
        if (!track) return 0;

        for (int node = 0; node < TRACK_MAX; ++node) {
                if (track[node].type < NODE_NONE ||
                    track[node].type > NODE_EXIT) {
                        return 0;
                }
                if (track[node].reverse &&
                    node_pointer_index(track, track[node].reverse) < 0) {
                        return 0;
                }
                int count = edge_count(&track[node]);
                for (int direction = 0; direction < count; ++direction) {
                        track_edge *edge = &track[node].edge[direction];
                        if (!edge->dest) continue;
                        if (edge->dist < 0 ||
                            node_pointer_index(track, edge->dest) < 0) {
                                return 0;
                        }
                }
        }
        return 1;
}

static int route_is_valid(const track_route *route) {
        if (!route || route->node_count < 1 ||
            route->node_count > TRACK_MAX) {
                return 0;
        }
        for (int offset = 0; offset < route->node_count; ++offset) {
                if (route->nodes[offset] < 0 ||
                    route->nodes[offset] >= TRACK_MAX) {
                        return 0;
                }
        }
        return 1;
}

static int physical_node_is_unavailable(
        track_node *track, int node,
        const unsigned char unavailable_by_node[TRACK_MAX]) {
        if (!unavailable_by_node) return 0;
        if (node < 0 || node >= TRACK_MAX) return -1;
        if (unavailable_by_node[node]) return 1;
        if (track[node].reverse) {
                int reverse = node_pointer_index(track, track[node].reverse);
                if (reverse < 0) return -1;
                if (unavailable_by_node[reverse]) return 1;
        }
        return 0;
}

static int transition_distance(track_node *track, int from, int to,
                               int *distance_mm, int *is_reversal) {
        if (!track || !distance_mm || !is_reversal ||
            from < 0 || from >= TRACK_MAX || to < 0 || to >= TRACK_MAX) {
                return -1;
        }

        *distance_mm = 0;
        *is_reversal = 0;
        if (track[from].reverse) {
                int reverse = node_pointer_index(track, track[from].reverse);
                if (reverse < 0) return -1;
                if (reverse == to) {
                        *is_reversal = 1;
                        return 0;
                }
        }

        int count = edge_count(&track[from]);
        for (int direction = 0; direction < count; ++direction) {
                track_edge *edge = &track[from].edge[direction];
                if (!edge->dest) continue;
                int next = node_pointer_index(track, edge->dest);
                if (next < 0 || edge->dist < 0) return -1;
                if (next == to) {
                        *distance_mm = edge->dist;
                        return 0;
                }
        }
        return -1;
}

int TrackFindNodeByName(track_node *track, const char *name) {
        if (!track || !name) return -1;

        for (int i = 0; i < TRACK_MAX; ++i) {
                if (route_streq(track[i].name, name)) return i;
        }
        return -1;
}

static void clear_route(track_route *route) {
        if (!route) return;
        route->node_count = 0;
        route->distance_mm = 0;
        route->optimization_cost_mm = 0;
        route->reversal_count = 0;
}

static int find_route(
        track_node *track, int start_index, int destination_index,
        route_reversal_policy reversal_policy, int reversal_penalty_mm,
        const unsigned char unavailable_by_node[TRACK_MAX],
        track_route *route) {
        int cost[TRACK_MAX];
        int previous[TRACK_MAX];
        unsigned char visited[TRACK_MAX];
        int reversed[TRACK_MAX];

        clear_route(route);
        if (!track || !route || start_index < 0 ||
            start_index >= TRACK_MAX || destination_index < 0 ||
            destination_index >= TRACK_MAX || reversal_penalty_mm < 0 ||
            reversal_policy < ROUTE_REVERSAL_NONE ||
            reversal_policy > ROUTE_REVERSAL_SENSOR_ONLY ||
            !graph_is_valid(track) ||
            track[start_index].type == NODE_NONE ||
            track[destination_index].type == NODE_NONE) {
                return -1;
        }

        int start_unavailable = physical_node_is_unavailable(
                track, start_index, unavailable_by_node);
        int destination_unavailable = physical_node_is_unavailable(
                track, destination_index, unavailable_by_node);
        if (start_unavailable != 0 || destination_unavailable != 0) return -1;

        for (int i = 0; i < TRACK_MAX; ++i) {
                cost[i] = INT_MAX;
                previous[i] = -1;
                visited[i] = 0;
        }
        cost[start_index] = 0;

        for (int step = 0; step < TRACK_MAX; ++step) {
                int current = -1;
                for (int i = 0; i < TRACK_MAX; ++i) {
                        if (!visited[i] && cost[i] != INT_MAX &&
                            (current < 0 || cost[i] < cost[current])) {
                                current = i;
                        }
                }
                if (current < 0) break;
                if (current == destination_index) break;
                visited[current] = 1;

                int count = edge_count(&track[current]);
                for (int direction = 0; direction < count; ++direction) {
                        track_edge *edge = &track[current].edge[direction];
                        if (!edge->dest) continue;
                        int next = node_pointer_index(track, edge->dest);
                        if (next < 0 || edge->dist < 0) return -1;
                        int unavailable = physical_node_is_unavailable(
                                track, next, unavailable_by_node);
                        if (unavailable < 0) return -1;
                        if (unavailable) continue;
                        if (cost[current] > INT_MAX - edge->dist) return -1;
                        int candidate = cost[current] + edge->dist;
                        if (candidate == INT_MAX) return -1;
                        if (candidate < cost[next]) {
                                cost[next] = candidate;
                                previous[next] = current;
                        }
                }

                int reversal_allowed =
                        reversal_policy == ROUTE_REVERSAL_ANY_NODE ||
                        (reversal_policy == ROUTE_REVERSAL_SENSOR_ONLY &&
                         track[current].type == NODE_SENSOR);
                if (reversal_allowed && track[current].reverse) {
                        int next = node_pointer_index(
                                track, track[current].reverse);
                        if (next < 0) return -1;
                        if (reversal_policy == ROUTE_REVERSAL_SENSOR_ONLY &&
                            track[next].type != NODE_SENSOR) {
                                return -1;
                        }
                        int unavailable = physical_node_is_unavailable(
                                track, next, unavailable_by_node);
                        if (unavailable < 0) return -1;
                        if (!unavailable) {
                                if (cost[current] >
                                    INT_MAX - reversal_penalty_mm) {
                                        return -1;
                                }
                                int candidate =
                                        cost[current] + reversal_penalty_mm;
                                if (candidate == INT_MAX) return -1;
                                if (candidate < cost[next]) {
                                        cost[next] = candidate;
                                        previous[next] = current;
                                }
                        }
                }
        }

        if (cost[destination_index] == INT_MAX) return -1;

        int count = 0;
        for (int node = destination_index; node >= 0;
             node = previous[node]) {
                if (count >= TRACK_MAX) {
                        clear_route(route);
                        return -1;
                }
                reversed[count++] = node;
                if (node == start_index) break;
        }
        if (count < 1 || reversed[count - 1] != start_index) {
                clear_route(route);
                return -1;
        }

        route->node_count = count;
        route->optimization_cost_mm = cost[destination_index];
        for (int i = 0; i < count; ++i) {
                route->nodes[i] = reversed[count - i - 1];
        }

        int physical_distance = 0;
        for (int i = 0; i + 1 < count; ++i) {
                int edge_distance;
                int is_reversal;
                if (transition_distance(track, route->nodes[i],
                                        route->nodes[i + 1],
                                        &edge_distance, &is_reversal) < 0) {
                        clear_route(route);
                        return -1;
                }
                if (is_reversal) {
                        ++route->reversal_count;
                } else {
                        if (physical_distance >
                            INT_MAX - edge_distance) {
                                clear_route(route);
                                return -1;
                        }
                        physical_distance += edge_distance;
                }
        }
        route->distance_mm = physical_distance;
        return 0;
}

int TrackFindShortestRoute(track_node *track, int start_index,
                           int destination_index, track_route *route) {
        return find_route(track, start_index, destination_index,
                          ROUTE_REVERSAL_NONE, 0, 0, route);
}

int TrackFindShortestRouteWithReversals(track_node *track, int start_index,
                                        int destination_index,
                                        int reversal_penalty_mm,
                                        track_route *route) {
        return find_route(track, start_index, destination_index,
                          ROUTE_REVERSAL_ANY_NODE, reversal_penalty_mm,
                          0, route);
}

int TrackFindShortestRouteWithUnavailable(
        track_node *track, int start_index, int destination_index,
        const unsigned char unavailable_by_node[TRACK_MAX],
        track_route *route) {
        return find_route(track, start_index, destination_index,
                          ROUTE_REVERSAL_NONE, 0,
                          unavailable_by_node, route);
}

int TrackFindShortestRouteWithSensorReversalsAndUnavailable(
        track_node *track, int start_index, int destination_index,
        int reversal_penalty_mm,
        const unsigned char unavailable_by_node[TRACK_MAX],
        track_route *route) {
        return find_route(track, start_index, destination_index,
                          ROUTE_REVERSAL_SENSOR_ONLY,
                          reversal_penalty_mm,
                          unavailable_by_node, route);
}

int TrackFindShortestRouteWithSensorReversals(
        track_node *track, int start_index, int destination_index,
        int reversal_penalty_mm, track_route *route) {
        return TrackFindShortestRouteWithSensorReversalsAndUnavailable(
                track, start_index, destination_index,
                reversal_penalty_mm, 0, route);
}

int TrackRouteDistanceBetweenOffsets(
        track_node *track, const track_route *route,
        int from_offset, int to_offset, int *distance_mm) {
        if (!track || !route_is_valid(route) || !distance_mm ||
            !graph_is_valid(track) || from_offset < 0 ||
            to_offset < from_offset || to_offset >= route->node_count) {
                return -1;
        }

        int total = 0;
        for (int offset = from_offset; offset < to_offset; ++offset) {
                int edge_distance;
                int is_reversal;
                if (transition_distance(track, route->nodes[offset],
                                        route->nodes[offset + 1],
                                        &edge_distance, &is_reversal) < 0) {
                        return -1;
                }
                if (!is_reversal) {
                        if (total > INT_MAX - edge_distance) return -1;
                        total += edge_distance;
                }
        }
        *distance_mm = total;
        return 0;
}

static int route_branch_direction(track_node *track,
                                  const track_route *route,
                                  int branch_index) {
        /*
         * A guard may revisit a turnout. The nearest completed traversal is
         * the setting that governs the next extension, not the first visit.
         */
        for (int i = route->node_count - 2; i >= 0; --i) {
                if (route->nodes[i] != branch_index) continue;
                int next = route->nodes[i + 1];
                if (track[branch_index].edge[DIR_STRAIGHT].dest ==
                    &track[next]) {
                        return DIR_STRAIGHT;
                }
                if (track[branch_index].edge[DIR_CURVED].dest ==
                    &track[next]) {
                        return DIR_CURVED;
                }
        }
        return track[branch_index].edge[DIR_STRAIGHT].dest ?
                DIR_STRAIGHT : DIR_CURVED;
}

static int route_has_node(const track_route *route, int node_index) {
        for (int i = 0; i < route->node_count; ++i) {
                if (route->nodes[i] == node_index) return 1;
        }
        return 0;
}

int TrackExtendRouteAhead(track_node *track, track_route *route,
                          int additional_distance_mm) {
        if (!track || !route_is_valid(route) ||
            !graph_is_valid(track) || additional_distance_mm < 0 ||
            route->distance_mm < 0 ||
            route->optimization_cost_mm < 0) {
                return -1;
        }

        int validated_distance;
        if (TrackRouteDistanceBetweenOffsets(
                    track, route, 0, route->node_count - 1,
                    &validated_distance) < 0) {
                return -1;
        }
        (void)validated_distance;

        track_route candidate = *route;
        int added_mm = 0;
        int current = candidate.nodes[candidate.node_count - 1];
        int traversal_steps = 0;
        while (added_mm < additional_distance_mm &&
               traversal_steps < TRACK_MAX * 32) {
                if (current < 0 || current >= TRACK_MAX) return -1;
                track_node *node = &track[current];
                if (node->type == NODE_EXIT || node->type == NODE_NONE) break;

                int direction = DIR_AHEAD;
                if (node->type == NODE_BRANCH) {
                        direction = route_branch_direction(
                                track, &candidate, current);
                }
                track_edge *edge = &node->edge[direction];
                if (!edge->dest) break;
                if (edge->dist < 0) return -1;

                int next = node_pointer_index(track, edge->dest);
                if (next < 0) return -1;
                if (candidate.node_count < TRACK_MAX) {
                        candidate.nodes[candidate.node_count++] = next;
                } else if (!route_has_node(&candidate, next)) {
                        break;
                }
                if (candidate.distance_mm > INT_MAX - edge->dist ||
                    candidate.optimization_cost_mm >
                            INT_MAX - edge->dist ||
                    added_mm > INT_MAX - edge->dist) {
                        return -1;
                }
                candidate.distance_mm += edge->dist;
                candidate.optimization_cost_mm += edge->dist;
                added_mm += edge->dist;
                current = next;
                ++traversal_steps;
        }
        *route = candidate;
        return added_mm;
}

static int build_turnout_plan_range(
        track_node *track, const track_route *route,
        int first_offset, int last_offset, int reject_conflicts,
        track_turnout_plan *plan) {
        if (!track || !route_is_valid(route) || !plan ||
            !graph_is_valid(track) || first_offset < 0 ||
            last_offset < first_offset ||
            last_offset >= route->node_count) {
                return -1;
        }

        plan->action_count = 0;
        for (int offset = first_offset; offset < last_offset; ++offset) {
                int node_index = route->nodes[offset];
                int next_index = route->nodes[offset + 1];
                int edge_distance;
                int is_reversal;
                if (transition_distance(track, node_index, next_index,
                                        &edge_distance,
                                        &is_reversal) < 0) {
                        return -1;
                }
                (void)edge_distance;
                if (is_reversal) continue;

                track_node *node = &track[node_index];
                track_node *next = &track[next_index];
                if (node->type != NODE_BRANCH) continue;

                int direction = -1;
                if (node->edge[DIR_STRAIGHT].dest == next) {
                        direction = DIR_STRAIGHT;
                } else if (node->edge[DIR_CURVED].dest == next) {
                        direction = DIR_CURVED;
                } else {
                        return -1;
                }

                if (reject_conflicts) {
                        for (int action_index = 0;
                             action_index < plan->action_count;
                             ++action_index) {
                                track_turnout_action *prior =
                                        &plan->actions[action_index];
                                if (prior->switch_number == node->num &&
                                    prior->direction != direction) {
                                        return -1;
                                }
                        }
                }
                if (plan->action_count >= TRACK_MAX) return -1;
                track_turnout_action *action =
                        &plan->actions[plan->action_count++];
                action->switch_number = node->num;
                action->direction = direction;
                action->route_node_offset = offset;
        }
        return 0;
}

int TrackBuildTurnoutPlan(track_node *track, const track_route *route,
                          track_turnout_plan *plan) {
        if (!route_is_valid(route)) return -1;
        return build_turnout_plan_range(
                track, route, 0, route->node_count - 1, 0, plan);
}

int TrackRouteMotionLegEnd(track_node *track, const track_route *route,
                           int leg_start_offset) {
        if (!track || !route_is_valid(route) || !graph_is_valid(track) ||
            leg_start_offset < 0 ||
            leg_start_offset >= route->node_count) {
                return -1;
        }

        for (int offset = leg_start_offset;
             offset + 1 < route->node_count; ++offset) {
                int edge_distance;
                int is_reversal;
                if (transition_distance(track, route->nodes[offset],
                                        route->nodes[offset + 1],
                                        &edge_distance,
                                        &is_reversal) < 0) {
                        return -1;
                }
                (void)edge_distance;
                if (is_reversal) return offset;
        }
        return route->node_count - 1;
}

int TrackBuildTurnoutPlanForLeg(track_node *track,
                                const track_route *route,
                                int leg_start_offset,
                                track_turnout_plan *plan) {
        int leg_end = TrackRouteMotionLegEnd(
                track, route, leg_start_offset);
        if (leg_end < 0) return -1;
        return build_turnout_plan_range(
                track, route, leg_start_offset, leg_end, 1, plan);
}
