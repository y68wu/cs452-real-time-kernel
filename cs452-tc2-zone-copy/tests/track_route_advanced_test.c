#include <limits.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "track_data.h"
#include "track_route.h"

static const char *const starts[] = {
        "EN5", "EN4", "EN7", "EN10", "EN9", "EN3"
};

static const char *const destination_a[] = {
        "A12", "B15", "C12", "C9", "E5", "E14", "D7", "C15"
};

static const char *const destination_b[] = {
        "A11", "B16", "C11", "C10", "E6", "E13", "D8", "C16"
};

static int fail(const char *message) {
        fprintf(stderr, "%s\n", message);
        return 1;
}

static int route_contains_physical(track_node *track,
                                   const track_route *route,
                                   int node) {
        int reverse = track[node].reverse ?
                (int)(track[node].reverse - track) : -1;
        for (int i = 0; i < route->node_count; ++i) {
                if (route->nodes[i] == node ||
                    route->nodes[i] == reverse) {
                        return 1;
                }
        }
        return 0;
}

static int append_node(track_route *route, int node) {
        if (route->node_count >= TRACK_MAX) return -1;
        route->nodes[route->node_count++] = node;
        return 0;
}

static int build_recent_branch_route(track_node *track,
                                     track_route *route,
                                     int *branch_out,
                                     int *recent_next_out) {
        for (int branch = 0; branch < TRACK_MAX; ++branch) {
                if (track[branch].type != NODE_BRANCH ||
                    !track[branch].edge[DIR_STRAIGHT].dest ||
                    !track[branch].edge[DIR_CURVED].dest) {
                        continue;
                }
                int straight =
                        (int)(track[branch].edge[DIR_STRAIGHT].dest - track);
                int curved =
                        (int)(track[branch].edge[DIR_CURVED].dest - track);
                track_route straight_return;
                track_route curved_return;
                if (TrackFindShortestRoute(
                            track, straight, branch,
                            &straight_return) < 0 ||
                    TrackFindShortestRoute(
                            track, curved, branch,
                            &curved_return) < 0) {
                        continue;
                }
                int required = 1 + straight_return.node_count +
                        curved_return.node_count;
                if (required >= TRACK_MAX) continue;

                route->node_count = 0;
                route->distance_mm = 0;
                route->optimization_cost_mm = 0;
                route->reversal_count = 0;
                if (append_node(route, branch) < 0) return -1;
                for (int i = 0; i < straight_return.node_count; ++i) {
                        if (append_node(route,
                                        straight_return.nodes[i]) < 0) {
                                return -1;
                        }
                }
                /*
                 * straight_return ends at branch. Traverse curved next, then
                 * append the rest of the curved-to-branch return route.
                 */
                for (int i = 0; i < curved_return.node_count; ++i) {
                        if (append_node(route,
                                        curved_return.nodes[i]) < 0) {
                                return -1;
                        }
                }
                int distance;
                if (TrackRouteDistanceBetweenOffsets(
                            track, route, 0, route->node_count - 1,
                            &distance) < 0) {
                        continue;
                }
                route->distance_mm = distance;
                route->optimization_cost_mm = distance;
                *branch_out = branch;
                *recent_next_out = curved;
                return 0;
        }
        return -1;
}

static int test_candidates_and_unavailable(track_node *track) {
        int paired_side_shorter = 0;
        for (unsigned int start_index = 0;
             start_index < sizeof(starts) / sizeof(starts[0]);
             ++start_index) {
                int start = TrackFindNodeByName(
                        track, starts[start_index]);
                if (start < 0) return fail("missing A-F start node");
                for (unsigned int destination = 0;
                     destination <
                             sizeof(destination_a) /
                                     sizeof(destination_a[0]);
                     ++destination) {
                        int side_a = TrackFindNodeByName(
                                track, destination_a[destination]);
                        int side_b = TrackFindNodeByName(
                                track, destination_b[destination]);
                        track_route route_a;
                        track_route route_b;
                        if (side_a < 0 || side_b < 0 ||
                            TrackFindShortestRoute(
                                    track, start, side_a,
                                    &route_a) < 0 ||
                            TrackFindShortestRoute(
                                    track, start, side_b,
                                    &route_b) < 0) {
                                return fail("one D-side forward candidate is missing");
                        }
                        if (route_b.distance_mm < route_a.distance_mm) {
                                ++paired_side_shorter;
                        }
                }
        }
        if (paired_side_shorter != 18) {
                return fail("expected exactly 18/48 shorter paired approaches");
        }

        int start = TrackFindNodeByName(track, "EN5");
        int destination = TrackFindNodeByName(track, "B15");
        int blocked = TrackFindNodeByName(track, "E10");
        unsigned char unavailable[TRACK_MAX];
        memset(unavailable, 0, sizeof(unavailable));
        unavailable[blocked] = 1;
        track_route baseline;
        track_route alternate;
        if (TrackFindShortestRoute(
                    track, start, destination, &baseline) < 0 ||
            TrackFindShortestRouteWithUnavailable(
                    track, start, destination,
                    unavailable, &alternate) < 0) {
                return fail("E10 alternate route was not found");
        }
        if (baseline.distance_mm != 4724 ||
            alternate.distance_mm != 4834 ||
            route_contains_physical(track, &alternate, blocked)) {
                return fail("E10 alternate route has unexpected distance/footprint");
        }
        return 0;
}

static int test_sensor_reversals_and_legs(track_node *track) {
        int routes_with_reversals = 0;
        int checked_legs = 0;
        for (unsigned int start_index = 0;
             start_index < sizeof(starts) / sizeof(starts[0]);
             ++start_index) {
                int start = TrackFindNodeByName(
                        track, starts[start_index]);
                for (unsigned int destination = 0;
                     destination <
                             sizeof(destination_a) /
                                     sizeof(destination_a[0]);
                     ++destination) {
                        const char *sides[] = {
                                destination_a[destination],
                                destination_b[destination]
                        };
                        for (int side = 0; side < 2; ++side) {
                                int target = TrackFindNodeByName(
                                        track, sides[side]);
                                track_route route;
                                if (TrackFindShortestRouteWithSensorReversals(
                                            track, start, target,
                                            400, &route) < 0) {
                                        return fail("sensor-only reversal route missing");
                                }
                                int counted_reversals = 0;
                                for (int i = 0;
                                     i + 1 < route.node_count; ++i) {
                                        int node = route.nodes[i];
                                        int next = route.nodes[i + 1];
                                        if (track[node].reverse ==
                                            &track[next]) {
                                                ++counted_reversals;
                                                if (track[node].type !=
                                                            NODE_SENSOR ||
                                                    track[next].type !=
                                                            NODE_SENSOR) {
                                                        return fail("automatic reversal is not sensor-only");
                                                }
                                        }
                                }
                                if (counted_reversals !=
                                    route.reversal_count) {
                                        return fail("reversal count does not match route transitions");
                                }
                                if (counted_reversals > 0) {
                                        ++routes_with_reversals;
                                }

                                int leg_start = 0;
                                for (;;) {
                                        int leg_end =
                                                TrackRouteMotionLegEnd(
                                                        track, &route,
                                                        leg_start);
                                        track_turnout_plan plan;
                                        if (leg_end < leg_start ||
                                            TrackBuildTurnoutPlanForLeg(
                                                    track, &route,
                                                    leg_start,
                                                    &plan) < 0) {
                                                return fail("motion-leg turnout plan failed");
                                        }
                                        for (int action = 0;
                                             action < plan.action_count;
                                             ++action) {
                                                int offset =
                                                        plan.actions[action]
                                                                .route_node_offset;
                                                if (offset < leg_start ||
                                                    offset >= leg_end) {
                                                        return fail("leg plan crossed its reversal boundary");
                                                }
                                        }
                                        ++checked_legs;
                                        if (leg_end ==
                                            route.node_count - 1) {
                                                break;
                                        }
                                        if (track[route.nodes[leg_end]]
                                                            .reverse !=
                                            &track[route.nodes[
                                                    leg_end + 1]]) {
                                                return fail("leg boundary is not a reversal");
                                        }
                                        leg_start = leg_end + 1;
                                }
                        }
                }
        }
        if (routes_with_reversals == 0 || checked_legs == 0) {
                return fail("sensor-only reversal/leg tests were vacuous");
        }
        return 0;
}

static int test_invalid_weights_and_overflow(void) {
        track_node graph[TRACK_MAX];
        memset(graph, 0, sizeof(graph));
        graph[0].type = NODE_ENTER;
        graph[1].type = NODE_SENSOR;
        graph[2].type = NODE_SENSOR;
        graph[0].edge[DIR_AHEAD].dest = &graph[1];
        graph[0].edge[DIR_AHEAD].dist = 1;
        graph[1].edge[DIR_AHEAD].dest = &graph[2];
        graph[1].edge[DIR_AHEAD].dist = -1;
        track_route route;
        if (TrackFindShortestRoute(graph, 0, 2, &route) >= 0) {
                return fail("negative edge weight was accepted");
        }

        graph[1].edge[DIR_AHEAD].dist = 10;
        graph[0].edge[DIR_AHEAD].dist = INT_MAX - 5;
        if (TrackFindShortestRoute(graph, 0, 2, &route) >= 0) {
                return fail("overflowing path cost was accepted");
        }

        graph[0].edge[DIR_AHEAD].dist = 1;
        graph[0].edge[DIR_AHEAD].dest =
                (track_node *)((uintptr_t)(void *)&graph[0] + 1u);
        if (TrackFindShortestRoute(graph, 0, 2, &route) >= 0) {
                return fail("misaligned edge destination was accepted");
        }
        return 0;
}

static int test_distance_extension_and_recent_branch(track_node *track) {
        int start = TrackFindNodeByName(track, "EN5");
        int destination = TrackFindNodeByName(track, "A12");
        track_route route;
        if (TrackFindShortestRoute(
                    track, start, destination, &route) < 0) {
                return fail("distance test route missing");
        }
        int recomputed;
        if (TrackRouteDistanceBetweenOffsets(
                    track, &route, 0, route.node_count - 1,
                    &recomputed) < 0 ||
            recomputed != route.distance_mm) {
                return fail("whole-route physical distance mismatch");
        }
        int real_distance = recomputed;
        route.distance_mm = 1;
        if (TrackRouteDistanceBetweenOffsets(
                    track, &route, 0, route.node_count - 1,
                    &recomputed) < 0 ||
            recomputed != real_distance) {
                return fail("offset distance trusted stale route.distance_mm");
        }

        track_route repeated;
        int branch;
        int recent_next;
        if (build_recent_branch_route(
                    track, &repeated, &branch, &recent_next) < 0) {
                return fail("could not construct a repeated-branch walk");
        }
        int old_count = repeated.node_count;
        if (repeated.nodes[old_count - 1] != branch ||
            TrackExtendRouteAhead(track, &repeated, 1) <= 0 ||
            repeated.node_count != old_count + 1 ||
            repeated.nodes[old_count] != recent_next) {
                return fail("route extension did not use most recent branch traversal");
        }

        track_route overflow = repeated;
        overflow.distance_mm = INT_MAX;
        overflow.optimization_cost_mm = INT_MAX;
        int before_count = overflow.node_count;
        if (TrackExtendRouteAhead(track, &overflow, 1) >= 0 ||
            overflow.node_count != before_count ||
            overflow.distance_mm != INT_MAX) {
                return fail("overflowing extension did not fail atomically");
        }
        return 0;
}

static int test_a_to_d1_physical_mapping(track_node *track) {
        int start = TrackFindNodeByName(track, "EN5");
        int anchor = TrackFindNodeByName(track, "A12");
        track_route route;
        track_turnout_plan plan;
        const int expected_switches[] = {11, 12, 4};
        const int expected_directions[] = {
                DIR_STRAIGHT, DIR_CURVED, DIR_STRAIGHT
        };

        if (TrackFindShortestRouteWithSensorReversals(
                    track, start, anchor, 400, &route) < 0 ||
            route.distance_mm != 2485 ||
            route.reversal_count != 2 ||
            TrackBuildTurnoutPlan(track, &route, &plan) < 0 ||
            plan.action_count != 3 ||
            track[anchor].edge[DIR_AHEAD].dist != 814 ||
            !track[anchor].edge[DIR_AHEAD].dest ||
            strcmp(track[anchor].edge[DIR_AHEAD].dest->name, "A16") != 0) {
                return fail("A-to-D1/A12 physical route or endpoint edge mismatch");
        }
        for (int i = 0; i < plan.action_count; ++i) {
                if (plan.actions[i].switch_number != expected_switches[i] ||
                    plan.actions[i].direction != expected_directions[i]) {
                return fail("A-to-D1/A12 turnout sequence mismatch");
                }
        }
        return 0;
}

int main(void) {
        track_node track[TRACK_MAX];
        init_trackb(track);

        if (test_candidates_and_unavailable(track) ||
            test_sensor_reversals_and_legs(track) ||
            test_invalid_weights_and_overflow() ||
            test_distance_extension_and_recent_branch(track) ||
            test_a_to_d1_physical_mapping(track)) {
                return 1;
        }
        puts("validated unavailable routing, paired approaches, sensor-only reversals, motion legs, distance and extension safety");
        return 0;
}
