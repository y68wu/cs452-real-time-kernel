#include <stdio.h>

#include "tc2_route_monitor.h"
#include "track_data.h"

static int fail(const char *message) {
        fprintf(stderr, "%s\n", message);
        return 1;
}

static int append_node(track_route *route, int node) {
        if (route->node_count >= TRACK_MAX) return -1;
        route->nodes[route->node_count++] = node;
        return 0;
}

static int build_repeated_sensor_route(track_node *track,
                                       track_route *route,
                                       int *repeated_sensor) {
        for (int sensor = 0; sensor < TRACK_MAX; ++sensor) {
                if (track[sensor].type != NODE_SENSOR ||
                    !track[sensor].edge[DIR_AHEAD].dest) {
                        continue;
                }
                int next =
                        (int)(track[sensor].edge[DIR_AHEAD].dest - track);
                track_route return_route;
                if (TrackFindShortestRoute(
                            track, next, sensor,
                            &return_route) < 0 ||
                    return_route.node_count + 1 > TRACK_MAX) {
                        continue;
                }

                int distinct_intermediate_sensor = 0;
                for (int offset = 0;
                     offset + 1 < return_route.node_count; ++offset) {
                        int node = return_route.nodes[offset];
                        if (track[node].type == NODE_SENSOR &&
                            node != sensor) {
                                distinct_intermediate_sensor = 1;
                                break;
                        }
                }
                if (!distinct_intermediate_sensor) continue;

                route->node_count = 0;
                route->distance_mm = 0;
                route->optimization_cost_mm = 0;
                route->reversal_count = 0;
                if (append_node(route, sensor) < 0) return -1;
                for (int offset = 0;
                     offset < return_route.node_count; ++offset) {
                        if (append_node(
                                    route,
                                    return_route.nodes[offset]) < 0) {
                                return -1;
                        }
                }
                int distance;
                if (TrackRouteDistanceBetweenOffsets(
                            track, route, 0,
                            route->node_count - 1,
                            &distance) < 0) {
                        continue;
                }
                route->distance_mm = distance;
                route->optimization_cost_mm = distance;
                *repeated_sensor = sensor;
                return 0;
        }
        return -1;
}

static int find_unexpected_sensor(track_node *track, int first, int second) {
        for (int node = 0; node < TRACK_MAX; ++node) {
                if (track[node].type == NODE_SENSOR &&
                    node != first && node != second) {
                        return node;
                }
        }
        return -1;
}

static int test_missing_duplicate_spurious(track_node *track) {
        int start = TrackFindNodeByName(track, "EN5");
        int target = TrackFindNodeByName(track, "E5");
        track_route route;
        if (TrackFindShortestRoute(track, start, target, &route) < 0) {
                return fail("monitor route missing");
        }

        tc2_route_monitor monitor;
        if (Tc2RouteMonitorInit(
                    track, &route, 0, route.node_count - 1,
                    10, &monitor) < 0) {
                return fail("monitor init failed");
        }
        tc2_route_sensor_point next;
        tc2_route_sensor_point next_next;
        if (Tc2RouteExpectedSensors(
                    track, &route, monitor.confirmed_offset,
                    monitor.terminal_offset,
                    &next, &next_next) != 2) {
                return fail("route did not expose next/next-next sensors");
        }

        tc2_route_observation observation;
        int original_offset = monitor.confirmed_offset;
        if (Tc2RouteMonitorObserve(
                    track, &route, &monitor,
                    next_next.node_index, 11,
                    &observation) < 0 ||
            observation.classification !=
                    TC2_ROUTE_SENSOR_MISSING_ONE ||
            observation.missing.route_offset != next.route_offset ||
            observation.matched.route_offset !=
                    next_next.route_offset ||
            monitor.confirmed_offset != next_next.route_offset ||
            monitor.confirmed_offset == original_offset) {
                return fail("single-missing progress was not classified/advanced");
        }

        int after_missing_offset = monitor.confirmed_offset;
        if (Tc2RouteMonitorObserve(
                    track, &route, &monitor,
                    next_next.node_index, 11,
                    &observation) < 0 ||
            observation.classification !=
                    TC2_ROUTE_SENSOR_DUPLICATE ||
            monitor.confirmed_offset != after_missing_offset) {
                return fail("duplicate sequence advanced progress");
        }

        if (Tc2RouteExpectedSensors(
                    track, &route, monitor.confirmed_offset,
                    monitor.terminal_offset,
                    &next, &next_next) < 1) {
                return fail("no sensor remains after missing test");
        }
        int unexpected = find_unexpected_sensor(
                track, next.node_index,
                next_next.node_index);
        if (unexpected < 0 ||
            Tc2RouteMonitorObserve(
                    track, &route, &monitor,
                    unexpected, 12, &observation) < 0 ||
            observation.classification !=
                    TC2_ROUTE_SENSOR_SPURIOUS ||
            monitor.confirmed_offset != after_missing_offset) {
                return fail("spurious sensor advanced progress");
        }

        if (Tc2RouteMonitorObserve(
                    track, &route, &monitor,
                    next.node_index, 13,
                    &observation) < 0 ||
            observation.classification !=
                    TC2_ROUTE_SENSOR_NORMAL ||
            monitor.confirmed_offset != next.route_offset) {
                return fail("normal sensor did not advance");
        }

        int before_second_skip = monitor.confirmed_offset;
        int count = Tc2RouteExpectedSensors(
                track, &route, monitor.confirmed_offset,
                monitor.terminal_offset,
                &next, &next_next);
        if (count == 2) {
                if (Tc2RouteMonitorObserve(
                            track, &route, &monitor,
                            next_next.node_index, 14,
                            &observation) < 0 ||
                    observation.classification !=
                            TC2_ROUTE_SENSOR_SPURIOUS ||
                    monitor.confirmed_offset != before_second_skip) {
                        return fail("second missing sensor was incorrectly tolerated");
                }
        }
        return 0;
}

static int test_sequence_wrap(track_node *track) {
        int start = TrackFindNodeByName(track, "EN5");
        int target = TrackFindNodeByName(track, "E5");
        track_route route;
        if (TrackFindShortestRoute(
                    track, start, target, &route) < 0) {
                return fail("wrap monitor route missing");
        }
        tc2_route_monitor monitor;
        if (Tc2RouteMonitorInit(
                    track, &route, 0,
                    route.node_count - 1,
                    0xffffffffu, &monitor) < 0) {
                return fail("wrap monitor init failed");
        }
        tc2_route_sensor_point next;
        tc2_route_sensor_point next_next;
        if (Tc2RouteExpectedSensors(
                    track, &route,
                    monitor.confirmed_offset,
                    monitor.terminal_offset,
                    &next, &next_next) < 1) {
                return fail("wrap monitor expected sensor missing");
        }
        tc2_route_observation observation;
        if (Tc2RouteMonitorObserve(
                    track, &route, &monitor,
                    next.node_index, 1,
                    &observation) < 0 ||
            observation.classification !=
                    TC2_ROUTE_SENSOR_NORMAL ||
            monitor.last_sequence != 1 ||
            monitor.confirmed_offset != next.route_offset) {
                return fail("post-wrap sequence did not advance");
        }
        int confirmed = monitor.confirmed_offset;
        if (Tc2RouteMonitorObserve(
                    track, &route, &monitor,
                    next.node_index, 0xffffffffu,
                    &observation) < 0 ||
            observation.classification !=
                    TC2_ROUTE_SENSOR_DUPLICATE ||
            monitor.confirmed_offset != confirmed) {
                return fail("pre-wrap sequence was not rejected as old");
        }
        return 0;
}

static int test_physical_sensor_pair_alias(track_node *track) {
        int start = TrackFindNodeByName(track, "EN5");
        int target = TrackFindNodeByName(track, "E5");
        track_route route;
        if (start < 0 || target < 0 ||
            TrackFindShortestRoute(track, start, target, &route) < 0) {
                return fail("physical-pair monitor route missing");
        }
        tc2_route_monitor monitor;
        if (Tc2RouteMonitorInit(
                    track, &route, 0, route.node_count - 1,
                    40, &monitor) < 0) {
                return fail("physical-pair monitor init failed");
        }
        tc2_route_sensor_point next;
        tc2_route_sensor_point next_next;
        if (Tc2RouteExpectedSensors(
                    track, &route, monitor.confirmed_offset,
                    monitor.terminal_offset,
                    &next, &next_next) < 1 ||
            !track[next.node_index].reverse ||
            track[next.node_index].reverse->type != NODE_SENSOR) {
                return fail("physical-pair expected sensor missing");
        }
        int paired = (int)(
                track[next.node_index].reverse - track);
        tc2_route_observation observation;
        if (Tc2RouteMonitorObserve(
                    track, &route, &monitor, paired, 41,
                    &observation) < 0 ||
            observation.classification !=
                    TC2_ROUTE_SENSOR_NORMAL ||
            observation.matched.node_index != next.node_index ||
            observation.matched.route_offset != next.route_offset ||
            monitor.confirmed_offset != next.route_offset) {
                return fail(
                        "opposite directed half did not confirm physical "
                        "sensor progress");
        }
        return 0;
}

static int test_repeated_occurrence(track_node *track) {
        track_route route;
        int repeated_sensor;
        if (build_repeated_sensor_route(
                    track, &route, &repeated_sensor) < 0 ||
            route.nodes[0] != repeated_sensor ||
            route.nodes[route.node_count - 1] != repeated_sensor) {
                return fail("could not build repeated sensor occurrence route");
        }

        int first_distance;
        int repeated_distance;
        if (Tc2RouteDistanceAtOffset(
                    track, &route, 0, &first_distance) < 0 ||
            Tc2RouteDistanceAtOffset(
                    track, &route, route.node_count - 1,
                    &repeated_distance) < 0 ||
            first_distance != 0 || repeated_distance <= first_distance) {
                return fail("repeated occurrence distance did not use route offsets");
        }

        route.distance_mm = 1;
        int distance_again;
        if (Tc2RouteDistanceAtOffset(
                    track, &route, route.node_count - 1,
                    &distance_again) < 0 ||
            distance_again != repeated_distance) {
                return fail("monitor distance trusted stale route.distance_mm");
        }

        tc2_route_monitor monitor;
        if (Tc2RouteMonitorInit(
                    track, &route, 0, route.node_count - 1,
                    20, &monitor) < 0) {
                return fail("repeated occurrence monitor init failed");
        }
        unsigned int sequence = 21;
        int saw_repeated_occurrence = 0;
        int saw_duplicate = 0;
        for (;;) {
                tc2_route_sensor_point next;
                tc2_route_sensor_point next_next;
                int expected = Tc2RouteExpectedSensors(
                        track, &route, monitor.confirmed_offset,
                        monitor.terminal_offset,
                        &next, &next_next);
                if (expected < 0) {
                        return fail("expected-sensor query failed on loop");
                }
                if (expected == 0) break;

                tc2_route_observation observation;
                if (Tc2RouteMonitorObserve(
                            track, &route, &monitor,
                            next.node_index, sequence,
                            &observation) < 0 ||
                    observation.classification !=
                            TC2_ROUTE_SENSOR_NORMAL ||
                    observation.matched.route_offset !=
                            next.route_offset) {
                        return fail("normal loop occurrence did not advance by offset");
                }
                if (!saw_duplicate) {
                        int saved_offset = monitor.confirmed_offset;
                        if (Tc2RouteMonitorObserve(
                                    track, &route, &monitor,
                                    next.node_index, sequence,
                                    &observation) < 0 ||
                            observation.classification !=
                                    TC2_ROUTE_SENSOR_DUPLICATE ||
                            monitor.confirmed_offset != saved_offset) {
                                return fail("loop duplicate was not rejected by sequence");
                        }
                        saw_duplicate = 1;
                }
                if (next.node_index == repeated_sensor &&
                    next.route_offset > 0) {
                        saw_repeated_occurrence = 1;
                }
                ++sequence;
        }
        if (!saw_repeated_occurrence ||
            monitor.confirmed_offset != route.node_count - 1) {
                return fail("second occurrence of repeated sensor was not confirmed");
        }
        return 0;
}

static int test_terminal_guard_and_reversal_rejection(track_node *track) {
        int start = TrackFindNodeByName(track, "EN5");
        int target = TrackFindNodeByName(track, "B15");
        track_route route;
        if (TrackFindShortestRoute(track, start, target, &route) < 0) {
                return fail("guard monitor route missing");
        }
        int terminal_offset = route.node_count - 1;
        if (TrackExtendRouteAhead(track, &route, 3000) < 0) {
                return fail("guard extension failed");
        }
        int guard_sensor = -1;
        for (int offset = terminal_offset + 1;
             offset < route.node_count; ++offset) {
                if (track[route.nodes[offset]].type == NODE_SENSOR) {
                        guard_sensor = route.nodes[offset];
                        break;
                }
        }
        if (guard_sensor < 0) {
                return fail("guard extension had no sensor for boundary test");
        }

        tc2_route_monitor monitor;
        if (Tc2RouteMonitorInit(
                    track, &route, 0, terminal_offset,
                    30, &monitor) < 0) {
                return fail("guard-bounded monitor init failed");
        }
        unsigned int sequence = 31;
        for (;;) {
                tc2_route_sensor_point next;
                tc2_route_sensor_point next_next;
                int count = Tc2RouteExpectedSensors(
                        track, &route, monitor.confirmed_offset,
                        monitor.terminal_offset,
                        &next, &next_next);
                if (count < 0) return fail("guard expected query failed");
                if (count == 0) break;
                tc2_route_observation observation;
                if (Tc2RouteMonitorObserve(
                            track, &route, &monitor,
                            next.node_index, sequence++,
                            &observation) < 0 ||
                    observation.classification !=
                            TC2_ROUTE_SENSOR_NORMAL) {
                        return fail("normal progress to target failed");
                }
        }
        tc2_route_observation observation;
        if (Tc2RouteMonitorObserve(
                    track, &route, &monitor,
                    guard_sensor, sequence,
                    &observation) < 0 ||
            observation.classification !=
                    TC2_ROUTE_SENSOR_SPURIOUS ||
            monitor.confirmed_offset != terminal_offset) {
                return fail("guard sensor crossed terminal occurrence");
        }

        const char *starts[] = {"EN5", "EN4", "EN7", "EN10", "EN9", "EN3"};
        const char *targets[] = {"A12", "B15", "C12", "C9",
                                 "E5", "E14", "D7", "C15"};
        int found_reversal = 0;
        for (int s = 0; s < 6 && !found_reversal; ++s) {
                for (int d = 0; d < 8 && !found_reversal; ++d) {
                        track_route reversed;
                        if (TrackFindShortestRouteWithSensorReversals(
                                    track,
                                    TrackFindNodeByName(track, starts[s]),
                                    TrackFindNodeByName(track, targets[d]),
                                    400, &reversed) == 0 &&
                            reversed.reversal_count > 0) {
                                if (Tc2RouteMonitorInit(
                                            track, &reversed, 0,
                                            reversed.node_count - 1,
                                            1, &monitor) >= 0) {
                                        return fail("single-leg monitor accepted a reversal");
                                }
                                found_reversal = 1;
                        }
                }
        }
        if (!found_reversal) {
                return fail("reversal rejection test was vacuous");
        }
        return 0;
}

int main(void) {
        track_node track[TRACK_MAX];
        init_trackb(track);
        if (test_missing_duplicate_spurious(track) ||
            test_sequence_wrap(track) ||
            test_physical_sensor_pair_alias(track) ||
            test_repeated_occurrence(track) ||
            test_terminal_guard_and_reversal_rejection(track)) {
                return 1;
        }
        puts("validated occurrence-aware route distance, uint32 sequence wrap, and ordered normal/missing/duplicate/spurious monitoring");
        return 0;
}
