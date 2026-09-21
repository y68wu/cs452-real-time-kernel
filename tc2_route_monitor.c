#include "tc2_route_monitor.h"

static int event_sequence_after(unsigned int candidate,
                                unsigned int reference) {
        if (candidate == 0 || candidate == reference) return 0;
        if (reference == 0) return 1;
        return candidate - reference < (1u << 31);
}

static void clear_sensor_point(tc2_route_sensor_point *point) {
        if (!point) return;
        point->route_offset = -1;
        point->node_index = -1;
        point->distance_mm = -1;
}

static void clear_observation(tc2_route_observation *observation) {
        if (!observation) return;
        observation->classification = TC2_ROUTE_SENSOR_SPURIOUS;
        clear_sensor_point(&observation->expected);
        clear_sensor_point(&observation->expected_next);
        clear_sensor_point(&observation->matched);
        clear_sensor_point(&observation->missing);
        observation->advance_mm = 0;
}

static int monitor_route_shape_is_valid(const track_route *route) {
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

/*
 * One physical Track-D detector has two directed SENSOR nodes.  The S88
 * attribution layer may report either directed half, while a planned route
 * contains only the half matching the train's travel direction.  Match the
 * physical pair here, but keep the route's directed node as the observation
 * result so route progress remains monotonic.
 */
static int same_physical_sensor(track_node *track,
                                int observed, int expected) {
        if (!track || observed < 0 || observed >= TRACK_MAX ||
            expected < 0 || expected >= TRACK_MAX ||
            track[observed].type != NODE_SENSOR ||
            track[expected].type != NODE_SENSOR) {
                return 0;
        }
        return observed == expected ||
                track[expected].reverse == &track[observed];
}

int Tc2RouteDistanceAtOffset(track_node *track, const track_route *route,
                             int offset, int *distance_mm) {
        if (!monitor_route_shape_is_valid(route) || !distance_mm ||
            offset < 0 || offset >= route->node_count) {
                return -1;
        }
        return TrackRouteDistanceBetweenOffsets(
                track, route, 0, offset, distance_mm);
}

static int fill_sensor_point(track_node *track, const track_route *route,
                             int offset,
                             tc2_route_sensor_point *point) {
        if (!point || !track || !monitor_route_shape_is_valid(route) ||
            offset < 0 || offset >= route->node_count ||
            track[route->nodes[offset]].type != NODE_SENSOR) {
                return -1;
        }

        int distance;
        if (Tc2RouteDistanceAtOffset(track, route, offset, &distance) < 0) {
                return -1;
        }
        point->route_offset = offset;
        point->node_index = route->nodes[offset];
        point->distance_mm = distance;
        return 0;
}

int Tc2RouteExpectedSensors(track_node *track, const track_route *route,
                            int after_offset, int terminal_offset,
                            tc2_route_sensor_point *next,
                            tc2_route_sensor_point *next_next) {
        if (!track || !monitor_route_shape_is_valid(route) ||
            !next || !next_next || after_offset < -1 ||
            terminal_offset < 0 ||
            terminal_offset >= route->node_count ||
            after_offset > terminal_offset) {
                return -1;
        }

        clear_sensor_point(next);
        clear_sensor_point(next_next);
        int count = 0;
        for (int offset = after_offset + 1;
             offset <= terminal_offset && count < 2; ++offset) {
                int node = route->nodes[offset];
                if (track[node].type != NODE_SENSOR) continue;
                tc2_route_sensor_point *point =
                        count == 0 ? next : next_next;
                if (fill_sensor_point(track, route, offset, point) < 0) {
                        clear_sensor_point(next);
                        clear_sensor_point(next_next);
                        return -1;
                }
                ++count;
        }
        return count;
}

int Tc2RouteMonitorInit(track_node *track, const track_route *route,
                        int origin_offset, int terminal_offset,
                        unsigned int baseline_sequence,
                        tc2_route_monitor *monitor) {
        if (!track || !monitor || !monitor_route_shape_is_valid(route) ||
            origin_offset < 0 || terminal_offset < origin_offset ||
            terminal_offset >= route->node_count) {
                return -1;
        }

        int origin_distance;
        int range_distance;
        if (Tc2RouteDistanceAtOffset(
                    track, route, origin_offset, &origin_distance) < 0 ||
            TrackRouteDistanceBetweenOffsets(
                    track, route, origin_offset, terminal_offset,
                    &range_distance) < 0) {
                return -1;
        }
        (void)range_distance;

        for (int offset = origin_offset;
             offset < terminal_offset; ++offset) {
                int node = route->nodes[offset];
                int next = route->nodes[offset + 1];
                if (track[node].reverse == &track[next]) {
                        return -1;
                }
        }

        monitor->origin_offset = origin_offset;
        monitor->terminal_offset = terminal_offset;
        monitor->confirmed_offset = origin_offset;
        monitor->confirmed_distance_mm = origin_distance;
        monitor->last_sequence = baseline_sequence;
        monitor->last_observed_node = -1;
        monitor->missing_sensor_count = 0;
        return 0;
}

static int monitor_state_is_valid(track_node *track,
                                  const track_route *route,
                                  const tc2_route_monitor *monitor) {
        if (!track || !monitor || !monitor_route_shape_is_valid(route) ||
            monitor->origin_offset < 0 ||
            monitor->terminal_offset < monitor->origin_offset ||
            monitor->terminal_offset >= route->node_count ||
            monitor->confirmed_offset < monitor->origin_offset ||
            monitor->confirmed_offset > monitor->terminal_offset ||
            monitor->missing_sensor_count < 0 ||
            monitor->missing_sensor_count > 1) {
                return 0;
        }
        int confirmed_distance;
        return Tc2RouteDistanceAtOffset(
                       track, route, monitor->confirmed_offset,
                       &confirmed_distance) == 0 &&
                confirmed_distance == monitor->confirmed_distance_mm;
}

int Tc2RouteMonitorObserve(track_node *track, const track_route *route,
                           tc2_route_monitor *monitor,
                           int sensor_node, unsigned int event_sequence,
                           tc2_route_observation *observation) {
        if (!track || !observation ||
            !monitor_state_is_valid(track, route, monitor) ||
            sensor_node < 0 || sensor_node >= TRACK_MAX ||
            track[sensor_node].type != NODE_SENSOR ||
            event_sequence == 0) {
                return -1;
        }

        clear_observation(observation);
        int expected_count = Tc2RouteExpectedSensors(
                track, route, monitor->confirmed_offset,
                monitor->terminal_offset,
                &observation->expected,
                &observation->expected_next);
        if (expected_count < 0) return -1;

        if (!event_sequence_after(
                    event_sequence,
                    monitor->last_sequence)) {
                observation->classification =
                        TC2_ROUTE_SENSOR_DUPLICATE;
                return 0;
        }

        monitor->last_sequence = event_sequence;
        monitor->last_observed_node = sensor_node;
        if (expected_count >= 1 &&
            same_physical_sensor(
                    track, sensor_node,
                    observation->expected.node_index)) {
                observation->classification = TC2_ROUTE_SENSOR_NORMAL;
                observation->matched = observation->expected;
        } else if (expected_count >= 2 &&
                   same_physical_sensor(
                           track, sensor_node,
                           observation->expected_next.node_index) &&
                   monitor->missing_sensor_count == 0) {
                observation->classification =
                        TC2_ROUTE_SENSOR_MISSING_ONE;
                observation->matched = observation->expected_next;
                observation->missing = observation->expected;
                monitor->missing_sensor_count = 1;
        } else {
                observation->classification =
                        TC2_ROUTE_SENSOR_SPURIOUS;
                return 0;
        }

        if (observation->matched.distance_mm <
            monitor->confirmed_distance_mm) {
                return -1;
        }
        observation->advance_mm =
                observation->matched.distance_mm -
                monitor->confirmed_distance_mm;
        monitor->confirmed_offset =
                observation->matched.route_offset;
        monitor->confirmed_distance_mm =
                observation->matched.distance_mm;
        return 0;
}
