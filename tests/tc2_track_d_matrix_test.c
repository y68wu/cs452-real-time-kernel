#include <stdint.h>
#include <stdio.h>

#include "../tc2_motion_model.h"
#include "../tc2_prediction_model.h"
#include "../tc2_route_monitor.h"

enum {
        SPEED_MIN = 1,
        SPEED_MAX = 120,
        EXPECTED_OPERATOR_ROUTES =
                TC2_TRACK_START_COUNT * TC2_TRACK_DESTINATION_COUNT,
        EXPECTED_DIRECTED_ROUTES =
                EXPECTED_OPERATOR_ROUTES *
                TC2_TRACK_DESTINATION_SIDE_COUNT,
        EXPECTED_PREDICTIONS =
                EXPECTED_DIRECTED_ROUTES * SPEED_MAX
};

static int fail_case(const char *message, int start, int destination,
                     int side, int speed) {
        fprintf(stderr,
                "%s start=%d destination=D%d side=%d speed=%d\n",
                message, start, destination + 1, side, speed);
        return -1;
}

static int route_for(track_node track[TRACK_MAX], int start_index,
                     int destination_index, int side_index,
                     track_route *route) {
        const tc2_track_start_definition *start =
                Tc2TrackStartDefinition(start_index);
        const tc2_track_destination_side *side =
                Tc2TrackDestinationSide(destination_index, side_index);
        int from = start ?
                TrackFindNodeByName(track, start->enter_node) : -1;
        int to = side ?
                TrackFindNodeByName(track, side->anchor_sensor) : -1;
        if (!route || from < 0 || to < 0 ||
            track[from].type != NODE_ENTER ||
            track[to].type != NODE_SENSOR ||
            TrackFindShortestRoute(track, from, to, route) < 0 ||
            route->node_count < 2 || route->distance_mm <= 0 ||
            route->reversal_count != 0 ||
            route->nodes[0] != from ||
            route->nodes[route->node_count - 1] != to) {
                return -1;
        }
        return 0;
}

static int check_route_edges(track_node track[TRACK_MAX],
                             const track_route *route) {
        int total = 0;
        if (TrackRouteDistanceBetweenOffsets(
                    track, route, 0, route->node_count - 1,
                    &total) < 0 ||
            total != route->distance_mm ||
            route->optimization_cost_mm < route->distance_mm) {
                return -1;
        }

        for (int offset = 0; offset + 1 < route->node_count; ++offset) {
                int edge_distance = -1;
                if (TrackRouteDistanceBetweenOffsets(
                            track, route, offset, offset + 1,
                            &edge_distance) < 0 ||
                    edge_distance < 0) {
                        return -1;
                }
        }
        return 0;
}

static int check_turnout_sequence(track_node track[TRACK_MAX],
                                  const track_route *route) {
        track_turnout_plan plan;
        if (TrackBuildTurnoutPlan(track, route, &plan) < 0) return -1;

        int expected_actions = 0;
        for (int offset = 0; offset + 1 < route->node_count; ++offset) {
                int node_index = route->nodes[offset];
                int next_index = route->nodes[offset + 1];
                track_node *node = &track[node_index];
                if (node->type != NODE_BRANCH) continue;
                if (expected_actions >= plan.action_count) return -1;

                int direction = -1;
                if (node->edge[DIR_STRAIGHT].dest ==
                    &track[next_index]) {
                        direction = DIR_STRAIGHT;
                } else if (node->edge[DIR_CURVED].dest ==
                           &track[next_index]) {
                        direction = DIR_CURVED;
                }
                const track_turnout_action *action =
                        &plan.actions[expected_actions];
                if (direction < 0 ||
                    action->route_node_offset != offset ||
                    action->switch_number != node->num ||
                    action->direction != direction) {
                        return -1;
                }
                ++expected_actions;
        }
        if (expected_actions != plan.action_count) return -1;

        for (int first = 0; first < plan.action_count; ++first) {
                if (first > 0 &&
                    plan.actions[first - 1].route_node_offset >=
                            plan.actions[first].route_node_offset) {
                        return -1;
                }
                for (int second = first + 1;
                     second < plan.action_count; ++second) {
                        if (plan.actions[first].switch_number ==
                                    plan.actions[second].switch_number &&
                            plan.actions[first].direction !=
                                    plan.actions[second].direction) {
                                return -1;
                        }
                }
        }
        return 0;
}

static int check_sensor_sequence(track_node track[TRACK_MAX],
                                 const track_route *route) {
        tc2_route_monitor monitor;
        if (Tc2RouteMonitorInit(
                    track, route, 0, route->node_count - 1,
                    1u, &monitor) < 0) {
                return -1;
        }

        int cursor = 0;
        unsigned int sequence = 2u;
        int observed = 0;
        while (cursor < route->node_count - 1) {
                tc2_route_sensor_point next;
                tc2_route_sensor_point next_next;
                int count = Tc2RouteExpectedSensors(
                        track, route, cursor,
                        route->node_count - 1, &next, &next_next);
                if (count < 1 || next.route_offset <= cursor ||
                    next.route_offset >= route->node_count ||
                    next.node_index != route->nodes[next.route_offset] ||
                    next.distance_mm < monitor.confirmed_distance_mm) {
                        return -1;
                }

                track_node *sensor = &track[next.node_index];
                if (sensor->type != NODE_SENSOR || !sensor->name ||
                    !sensor->reverse ||
                    sensor->reverse->type != NODE_SENSOR ||
                    sensor->reverse->reverse != sensor ||
                    sensor->reverse == sensor) {
                        return -1;
                }
                if (count == 2 &&
                    (next_next.route_offset <= next.route_offset ||
                     next_next.distance_mm < next.distance_mm)) {
                        return -1;
                }

                tc2_route_observation observation;
                if (Tc2RouteMonitorObserve(
                            track, route, &monitor, next.node_index,
                            sequence++, &observation) < 0 ||
                    observation.classification !=
                            TC2_ROUTE_SENSOR_NORMAL ||
                    observation.matched.route_offset !=
                            next.route_offset ||
                    monitor.confirmed_offset != next.route_offset) {
                        return -1;
                }
                cursor = next.route_offset;
                ++observed;
        }
        return observed > 0 &&
               monitor.confirmed_offset == route->node_count - 1 ?
                0 : -1;
}

static int check_speed_matrix(track_node track[TRACK_MAX],
                              const track_route *route,
                              int start_index, int destination_index,
                              int side_index, int *prediction_count) {
        const tc2_track_destination_side *side =
                Tc2TrackDestinationSide(destination_index, side_index);
        int previous_velocity = -1;
        int previous_stop = -1;
        int previous_braking = -1;
        int64_t previous_ceiling = -1;

        for (int speed = SPEED_MIN; speed <= SPEED_MAX; ++speed) {
                tc2_prediction_request request = {
                        .train = 14,
                        .start_index = start_index,
                        .destination_index = destination_index,
                        .destination_side = side_index,
                        .speed = speed,
                        .reversal_count = route->reversal_count
                };
                tc2_prediction_plan prediction;
                if (Tc2PredictionBuildPlan(
                            track, route, &request,
                            &prediction) != TC2_PREDICTION_OK) {
                        return fail_case("prediction rejected",
                                         start_index, destination_index,
                                         side_index, speed);
                }

                int64_t physical =
                        (int64_t)route->distance_mm * 1000 +
                        (int64_t)side->base_offset_mm * 1000;
                int64_t expected_ceiling =
                        physical +
                        (int64_t)prediction
                                .conservative_braking_distance_mm * 1000;
                if (prediction.geometry_anchor_node !=
                            route->nodes[route->node_count - 1] ||
                    prediction.geometry_anchor_route_offset !=
                            route->node_count - 1 ||
                    prediction.geometry_base_offset_mm !=
                            side->base_offset_mm ||
                    prediction.physical_destination_distance_um !=
                            physical ||
                    prediction.minimum_reservation_ceiling_distance_um !=
                            expected_ceiling ||
                    prediction.minimum_reservation_ceiling_distance_um <
                            physical ||
                    prediction.minimum_reservation_ceiling_distance_um <
                            prediction.endpoint_distance_um ||
                    prediction.endpoint_distance_um < 0 ||
                    prediction.command_distance_um < 0 ||
                    prediction.command_distance_um >
                            prediction.endpoint_distance_um ||
                    prediction.velocity_um_per_tick <= 0 ||
                    prediction.point_stop_distance_um < 0 ||
                    prediction.conservative_braking_distance_mm <= 0) {
                        return fail_case("unsafe prediction bounds",
                                         start_index, destination_index,
                                         side_index, speed);
                }

                if (speed > SPEED_MIN &&
                    (prediction.velocity_um_per_tick <
                                    previous_velocity ||
                     prediction.point_stop_distance_um <
                                    previous_stop ||
                     prediction.conservative_braking_distance_mm <
                                    previous_braking ||
                     prediction.minimum_reservation_ceiling_distance_um <
                                    previous_ceiling)) {
                        return fail_case("non-monotone speed safety model",
                                         start_index, destination_index,
                                         side_index, speed);
                }

                track_route guarded = *route;
                int required_guard_mm =
                        side->base_offset_mm +
                        prediction.conservative_braking_distance_mm;
                int represented_guard_mm =
                        TrackExtendRouteAhead(
                                track, &guarded,
                                required_guard_mm);
                if (represented_guard_mm < required_guard_mm ||
                    check_route_edges(track, &guarded) < 0 ||
                    check_turnout_sequence(track, &guarded) < 0) {
                        return fail_case("unrepresentable reservation guard",
                                         start_index, destination_index,
                                         side_index, speed);
                }

                previous_velocity = prediction.velocity_um_per_tick;
                previous_stop = prediction.point_stop_distance_um;
                previous_braking =
                        prediction.conservative_braking_distance_mm;
                previous_ceiling =
                        prediction.minimum_reservation_ceiling_distance_um;
                ++*prediction_count;
        }
        return 0;
}

int main(void) {
        track_node track[TRACK_MAX];
        int directed_routes = 0;
        int predictions = 0;

        init_trackb(track);
        if (Tc2TrackCatalogValidate(track) < 0) {
                fprintf(stderr, "TC2 Track-D catalog validation failed\n");
                return 1;
        }

        for (int start = 0; start < TC2_TRACK_START_COUNT; ++start) {
                for (int destination = 0;
                     destination < TC2_TRACK_DESTINATION_COUNT;
                     ++destination) {
                        for (int side = 0;
                             side < TC2_TRACK_DESTINATION_SIDE_COUNT;
                             ++side) {
                                track_route route;
                                if (route_for(track, start, destination,
                                              side, &route) < 0) {
                                        fail_case("route unavailable",
                                                  start, destination,
                                                  side, 0);
                                        return 1;
                                }
                                if (check_route_edges(track, &route) < 0 ||
                                    check_turnout_sequence(
                                            track, &route) < 0 ||
                                    check_sensor_sequence(
                                            track, &route) < 0) {
                                        fail_case(
                                                "route/switch/sensor sequence invalid",
                                                start, destination,
                                                side, 0);
                                        return 1;
                                }
                                if (check_speed_matrix(
                                            track, &route, start,
                                            destination, side,
                                            &predictions) < 0) {
                                        return 1;
                                }
                                ++directed_routes;
                        }
                }
        }

        if (directed_routes != EXPECTED_DIRECTED_ROUTES ||
            predictions != EXPECTED_PREDICTIONS) {
                fprintf(stderr,
                        "matrix count mismatch routes=%d predictions=%d\n",
                        directed_routes, predictions);
                return 1;
        }
        printf("validated %d A-F x d1-d8 operator routes "
               "(%d directed sensor sides), %d speed predictions, "
               "turnout sequences, sensor replay, and reservation guards\n",
               EXPECTED_OPERATOR_ROUTES, directed_routes, predictions);
        return 0;
}
