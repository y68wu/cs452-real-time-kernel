#include <limits.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "../tc2_offline_planner.h"
#include "../tc2_route_projection.h"
#include "../tc2_track_model.h"
#include "../track_data.h"

#ifdef MODE_TC2

#define MATRIX_TRAIN 14

static int fail_at(
        const char *reason, int start, int speed,
        int destination, int status) {
        fprintf(stderr,
                "tc2_offline_planner_test: %s "
                "(train=%d start=%c speed=%d destination=D%d status=%d)\n",
                reason, MATRIX_TRAIN, 'A' + start, speed,
                destination + 1, status);
        return 1;
}

static int plan_is_fail_closed(
        const tc2_offline_plan *plan, int expected_status) {
        return plan &&
               plan->status == expected_status &&
               !plan->valid &&
               plan->selected_side == -1 &&
               plan->target_node == -1 &&
               plan->geometry_anchor_route_offset == -1 &&
               plan->physical_destination_route_offset == -1 &&
               plan->selected_motion_route.node_count == 0 &&
               plan->projection_header.status ==
                       TC2_DISPATCH_PROJECTION_NOT_READY &&
               !plan->projection_header.valid &&
               plan->projection_header.publication_serial ==
                       UINT64_C(0) &&
               plan->projection_header.waypoint_count == 0;
}

static int exact_route_reversal_count(
        track_node track[TRACK_MAX],
        const track_route *route) {
        if (!track || !route || route->node_count < 1 ||
            route->node_count > TRACK_MAX) {
                return -1;
        }
        int count = 0;
        for (int offset = 0;
             offset + 1 < route->node_count; ++offset) {
                int from = route->nodes[offset];
                int to = route->nodes[offset + 1];
                if (from < 0 || from >= TRACK_MAX ||
                    to < 0 || to >= TRACK_MAX) {
                        return -1;
                }
                if (track[from].reverse == &track[to]) {
                        ++count;
                }
        }
        return count;
}

static int wire_waypoint_matches(
        const tc2_dispatch_projection_waypoint *wire,
        const tc2_route_waypoint *source) {
        return wire && source &&
               wire->distance_um == source->distance_um &&
               wire->route_offset == source->route_offset &&
               wire->graph_node == source->graph_node &&
               wire->sensor_index == source->sensor_index &&
               wire->switch_number == source->switch_number &&
               wire->kind == (int)source->kind &&
               wire->turnout_direction ==
                       source->turnout_direction &&
               wire->destination_index ==
                       source->destination_index &&
               wire->ui_row == source->ui_row &&
               wire->ui_column == source->ui_column &&
               wire->ui_width == source->ui_width &&
               wire->reserved == 0;
}

static int check_projection_contract(
        track_node track[TRACK_MAX],
        const tc2_offline_plan *plan) {
        tc2_route_projection_selection selection = {
                .selected_motion_route =
                        &plan->selected_motion_route,
                .geometry_anchor_route_offset =
                        plan->geometry_anchor_route_offset,
                .physical_destination_route_offset =
                        plan->physical_destination_route_offset,
                .physical_destination_offset_mm =
                        plan->physical_destination_offset_mm,
        };
        tc2_route_projection projection;
        if (Tc2RouteProjectionBuildWithPrediction(
                    track, &selection,
                    &plan->prediction_request,
                    &plan->prediction_plan,
                    &projection) != TC2_ROUTE_PROJECTION_OK ||
            Tc2RouteProjectionValidateWithPrediction(
                    track, &selection,
                    &plan->prediction_request,
                    &plan->prediction_plan,
                    &projection) != TC2_ROUTE_PROJECTION_OK ||
            !projection.valid ||
            projection.waypoint_count !=
                    plan->projection_header.waypoint_count ||
            projection.sensor_count !=
                    plan->projection_header.sensor_count ||
            projection.turnout_count !=
                    plan->projection_header.turnout_count ||
            projection.reversal_count !=
                    plan->projection_header.reversal_count ||
            projection.physical_destination_distance_um !=
                    plan->projection_header
                            .physical_destination_distance_um ||
            projection.operator_destination_waypoint_index !=
                    plan->projection_header
                            .operator_destination_waypoint_index) {
                return -1;
        }

        for (int index = 0;
             index < projection.waypoint_count; ++index) {
                if (!wire_waypoint_matches(
                            &plan->waypoints[index],
                            &projection.waypoints[index])) {
                        return -1;
                }
        }
        return 0;
}

static int check_calibrated_a_to_d1(
        const tc2_offline_plan *plan) {
        static const int expected_switch[] = {8, 7, 18, 3, 2, 1};
        static const char expected_direction[] =
                {'S', 'C', 'S', 'C', 'S', 'S'};
        int turnout = 0;

        if (!plan || !plan->valid) return -1;
        for (int index = 0;
             index < plan->projection_header.waypoint_count; ++index) {
                const tc2_dispatch_projection_waypoint *point =
                        &plan->waypoints[index];
                if (point->kind != TC2_ROUTE_WAYPOINT_TURNOUT) continue;
                if (turnout >=
                            (int)(sizeof(expected_switch) /
                                  sizeof(expected_switch[0])) ||
                    point->switch_number != expected_switch[turnout] ||
                    Tc2TrackPhysicalTurnoutDirection(
                            point->turnout_direction) !=
                            expected_direction[turnout]) {
                        return -1;
                }
                ++turnout;
        }
        return turnout ==
                (int)(sizeof(expected_switch) /
                      sizeof(expected_switch[0])) ? 0 : -1;
}

static int check_forward_shortest_endpoint(
        track_node track[TRACK_MAX], int start_index,
        int destination_index, const tc2_offline_plan *plan) {
        const tc2_track_start_definition *start =
                Tc2TrackStartDefinition(start_index);
        int start_node = start ?
                TrackFindNodeByName(track, start->enter_node) : -1;
        int shortest = -1;

        if (!plan || !plan->valid || start_node < 0 ||
            plan->selected_motion_route.reversal_count != 0) {
                return -1;
        }
        for (int side = 0;
             side < TC2_TRACK_DESTINATION_SIDE_COUNT; ++side) {
                const tc2_track_destination_side *definition =
                        Tc2TrackDestinationSide(
                                destination_index, side);
                int anchor = definition ?
                        TrackFindNodeByName(
                                track,
                                definition->anchor_sensor) :
                        -1;
                track_route route;
                if (!definition || anchor < 0 ||
                    TrackFindShortestRoute(
                            track, start_node, anchor,
                            &route) < 0 ||
                    route.reversal_count != 0 ||
                    route.distance_mm >
                            INT_MAX -
                                    definition->base_offset_mm) {
                        return -1;
                }
                int distance =
                        route.distance_mm +
                        definition->base_offset_mm;
                if (shortest < 0 || distance < shortest) {
                        shortest = distance;
                }
        }
        return shortest >= 0 &&
               plan->path_distance_mm == shortest ? 0 : -1;
}

static int check_success_plan(
        track_node track[TRACK_MAX],
        int start, int speed, int destination,
        const tc2_offline_plan *plan) {
        const tc2_dispatch_projection_header *header =
                &plan->projection_header;
        int route_reversals = exact_route_reversal_count(
                track, &plan->selected_motion_route);
        if (!plan->valid ||
            plan->status != TC2_OFFLINE_PLAN_OK ||
            route_reversals < 0 ||
            route_reversals !=
                    plan->selected_motion_route.reversal_count ||
            route_reversals !=
                    plan->prediction_request.reversal_count ||
            route_reversals != header->reversal_count ||
            plan->selected_side < 0 ||
            plan->selected_side >=
                    TC2_TRACK_DESTINATION_SIDE_COUNT ||
            plan->target_node < 0 ||
            plan->target_node >= TRACK_MAX ||
            plan->selected_motion_route.node_count < 1 ||
            plan->selected_motion_route.node_count > TRACK_MAX ||
            plan->geometry_anchor_route_offset < 0 ||
            plan->geometry_anchor_route_offset >=
                    plan->selected_motion_route.node_count ||
            plan->physical_destination_route_offset <
                    plan->geometry_anchor_route_offset ||
            plan->physical_destination_route_offset >=
                    plan->selected_motion_route.node_count ||
            plan->physical_destination_offset_mm < 0 ||
            plan->path_distance_mm < 0 ||
            plan->optimization_cost_mm <
                    plan->path_distance_mm ||
            plan->prediction_request.train != MATRIX_TRAIN ||
            plan->prediction_request.start_index != start ||
            plan->prediction_request.speed != speed ||
            plan->prediction_request.destination_index !=
                    destination ||
            plan->prediction_request.destination_side !=
                    plan->selected_side ||
            header->status != TC2_DISPATCH_PROJECTION_OK ||
            !header->valid ||
            header->train != MATRIX_TRAIN ||
            header->job_state != TC2_JOB_READY ||
            header->start_index != start ||
            header->speed != speed ||
            header->destination_index != destination ||
            header->destination_side != plan->selected_side ||
            header->publication_serial == UINT64_C(0) ||
            header->plan_generation == 0 ||
            header->launch_epoch == 0 ||
            !header->scheduler_healthy ||
            header->waypoint_count < 2 ||
            header->waypoint_count >
                    TC2_OFFLINE_PLANNER_MAX_WAYPOINTS ||
            header->operator_destination_waypoint_index !=
                    header->waypoint_count - 1 ||
            header->physical_destination_distance_um !=
                    plan->prediction_plan
                            .physical_destination_distance_um ||
            plan->waypoints[header->waypoint_count - 1].kind !=
                    TC2_ROUTE_WAYPOINT_DESTINATION ||
            plan->waypoints[header->waypoint_count - 1]
                            .destination_index != destination ||
            plan->waypoints[header->waypoint_count - 1]
                            .distance_um !=
                    header->physical_destination_distance_um) {
                return -1;
        }

        int64_t previous_distance = -1;
        for (int index = 0;
             index < header->waypoint_count; ++index) {
                const tc2_dispatch_projection_waypoint *point =
                        &plan->waypoints[index];
                if (point->distance_um < previous_distance ||
                    point->reserved != 0 ||
                    point->route_offset < 0 ||
                    point->route_offset >=
                            plan->selected_motion_route.node_count ||
                    point->graph_node < 0 ||
                    point->graph_node >= TRACK_MAX) {
                        return -1;
                }
                previous_distance = point->distance_um;
        }
        return check_projection_contract(track, plan);
}

static int check_invalid_inputs(track_node track[TRACK_MAX]) {
        tc2_offline_plan plan;
        memset(&plan, 0xa5, sizeof(plan));
        int status = Tc2OfflinePlannerBuild(
                track, 0, 0, 20, 0, &plan);
        if (status != TC2_OFFLINE_PLAN_INVALID_ARGUMENT ||
            !plan_is_fail_closed(&plan, status)) {
                return 1;
        }

        static const int invalid_case[][4] = {
                {14, -1, 20, 0},
                {14, TC2_TRACK_START_COUNT, 20, 0},
                {14, 0, 0, 0},
                {14, 0, 121, 0},
                {14, 0, 20, -1},
                {14, 0, 20, TC2_TRACK_DESTINATION_COUNT}
        };
        for (unsigned int index = 0;
             index < sizeof(invalid_case) /
                     sizeof(invalid_case[0]); ++index) {
                memset(&plan, 0xa5, sizeof(plan));
                status = Tc2OfflinePlannerBuild(
                        track,
                        invalid_case[index][0],
                        invalid_case[index][1],
                        invalid_case[index][2],
                        invalid_case[index][3],
                        &plan);
                if (status !=
                            TC2_OFFLINE_PLAN_INVALID_ARGUMENT ||
                    !plan_is_fail_closed(&plan, status)) {
                        return 1;
                }
        }
        memset(&plan, 0xa5, sizeof(plan));
        status = Tc2OfflinePlannerBuild(
                0, 14, 0, 20, 0, &plan);
        if (status != TC2_OFFLINE_PLAN_INVALID_ARGUMENT ||
            !plan_is_fail_closed(&plan, status) ||
            Tc2OfflinePlannerBuild(
                    track, 14, 0, 20, 0, 0) !=
                    TC2_OFFLINE_PLAN_INVALID_ARGUMENT) {
                return 1;
        }
        return 0;
}

static int check_unreachable_and_bad_catalog(
        track_node track[TRACK_MAX]) {
        const tc2_track_start_definition *start =
                Tc2TrackStartDefinition(0);
        int start_node = start ?
                TrackFindNodeByName(track, start->enter_node) : -1;
        if (start_node < 0 || start_node >= TRACK_MAX ||
            track[start_node].type != NODE_ENTER) {
                return 1;
        }

        track_edge saved_edge =
                track[start_node].edge[DIR_AHEAD];
        track[start_node].edge[DIR_AHEAD].dest = 0;
        track[start_node].edge[DIR_AHEAD].dist = 0;
        tc2_offline_plan plan;
        memset(&plan, 0xa5, sizeof(plan));
        int status = Tc2OfflinePlannerBuild(
                track, MATRIX_TRAIN, 0, 20, 0, &plan);
        track[start_node].edge[DIR_AHEAD] = saved_edge;
        if (status != TC2_OFFLINE_PLAN_UNREACHABLE ||
            !plan_is_fail_closed(&plan, status)) {
                return 1;
        }

        node_type saved_type = track[start_node].type;
        track[start_node].type = NODE_NONE;
        memset(&plan, 0xa5, sizeof(plan));
        status = Tc2OfflinePlannerBuild(
                track, MATRIX_TRAIN, 0, 20, 0, &plan);
        track[start_node].type = saved_type;
        if (status != TC2_OFFLINE_PLAN_CATALOG_INVALID ||
            !plan_is_fail_closed(&plan, status)) {
                return 1;
        }
        return 0;
}

static int node_edge_count(const track_node *node) {
        if (!node) return 0;
        if (node->type == NODE_BRANCH) return 2;
        if (node->type == NODE_NONE ||
            node->type == NODE_EXIT) {
                return 0;
        }
        return 1;
}

/*
 * The unmodified Track-D matrix has a forward directed route for every
 * operator pair, so its deterministic lowest-cost choice does not need a
 * reversal.  Remove one directed edge at a time to exercise the planner's
 * sensor-reversal fallback on an otherwise valid catalog.
 */
static int check_sensor_reversal_fallback(
        track_node track[TRACK_MAX],
        int *observed_reversal_count) {
        if (!observed_reversal_count) return 1;
        *observed_reversal_count = 0;
        static const int speeds[] = {20, 80, 120};
        for (int node = 0; node < TRACK_MAX; ++node) {
                int edge_count = node_edge_count(&track[node]);
                for (int direction = 0;
                     direction < edge_count; ++direction) {
                        if (!track[node].edge[direction].dest) {
                                continue;
                        }
                        track_edge saved =
                                track[node].edge[direction];
                        track[node].edge[direction].dest = 0;
                        track[node].edge[direction].dist = 0;

                        for (int start = 0;
                             start < TC2_TRACK_START_COUNT;
                             ++start) {
                                for (int destination = 0;
                                     destination <
                                             TC2_TRACK_DESTINATION_COUNT;
                                     ++destination) {
                                        for (unsigned int speed_index = 0;
                                             speed_index <
                                                     sizeof(speeds) /
                                                     sizeof(speeds[0]);
                                             ++speed_index) {
                                                tc2_offline_plan first;
                                                int speed =
                                                        speeds[speed_index];
                                                int status =
                                                        Tc2OfflinePlannerBuild(
                                                                track,
                                                                MATRIX_TRAIN,
                                                                start,
                                                                speed,
                                                                destination,
                                                                &first);
                                                if (status !=
                                                            TC2_OFFLINE_PLAN_OK ||
                                                    first.selected_motion_route
                                                                    .reversal_count <
                                                            1) {
                                                        continue;
                                                }
                                                tc2_offline_plan second;
                                                int valid =
                                                        check_success_plan(
                                                                track, start,
                                                                speed,
                                                                destination,
                                                                &first) == 0 &&
                                                        Tc2OfflinePlannerBuild(
                                                                track,
                                                                MATRIX_TRAIN,
                                                                start,
                                                                speed,
                                                                destination,
                                                                &second) ==
                                                                TC2_OFFLINE_PLAN_OK &&
                                                        memcmp(
                                                                &first,
                                                                &second,
                                                                sizeof(first)) ==
                                                                0 &&
                                                        Tc2OfflinePlannerValidate(
                                                                track,
                                                                MATRIX_TRAIN,
                                                                start,
                                                                speed,
                                                                destination,
                                                                &first) ==
                                                                TC2_OFFLINE_PLAN_OK;
                                                *observed_reversal_count =
                                                        first.selected_motion_route
                                                                .reversal_count;
                                                track[node].edge[direction] =
                                                        saved;
                                                return valid &&
                                                               *observed_reversal_count >
                                                                       0 ?
                                                        0 : 1;
                                        }
                                }
                        }
                        track[node].edge[direction] = saved;
                }
        }
        return 1;
}

int main(void) {
        track_node track[TRACK_MAX];
        track_node track_before[TRACK_MAX];
        init_trackb(track);
        memcpy(track_before, track, sizeof(track));

        int synthetic_reversals = 0;
        if (Tc2TrackCatalogValidate(track) != 0 ||
            check_invalid_inputs(track) != 0 ||
            check_unreachable_and_bad_catalog(track) != 0 ||
            check_sensor_reversal_fallback(
                    track, &synthetic_reversals) != 0 ||
            synthetic_reversals < 1) {
                fprintf(stderr,
                        "tc2_offline_planner_test: "
                        "catalog, invalid-input, unreachable, "
                        "reversal, or fail-closed check failed\n");
                return 1;
        }

        int planned = 0;
        int unreachable = 0;
        int reversal_plans = 0;
        for (int start = 0;
             start < TC2_TRACK_START_COUNT; ++start) {
                for (int destination = 0;
                     destination <
                             TC2_TRACK_DESTINATION_COUNT;
                     ++destination) {
                        for (int speed = 1;
                             speed <= 120; ++speed) {
                                tc2_offline_plan first;
                                tc2_offline_plan second;
                                int status = Tc2OfflinePlannerBuild(
                                        track, MATRIX_TRAIN,
                                        start, speed,
                                        destination, &first);
                                if (status ==
                                    TC2_OFFLINE_PLAN_UNREACHABLE) {
                                        if (first.valid ||
                                            first.projection_header
                                                    .valid) {
                                                return fail_at(
                                                        "unreachable result "
                                                        "was not fail-closed",
                                                        start, speed,
                                                        destination,
                                                        status);
                                        }
                                        ++unreachable;
                                        continue;
                                }
                                if (status !=
                                    TC2_OFFLINE_PLAN_OK) {
                                        return fail_at(
                                                "reachable matrix member "
                                                "was rejected",
                                                start, speed,
                                                destination,
                                                status);
                                }
                                if (check_success_plan(
                                            track, start, speed,
                                            destination,
                                            &first) < 0) {
                                        return fail_at(
                                                "projection contract "
                                                "failed",
                                                start, speed,
                                                destination,
                                                status);
                                }
                                if (check_forward_shortest_endpoint(
                                            track, start,
                                            destination,
                                            &first) < 0) {
                                        return fail_at(
                                                "selected route is not the "
                                                "shortest forward endpoint",
                                                start, speed,
                                                destination,
                                                status);
                                }
                                if (start == 0 &&
                                    destination == 0 &&
                                    speed == 80 &&
                                    check_calibrated_a_to_d1(
                                            &first) < 0) {
                                        return fail_at(
                                                "calibrated A->d1 physical "
                                                "turnout sequence drifted",
                                                start, speed,
                                                destination,
                                                status);
                                }
                                if (Tc2OfflinePlannerBuild(
                                            track, MATRIX_TRAIN,
                                            start, speed,
                                            destination,
                                            &second) !=
                                            TC2_OFFLINE_PLAN_OK ||
                                    memcmp(
                                            &first, &second,
                                            sizeof(first)) != 0 ||
                                    Tc2OfflinePlannerValidate(
                                            track, MATRIX_TRAIN,
                                            start, speed,
                                            destination,
                                            &first) !=
                                            TC2_OFFLINE_PLAN_OK) {
                                        return fail_at(
                                                "deterministic rebuild "
                                                "failed",
                                                start, speed,
                                                destination,
                                                status);
                                }
                                if (first.selected_motion_route
                                            .reversal_count > 0) {
                                        ++reversal_plans;
                                }
                                ++planned;
                        }
                }
        }

        if (planned + unreachable !=
                    TC2_TRACK_START_COUNT *
                    TC2_TRACK_DESTINATION_COUNT * 120 ||
            memcmp(track_before, track, sizeof(track)) != 0) {
                fprintf(stderr,
                        "tc2_offline_planner_test: "
                        "matrix cardinality or purity failed "
                        "(planned=%d unreachable=%d reversals=%d)\n",
                        planned, unreachable, reversal_plans);
                return 1;
        }

        printf("tc2_offline_planner_test: PASS "
               "(planned=%d unreachable=%d "
               "matrix_reversals=%d synthetic_reversals=%d total=%d)\n",
               planned, unreachable, reversal_plans,
               synthetic_reversals,
               planned + unreachable);
        return 0;
}

#else

int main(void) {
        return 0;
}

#endif
