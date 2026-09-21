#include <limits.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "../tc2_motion_model.h"
#include "../tc2_route_projection.h"

enum {
        EXPECTED_PROJECTION_COUNT =
                TC2_TRACK_START_COUNT *
                TC2_TRACK_DESTINATION_COUNT *
                TC2_TRACK_DESTINATION_SIDE_COUNT * 120
};

typedef struct {
        track_route geometry_route;
        track_route selected_motion_route;
        tc2_prediction_request request;
        tc2_prediction_plan prediction;
        tc2_route_projection_selection selection;
        int represented_extension_mm;
} projection_fixture;

typedef struct {
        int projections;
        int exact_endpoint;
        int inside_edge_endpoint;
        int retained_safety_suffix;
        int semantic_suffix_isolation;
} projection_matrix_stats;

static int fail_case(
        const char *message, int start, int destination,
        int side, int speed) {
        fprintf(stderr,
                "%s start=%d destination=%d side=%d speed=%d\n",
                message, start, destination, side, speed);
        return 1;
}

static int valid_node(int node) {
        return node >= 0 && node < TRACK_MAX;
}

static int text_equal(const char *left, const char *right) {
        if (!left || !right) return 0;
        while (*left && *right && *left == *right) {
                ++left;
                ++right;
        }
        return *left == '\0' && *right == '\0';
}

static unsigned char decimal_width(int number) {
        unsigned char width = 1;
        while (number >= 10) {
                number /= 10;
                ++width;
        }
        return width;
}

static int route_scalar_at(
        track_node track[TRACK_MAX], const track_route *route,
        int offset, int64_t *scalar_um) {
        int distance_mm;
        if (!track || !route || !scalar_um || offset < 0 ||
            offset >= route->node_count ||
            TrackRouteDistanceBetweenOffsets(
                    track, route, 0, offset, &distance_mm) < 0 ||
            distance_mm < 0) {
                return -1;
        }
        *scalar_um = (int64_t)distance_mm * 1000;
        return 0;
}

static int route_distance_between(
        track_node track[TRACK_MAX], const track_route *route,
        int from, int to) {
        int distance_mm;
        if (TrackRouteDistanceBetweenOffsets(
                    track, route, from, to, &distance_mm) < 0) {
                return -1;
        }
        return distance_mm;
}

static int exact_reverse_transition(
        track_node track[TRACK_MAX], int previous, int current) {
        return valid_node(previous) && valid_node(current) &&
               track[previous].type == NODE_SENSOR &&
               track[current].type == NODE_SENSOR &&
               track[previous].reverse == &track[current] &&
               track[current].reverse == &track[previous];
}

static const tc2_track_d_switch_layout *switch_layout_for(
        int switch_number) {
        for (int index = 0;
             index < TC2_TRACK_D_SWITCH_COUNT; ++index) {
                const tc2_track_d_switch_layout *layout =
                        Tc2TrackDSwitchLayout((size_t)index);
                if (layout && layout->number == switch_number) {
                        return layout;
                }
        }
        return 0;
}

static const track_turnout_action *turnout_action_at(
        const track_turnout_plan *plan, int route_offset) {
        if (!plan) return 0;
        for (int index = 0; index < plan->action_count; ++index) {
                if (plan->actions[index].route_node_offset ==
                    route_offset) {
                        return &plan->actions[index];
                }
        }
        return 0;
}

static int request_equal(
        const tc2_prediction_request *left,
        const tc2_prediction_request *right) {
        return left && right &&
               left->train == right->train &&
               left->start_index == right->start_index &&
               left->destination_index == right->destination_index &&
               left->destination_side == right->destination_side &&
               left->speed == right->speed &&
               left->reversal_count == right->reversal_count;
}

static int prediction_equal(
        const tc2_prediction_plan *left,
        const tc2_prediction_plan *right) {
        return left && right &&
               left->geometry_anchor_node ==
                       right->geometry_anchor_node &&
               left->geometry_anchor_route_offset ==
                       right->geometry_anchor_route_offset &&
               left->geometry_base_offset_mm ==
                       right->geometry_base_offset_mm &&
               left->geometry_evidence ==
                       right->geometry_evidence &&
               left->profile_evidence ==
                       right->profile_evidence &&
               left->exact_profile == right->exact_profile &&
               left->velocity_um_per_tick ==
                       right->velocity_um_per_tick &&
               left->point_stop_distance_um ==
                       right->point_stop_distance_um &&
               left->profile_correction_mm ==
                       right->profile_correction_mm &&
               left->physical_destination_distance_um ==
                       right->physical_destination_distance_um &&
               left->corrected_endpoint_distance_um ==
                       right->corrected_endpoint_distance_um &&
               left->endpoint_distance_um ==
                       right->endpoint_distance_um &&
               left->endpoint_anchor_node ==
                       right->endpoint_anchor_node &&
               left->endpoint_anchor_route_offset ==
                       right->endpoint_anchor_route_offset &&
               left->endpoint_offset_um ==
                       right->endpoint_offset_um &&
               left->raw_command_distance_um ==
                       right->raw_command_distance_um &&
               left->command_distance_um ==
                       right->command_distance_um &&
               left->command_anchor_node ==
                       right->command_anchor_node &&
               left->command_anchor_route_offset ==
                       right->command_anchor_route_offset &&
               left->command_offset_um ==
                       right->command_offset_um &&
               left->action == right->action &&
               left->conservative_braking_distance_mm ==
                       right->conservative_braking_distance_mm &&
               left->minimum_reservation_ceiling_distance_um ==
                       right->minimum_reservation_ceiling_distance_um;
}

/*
 * Deliberately copy only values, not object representation.  Callers may
 * have arbitrary bytes in ABI padding; BuildWithPrediction must normalize
 * those bytes through its canonical prediction before storing the result.
 */
static void copy_prediction_fields(
        tc2_prediction_plan *destination,
        const tc2_prediction_plan *source) {
        destination->geometry_anchor_node =
                source->geometry_anchor_node;
        destination->geometry_anchor_route_offset =
                source->geometry_anchor_route_offset;
        destination->geometry_base_offset_mm =
                source->geometry_base_offset_mm;
        destination->geometry_evidence =
                source->geometry_evidence;
        destination->profile_evidence =
                source->profile_evidence;
        destination->exact_profile = source->exact_profile;
        destination->velocity_um_per_tick =
                source->velocity_um_per_tick;
        destination->point_stop_distance_um =
                source->point_stop_distance_um;
        destination->profile_correction_mm =
                source->profile_correction_mm;
        destination->physical_destination_distance_um =
                source->physical_destination_distance_um;
        destination->corrected_endpoint_distance_um =
                source->corrected_endpoint_distance_um;
        destination->endpoint_distance_um =
                source->endpoint_distance_um;
        destination->endpoint_anchor_node =
                source->endpoint_anchor_node;
        destination->endpoint_anchor_route_offset =
                source->endpoint_anchor_route_offset;
        destination->endpoint_offset_um =
                source->endpoint_offset_um;
        destination->raw_command_distance_um =
                source->raw_command_distance_um;
        destination->command_distance_um =
                source->command_distance_um;
        destination->command_anchor_node =
                source->command_anchor_node;
        destination->command_anchor_route_offset =
                source->command_anchor_route_offset;
        destination->command_offset_um =
                source->command_offset_um;
        destination->action = source->action;
        destination->conservative_braking_distance_mm =
                source->conservative_braking_distance_mm;
        destination->minimum_reservation_ceiling_distance_um =
                source->minimum_reservation_ceiling_distance_um;
}

static int route_active_equal(
        const track_route *left, const track_route *right) {
        if (!left || !right ||
            left->node_count != right->node_count ||
            left->distance_mm != right->distance_mm ||
            left->optimization_cost_mm !=
                    right->optimization_cost_mm ||
            left->reversal_count != right->reversal_count) {
                return 0;
        }
        for (int offset = 0; offset < left->node_count; ++offset) {
                if (left->nodes[offset] != right->nodes[offset]) {
                        return 0;
                }
        }
        return 1;
}

static int route_unused_nodes_are_zero(const track_route *route) {
        if (!route || route->node_count < 0 ||
            route->node_count > TRACK_MAX) {
                return 0;
        }
        for (int offset = route->node_count;
             offset < TRACK_MAX; ++offset) {
                if (route->nodes[offset] != 0) return 0;
        }
        return 1;
}

static int waypoint_equal(
        const tc2_route_waypoint *left,
        const tc2_route_waypoint *right) {
        return left && right &&
               left->kind == right->kind &&
               left->route_offset == right->route_offset &&
               left->graph_node == right->graph_node &&
               left->sensor_index == right->sensor_index &&
               left->switch_number == right->switch_number &&
               left->turnout_direction == right->turnout_direction &&
               left->destination_index == right->destination_index &&
               left->distance_um == right->distance_um &&
               left->ui_row == right->ui_row &&
               left->ui_column == right->ui_column &&
               left->ui_width == right->ui_width;
}

/*
 * Speed may change the retained safety suffix and prediction timing, but it
 * must not change the physical operator endpoint or any semantic waypoint at
 * or before it.
 */
static int semantic_projection_equal(
        const tc2_route_projection *left,
        const tc2_route_projection *right) {
        if (!left || !right || !left->valid || !right->valid ||
            left->geometry_anchor_route_offset !=
                    right->geometry_anchor_route_offset ||
            left->geometry_anchor_sensor_index !=
                    right->geometry_anchor_sensor_index ||
            left->physical_destination_route_offset !=
                    right->physical_destination_route_offset ||
            left->physical_destination_offset_mm !=
                    right->physical_destination_offset_mm ||
            left->physical_destination_distance_um !=
                    right->physical_destination_distance_um ||
            left->last_visible_route_offset !=
                    right->last_visible_route_offset ||
            left->operator_destination_waypoint_index !=
                    right->operator_destination_waypoint_index ||
            left->waypoint_count != right->waypoint_count ||
            left->sensor_count != right->sensor_count ||
            left->turnout_count != right->turnout_count ||
            left->reversal_count != right->reversal_count) {
                return 0;
        }
        for (int index = 0; index < left->waypoint_count; ++index) {
                if (!waypoint_equal(
                            &left->waypoints[index],
                            &right->waypoints[index])) {
                        return 0;
                }
        }
        return 1;
}

static int projection_is_initialized_empty(
        const tc2_route_projection *projection) {
        tc2_route_projection expected;
        Tc2RouteProjectionInitialize(&expected);
        return projection &&
               memcmp(projection, &expected, sizeof(expected)) == 0;
}

static int first_endpoint_ceiling(
        track_node track[TRACK_MAX], const track_route *route,
        int anchor_offset, int endpoint_offset_mm) {
        if (!route || anchor_offset < 0 ||
            anchor_offset >= route->node_count ||
            endpoint_offset_mm < 0) {
                return -1;
        }
        if (endpoint_offset_mm == 0) return anchor_offset;
        for (int offset = anchor_offset + 1;
             offset < route->node_count; ++offset) {
                int distance_mm = route_distance_between(
                        track, route, anchor_offset, offset);
                if (distance_mm < 0) return -1;
                if (distance_mm >= endpoint_offset_mm) return offset;
        }
        return -1;
}

static int finish_fixture(
        track_node track[TRACK_MAX], int start_index,
        int destination_index, int side_index, int speed,
        const track_route *geometry_route,
        projection_fixture *fixture) {
        const tc2_track_destination_side *side =
                Tc2TrackDestinationSide(
                        destination_index, side_index);
        if (!track || !side || !geometry_route || !fixture ||
            geometry_route->node_count < 1) {
                return -1;
        }

        memset(fixture, 0, sizeof(*fixture));
        fixture->geometry_route = *geometry_route;
        fixture->request.train = 14;
        fixture->request.start_index = start_index;
        fixture->request.destination_index = destination_index;
        fixture->request.destination_side = side_index;
        fixture->request.speed = speed;
        fixture->request.reversal_count =
                geometry_route->reversal_count;
        if (Tc2PredictionBuildPlan(
                    track, &fixture->geometry_route,
                    &fixture->request,
                    &fixture->prediction) != TC2_PREDICTION_OK ||
            fixture->prediction.conservative_braking_distance_mm !=
                    Tc2MotionBrakingDistanceMm(speed) ||
            side->base_offset_mm > INT_MAX -
                    fixture->prediction
                            .conservative_braking_distance_mm) {
                return -1;
        }

        fixture->selected_motion_route =
                fixture->geometry_route;
        int anchor_offset =
                fixture->geometry_route.node_count - 1;
        int required_extension_mm =
                side->base_offset_mm +
                fixture->prediction.conservative_braking_distance_mm;
        int represented_mm = TrackExtendRouteAhead(
                track, &fixture->selected_motion_route,
                required_extension_mm);
        if (represented_mm < required_extension_mm) return -1;

        /*
         * Match dispatcher selection exactly: a retained safety suffix may
         * not terminate on a branch whose outgoing setting is unspecified.
         */
        int last = fixture->selected_motion_route.nodes[
                fixture->selected_motion_route.node_count - 1];
        if (track[last].type == NODE_BRANCH &&
            fixture->selected_motion_route.node_count < TRACK_MAX) {
                int before =
                        fixture->selected_motion_route.node_count;
                int extra = TrackExtendRouteAhead(
                        track, &fixture->selected_motion_route, 1);
                if (extra < 1 ||
                    fixture->selected_motion_route.node_count <= before) {
                        return -1;
                }
        }

        fixture->represented_extension_mm =
                fixture->selected_motion_route.distance_mm -
                fixture->geometry_route.distance_mm;
        int ceiling = first_endpoint_ceiling(
                track, &fixture->selected_motion_route,
                anchor_offset, side->base_offset_mm);
        if (ceiling < 0 ||
            fixture->represented_extension_mm <
                    required_extension_mm) {
                return -1;
        }

        /*
         * Builders must not leak inactive caller storage. Poison it after
         * route construction so the retained output must canonicalize it.
         */
        for (int offset =
                     fixture->selected_motion_route.node_count;
             offset < TRACK_MAX; ++offset) {
                fixture->selected_motion_route.nodes[offset] =
                        0x5a5a5a5a;
        }
        fixture->selection.selected_motion_route =
                &fixture->selected_motion_route;
        fixture->selection.geometry_anchor_route_offset =
                anchor_offset;
        fixture->selection.physical_destination_route_offset =
                ceiling;
        fixture->selection.physical_destination_offset_mm =
                side->base_offset_mm;
        return 0;
}

static int build_fixture(
        track_node track[TRACK_MAX], int start_index,
        int destination_index, int side_index, int speed,
        projection_fixture *fixture) {
        const tc2_track_start_definition *start =
                Tc2TrackStartDefinition(start_index);
        const tc2_track_destination_side *side =
                Tc2TrackDestinationSide(
                        destination_index, side_index);
        int from = start ?
                TrackFindNodeByName(track, start->enter_node) : -1;
        int anchor = side ?
                TrackFindNodeByName(track, side->anchor_sensor) : -1;
        track_route geometry_route;
        memset(&geometry_route, 0, sizeof(geometry_route));
        if (!start || !side || from < 0 || anchor < 0 ||
            TrackFindShortestRoute(
                    track, from, anchor, &geometry_route) < 0 ||
            geometry_route.node_count < 2) {
                return -1;
        }
        return finish_fixture(
                track, start_index, destination_index,
                side_index, speed, &geometry_route, fixture);
}

static int check_waypoint(
        const tc2_route_waypoint *actual,
        tc2_route_waypoint_kind kind,
        int route_offset, int graph_node, int sensor_index,
        int switch_number, int turnout_direction,
        int destination_index, int64_t distance_um,
        unsigned char row, unsigned char column,
        unsigned char width) {
        return actual &&
               actual->kind == kind &&
               actual->route_offset == route_offset &&
               actual->graph_node == graph_node &&
               actual->sensor_index == sensor_index &&
               actual->switch_number == switch_number &&
               actual->turnout_direction == turnout_direction &&
               actual->destination_index == destination_index &&
               actual->distance_um == distance_um &&
               actual->ui_row == row &&
               actual->ui_column == column &&
               actual->ui_width == width;
}

static int check_projection(
        track_node track[TRACK_MAX],
        const projection_fixture *fixture,
        const tc2_route_projection *projection,
        projection_matrix_stats *stats) {
        const tc2_prediction_request *request =
                &fixture->request;
        const track_route *route =
                &fixture->selected_motion_route;
        int start = request->start_index;
        int destination = request->destination_index;
        int side = request->destination_side;
        int speed = request->speed;
        int anchor = fixture->selection
                .geometry_anchor_route_offset;
        int ceiling = fixture->selection
                .physical_destination_route_offset;
        int64_t anchor_scalar;
        int64_t ceiling_scalar;
        if (!projection || !projection->valid ||
            !request_equal(&projection->request, request) ||
            !prediction_equal(
                    &projection->prediction,
                    &fixture->prediction) ||
            !route_active_equal(
                    &projection->selected_motion_route, route) ||
            !route_unused_nodes_are_zero(
                    &projection->selected_motion_route) ||
            route_scalar_at(
                    track, route, anchor, &anchor_scalar) < 0 ||
            route_scalar_at(
                    track, route, ceiling, &ceiling_scalar) < 0) {
                return fail_case(
                        "retained projection contract mismatch",
                        start, destination, side, speed);
        }

        for (int offset = 0;
             offset < fixture->geometry_route.node_count; ++offset) {
                if (route->nodes[offset] !=
                    fixture->geometry_route.nodes[offset]) {
                        return fail_case(
                                "geometry prefix not retained",
                                start, destination, side, speed);
                }
        }

        int64_t physical_um =
                anchor_scalar +
                (int64_t)fixture->selection
                        .physical_destination_offset_mm * 1000;
        int expected_last_visible =
                ceiling_scalar == physical_um ?
                ceiling : ceiling - 1;
        int64_t full_scalar =
                (int64_t)route->distance_mm * 1000;
        if (projection->geometry_anchor_route_offset != anchor ||
            projection->geometry_anchor_sensor_index !=
                    track[route->nodes[anchor]].num ||
            projection->physical_destination_route_offset !=
                    ceiling ||
            projection->physical_destination_offset_mm !=
                    fixture->selection
                            .physical_destination_offset_mm ||
            projection->physical_destination_distance_um !=
                    physical_um ||
            projection->prediction
                    .physical_destination_distance_um != physical_um ||
            projection->last_visible_route_offset !=
                    expected_last_visible ||
            full_scalar < projection->prediction
                    .minimum_reservation_ceiling_distance_um ||
            ceiling_scalar < physical_um ||
            (ceiling > anchor &&
             route_scalar_at(
                     track, route, ceiling - 1,
                     &anchor_scalar) < 0) ||
            (ceiling > anchor && anchor_scalar >= physical_um)) {
                return fail_case(
                        "endpoint/ceiling scalar mismatch",
                        start, destination, side, speed);
        }

        if (ceiling_scalar == physical_um) {
                if (expected_last_visible != ceiling) {
                        return fail_case(
                                "exact endpoint not visible",
                                start, destination, side, speed);
                }
                if (stats) ++stats->exact_endpoint;
        } else {
                if (expected_last_visible != ceiling - 1) {
                        return fail_case(
                                "inside-edge ceiling not clipped",
                                start, destination, side, speed);
                }
                if (stats) ++stats->inside_edge_endpoint;
        }
        if (full_scalar > physical_um) {
                if (stats) ++stats->retained_safety_suffix;
        } else {
                return fail_case(
                        "full motion route lost safety suffix",
                        start, destination, side, speed);
        }

        track_turnout_plan turnout_plan;
        if (TrackBuildTurnoutPlan(
                    track, route, &turnout_plan) < 0) {
                return fail_case(
                        "full turnout plan invalid",
                        start, destination, side, speed);
        }
        const tc2_track_d_start_layout *start_layout =
                Tc2TrackDStartLayout((size_t)start);
        if (!start_layout || projection->waypoint_count < 2 ||
            !check_waypoint(
                    &projection->waypoints[0],
                    TC2_ROUTE_WAYPOINT_START,
                    0, route->nodes[0], -1, -1, -1, -1, 0,
                    start_layout->row,
                    start_layout->column, 1)) {
                return fail_case(
                        "START waypoint mismatch",
                        start, destination, side, speed);
        }

        int waypoint_index = 1;
        int sensor_count = 0;
        int turnout_count = 0;
        int reversal_count = 0;
        for (int offset = 1;
             offset <= expected_last_visible; ++offset) {
                int graph_node = route->nodes[offset];
                track_node *node = &track[graph_node];
                int64_t scalar_um;
                if (route_scalar_at(
                            track, route, offset,
                            &scalar_um) < 0 ||
                    scalar_um > physical_um) {
                        return fail_case(
                                "visible graph event crossed P",
                                start, destination, side, speed);
                }
                if (node->type == NODE_SENSOR) {
                        tc2_track_d_directed_sensor_cell cell;
                        tc2_route_waypoint_kind kind =
                                exact_reverse_transition(
                                        track,
                                        route->nodes[offset - 1],
                                        graph_node) ?
                                TC2_ROUTE_WAYPOINT_REVERSAL :
                                TC2_ROUTE_WAYPOINT_SENSOR;
                        if (Tc2TrackDDirectedSensorCell(
                                    node->num, &cell) < 0 ||
                            !text_equal(node->name, cell.name) ||
                            waypoint_index >=
                                    projection->waypoint_count ||
                            !check_waypoint(
                                    &projection->waypoints[
                                            waypoint_index],
                                    kind, offset, graph_node,
                                    node->num, -1, -1, -1,
                                    scalar_um, cell.row, cell.column,
                                    cell.width)) {
                                return fail_case(
                                        "directed sensor waypoint mismatch",
                                        start, destination, side, speed);
                        }
                        ++waypoint_index;
                        if (kind ==
                            TC2_ROUTE_WAYPOINT_REVERSAL) {
                                ++reversal_count;
                        } else {
                                ++sensor_count;
                        }
                } else if (node->type == NODE_BRANCH) {
                        const track_turnout_action *action =
                                turnout_action_at(
                                        &turnout_plan, offset);
                        const tc2_track_d_switch_layout *layout =
                                action ? switch_layout_for(
                                        action->switch_number) : 0;
                        if (!action || !layout ||
                            waypoint_index >=
                                    projection->waypoint_count ||
                            !check_waypoint(
                                    &projection->waypoints[
                                            waypoint_index],
                                    TC2_ROUTE_WAYPOINT_TURNOUT,
                                    offset, graph_node, -1,
                                    action->switch_number,
                                    action->direction, -1,
                                    scalar_um, layout->row,
                                    layout->column,
                                    (unsigned char)(
                                            2 +
                                            decimal_width(
                                                    layout->number)))) {
                                return fail_case(
                                        "turnout waypoint mismatch",
                                        start, destination, side, speed);
                        }
                        ++waypoint_index;
                        ++turnout_count;
                }
        }

        const tc2_track_d_destination_layout *destination_layout =
                Tc2TrackDDestinationLayout(
                        (size_t)destination);
        int destination_width =
                destination_layout && destination_layout->label ?
                (int)strlen(destination_layout->label) : -1;
        if (!destination_layout ||
            destination_width < 1 ||
            waypoint_index != projection->waypoint_count - 1 ||
            projection->operator_destination_waypoint_index !=
                    waypoint_index ||
            !check_waypoint(
                    &projection->waypoints[waypoint_index],
                    TC2_ROUTE_WAYPOINT_DESTINATION,
                    expected_last_visible,
                    route->nodes[expected_last_visible],
                    -1, -1, -1, destination, physical_um,
                    destination_layout->row,
                    destination_layout->column,
                    (unsigned char)destination_width) ||
            projection->sensor_count != sensor_count ||
            projection->turnout_count != turnout_count ||
            projection->reversal_count != reversal_count) {
                return fail_case(
                        "DESTINATION/count contract mismatch",
                        start, destination, side, speed);
        }

        int64_t prior_um = -1;
        for (int index = 0;
             index < projection->waypoint_count; ++index) {
                const tc2_route_waypoint *waypoint =
                        &projection->waypoints[index];
                if (waypoint->distance_um < prior_um ||
                    waypoint->distance_um > physical_um ||
                    waypoint->route_offset >
                            expected_last_visible) {
                        return fail_case(
                                "semantic suffix leaked past P",
                                start, destination, side, speed);
                }
                prior_um = waypoint->distance_um;
        }
        if (stats) ++stats->semantic_suffix_isolation;

        int destination_waypoint =
                projection->operator_destination_waypoint_index;
        if (Tc2RouteProjectionWaypointAtOrBefore(
                    projection, -1) != -1 ||
            Tc2RouteProjectionWaypointAtOrBefore(
                    projection, physical_um) !=
                    destination_waypoint ||
            Tc2RouteProjectionWaypointAtOrBefore(
                    projection, physical_um + 1) !=
                    destination_waypoint ||
            (physical_um > 0 &&
             Tc2RouteProjectionWaypointAtOrBefore(
                     projection, physical_um - 1) >=
                     destination_waypoint)) {
                return fail_case(
                        "waypoint query/tie ordering mismatch",
                        start, destination, side, speed);
        }

        if (Tc2RouteProjectionValidate(
                    track, &fixture->selection,
                    request, projection) !=
                TC2_ROUTE_PROJECTION_OK ||
            Tc2RouteProjectionValidateWithPrediction(
                    track, &fixture->selection,
                    request, &fixture->prediction,
                    projection) !=
                TC2_ROUTE_PROJECTION_OK) {
                return fail_case(
                        "public validation rejected projection",
                        start, destination, side, speed);
        }
        if (stats) ++stats->projections;
        return 0;
}

static int exhaustive_projection_matrix(
        track_node track[TRACK_MAX],
        projection_matrix_stats *stats) {
        memset(stats, 0, sizeof(*stats));
        for (int start = 0;
             start < TC2_TRACK_START_COUNT; ++start) {
                for (int destination = 0;
                     destination <
                             TC2_TRACK_DESTINATION_COUNT;
                     ++destination) {
                        for (int side = 0;
                             side <
                                     TC2_TRACK_DESTINATION_SIDE_COUNT;
                             ++side) {
                                tc2_route_projection baseline;
                                int have_baseline = 0;
                                for (int speed = 1;
                                     speed <= 120; ++speed) {
                                        projection_fixture fixture;
                                        tc2_route_projection first;
                                        tc2_route_projection second;
                                        if (build_fixture(
                                                    track, start,
                                                    destination, side,
                                                    speed,
                                                    &fixture) < 0) {
                                                return fail_case(
                                                        "fixture build failed",
                                                        start, destination,
                                                        side, speed);
                                        }
                                        memset(&first, 0xa5, sizeof(first));
                                        if (Tc2RouteProjectionBuild(
                                                    track,
                                                    &fixture.selection,
                                                    &fixture.request,
                                                    &first) !=
                                                TC2_ROUTE_PROJECTION_OK ||
                                            check_projection(
                                                    track, &fixture,
                                                    &first, stats)) {
                                                return fail_case(
                                                        "projection build/check failed",
                                                        start, destination,
                                                        side, speed);
                                        }
                                        memset(&second, 0x3c, sizeof(second));
                                        if (Tc2RouteProjectionBuildWithPrediction(
                                                    track,
                                                    &fixture.selection,
                                                    &fixture.request,
                                                    &fixture.prediction,
                                                    &second) !=
                                                    TC2_ROUTE_PROJECTION_OK ||
                                            memcmp(
                                                    &first, &second,
                                                    sizeof(first)) != 0) {
                                                return fail_case(
                                                        "derived/selected prediction differed",
                                                        start, destination,
                                                        side, speed);
                                        }
                                        if (!have_baseline) {
                                                baseline = first;
                                                have_baseline = 1;
                                        } else if (!semantic_projection_equal(
                                                           &baseline,
                                                           &first)) {
                                                return fail_case(
                                                        "speed changed semantic route through P",
                                                        start, destination,
                                                        side, speed);
                                        }
                                }
                        }
                }
        }
        if (stats->projections !=
                    EXPECTED_PROJECTION_COUNT ||
            stats->exact_endpoint != 2880 ||
            stats->inside_edge_endpoint !=
                    EXPECTED_PROJECTION_COUNT - 2880 ||
            stats->retained_safety_suffix !=
                    EXPECTED_PROJECTION_COUNT ||
            stats->semantic_suffix_isolation !=
                    EXPECTED_PROJECTION_COUNT) {
                fprintf(stderr,
                        "matrix coverage mismatch total=%d exact=%d "
                        "inside=%d suffix=%d isolation=%d expected=%d\n",
                        stats->projections,
                        stats->exact_endpoint,
                        stats->inside_edge_endpoint,
                        stats->retained_safety_suffix,
                        stats->semantic_suffix_isolation,
                        EXPECTED_PROJECTION_COUNT);
                return 1;
        }
        return 0;
}

static int wrong_ceiling_regression(
        track_node track[TRACK_MAX]) {
        projection_fixture fixture;
        if (build_fixture(track, 0, 6, 0, 120, &fixture) < 0) {
                fprintf(stderr, "wrong-ceiling fixture failed\n");
                return 1;
        }

        tc2_route_projection_selection wrong =
                fixture.selection;
        tc2_route_projection output;
        int tested_early = 0;
        int tested_late = 0;
        if (wrong.physical_destination_route_offset >
            wrong.geometry_anchor_route_offset) {
                --wrong.physical_destination_route_offset;
                memset(&output, 0xa5, sizeof(output));
                if (Tc2RouteProjectionBuild(
                            track, &wrong, &fixture.request,
                            &output) !=
                                TC2_ROUTE_PROJECTION_ROUTE_MISMATCH ||
                    !projection_is_initialized_empty(&output)) {
                        fprintf(stderr,
                                "early endpoint ceiling accepted\n");
                        return 1;
                }
                tested_early = 1;
        }

        wrong = fixture.selection;
        if (wrong.physical_destination_route_offset + 1 <
            fixture.selected_motion_route.node_count) {
                ++wrong.physical_destination_route_offset;
                memset(&output, 0x3c, sizeof(output));
                if (Tc2RouteProjectionBuild(
                            track, &wrong, &fixture.request,
                            &output) !=
                                TC2_ROUTE_PROJECTION_ROUTE_MISMATCH ||
                    !projection_is_initialized_empty(&output)) {
                        fprintf(stderr,
                                "non-minimal endpoint ceiling accepted\n");
                        return 1;
                }
                tested_late = 1;
        }
        if (!tested_early || !tested_late) {
                fprintf(stderr,
                        "wrong-ceiling fixture lacks both boundaries\n");
                return 1;
        }
        return 0;
}

static int full_route_retention_and_suffix_regression(
        track_node track[TRACK_MAX]) {
        projection_fixture fixture;
        tc2_route_projection projection;
        if (build_fixture(track, 5, 0, 0, 120, &fixture) < 0 ||
            Tc2RouteProjectionBuild(
                    track, &fixture.selection,
                    &fixture.request, &projection) !=
                    TC2_ROUTE_PROJECTION_OK) {
                fprintf(stderr,
                        "full-route retention fixture failed\n");
                return 1;
        }
        int last_visible = projection.last_visible_route_offset;
        if (fixture.selected_motion_route.node_count <=
                    last_visible + 1 ||
            !route_active_equal(
                    &projection.selected_motion_route,
                    &fixture.selected_motion_route)) {
                fprintf(stderr,
                        "full selected route was clipped at P\n");
                return 1;
        }
        for (int index = 0;
             index < projection.waypoint_count; ++index) {
                if (projection.waypoints[index].route_offset >
                    last_visible) {
                        fprintf(stderr,
                                "safety suffix became UI travel\n");
                        return 1;
                }
        }
        return 0;
}

/*
 * Full-route reversal accounting is independent of the geometry-prefix
 * request.  Add a zero-distance reverse transition strictly inside the
 * retained safety suffix and prove that it is audited but never rendered as
 * semantic travel.
 */
static int full_suffix_reversal_regression(
        track_node track[TRACK_MAX]) {
        projection_fixture fixture;
        tc2_route_projection baseline;
        tc2_route_projection reversed;
        int found = 0;
        for (int start = 0;
             start < TC2_TRACK_START_COUNT && !found; ++start) {
                for (int destination = 0;
                     destination < TC2_TRACK_DESTINATION_COUNT &&
                             !found;
                     ++destination) {
                        for (int side = 0;
                             side <
                                     TC2_TRACK_DESTINATION_SIDE_COUNT &&
                                     !found;
                             ++side) {
                                if (build_fixture(
                                            track, start, destination,
                                            side, 120,
                                            &fixture) < 0) {
                                        continue;
                                }
                                int count = fixture
                                        .selected_motion_route
                                        .node_count;
                                int last = count > 0 ?
                                        fixture
                                                .selected_motion_route
                                                .nodes[count - 1] :
                                        -1;
                                if (count < TRACK_MAX &&
                                    valid_node(last) &&
                                    track[last].type == NODE_SENSOR &&
                                    track[last].reverse) {
                                        found = 1;
                                }
                        }
                }
        }
        if (!found ||
            Tc2RouteProjectionBuild(
                    track, &fixture.selection, &fixture.request,
                    &baseline) != TC2_ROUTE_PROJECTION_OK) {
                fprintf(stderr,
                        "full suffix reversal fixture unavailable\n");
                return 1;
        }

        int old_count =
                fixture.selected_motion_route.node_count;
        int last =
                fixture.selected_motion_route.nodes[old_count - 1];
        int reverse = (int)(track[last].reverse - track);
        if (!valid_node(reverse)) {
                fprintf(stderr,
                        "full suffix reversal node invalid\n");
                return 1;
        }
        fixture.selected_motion_route.nodes[old_count] = reverse;
        fixture.selected_motion_route.node_count = old_count + 1;
        ++fixture.selected_motion_route.reversal_count;
        if (Tc2RouteProjectionBuild(
                    track, &fixture.selection, &fixture.request,
                    &reversed) != TC2_ROUTE_PROJECTION_OK ||
            !route_active_equal(
                    &reversed.selected_motion_route,
                    &fixture.selected_motion_route) ||
            reversed.selected_motion_route.reversal_count !=
                    fixture.request.reversal_count + 1 ||
            reversed.reversal_count != baseline.reversal_count ||
            !semantic_projection_equal(&baseline, &reversed) ||
            reversed.waypoints[
                    reversed.waypoint_count - 1]
                    .route_offset >= old_count ||
            Tc2RouteProjectionValidate(
                    track, &fixture.selection, &fixture.request,
                    &reversed) != TC2_ROUTE_PROJECTION_OK) {
                fprintf(stderr,
                        "full suffix reversal leaked or was rejected\n");
                return 1;
        }
        return 0;
}

static int selected_prediction_regression(
        track_node track[TRACK_MAX]) {
        projection_fixture fixture;
        tc2_route_projection projection;
        if (build_fixture(track, 0, 6, 0, 100, &fixture) < 0 ||
            fixture.prediction.profile_evidence !=
                    TC2_PREDICTION_PROFILE_EXACT_T14_A_D7 ||
            !fixture.prediction.exact_profile) {
                fprintf(stderr,
                        "exact selected-prediction fixture failed\n");
                return 1;
        }

        tc2_prediction_plan transferred =
                fixture.prediction;
        transferred.profile_evidence =
                TC2_PREDICTION_PROFILE_TRANSFERRED_T14_A_D7;
        transferred.exact_profile = 0;
        if (Tc2RouteProjectionBuildWithPrediction(
                    track, &fixture.selection, &fixture.request,
                    &transferred, &projection) !=
                    TC2_ROUTE_PROJECTION_OK ||
            !prediction_equal(
                    &projection.prediction, &transferred)) {
                fprintf(stderr,
                        "permitted EXACT-to-TRANSFERRED downgrade failed\n");
                return 1;
        }

        tc2_prediction_plan tampered = fixture.prediction;
        ++tampered.physical_destination_distance_um;
        memset(&projection, 0xa5, sizeof(projection));
        if (Tc2RouteProjectionBuildWithPrediction(
                    track, &fixture.selection, &fixture.request,
                    &tampered, &projection) !=
                        TC2_ROUTE_PROJECTION_PREDICTION ||
            !projection_is_initialized_empty(&projection)) {
                fprintf(stderr,
                        "tampered selected prediction accepted\n");
                return 1;
        }
        return 0;
}

static int selected_prediction_padding_regression(
        track_node track[TRACK_MAX]) {
        projection_fixture fixture;
        tc2_prediction_plan first_prediction;
        tc2_prediction_plan second_prediction;
        tc2_route_projection first;
        tc2_route_projection second;
        if (build_fixture(track, 0, 6, 0, 100, &fixture) < 0) {
                fprintf(stderr,
                        "prediction padding fixture failed\n");
                return 1;
        }
        memset(&first_prediction, 0xa5, sizeof(first_prediction));
        copy_prediction_fields(
                &first_prediction, &fixture.prediction);
        memset(&second_prediction, 0x3c, sizeof(second_prediction));
        copy_prediction_fields(
                &second_prediction, &fixture.prediction);
        memset(&first, 0x5a, sizeof(first));
        memset(&second, 0xc3, sizeof(second));
        if (Tc2RouteProjectionBuildWithPrediction(
                    track, &fixture.selection, &fixture.request,
                    &first_prediction, &first) !=
                        TC2_ROUTE_PROJECTION_OK ||
            Tc2RouteProjectionBuildWithPrediction(
                    track, &fixture.selection, &fixture.request,
                    &second_prediction, &second) !=
                        TC2_ROUTE_PROJECTION_OK ||
            memcmp(&first, &second, sizeof(first)) != 0 ||
            !prediction_equal(
                    &first.prediction, &fixture.prediction)) {
                fprintf(stderr,
                        "selected prediction padding was not canonicalized\n");
                return 1;
        }
        return 0;
}

static int alias_no_write_regression(
        track_node track[TRACK_MAX]) {
        projection_fixture fixture;
        if (build_fixture(track, 0, 6, 0, 100, &fixture) < 0) {
                fprintf(stderr, "alias fixture failed\n");
                return 1;
        }

        union {
                tc2_route_projection output;
                tc2_route_projection_selection selection;
        } selection_alias;
        unsigned char selection_before[
                sizeof(selection_alias)];
        memset(&selection_alias, 0xa5, sizeof(selection_alias));
        selection_alias.selection = fixture.selection;
        memcpy(selection_before, &selection_alias,
               sizeof(selection_before));
        if (Tc2RouteProjectionBuild(
                    track, &selection_alias.selection,
                    &fixture.request,
                    &selection_alias.output) !=
                        TC2_ROUTE_PROJECTION_INVALID ||
            memcmp(selection_before, &selection_alias,
                   sizeof(selection_before)) != 0) {
                fprintf(stderr,
                        "selection alias wrote output\n");
                return 1;
        }

        union {
                tc2_route_projection output;
                tc2_prediction_request request;
        } request_alias;
        unsigned char request_before[sizeof(request_alias)];
        memset(&request_alias, 0x3c, sizeof(request_alias));
        request_alias.request = fixture.request;
        memcpy(request_before, &request_alias,
               sizeof(request_before));
        if (Tc2RouteProjectionBuild(
                    track, &fixture.selection,
                    &request_alias.request,
                    &request_alias.output) !=
                        TC2_ROUTE_PROJECTION_INVALID ||
            memcmp(request_before, &request_alias,
                   sizeof(request_before)) != 0) {
                fprintf(stderr,
                        "request alias wrote output\n");
                return 1;
        }

        union {
                tc2_route_projection output;
                track_route route;
        } route_alias;
        tc2_route_projection_selection route_selection =
                fixture.selection;
        unsigned char route_before[sizeof(route_alias)];
        memset(&route_alias, 0x5a, sizeof(route_alias));
        route_alias.route = fixture.selected_motion_route;
        route_selection.selected_motion_route =
                &route_alias.route;
        memcpy(route_before, &route_alias, sizeof(route_before));
        if (Tc2RouteProjectionBuild(
                    track, &route_selection, &fixture.request,
                    &route_alias.output) !=
                        TC2_ROUTE_PROJECTION_INVALID ||
            memcmp(route_before, &route_alias,
                   sizeof(route_before)) != 0) {
                fprintf(stderr,
                        "selected route alias wrote output\n");
                return 1;
        }

        union {
                tc2_route_projection output;
                tc2_prediction_plan prediction;
        } prediction_alias;
        unsigned char prediction_before[
                sizeof(prediction_alias)];
        memset(&prediction_alias, 0xc3,
               sizeof(prediction_alias));
        prediction_alias.prediction = fixture.prediction;
        memcpy(prediction_before, &prediction_alias,
               sizeof(prediction_before));
        if (Tc2RouteProjectionBuildWithPrediction(
                    track, &fixture.selection, &fixture.request,
                    &prediction_alias.prediction,
                    &prediction_alias.output) !=
                        TC2_ROUTE_PROJECTION_INVALID ||
            memcmp(prediction_before, &prediction_alias,
                   sizeof(prediction_before)) != 0) {
                fprintf(stderr,
                        "prediction alias wrote output\n");
                return 1;
        }

        union {
                tc2_route_projection output;
                track_node track[TRACK_MAX];
        } track_alias;
        unsigned char track_before[sizeof(track_alias)];
        memset(&track_alias, 0x96, sizeof(track_alias));
        init_trackb(track_alias.track);
        memcpy(track_before, &track_alias, sizeof(track_before));
        if (Tc2RouteProjectionBuild(
                    track_alias.track, &fixture.selection,
                    &fixture.request, &track_alias.output) !=
                        TC2_ROUTE_PROJECTION_INVALID ||
            memcmp(track_before, &track_alias,
                   sizeof(track_before)) != 0) {
                fprintf(stderr,
                        "track alias wrote output\n");
                return 1;
        }

        tc2_route_projection *overflow_output =
                (tc2_route_projection *)(uintptr_t)(
                        UINTPTR_MAX -
                        sizeof(tc2_route_projection) / 2);
        if (Tc2RouteProjectionBuild(
                    track, &fixture.selection, &fixture.request,
                    overflow_output) !=
                        TC2_ROUTE_PROJECTION_INVALID) {
                fprintf(stderr,
                        "overflowing output range was accepted\n");
                return 1;
        }
        return 0;
}

static int invalid_input_regression(
        track_node track[TRACK_MAX]) {
        projection_fixture fixture;
        tc2_route_projection projection;
        if (build_fixture(track, 0, 6, 0, 80, &fixture) < 0 ||
            Tc2RouteProjectionBuild(
                    track, &fixture.selection,
                    &fixture.request, &projection) !=
                    TC2_ROUTE_PROJECTION_OK) {
                fprintf(stderr, "invalid-input fixture failed\n");
                return 1;
        }

        tc2_route_projection tampered = projection;
        ++tampered.waypoints[
                tampered.operator_destination_waypoint_index]
                .distance_um;
        if (Tc2RouteProjectionValidate(
                    track, &fixture.selection, &fixture.request,
                    &tampered) !=
                TC2_ROUTE_PROJECTION_ROUTE_MISMATCH) {
                fprintf(stderr,
                        "tampered projection validated\n");
                return 1;
        }

        tc2_prediction_request invalid_request =
                fixture.request;
        invalid_request.speed = 121;
        memset(&projection, 0xa5, sizeof(projection));
        if (Tc2RouteProjectionBuild(
                    track, &fixture.selection, &invalid_request,
                    &projection) != TC2_ROUTE_PROJECTION_INVALID ||
            !projection_is_initialized_empty(&projection)) {
                fprintf(stderr,
                        "invalid request not fail-closed\n");
                return 1;
        }

        tc2_route_projection_selection wrong_offset =
                fixture.selection;
        ++wrong_offset.physical_destination_offset_mm;
        memset(&projection, 0x3c, sizeof(projection));
        if (Tc2RouteProjectionBuild(
                    track, &wrong_offset, &fixture.request,
                    &projection) !=
                        TC2_ROUTE_PROJECTION_ROUTE_MISMATCH ||
            !projection_is_initialized_empty(&projection)) {
                fprintf(stderr,
                        "wrong physical offset accepted\n");
                return 1;
        }

        track_route invalid_route =
                fixture.selected_motion_route;
        invalid_route.optimization_cost_mm =
                invalid_route.distance_mm - 1;
        tc2_route_projection_selection invalid_selection =
                fixture.selection;
        invalid_selection.selected_motion_route =
                &invalid_route;
        memset(&projection, 0x96, sizeof(projection));
        if (Tc2RouteProjectionBuild(
                    track, &invalid_selection, &fixture.request,
                    &projection) != TC2_ROUTE_PROJECTION_INVALID ||
            !projection_is_initialized_empty(&projection)) {
                fprintf(stderr,
                        "invalid optimization scalar not fail-closed\n");
                return 1;
        }

        memset(&projection, 0xa5, sizeof(projection));
        if (Tc2RouteProjectionBuild(
                    track, 0, &fixture.request,
                    &projection) != TC2_ROUTE_PROJECTION_INVALID ||
            !projection_is_initialized_empty(&projection) ||
            Tc2RouteProjectionWaypointAtOrBefore(0, 0) != -1 ||
            !text_equal(
                    Tc2RouteWaypointKindName(
                            TC2_ROUTE_WAYPOINT_DESTINATION),
                    "DESTINATION") ||
            !text_equal(
                    Tc2RouteWaypointKindName(
                            TC2_ROUTE_WAYPOINT_INVALID),
                    "INVALID")) {
                fprintf(stderr,
                        "invalid/null utility contract failed\n");
                return 1;
        }
        return 0;
}

int main(void) {
        track_node track[TRACK_MAX];
        init_trackb(track);
        if (Tc2TrackCatalogValidate(track) < 0 ||
            Tc2TrackDLayoutValidate(0, 0) < 0) {
                fprintf(stderr,
                        "Track D graph/layout validation failed\n");
                return 1;
        }

        projection_matrix_stats stats;
        if (exhaustive_projection_matrix(track, &stats) ||
            wrong_ceiling_regression(track) ||
            full_route_retention_and_suffix_regression(track) ||
            full_suffix_reversal_regression(track) ||
            selected_prediction_regression(track) ||
            selected_prediction_padding_regression(track) ||
            alias_no_write_regression(track) ||
            invalid_input_regression(track)) {
                return 1;
        }

        printf("tc2_route_projection_test: PASS "
               "(%d A-F x d1-d8 x sides x speeds, sensor exact=%d, "
               "inside-edge=%d, "
               "full-route suffix isolated)\n",
               stats.projections, stats.exact_endpoint,
               stats.inside_edge_endpoint);
        return 0;
}
