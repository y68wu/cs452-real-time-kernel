#include "tc2_route_projection.h"

#ifndef MODE_TC2

/*
 * Keep the source in the shared wildcard build without exporting TC2-only
 * route/UI symbols in TC1, PERF, K4, CLOCK, or RPS images.
 */
typedef int tc2_route_projection_disabled_translation_unit;

#else

static void zero_bytes(void *value, unsigned int size) {
        unsigned char *bytes = (unsigned char *)value;
        for (unsigned int index = 0; index < size; ++index) {
                bytes[index] = 0;
        }
}

static int bytes_equal(
        const void *left_value, const void *right_value,
        unsigned int size) {
        const unsigned char *left =
                (const unsigned char *)left_value;
        const unsigned char *right =
                (const unsigned char *)right_value;
        for (unsigned int index = 0; index < size; ++index) {
                if (left[index] != right[index]) return 0;
        }
        return 1;
}

static int text_equal(const char *left, const char *right) {
        if (!left || !right) return 0;
        while (*left && *right && *left == *right) {
                ++left;
                ++right;
        }
        return *left == '\0' && *right == '\0';
}

static unsigned char text_width(const char *text) {
        unsigned char width = 0;
        if (!text) return 0;
        while (*text && width < 255) {
                ++text;
                ++width;
        }
        return width;
}

static int valid_node_index(int node) {
        return node >= 0 && node < TRACK_MAX;
}

static unsigned char decimal_width(int number) {
        unsigned char width = 1;
        while (number >= 10) {
                number /= 10;
                ++width;
        }
        return width;
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

void Tc2RouteProjectionInitialize(
        tc2_route_projection *projection) {
        if (!projection) return;
        zero_bytes(projection, (unsigned int)sizeof(*projection));
        projection->geometry_anchor_route_offset = -1;
        projection->geometry_anchor_sensor_index = -1;
        projection->physical_destination_route_offset = -1;
        projection->last_visible_route_offset = -1;
        projection->operator_destination_waypoint_index = -1;
}

static void initialize_waypoint(tc2_route_waypoint *waypoint) {
        zero_bytes(waypoint, (unsigned int)sizeof(*waypoint));
        waypoint->route_offset = -1;
        waypoint->graph_node = -1;
        waypoint->sensor_index = -1;
        waypoint->switch_number = -1;
        waypoint->turnout_direction = -1;
        waypoint->destination_index = -1;
        waypoint->distance_um = -1;
}

static int append_waypoint(
        tc2_route_projection *projection,
        tc2_route_waypoint_kind kind,
        int route_offset, int graph_node, int64_t distance_um,
        tc2_route_waypoint **waypoint) {
        if (!projection || !waypoint ||
            projection->waypoint_count < 0 ||
            projection->waypoint_count >=
                    TC2_ROUTE_PROJECTION_MAX_WAYPOINTS) {
                return TC2_ROUTE_PROJECTION_CAPACITY;
        }
        *waypoint =
                &projection->waypoints[projection->waypoint_count++];
        initialize_waypoint(*waypoint);
        (*waypoint)->kind = kind;
        (*waypoint)->route_offset = route_offset;
        (*waypoint)->graph_node = graph_node;
        (*waypoint)->distance_um = distance_um;
        return TC2_ROUTE_PROJECTION_OK;
}

static int ranges_overlap(
        const void *left_value, unsigned int left_size,
        const void *right_value, unsigned int right_size) {
        if (!left_value || !right_value ||
            left_size == 0 || right_size == 0) {
                return 0;
        }
        uintptr_t left = (uintptr_t)left_value;
        uintptr_t right = (uintptr_t)right_value;
        if (left > UINTPTR_MAX - (uintptr_t)(left_size - 1) ||
            right > UINTPTR_MAX - (uintptr_t)(right_size - 1)) {
                return 1;
        }
        uintptr_t left_last = left + (uintptr_t)(left_size - 1);
        uintptr_t right_last = right + (uintptr_t)(right_size - 1);
        return left <= right_last && right <= left_last;
}

static int output_aliases_input(
        track_node track[TRACK_MAX],
        const tc2_route_projection_selection *selection,
        const tc2_prediction_request *request,
        const tc2_prediction_plan *selected_prediction,
        tc2_route_projection *output) {
        if (!output) return 0;
        if (track && ranges_overlap(
                    output, (unsigned int)sizeof(*output),
                    track,
                    (unsigned int)(sizeof(track_node) * TRACK_MAX))) {
                return 1;
        }
        if (selection && ranges_overlap(
                    output, (unsigned int)sizeof(*output),
                    selection, (unsigned int)sizeof(*selection))) {
                return 1;
        }
        if (selection && selection->selected_motion_route &&
            ranges_overlap(
                    output, (unsigned int)sizeof(*output),
                    selection->selected_motion_route,
                    (unsigned int)sizeof(
                            *selection->selected_motion_route))) {
                return 1;
        }
        if (request && ranges_overlap(
                    output, (unsigned int)sizeof(*output),
                    request, (unsigned int)sizeof(*request))) {
                return 1;
        }
        if (selected_prediction && ranges_overlap(
                    output, (unsigned int)sizeof(*output),
                    selected_prediction,
                    (unsigned int)sizeof(*selected_prediction))) {
                return 1;
        }
        return 0;
}

static int prediction_equal_except_permitted_downgrade(
        const tc2_prediction_plan *canonical,
        const tc2_prediction_plan *selected) {
        if (!canonical || !selected) return 0;

        int permitted_profile =
                selected->profile_evidence ==
                        canonical->profile_evidence &&
                selected->exact_profile ==
                        canonical->exact_profile;
        if (canonical->profile_evidence ==
                    TC2_PREDICTION_PROFILE_EXACT_T14_A_D7 &&
            canonical->exact_profile == 1 &&
            selected->profile_evidence ==
                    TC2_PREDICTION_PROFILE_TRANSFERRED_T14_A_D7 &&
            selected->exact_profile == 0) {
                permitted_profile = 1;
        }

        return permitted_profile &&
               selected->geometry_anchor_node ==
                       canonical->geometry_anchor_node &&
               selected->geometry_anchor_route_offset ==
                       canonical->geometry_anchor_route_offset &&
               selected->geometry_base_offset_mm ==
                       canonical->geometry_base_offset_mm &&
               selected->geometry_evidence ==
                       canonical->geometry_evidence &&
               selected->velocity_um_per_tick ==
                       canonical->velocity_um_per_tick &&
               selected->point_stop_distance_um ==
                       canonical->point_stop_distance_um &&
               selected->profile_correction_mm ==
                       canonical->profile_correction_mm &&
               selected->physical_destination_distance_um ==
                       canonical->physical_destination_distance_um &&
               selected->corrected_endpoint_distance_um ==
                       canonical->corrected_endpoint_distance_um &&
               selected->endpoint_distance_um ==
                       canonical->endpoint_distance_um &&
               selected->endpoint_anchor_node ==
                       canonical->endpoint_anchor_node &&
               selected->endpoint_anchor_route_offset ==
                       canonical->endpoint_anchor_route_offset &&
               selected->endpoint_offset_um ==
                       canonical->endpoint_offset_um &&
               selected->raw_command_distance_um ==
                       canonical->raw_command_distance_um &&
               selected->command_distance_um ==
                       canonical->command_distance_um &&
               selected->command_anchor_node ==
                       canonical->command_anchor_node &&
               selected->command_anchor_route_offset ==
                       canonical->command_anchor_route_offset &&
               selected->command_offset_um ==
                       canonical->command_offset_um &&
               selected->action == canonical->action &&
               selected->conservative_braking_distance_mm ==
                       canonical->conservative_braking_distance_mm &&
               selected->minimum_reservation_ceiling_distance_um ==
                       canonical->minimum_reservation_ceiling_distance_um;
}

static int prediction_status_to_projection(int status) {
        if (status == TC2_PREDICTION_INVALID) {
                return TC2_ROUTE_PROJECTION_INVALID;
        }
        if (status == TC2_PREDICTION_ROUTE_MISMATCH) {
                return TC2_ROUTE_PROJECTION_ROUTE_MISMATCH;
        }
        return TC2_ROUTE_PROJECTION_PREDICTION;
}

static int catalog_matches_layout(
        track_node track[TRACK_MAX],
        const tc2_prediction_request *request,
        const tc2_prediction_plan *prediction) {
        if (!track || !request || !prediction ||
            Tc2TrackCatalogValidate(track) < 0 ||
            Tc2TrackDLayoutValidate(0, 0) < 0) {
                return 0;
        }

        const tc2_track_start_definition *start =
                Tc2TrackStartDefinition(request->start_index);
        const tc2_track_d_start_layout *start_layout =
                Tc2TrackDStartLayout(
                        (size_t)request->start_index);
        const tc2_track_destination_definition *destination =
                Tc2TrackDestinationDefinition(
                        request->destination_index);
        const tc2_track_destination_side *side =
                Tc2TrackDestinationSide(
                        request->destination_index,
                        request->destination_side);
        const tc2_track_d_destination_layout *destination_layout =
                Tc2TrackDDestinationLayout(
                        (size_t)request->destination_index);
        if (!start || !start_layout || !destination || !side ||
            !destination_layout || !start->label ||
            start->label[0] != start_layout->label ||
            start->label[1] != '\0' ||
            !text_equal(
                    start->enter_node, start_layout->entry_node) ||
            !text_equal(
                    destination->label,
                    destination_layout->label)) {
                return 0;
        }

        const char *layout_anchor =
                request->destination_side == 0 ?
                destination_layout->forward_anchor :
                destination_layout->reverse_anchor;
        int layout_offset =
                request->destination_side == 0 ?
                destination_layout->forward_offset_mm :
                destination_layout->reverse_offset_mm;
        if (!text_equal(side->anchor_sensor, layout_anchor) ||
            side->base_offset_mm != layout_offset ||
            prediction->geometry_base_offset_mm != layout_offset) {
                return 0;
        }

        int start_node =
                TrackFindNodeByName(track, start->enter_node);
        int anchor_node =
                TrackFindNodeByName(track, side->anchor_sensor);
        return valid_node_index(start_node) &&
               valid_node_index(anchor_node) &&
               anchor_node == prediction->geometry_anchor_node;
}

static void copy_request(
        tc2_prediction_request *destination,
        const tc2_prediction_request *source) {
        destination->train = source->train;
        destination->start_index = source->start_index;
        destination->destination_index =
                source->destination_index;
        destination->destination_side =
                source->destination_side;
        destination->speed = source->speed;
        destination->reversal_count =
                source->reversal_count;
}

static void copy_route(
        track_route *destination, const track_route *source) {
        zero_bytes(destination, (unsigned int)sizeof(*destination));
        destination->node_count = source->node_count;
        destination->distance_mm = source->distance_mm;
        destination->optimization_cost_mm =
                source->optimization_cost_mm;
        destination->reversal_count = source->reversal_count;
        for (int offset = 0; offset < source->node_count; ++offset) {
                destination->nodes[offset] = source->nodes[offset];
        }
}

static int build_route_prefix(
        track_node track[TRACK_MAX], const track_route *source,
        int last, track_route *prefix) {
        if (!track || !source || !prefix || last < 0 ||
            last >= source->node_count) {
                return -1;
        }
        zero_bytes(prefix, (unsigned int)sizeof(*prefix));
        prefix->node_count = last + 1;
        for (int offset = 0; offset <= last; ++offset) {
                if (!valid_node_index(source->nodes[offset])) return -1;
                prefix->nodes[offset] = source->nodes[offset];
        }
        if (TrackRouteDistanceBetweenOffsets(
                    track, source, 0, last,
                    &prefix->distance_mm) < 0) {
                return -1;
        }
        prefix->optimization_cost_mm = prefix->distance_mm;
        for (int offset = 1; offset <= last; ++offset) {
                int previous = source->nodes[offset - 1];
                int current = source->nodes[offset];
                if (track[previous].type == NODE_SENSOR &&
                    track[current].type == NODE_SENSOR &&
                    track[previous].reverse == &track[current] &&
                    track[current].reverse == &track[previous]) {
                        ++prefix->reversal_count;
                }
        }
        return 0;
}

static void copy_prediction(
        tc2_prediction_plan *destination,
        const tc2_prediction_plan *source) {
        unsigned char *destination_bytes =
                (unsigned char *)destination;
        const unsigned char *source_bytes =
                (const unsigned char *)source;
        for (unsigned int index = 0;
             index < (unsigned int)sizeof(*destination); ++index) {
                destination_bytes[index] = source_bytes[index];
        }
}

static int exact_reverse_transition(
        track_node track[TRACK_MAX], int previous, int current) {
        return valid_node_index(previous) &&
               valid_node_index(current) &&
               track[previous].type == NODE_SENSOR &&
               track[current].type == NODE_SENSOR &&
               track[previous].reverse == &track[current] &&
               track[current].reverse == &track[previous];
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

static int route_scalars_and_reversals(
        track_node track[TRACK_MAX], const track_route *route,
        int64_t cumulative_um[TRACK_MAX], int *reversals) {
        if (!track || !route || !cumulative_um || !reversals ||
            route->node_count <= 0 ||
            route->node_count > TRACK_MAX) {
                return -1;
        }
        *reversals = 0;
        cumulative_um[0] = 0;
        for (int offset = 0; offset < route->node_count; ++offset) {
                if (!valid_node_index(route->nodes[offset])) return -1;
                if (offset == 0) continue;
                int edge_distance_mm;
                if (TrackRouteDistanceBetweenOffsets(
                            track, route, offset - 1, offset,
                            &edge_distance_mm) < 0 ||
                    edge_distance_mm < 0 ||
                    (int64_t)edge_distance_mm >
                            (INT64_MAX -
                             cumulative_um[offset - 1]) / 1000) {
                        return -1;
                }
                cumulative_um[offset] =
                        cumulative_um[offset - 1] +
                        (int64_t)edge_distance_mm * 1000;
                if (exact_reverse_transition(
                            track, route->nodes[offset - 1],
                            route->nodes[offset])) {
                        ++*reversals;
                }
        }
        return cumulative_um[route->node_count - 1] ==
                       (int64_t)route->distance_mm * 1000 ?
                0 : -1;
}

static int build_projection(
        track_node track[TRACK_MAX],
        const tc2_route_projection_selection *selection,
        const tc2_prediction_request *request,
        const tc2_prediction_plan *selected_prediction,
        int derive_prediction,
        tc2_route_projection *output) {
        if (!output) return TC2_ROUTE_PROJECTION_INVALID;
        /*
         * Aliasing is the sole no-write failure: clearing output would
         * corrupt the very input needed to describe the failure.
         */
        if (output_aliases_input(
                    track, selection, request,
                    selected_prediction, output)) {
                return TC2_ROUTE_PROJECTION_INVALID;
        }
        Tc2RouteProjectionInitialize(output);

        int status = TC2_ROUTE_PROJECTION_INVALID;
        if (!track || !selection || !request ||
            !selection->selected_motion_route ||
            (!derive_prediction && !selected_prediction)) {
                goto fail;
        }
        const track_route *route =
                selection->selected_motion_route;
        int anchor_offset =
                selection->geometry_anchor_route_offset;
        int ceiling_offset =
                selection->physical_destination_route_offset;
        int endpoint_offset_mm =
                selection->physical_destination_offset_mm;
        if (route->node_count <= 0 ||
            route->node_count > TRACK_MAX ||
            route->distance_mm < 0 ||
            route->optimization_cost_mm < route->distance_mm ||
            anchor_offset < 0 ||
            anchor_offset >= route->node_count ||
            ceiling_offset < anchor_offset ||
            ceiling_offset >= route->node_count ||
            endpoint_offset_mm < 0) {
                goto fail;
        }

        track_route geometry_route;
        if (build_route_prefix(
                    track, route, anchor_offset,
                    &geometry_route) < 0) {
                status = TC2_ROUTE_PROJECTION_ROUTE_MISMATCH;
                goto fail;
        }

        tc2_prediction_plan canonical_prediction;
        int prediction_status = Tc2PredictionBuildPlan(
                track, &geometry_route, request,
                &canonical_prediction);
        if (prediction_status != TC2_PREDICTION_OK) {
                status = prediction_status_to_projection(
                        prediction_status);
                goto fail;
        }
        if (!derive_prediction &&
            !prediction_equal_except_permitted_downgrade(
                    &canonical_prediction, selected_prediction)) {
                status = TC2_ROUTE_PROJECTION_PREDICTION;
                goto fail;
        }
        if (!derive_prediction) {
                /*
                 * Only the documented evidence downgrade may differ.  Keep
                 * the canonical zeroed representation so caller padding can
                 * never make projection bytes nondeterministic.
                 */
                canonical_prediction.profile_evidence =
                        selected_prediction->profile_evidence;
                canonical_prediction.exact_profile =
                        selected_prediction->exact_profile;
        }
        const tc2_prediction_plan *prediction =
                &canonical_prediction;
        if (!catalog_matches_layout(track, request, prediction)) {
                status = TC2_ROUTE_PROJECTION_LAYOUT_MISMATCH;
                goto fail;
        }

        track_turnout_plan turnout_plan;
        if (TrackBuildTurnoutPlan(
                    track, route, &turnout_plan) < 0) {
                status = TC2_ROUTE_PROJECTION_ROUTE_MISMATCH;
                goto fail;
        }

        int64_t cumulative_um[TRACK_MAX];
        int route_reversals;
        if (route_scalars_and_reversals(
                    track, route, cumulative_um,
                    &route_reversals) < 0 ||
            route_reversals != route->reversal_count ||
            geometry_route.reversal_count !=
                    request->reversal_count ||
            prediction->geometry_anchor_route_offset !=
                    anchor_offset ||
            prediction->geometry_anchor_node !=
                    route->nodes[anchor_offset] ||
            track[route->nodes[anchor_offset]].type != NODE_SENSOR) {
                status = TC2_ROUTE_PROJECTION_ROUTE_MISMATCH;
                goto fail;
        }

        const tc2_track_start_definition *start =
                Tc2TrackStartDefinition(request->start_index);
        int expected_start = start ?
                TrackFindNodeByName(track, start->enter_node) : -1;
        int64_t physical_destination_um =
                cumulative_um[anchor_offset] +
                (int64_t)endpoint_offset_mm * 1000;
        if (!valid_node_index(expected_start) ||
            route->nodes[0] != expected_start ||
            prediction->physical_destination_distance_um !=
                    physical_destination_um ||
            prediction->geometry_base_offset_mm !=
                    endpoint_offset_mm ||
            physical_destination_um >
                    cumulative_um[route->node_count - 1] ||
            (int64_t)route->distance_mm * 1000 <
                    prediction->
                            minimum_reservation_ceiling_distance_um) {
                status = TC2_ROUTE_PROJECTION_ROUTE_MISMATCH;
                goto fail;
        }

        int computed_ceiling = -1;
        if (endpoint_offset_mm == 0) {
                computed_ceiling = anchor_offset;
        } else {
                for (int offset = anchor_offset + 1;
                     offset < route->node_count; ++offset) {
                        if (cumulative_um[offset] >=
                            physical_destination_um) {
                                computed_ceiling = offset;
                                break;
                        }
                }
        }
        if (computed_ceiling != ceiling_offset ||
            cumulative_um[ceiling_offset] <
                    physical_destination_um ||
            (ceiling_offset > anchor_offset &&
             cumulative_um[ceiling_offset - 1] >=
                    physical_destination_um)) {
                status = TC2_ROUTE_PROJECTION_ROUTE_MISMATCH;
                goto fail;
        }

        int last_visible =
                cumulative_um[ceiling_offset] ==
                        physical_destination_um ?
                ceiling_offset : ceiling_offset - 1;
        if (last_visible < anchor_offset ||
            last_visible >= route->node_count) {
                status = TC2_ROUTE_PROJECTION_ROUTE_MISMATCH;
                goto fail;
        }

        copy_request(&output->request, request);
        copy_route(&output->selected_motion_route, route);
        copy_prediction(&output->prediction, prediction);
        output->geometry_anchor_route_offset = anchor_offset;
        output->geometry_anchor_sensor_index =
                track[route->nodes[anchor_offset]].num;
        output->physical_destination_route_offset =
                ceiling_offset;
        output->physical_destination_offset_mm =
                endpoint_offset_mm;
        output->physical_destination_distance_um =
                physical_destination_um;
        output->last_visible_route_offset = last_visible;

        const tc2_track_d_start_layout *start_layout =
                Tc2TrackDStartLayout(
                        (size_t)request->start_index);
        tc2_route_waypoint *waypoint;
        status = append_waypoint(
                output, TC2_ROUTE_WAYPOINT_START, 0,
                route->nodes[0], 0, &waypoint);
        if (status != TC2_ROUTE_PROJECTION_OK) goto fail;
        if (!start_layout) {
                status = TC2_ROUTE_PROJECTION_LAYOUT_MISMATCH;
                goto fail;
        }
        waypoint->ui_row = start_layout->row;
        waypoint->ui_column = start_layout->column;
        waypoint->ui_width = 1;

        int visible_reversals = 0;
        for (int offset = 1; offset <= last_visible; ++offset) {
                int node_index = route->nodes[offset];
                track_node *node = &track[node_index];
                if (node->type == NODE_SENSOR) {
                        tc2_track_d_directed_sensor_cell cell;
                        if (Tc2TrackDDirectedSensorCell(
                                    node->num, &cell) < 0 ||
                            !text_equal(node->name, cell.name)) {
                                status =
                                        TC2_ROUTE_PROJECTION_LAYOUT_MISMATCH;
                                goto fail;
                        }
                        tc2_route_waypoint_kind kind =
                                exact_reverse_transition(
                                        track,
                                        route->nodes[offset - 1],
                                        node_index) ?
                                TC2_ROUTE_WAYPOINT_REVERSAL :
                                TC2_ROUTE_WAYPOINT_SENSOR;
                        status = append_waypoint(
                                output, kind, offset, node_index,
                                cumulative_um[offset], &waypoint);
                        if (status != TC2_ROUTE_PROJECTION_OK) {
                                goto fail;
                        }
                        waypoint->sensor_index = node->num;
                        waypoint->ui_row = cell.row;
                        waypoint->ui_column = cell.column;
                        waypoint->ui_width = cell.width;
                        if (kind == TC2_ROUTE_WAYPOINT_REVERSAL) {
                                ++visible_reversals;
                        } else {
                                ++output->sensor_count;
                        }
                } else if (node->type == NODE_BRANCH) {
                        const track_turnout_action *action =
                                turnout_action_at(
                                        &turnout_plan, offset);
                        const tc2_track_d_switch_layout *layout =
                                action ? switch_layout_for(
                                        action->switch_number) : 0;
                        if (!action || !layout ||
                            action->switch_number != node->num ||
                            (action->direction != DIR_STRAIGHT &&
                             action->direction != DIR_CURVED)) {
                                status =
                                        TC2_ROUTE_PROJECTION_ROUTE_MISMATCH;
                                goto fail;
                        }
                        status = append_waypoint(
                                output, TC2_ROUTE_WAYPOINT_TURNOUT,
                                offset, node_index,
                                cumulative_um[offset], &waypoint);
                        if (status != TC2_ROUTE_PROJECTION_OK) {
                                goto fail;
                        }
                        waypoint->switch_number =
                                action->switch_number;
                        waypoint->turnout_direction =
                                action->direction;
                        waypoint->ui_row = layout->row;
                        waypoint->ui_column = layout->column;
                        waypoint->ui_width =
                                (unsigned char)(
                                        2 +
                                        decimal_width(
                                                layout->number));
                        ++output->turnout_count;
                }
        }

        const tc2_track_d_destination_layout *destination_layout =
                Tc2TrackDDestinationLayout(
                        (size_t)request->destination_index);
        status = append_waypoint(
                output, TC2_ROUTE_WAYPOINT_DESTINATION,
                last_visible, route->nodes[last_visible],
                physical_destination_um, &waypoint);
        if (status != TC2_ROUTE_PROJECTION_OK) goto fail;
        if (!destination_layout) {
                status = TC2_ROUTE_PROJECTION_LAYOUT_MISMATCH;
                goto fail;
        }
        waypoint->destination_index =
                request->destination_index;
        waypoint->ui_row = destination_layout->row;
        waypoint->ui_column = destination_layout->column;
        waypoint->ui_width =
                text_width(destination_layout->label);
        output->operator_destination_waypoint_index =
                output->waypoint_count - 1;
        output->reversal_count = visible_reversals;
        output->valid = 1;
        return TC2_ROUTE_PROJECTION_OK;

fail:
        Tc2RouteProjectionInitialize(output);
        return status;
}

static int build_current_projection(
        track_node track[TRACK_MAX],
        const tc2_route_projection_selection *selection,
        const tc2_prediction_request *request,
        tc2_route_projection *output) {
        if (!output) return TC2_ROUTE_PROJECTION_INVALID;
        if (output_aliases_input(
                    track, selection, request, 0, output)) {
                return TC2_ROUTE_PROJECTION_INVALID;
        }
        Tc2RouteProjectionInitialize(output);

        int status = TC2_ROUTE_PROJECTION_INVALID;
        if (!track || !selection || !request ||
            request->start_index != -1 ||
            !selection->selected_motion_route) {
                goto fail;
        }
        const track_route *route =
                selection->selected_motion_route;
        int anchor_offset =
                selection->geometry_anchor_route_offset;
        int ceiling_offset =
                selection->physical_destination_route_offset;
        int endpoint_offset_mm =
                selection->physical_destination_offset_mm;
        if (route->node_count <= 0 ||
            route->node_count > TRACK_MAX ||
            route->distance_mm < 0 ||
            route->optimization_cost_mm < route->distance_mm ||
            anchor_offset < 0 ||
            anchor_offset >= route->node_count ||
            ceiling_offset < anchor_offset ||
            ceiling_offset >= route->node_count ||
            endpoint_offset_mm < 0 ||
            request->destination_index < 0 ||
            request->destination_index >=
                    TC2_TRACK_D_DESTINATION_COUNT) {
                goto fail;
        }

        track_turnout_plan turnout_plan;
        int64_t cumulative_um[TRACK_MAX];
        int route_reversals;
        if (TrackBuildTurnoutPlan(
                    track, route, &turnout_plan) < 0 ||
            route_scalars_and_reversals(
                    track, route, cumulative_um,
                    &route_reversals) < 0 ||
            route_reversals != route->reversal_count ||
            track[route->nodes[anchor_offset]].type !=
                    NODE_SENSOR) {
                status = TC2_ROUTE_PROJECTION_ROUTE_MISMATCH;
                goto fail;
        }

        int64_t physical_destination_um =
                cumulative_um[anchor_offset] +
                (int64_t)endpoint_offset_mm * 1000;
        if (physical_destination_um < 0 ||
            physical_destination_um >
                    cumulative_um[route->node_count - 1]) {
                status = TC2_ROUTE_PROJECTION_ROUTE_MISMATCH;
                goto fail;
        }
        int computed_ceiling = -1;
        if (endpoint_offset_mm == 0) {
                computed_ceiling = anchor_offset;
        } else {
                for (int offset = anchor_offset + 1;
                     offset < route->node_count; ++offset) {
                        if (cumulative_um[offset] >=
                            physical_destination_um) {
                                computed_ceiling = offset;
                                break;
                        }
                }
        }
        if (computed_ceiling != ceiling_offset ||
            cumulative_um[ceiling_offset] <
                    physical_destination_um ||
            (ceiling_offset > anchor_offset &&
             cumulative_um[ceiling_offset - 1] >=
                    physical_destination_um)) {
                status = TC2_ROUTE_PROJECTION_ROUTE_MISMATCH;
                goto fail;
        }
        int last_visible =
                cumulative_um[ceiling_offset] ==
                        physical_destination_um ?
                ceiling_offset : ceiling_offset - 1;
        if (last_visible < 0 ||
            last_visible >= route->node_count) {
                status = TC2_ROUTE_PROJECTION_ROUTE_MISMATCH;
                goto fail;
        }

        copy_request(&output->request, request);
        copy_route(&output->selected_motion_route, route);
        output->geometry_anchor_route_offset = anchor_offset;
        output->geometry_anchor_sensor_index =
                track[route->nodes[anchor_offset]].num;
        output->physical_destination_route_offset =
                ceiling_offset;
        output->physical_destination_offset_mm =
                endpoint_offset_mm;
        output->physical_destination_distance_um =
                physical_destination_um;
        output->last_visible_route_offset = last_visible;

        tc2_route_waypoint *waypoint;
        status = append_waypoint(
                output, TC2_ROUTE_WAYPOINT_START, 0,
                route->nodes[0], 0, &waypoint);
        if (status != TC2_ROUTE_PROJECTION_OK) goto fail;
        track_node *first_node = &track[route->nodes[0]];
        if (first_node->type == NODE_SENSOR) {
                tc2_track_d_directed_sensor_cell cell;
                if (Tc2TrackDDirectedSensorCell(
                            first_node->num, &cell) < 0 ||
                    !text_equal(first_node->name, cell.name)) {
                        status =
                                TC2_ROUTE_PROJECTION_LAYOUT_MISMATCH;
                        goto fail;
                }
                waypoint->ui_row = cell.row;
                waypoint->ui_column = cell.column;
                waypoint->ui_width = cell.width;
        } else if (first_node->type == NODE_BRANCH) {
                const track_turnout_action *action =
                        turnout_action_at(&turnout_plan, 0);
                const tc2_track_d_switch_layout *layout =
                        action ? switch_layout_for(
                                action->switch_number) : 0;
                if (!action || !layout) {
                        status =
                                TC2_ROUTE_PROJECTION_LAYOUT_MISMATCH;
                        goto fail;
                }
                waypoint->ui_row = layout->row;
                waypoint->ui_column = layout->column;
                waypoint->ui_width =
                        (unsigned char)(
                                2 + decimal_width(layout->number));
        } else {
                status = TC2_ROUTE_PROJECTION_LAYOUT_MISMATCH;
                goto fail;
        }

        int visible_reversals = 0;
        for (int offset = 1; offset <= last_visible; ++offset) {
                int node_index = route->nodes[offset];
                track_node *node = &track[node_index];
                if (node->type == NODE_SENSOR) {
                        tc2_track_d_directed_sensor_cell cell;
                        if (Tc2TrackDDirectedSensorCell(
                                    node->num, &cell) < 0 ||
                            !text_equal(node->name, cell.name)) {
                                status =
                                        TC2_ROUTE_PROJECTION_LAYOUT_MISMATCH;
                                goto fail;
                        }
                        tc2_route_waypoint_kind kind =
                                exact_reverse_transition(
                                        track,
                                        route->nodes[offset - 1],
                                        node_index) ?
                                TC2_ROUTE_WAYPOINT_REVERSAL :
                                TC2_ROUTE_WAYPOINT_SENSOR;
                        status = append_waypoint(
                                output, kind, offset, node_index,
                                cumulative_um[offset], &waypoint);
                        if (status != TC2_ROUTE_PROJECTION_OK) {
                                goto fail;
                        }
                        waypoint->sensor_index = node->num;
                        waypoint->ui_row = cell.row;
                        waypoint->ui_column = cell.column;
                        waypoint->ui_width = cell.width;
                        if (kind == TC2_ROUTE_WAYPOINT_REVERSAL) {
                                ++visible_reversals;
                        } else {
                                ++output->sensor_count;
                        }
                } else if (node->type == NODE_BRANCH) {
                        const track_turnout_action *action =
                                turnout_action_at(
                                        &turnout_plan, offset);
                        const tc2_track_d_switch_layout *layout =
                                action ? switch_layout_for(
                                        action->switch_number) : 0;
                        if (!action || !layout ||
                            action->switch_number != node->num ||
                            (action->direction != DIR_STRAIGHT &&
                             action->direction != DIR_CURVED)) {
                                status =
                                        TC2_ROUTE_PROJECTION_ROUTE_MISMATCH;
                                goto fail;
                        }
                        status = append_waypoint(
                                output, TC2_ROUTE_WAYPOINT_TURNOUT,
                                offset, node_index,
                                cumulative_um[offset], &waypoint);
                        if (status != TC2_ROUTE_PROJECTION_OK) {
                                goto fail;
                        }
                        waypoint->switch_number =
                                action->switch_number;
                        waypoint->turnout_direction =
                                action->direction;
                        waypoint->ui_row = layout->row;
                        waypoint->ui_column = layout->column;
                        waypoint->ui_width =
                                (unsigned char)(
                                        2 + decimal_width(
                                                layout->number));
                        ++output->turnout_count;
                }
        }

        const tc2_track_d_destination_layout *destination_layout =
                Tc2TrackDDestinationLayout(
                        (size_t)request->destination_index);
        status = append_waypoint(
                output, TC2_ROUTE_WAYPOINT_DESTINATION,
                last_visible, route->nodes[last_visible],
                physical_destination_um, &waypoint);
        if (status != TC2_ROUTE_PROJECTION_OK ||
            !destination_layout) {
                status = TC2_ROUTE_PROJECTION_LAYOUT_MISMATCH;
                goto fail;
        }
        waypoint->destination_index =
                request->destination_index;
        waypoint->ui_row = destination_layout->row;
        waypoint->ui_column = destination_layout->column;
        waypoint->ui_width =
                text_width(destination_layout->label);
        output->operator_destination_waypoint_index =
                output->waypoint_count - 1;
        output->reversal_count = visible_reversals;
        output->valid = 1;
        return TC2_ROUTE_PROJECTION_OK;

fail:
        Tc2RouteProjectionInitialize(output);
        return status;
}

int Tc2RouteProjectionBuild(
        track_node track[TRACK_MAX],
        const tc2_route_projection_selection *selection,
        const tc2_prediction_request *request,
        tc2_route_projection *output) {
        return build_projection(
                track, selection, request, 0, 1, output);
}

int Tc2RouteProjectionBuildWithPrediction(
        track_node track[TRACK_MAX],
        const tc2_route_projection_selection *selection,
        const tc2_prediction_request *request,
        const tc2_prediction_plan *selected_prediction,
        tc2_route_projection *output) {
        return build_projection(
                track, selection, request,
                selected_prediction, 0, output);
}

int Tc2RouteProjectionBuildCurrent(
        track_node track[TRACK_MAX],
        const tc2_route_projection_selection *selection,
        const tc2_prediction_request *request,
        tc2_route_projection *output) {
        return build_current_projection(
                track, selection, request, output);
}

int Tc2RouteProjectionValidate(
        track_node track[TRACK_MAX],
        const tc2_route_projection_selection *selection,
        const tc2_prediction_request *request,
        const tc2_route_projection *projection) {
        if (!projection) return TC2_ROUTE_PROJECTION_INVALID;
        tc2_route_projection expected;
        int status = Tc2RouteProjectionBuild(
                track, selection, request, &expected);
        if (status != TC2_ROUTE_PROJECTION_OK) return status;
        return bytes_equal(
                &expected, projection,
                (unsigned int)sizeof(expected)) ?
                TC2_ROUTE_PROJECTION_OK :
                TC2_ROUTE_PROJECTION_ROUTE_MISMATCH;
}

int Tc2RouteProjectionValidateWithPrediction(
        track_node track[TRACK_MAX],
        const tc2_route_projection_selection *selection,
        const tc2_prediction_request *request,
        const tc2_prediction_plan *selected_prediction,
        const tc2_route_projection *projection) {
        if (!projection) return TC2_ROUTE_PROJECTION_INVALID;
        tc2_route_projection expected;
        int status = Tc2RouteProjectionBuildWithPrediction(
                track, selection, request,
                selected_prediction, &expected);
        if (status != TC2_ROUTE_PROJECTION_OK) return status;
        return bytes_equal(
                &expected, projection,
                (unsigned int)sizeof(expected)) ?
                TC2_ROUTE_PROJECTION_OK :
                TC2_ROUTE_PROJECTION_ROUTE_MISMATCH;
}

int Tc2RouteProjectionValidateCurrent(
        track_node track[TRACK_MAX],
        const tc2_route_projection_selection *selection,
        const tc2_prediction_request *request,
        const tc2_route_projection *projection) {
        if (!projection) return TC2_ROUTE_PROJECTION_INVALID;
        tc2_route_projection expected;
        int status = build_current_projection(
                track, selection, request, &expected);
        if (status != TC2_ROUTE_PROJECTION_OK) return status;
        return bytes_equal(
                       &expected, projection,
                       (unsigned int)sizeof(expected)) ?
                TC2_ROUTE_PROJECTION_OK :
                TC2_ROUTE_PROJECTION_ROUTE_MISMATCH;
}

int Tc2RouteProjectionWaypointAtOrBefore(
        const tc2_route_projection *projection,
        int64_t distance_um) {
        if (!projection || !projection->valid ||
            distance_um < 0 || projection->waypoint_count <= 0 ||
            projection->waypoint_count >
                    TC2_ROUTE_PROJECTION_MAX_WAYPOINTS) {
                return -1;
        }

        int selected = -1;
        int64_t prior_distance = -1;
        for (int index = 0;
             index < projection->waypoint_count; ++index) {
                int64_t waypoint_distance =
                        projection->waypoints[index].distance_um;
                if (waypoint_distance < 0 ||
                    waypoint_distance < prior_distance) {
                        return -1;
                }
                prior_distance = waypoint_distance;
                if (waypoint_distance <= distance_um) {
                        selected = index;
                } else {
                        break;
                }
        }
        return selected;
}

const char *Tc2RouteWaypointKindName(
        tc2_route_waypoint_kind kind) {
        if (kind == TC2_ROUTE_WAYPOINT_START) return "START";
        if (kind == TC2_ROUTE_WAYPOINT_SENSOR) return "SENSOR";
        if (kind == TC2_ROUTE_WAYPOINT_TURNOUT) return "TURNOUT";
        if (kind == TC2_ROUTE_WAYPOINT_REVERSAL) return "REVERSAL";
        if (kind == TC2_ROUTE_WAYPOINT_DESTINATION) {
                return "DESTINATION";
        }
        return "INVALID";
}

#endif
