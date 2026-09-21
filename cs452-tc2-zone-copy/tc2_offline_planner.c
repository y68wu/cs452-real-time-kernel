#include "tc2_offline_planner.h"

#ifdef MODE_TC2

#include <limits.h>
#include <stdint.h>

#include "tc2_motion_model.h"
#include "tc2_track_model.h"

#define TC2_OFFLINE_REVERSAL_PENALTY_MM 600
#define TC2_OFFLINE_TRAIN_BODY_MM 102

typedef struct {
        track_route route;
        int target_node;
        int target_offset;
        int destination_offset;
        int endpoint_offset_mm;
        int path_distance_mm;
        int path_cost_mm;
        int side;
        tc2_prediction_request request;
        tc2_prediction_plan prediction;
} offline_candidate;

static void zero_bytes(void *value, unsigned int count) {
        unsigned char *bytes = (unsigned char *)value;
        if (!bytes) return;
        for (unsigned int index = 0; index < count; ++index) {
                bytes[index] = 0;
        }
}

static int bytes_equal(
        const void *left_value, const void *right_value,
        unsigned int count) {
        const unsigned char *left =
                (const unsigned char *)left_value;
        const unsigned char *right =
                (const unsigned char *)right_value;
        if (!left || !right) return 0;
        for (unsigned int index = 0; index < count; ++index) {
                if (left[index] != right[index]) return 0;
        }
        return 1;
}

static int valid_node(int node) {
        return node >= 0 && node < TRACK_MAX;
}

static void canonicalize_route(track_route *route) {
        if (!route) return;
        int first_unused = route->node_count;
        if (first_unused < 0) first_unused = 0;
        if (first_unused > TRACK_MAX) first_unused = TRACK_MAX;
        for (int offset = first_unused; offset < TRACK_MAX; ++offset) {
                route->nodes[offset] = 0;
        }
}

static int exact_reversal(
        track_node track[TRACK_MAX], int from, int to) {
        return track && valid_node(from) && valid_node(to) &&
               track[from].reverse == &track[to];
}

static int route_nodes_equal(
        const track_route *left, const track_route *right) {
        if (!left || !right ||
            left->node_count != right->node_count) {
                return 0;
        }
        for (int offset = 0; offset < left->node_count; ++offset) {
                if (left->nodes[offset] != right->nodes[offset]) {
                        return 0;
                }
        }
        return 1;
}

static int route_slice(
        track_node track[TRACK_MAX], const track_route *source,
        int first, int last, track_route *slice) {
        if (!track || !source || !slice || first < 0 ||
            last < first || last >= source->node_count ||
            source->node_count > TRACK_MAX) {
                return -1;
        }
        zero_bytes(slice, (unsigned int)sizeof(*slice));
        for (int offset = first; offset <= last; ++offset) {
                int node = source->nodes[offset];
                if (!valid_node(node) ||
                    slice->node_count >= TRACK_MAX) {
                        return -1;
                }
                slice->nodes[slice->node_count++] = node;
                if (offset < last &&
                    exact_reversal(
                            track, source->nodes[offset],
                            source->nodes[offset + 1])) {
                        ++slice->reversal_count;
                }
        }
        if (TrackRouteDistanceBetweenOffsets(
                    track, source, first, last,
                    &slice->distance_mm) < 0) {
                return -1;
        }
        slice->optimization_cost_mm = slice->distance_mm;
        canonicalize_route(slice);
        return 0;
}

static int paired_turnout_mate(int switch_number) {
        switch (switch_number) {
        case 153: return 154;
        case 154: return 153;
        case 155: return 156;
        case 156: return 155;
        default: return 0;
        }
}

static int turnout_plan_consistent(
        const track_turnout_plan *plan) {
        if (!plan || plan->action_count < 0 ||
            plan->action_count > TRACK_MAX) {
                return 0;
        }
        for (int left = 0; left < plan->action_count; ++left) {
                int left_switch =
                        plan->actions[left].switch_number;
                int left_direction =
                        plan->actions[left].direction;
                if ((left_direction != DIR_STRAIGHT &&
                     left_direction != DIR_CURVED) ||
                    left_switch <= 0) {
                        return 0;
                }
                for (int right = left + 1;
                     right < plan->action_count; ++right) {
                        int right_switch =
                                plan->actions[right]
                                        .switch_number;
                        int right_direction =
                                plan->actions[right]
                                        .direction;
                        if (left_switch == right_switch &&
                            left_direction != right_direction) {
                                return 0;
                        }
                        if (paired_turnout_mate(left_switch) ==
                                    right_switch &&
                            left_direction == DIR_CURVED &&
                            right_direction == DIR_CURVED) {
                                return 0;
                        }
                }
        }
        return 1;
}

static int checked_turnout_plan(
        track_node track[TRACK_MAX], const track_route *route,
        int first, int last) {
        track_route slice;
        track_turnout_plan plan;
        zero_bytes(&plan, (unsigned int)sizeof(plan));
        if (route_slice(
                    track, route, first, last, &slice) < 0 ||
            TrackBuildTurnoutPlan(track, &slice, &plan) < 0) {
                return -1;
        }
        return turnout_plan_consistent(&plan) ? 0 : -1;
}

static int has_consecutive_reversals(
        track_node track[TRACK_MAX], const track_route *route,
        int target_offset) {
        if (!track || !route || route->node_count < 1 ||
            route->node_count > TRACK_MAX ||
            target_offset < 0 ||
            target_offset >= route->node_count) {
                return 1;
        }
        int previous = 0;
        for (int offset = 0; offset < target_offset; ++offset) {
                int reversal = exact_reversal(
                        track, route->nodes[offset],
                        route->nodes[offset + 1]);
                if (previous && reversal) return 1;
                previous = reversal;
        }
        return 0;
}

static int contains_named_node_after(
        track_node track[TRACK_MAX], const track_route *route,
        int after_offset, const char *name, int through_offset) {
        if (!name) return 1;
        int node = TrackFindNodeByName(track, name);
        if (!route || !valid_node(node) || after_offset < -1 ||
            through_offset < 0 ||
            through_offset >= route->node_count) {
                return 0;
        }
        for (int offset = after_offset + 1;
             offset <= through_offset; ++offset) {
                if (route->nodes[offset] == node) return 1;
        }
        return 0;
}

static int ceiling_after_offset(
        track_node track[TRACK_MAX], const track_route *route,
        int anchor_offset, int additional_mm,
        int *ceiling_offset) {
        if (!track || !route || !ceiling_offset ||
            additional_mm < 0 || anchor_offset < 0 ||
            anchor_offset >= route->node_count) {
                return -1;
        }
        if (additional_mm == 0) {
                *ceiling_offset = anchor_offset;
                return 0;
        }
        for (int offset = anchor_offset + 1;
             offset < route->node_count; ++offset) {
                int distance = 0;
                if (TrackRouteDistanceBetweenOffsets(
                            track, route, anchor_offset, offset,
                            &distance) < 0) {
                        return -1;
                }
                if (distance >= additional_mm) {
                        *ceiling_offset = offset;
                        return 0;
                }
        }
        return -1;
}

static int reversal_clearance_safe(
        track_node track[TRACK_MAX], const track_route *route,
        int destination_offset, int speed) {
        if (!track || !route || destination_offset < 0 ||
            destination_offset >= route->node_count) {
                return 0;
        }
        int uncertainty = Tc2MotionUncertaintyMm(speed);
        if (uncertainty < 0 ||
            uncertainty > INT_MAX - TC2_OFFLINE_TRAIN_BODY_MM) {
                return 0;
        }
        int required =
                TC2_OFFLINE_TRAIN_BODY_MM + uncertainty;
        for (int offset = 0;
             offset < destination_offset; ++offset) {
                if (!exact_reversal(
                            track, route->nodes[offset],
                            route->nodes[offset + 1])) {
                        continue;
                }
                for (int candidate = offset + 1;
                     candidate <= destination_offset;
                     ++candidate) {
                        int node = route->nodes[candidate];
                        if (!valid_node(node)) return 0;
                        if (track[node].type != NODE_BRANCH) {
                                continue;
                        }
                        int distance = 0;
                        if (TrackRouteDistanceBetweenOffsets(
                                    track, route, offset + 1,
                                    candidate, &distance) < 0 ||
                            distance < required) {
                                return 0;
                        }
                        break;
                }
        }
        return 1;
}

static int validate_and_guard_candidate(
        track_node track[TRACK_MAX], int speed,
        int endpoint_offset_mm, const char *straight_guide,
        track_route *route, int *target_offset,
        int *destination_offset, int *path_distance,
        int *path_cost) {
        if (!track || !route || !target_offset ||
            !destination_offset || !path_distance || !path_cost ||
            route->node_count < 1 ||
            route->node_count > TRACK_MAX ||
            endpoint_offset_mm < 0 ||
            route->distance_mm < 0 ||
            route->optimization_cost_mm < route->distance_mm ||
            route->distance_mm > INT_MAX - endpoint_offset_mm ||
            route->optimization_cost_mm >
                    INT_MAX - endpoint_offset_mm) {
                return -1;
        }

        *target_offset = route->node_count - 1;
        *path_distance =
                route->distance_mm + endpoint_offset_mm;
        *path_cost =
                route->optimization_cost_mm + endpoint_offset_mm;
        if (has_consecutive_reversals(
                    track, route, *target_offset)) {
                return -1;
        }

        int braking = Tc2MotionBrakingDistanceMm(speed);
        if (braking < 0 ||
            endpoint_offset_mm > INT_MAX - braking) {
                return -1;
        }
        int required_extension =
                endpoint_offset_mm + braking;
        int represented = TrackExtendRouteAhead(
                track, route, required_extension);
        if (represented >= required_extension &&
            route->node_count < TRACK_MAX) {
                int last = route->nodes[
                        route->node_count - 1];
                if (!valid_node(last)) return -1;
                if (track[last].type == NODE_BRANCH) {
                        int before = route->node_count;
                        int extra = TrackExtendRouteAhead(
                                track, route, 1);
                        if (extra <= 0 ||
                            route->node_count <= before) {
                                return -1;
                        }
                }
        }
        canonicalize_route(route);

        if (represented < required_extension ||
            ceiling_after_offset(
                    track, route, *target_offset,
                    endpoint_offset_mm,
                    destination_offset) < 0 ||
            !contains_named_node_after(
                    track, route, *target_offset,
                    straight_guide, *destination_offset) ||
            !reversal_clearance_safe(
                    track, route, *destination_offset,
                    speed)) {
                return -1;
        }

        int leg_start = 0;
        while (leg_start <= *destination_offset) {
                int leg_end = TrackRouteMotionLegEnd(
                        track, route, leg_start);
                if (leg_end >= *destination_offset) {
                        leg_end = route->node_count - 1;
                }
                if (leg_end < leg_start ||
                    checked_turnout_plan(
                            track, route, leg_start,
                            leg_end) < 0) {
                        return -1;
                }
                if (leg_end >= *destination_offset) break;
                leg_start = leg_end + 1;
        }
        return 0;
}

/*
 * Construct the same two graph candidates as the live dispatcher.  The
 * caller receives whether the directed graph had any path independently of
 * whether a path passed the conservative execution guards.
 */
static int select_candidate_route(
        track_node track[TRACK_MAX], int start, int target,
        int speed, int endpoint_offset_mm,
        const char *straight_guide, offline_candidate *output,
        int *raw_path_seen) {
        if (!track || !output || !raw_path_seen) return -1;

        track_route forward;
        track_route reversal;
        zero_bytes(&forward, (unsigned int)sizeof(forward));
        zero_bytes(&reversal, (unsigned int)sizeof(reversal));

        int forward_target = -1;
        int forward_destination = -1;
        int forward_distance = 0;
        int forward_cost = 0;
        int reversal_target = -1;
        int reversal_destination = -1;
        int reversal_distance = 0;
        int reversal_cost = 0;

        int forward_found =
                TrackFindShortestRoute(
                        track, start, target, &forward) == 0;
        int reversal_found =
                TrackFindShortestRouteWithSensorReversals(
                        track, start, target,
                        TC2_OFFLINE_REVERSAL_PENALTY_MM,
                        &reversal) == 0;
        if (forward_found || reversal_found) *raw_path_seen = 1;
        canonicalize_route(&forward);
        canonicalize_route(&reversal);

        int have_forward =
                forward_found &&
                validate_and_guard_candidate(
                        track, speed, endpoint_offset_mm,
                        straight_guide, &forward,
                        &forward_target, &forward_destination,
                        &forward_distance, &forward_cost) == 0;
        int have_reversal =
                reversal_found &&
                validate_and_guard_candidate(
                        track, speed, endpoint_offset_mm,
                        straight_guide, &reversal,
                        &reversal_target,
                        &reversal_destination,
                        &reversal_distance,
                        &reversal_cost) == 0;
        if (!have_forward && !have_reversal) return -1;

        zero_bytes(output, (unsigned int)sizeof(*output));
        if (have_forward) {
                output->route = forward;
                output->target_offset = forward_target;
                output->destination_offset =
                        forward_destination;
                output->path_distance_mm = forward_distance;
                output->path_cost_mm = forward_cost;
        } else {
                output->route = reversal;
                output->target_offset = reversal_target;
                output->destination_offset =
                        reversal_destination;
                output->path_distance_mm = reversal_distance;
                output->path_cost_mm = reversal_cost;
        }
        output->target_node = target;
        output->endpoint_offset_mm = endpoint_offset_mm;
        canonicalize_route(&output->route);
        return 0;
}

static int build_candidate_prediction(
        track_node track[TRACK_MAX], int train, int start_index,
        int speed, int destination_index, int side,
        offline_candidate *candidate) {
        track_route geometry_route;
        if (!candidate ||
            route_slice(
                    track, &candidate->route, 0,
                    candidate->target_offset,
                    &geometry_route) < 0) {
                return -1;
        }

        zero_bytes(
                &candidate->request,
                (unsigned int)sizeof(candidate->request));
        candidate->request.train = train;
        candidate->request.start_index = start_index;
        candidate->request.destination_index =
                destination_index;
        candidate->request.destination_side = side;
        candidate->request.speed = speed;
        candidate->request.reversal_count =
                geometry_route.reversal_count;

        if (Tc2PredictionBuildPlan(
                    track, &geometry_route,
                    &candidate->request,
                    &candidate->prediction) !=
            TC2_PREDICTION_OK) {
                return -1;
        }

        if (candidate->prediction.exact_profile) {
                track_route canonical;
                zero_bytes(
                        &canonical,
                        (unsigned int)sizeof(canonical));
                int canonical_ok =
                        TrackFindShortestRoute(
                                track,
                                geometry_route.nodes[0],
                                geometry_route.nodes[
                                        geometry_route.node_count -
                                        1],
                                &canonical) == 0;
                canonicalize_route(&canonical);
                if (!canonical_ok ||
                    !route_nodes_equal(
                            &canonical, &geometry_route)) {
                        candidate->prediction.exact_profile = 0;
                        candidate->prediction.profile_evidence =
                                TC2_PREDICTION_PROFILE_TRANSFERRED_T14_A_D7;
                }
        }

        if ((int64_t)candidate->route.distance_mm * 1000 <
            candidate->prediction
                    .minimum_reservation_ceiling_distance_um) {
                return -1;
        }
        candidate->side = side;
        return 0;
}

static int range_fits_i16(int value) {
        return value >= INT16_MIN && value <= INT16_MAX;
}

static int range_fits_i8(int value) {
        return value >= INT8_MIN && value <= INT8_MAX;
}

static int copy_wire_projection(
        const tc2_route_projection *projection,
        tc2_offline_plan *plan) {
        if (!projection || !plan || !projection->valid ||
            projection->waypoint_count < 1 ||
            projection->waypoint_count >
                    TC2_OFFLINE_PLANNER_MAX_WAYPOINTS) {
                return -1;
        }
        for (int index = 0;
             index < projection->waypoint_count; ++index) {
                const tc2_route_waypoint *source =
                        &projection->waypoints[index];
                tc2_dispatch_projection_waypoint *destination =
                        &plan->waypoints[index];
                if (!range_fits_i16(source->route_offset) ||
                    !range_fits_i16(source->graph_node) ||
                    !range_fits_i16(source->sensor_index) ||
                    !range_fits_i16(source->switch_number) ||
                    !range_fits_i8((int)source->kind) ||
                    !range_fits_i8(
                            source->turnout_direction) ||
                    !range_fits_i8(
                            source->destination_index)) {
                        return -1;
                }
                destination->distance_um =
                        source->distance_um;
                destination->route_offset =
                        (int16_t)source->route_offset;
                destination->graph_node =
                        (int16_t)source->graph_node;
                destination->sensor_index =
                        (int16_t)source->sensor_index;
                destination->switch_number =
                        (int16_t)source->switch_number;
                destination->kind = (int8_t)source->kind;
                destination->turnout_direction =
                        (int8_t)source->turnout_direction;
                destination->destination_index =
                        (int8_t)source->destination_index;
                destination->ui_row = source->ui_row;
                destination->ui_column =
                        source->ui_column;
                destination->ui_width = source->ui_width;
                destination->reserved = 0;
        }
        return 0;
}

static uint64_t hash_feed_u64(uint64_t hash, uint64_t value) {
        for (int shift = 0; shift < 64; shift += 8) {
                hash ^= (value >> shift) & UINT64_C(0xff);
                hash *= UINT64_C(1099511628211);
        }
        return hash;
}

static uint64_t publication_hash(
        const tc2_offline_plan *plan) {
        uint64_t hash = UINT64_C(1469598103934665603);
#define FEED(value) \
        do { hash = hash_feed_u64( \
                hash, (uint64_t)(int64_t)(value)); } while (0)
        FEED(plan->status);
        FEED(plan->valid);
        FEED(plan->selected_side);
        FEED(plan->target_node);
        FEED(plan->geometry_anchor_route_offset);
        FEED(plan->physical_destination_route_offset);
        FEED(plan->physical_destination_offset_mm);
        FEED(plan->path_distance_mm);
        FEED(plan->optimization_cost_mm);
        FEED(plan->selected_motion_route.node_count);
        FEED(plan->selected_motion_route.distance_mm);
        FEED(plan->selected_motion_route.optimization_cost_mm);
        FEED(plan->selected_motion_route.reversal_count);
        for (int offset = 0;
             offset < plan->selected_motion_route.node_count;
             ++offset) {
                FEED(plan->selected_motion_route.nodes[offset]);
        }
        FEED(plan->prediction_request.train);
        FEED(plan->prediction_request.start_index);
        FEED(plan->prediction_request.destination_index);
        FEED(plan->prediction_request.destination_side);
        FEED(plan->prediction_request.speed);
        FEED(plan->prediction_request.reversal_count);
        FEED(plan->prediction_plan.geometry_anchor_node);
        FEED(plan->prediction_plan.geometry_anchor_route_offset);
        FEED(plan->prediction_plan.geometry_base_offset_mm);
        FEED(plan->prediction_plan.geometry_evidence);
        FEED(plan->prediction_plan.profile_evidence);
        FEED(plan->prediction_plan.exact_profile);
        FEED(plan->prediction_plan.velocity_um_per_tick);
        FEED(plan->prediction_plan.point_stop_distance_um);
        FEED(plan->prediction_plan.profile_correction_mm);
        FEED(plan->prediction_plan.physical_destination_distance_um);
        FEED(plan->prediction_plan.corrected_endpoint_distance_um);
        FEED(plan->prediction_plan.endpoint_distance_um);
        FEED(plan->prediction_plan.endpoint_anchor_node);
        FEED(plan->prediction_plan.endpoint_anchor_route_offset);
        FEED(plan->prediction_plan.endpoint_offset_um);
        FEED(plan->prediction_plan.raw_command_distance_um);
        FEED(plan->prediction_plan.command_distance_um);
        FEED(plan->prediction_plan.command_anchor_node);
        FEED(plan->prediction_plan.command_anchor_route_offset);
        FEED(plan->prediction_plan.command_offset_um);
        FEED(plan->prediction_plan.action);
        FEED(plan->prediction_plan.conservative_braking_distance_mm);
        FEED(plan->prediction_plan.minimum_reservation_ceiling_distance_um);
        FEED(plan->projection_header.status);
        FEED(plan->projection_header.valid);
        FEED(plan->projection_header.train);
        FEED(plan->projection_header.job_state);
        FEED(plan->projection_header.physical_destination_distance_um);
        FEED(plan->projection_header.scheduler_healthy);
        FEED(plan->projection_header.start_index);
        FEED(plan->projection_header.destination_index);
        FEED(plan->projection_header.destination_side);
        FEED(plan->projection_header.speed);
        FEED(plan->projection_header.waypoint_count);
        FEED(plan->projection_header.sensor_count);
        FEED(plan->projection_header.turnout_count);
        FEED(plan->projection_header.reversal_count);
        FEED(plan->projection_header.geometry_anchor_route_offset);
        FEED(plan->projection_header.physical_destination_route_offset);
        FEED(plan->projection_header.last_visible_route_offset);
        FEED(plan->projection_header.operator_destination_waypoint_index);
        FEED(plan->projection_header.physical_destination_offset_mm);
        for (int index = 0;
             index < plan->projection_header.waypoint_count;
             ++index) {
                const tc2_dispatch_projection_waypoint *point =
                        &plan->waypoints[index];
                FEED(point->distance_um);
                FEED(point->route_offset);
                FEED(point->graph_node);
                FEED(point->sensor_index);
                FEED(point->switch_number);
                FEED(point->kind);
                FEED(point->turnout_direction);
                FEED(point->destination_index);
                FEED(point->ui_row);
                FEED(point->ui_column);
                FEED(point->ui_width);
                FEED(point->reserved);
        }
#undef FEED
        return hash ? hash : UINT64_C(1);
}

void Tc2OfflinePlannerInitialize(tc2_offline_plan *plan) {
        if (!plan) return;
        zero_bytes(plan, (unsigned int)sizeof(*plan));
        plan->status = TC2_OFFLINE_PLAN_INVALID_ARGUMENT;
        plan->selected_side = -1;
        plan->target_node = -1;
        plan->geometry_anchor_route_offset = -1;
        plan->physical_destination_route_offset = -1;
        plan->projection_header.status =
                TC2_DISPATCH_PROJECTION_NOT_READY;
}

static int fail_plan(tc2_offline_plan *plan, int status) {
        Tc2OfflinePlannerInitialize(plan);
        if (plan) plan->status = status;
        return status;
}

int Tc2OfflinePlannerBuild(
        track_node track[TRACK_MAX],
        int train,
        int start_index,
        int speed,
        int destination_index,
        tc2_offline_plan *plan) {
        if (!plan) return TC2_OFFLINE_PLAN_INVALID_ARGUMENT;
        Tc2OfflinePlannerInitialize(plan);
        if (!track || train < 1 || train > 255 ||
            start_index < 0 ||
            start_index >= TC2_TRACK_START_COUNT ||
            speed < 1 || speed > 120 ||
            destination_index < 0 ||
            destination_index >=
                    TC2_TRACK_DESTINATION_COUNT) {
                return fail_plan(
                        plan,
                        TC2_OFFLINE_PLAN_INVALID_ARGUMENT);
        }
        if (Tc2TrackCatalogValidate(track) < 0) {
                return fail_plan(
                        plan,
                        TC2_OFFLINE_PLAN_CATALOG_INVALID);
        }

        const tc2_track_start_definition *start =
                Tc2TrackStartDefinition(start_index);
        int start_node = start ?
                TrackFindNodeByName(track, start->enter_node) :
                -1;
        if (!valid_node(start_node)) {
                return fail_plan(
                        plan,
                        TC2_OFFLINE_PLAN_CATALOG_INVALID);
        }

        int found = 0;
        int raw_path_seen = 0;
        int guarded_path_seen = 0;
        int prediction_path_seen = 0;
        offline_candidate selected;
        zero_bytes(&selected, (unsigned int)sizeof(selected));
        for (int side = 0;
             side < TC2_TRACK_DESTINATION_SIDE_COUNT;
             ++side) {
                const tc2_track_destination_side *definition =
                        Tc2TrackDestinationSide(
                                destination_index, side);
                int target = definition ?
                        TrackFindNodeByName(
                                track,
                                definition->anchor_sensor) :
                        -1;
                if (!definition || !valid_node(target)) {
                        continue;
                }

                offline_candidate candidate;
                if (select_candidate_route(
                            track, start_node, target, speed,
                            definition->base_offset_mm,
                            definition->straight_guide,
                            &candidate, &raw_path_seen) < 0) {
                        continue;
                }
                guarded_path_seen = 1;
                if (build_candidate_prediction(
                            track, train, start_index, speed,
                            destination_index, side,
                            &candidate) < 0) {
                        continue;
                }
                prediction_path_seen = 1;

                if (!found ||
                    candidate.path_distance_mm <
                            selected.path_distance_mm ||
                    (candidate.path_distance_mm ==
                             selected.path_distance_mm &&
                     candidate.path_cost_mm <
                             selected.path_cost_mm)) {
                        selected = candidate;
                        found = 1;
                }
        }

        if (!found) {
                if (!raw_path_seen) {
                        return fail_plan(
                                plan,
                                TC2_OFFLINE_PLAN_UNREACHABLE);
                }
                if (!guarded_path_seen) {
                        return fail_plan(
                                plan,
                                TC2_OFFLINE_PLAN_ROUTE_GUARD);
                }
                if (!prediction_path_seen) {
                        return fail_plan(
                                plan,
                                TC2_OFFLINE_PLAN_PREDICTION);
                }
                return fail_plan(
                        plan, TC2_OFFLINE_PLAN_INTERNAL);
        }

        plan->selected_side = selected.side;
        plan->target_node = selected.target_node;
        plan->geometry_anchor_route_offset =
                selected.target_offset;
        plan->physical_destination_route_offset =
                selected.destination_offset;
        plan->physical_destination_offset_mm =
                selected.endpoint_offset_mm;
        plan->path_distance_mm =
                selected.path_distance_mm;
        plan->optimization_cost_mm =
                selected.path_cost_mm;
        plan->selected_motion_route = selected.route;
        canonicalize_route(&plan->selected_motion_route);
        plan->prediction_request = selected.request;
        plan->prediction_plan = selected.prediction;

        tc2_route_projection_selection projection_selection;
        projection_selection.selected_motion_route =
                &plan->selected_motion_route;
        projection_selection.geometry_anchor_route_offset =
                plan->geometry_anchor_route_offset;
        projection_selection.physical_destination_route_offset =
                plan->physical_destination_route_offset;
        projection_selection.physical_destination_offset_mm =
                plan->physical_destination_offset_mm;
        tc2_route_projection projection;
        if (Tc2RouteProjectionBuildWithPrediction(
                    track, &projection_selection,
                    &plan->prediction_request,
                    &plan->prediction_plan,
                    &projection) != TC2_ROUTE_PROJECTION_OK ||
            Tc2RouteProjectionValidateWithPrediction(
                    track, &projection_selection,
                    &plan->prediction_request,
                    &plan->prediction_plan,
                    &projection) != TC2_ROUTE_PROJECTION_OK ||
            copy_wire_projection(&projection, plan) < 0) {
                return fail_plan(
                        plan, TC2_OFFLINE_PLAN_PROJECTION);
        }

        tc2_dispatch_projection_header *header =
                &plan->projection_header;
        header->status = TC2_DISPATCH_PROJECTION_OK;
        header->valid = 1;
        header->train = train;
        header->job_state = TC2_JOB_READY;
        header->physical_destination_distance_um =
                projection.physical_destination_distance_um;
        header->scheduler_healthy = 1;
        header->start_index = start_index;
        header->destination_index = destination_index;
        header->destination_side = selected.side;
        header->speed = speed;
        header->waypoint_count = projection.waypoint_count;
        header->sensor_count = projection.sensor_count;
        header->turnout_count = projection.turnout_count;
        header->reversal_count = projection.reversal_count;
        header->geometry_anchor_route_offset =
                projection.geometry_anchor_route_offset;
        header->physical_destination_route_offset =
                projection.physical_destination_route_offset;
        header->last_visible_route_offset =
                projection.last_visible_route_offset;
        header->operator_destination_waypoint_index =
                projection.operator_destination_waypoint_index;
        header->physical_destination_offset_mm =
                projection.physical_destination_offset_mm;

        plan->valid = 1;
        plan->status = TC2_OFFLINE_PLAN_OK;
        header->publication_serial = publication_hash(plan);
        header->plan_generation =
                (uint32_t)header->publication_serial;
        if (header->plan_generation == 0) {
                header->plan_generation = 1;
        }
        header->launch_epoch =
                (uint32_t)(
                        header->publication_serial >> 32);
        if (header->launch_epoch == 0) {
                header->launch_epoch = 1;
        }
        return TC2_OFFLINE_PLAN_OK;
}

int Tc2OfflinePlannerValidate(
        track_node track[TRACK_MAX],
        int train,
        int start_index,
        int speed,
        int destination_index,
        const tc2_offline_plan *plan) {
        if (!plan || !plan->valid ||
            plan->status != TC2_OFFLINE_PLAN_OK) {
                return TC2_OFFLINE_PLAN_INVALID_ARGUMENT;
        }
        tc2_offline_plan expected;
        int status = Tc2OfflinePlannerBuild(
                track, train, start_index, speed,
                destination_index, &expected);
        if (status != TC2_OFFLINE_PLAN_OK) return status;
        return bytes_equal(
                &expected, plan,
                (unsigned int)sizeof(expected)) ?
                TC2_OFFLINE_PLAN_OK :
                TC2_OFFLINE_PLAN_INTERNAL;
}

#endif
