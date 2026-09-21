#include "tc2_prediction_model.h"

#ifdef MODE_TC2

#include <limits.h>

#include "tc2_motion_model.h"

static void zero_plan(tc2_prediction_plan *plan) {
        unsigned char *bytes = (unsigned char *)plan;
        for (unsigned int i = 0;
             i < (unsigned int)sizeof(*plan); ++i) {
                bytes[i] = 0;
        }
}

static int checked_add_i64(
        int64_t left, int64_t right, int64_t *result) {
        if (!result ||
            (right > 0 && left > INT64_MAX - right) ||
            (right < 0 && left < INT64_MIN - right)) {
                return -1;
        }
        *result = left + right;
        return 0;
}

static int route_node_is_valid(int node) {
        return node >= 0 && node < TRACK_MAX;
}

static int locate_scalar_on_route(
        track_node track[TRACK_MAX], const track_route *route,
        int64_t scalar_um, int *anchor_node,
        int *anchor_route_offset, int64_t *offset_um) {
        if (!track || !route || !anchor_node ||
            !anchor_route_offset || !offset_um ||
            scalar_um < 0 || route->node_count <= 0 ||
            route->node_count > TRACK_MAX ||
            !route_node_is_valid(route->nodes[0])) {
                return TC2_PREDICTION_INVALID;
        }

        int selected_offset = 0;
        int64_t selected_distance_um = 0;
        int64_t cumulative_um = 0;
        for (int offset = 1; offset < route->node_count; ++offset) {
                int edge_distance_mm;
                if (!route_node_is_valid(route->nodes[offset]) ||
                    TrackRouteDistanceBetweenOffsets(
                            track, route, offset - 1, offset,
                            &edge_distance_mm) < 0 ||
                    edge_distance_mm < 0) {
                        return TC2_PREDICTION_ROUTE_MISMATCH;
                }
                int64_t edge_distance_um =
                        (int64_t)edge_distance_mm * 1000;
                if (checked_add_i64(
                            cumulative_um, edge_distance_um,
                            &cumulative_um) < 0) {
                        return TC2_PREDICTION_RANGE;
                }
                if (cumulative_um <= scalar_um) {
                        selected_offset = offset;
                        selected_distance_um = cumulative_um;
                } else {
                        break;
                }
        }

        *anchor_node = route->nodes[selected_offset];
        *anchor_route_offset = selected_offset;
        *offset_um = scalar_um - selected_distance_um;
        return TC2_PREDICTION_OK;
}

static tc2_prediction_profile_evidence profile_evidence_for(
        const tc2_prediction_request *request) {
        if (request->train == 14 &&
            request->start_index == 0 &&
            request->destination_index == 6 &&
            request->destination_side == 0 &&
            request->reversal_count == 0) {
                return TC2_PREDICTION_PROFILE_EXACT_T14_A_D7;
        }
        return TC2_PREDICTION_PROFILE_TRANSFERRED_T14_A_D7;
}

int Tc2PredictionBuildPlan(
        track_node track[TRACK_MAX],
        const track_route *route,
        const tc2_prediction_request *request,
        tc2_prediction_plan *plan) {
        if (!plan) return TC2_PREDICTION_INVALID;
        zero_plan(plan);
        if (!track || !route || !request ||
            request->train < 1 || request->train > 255 ||
            request->start_index < 0 ||
            request->start_index >= TC2_TRACK_START_COUNT ||
            request->destination_index < 0 ||
            request->destination_index >=
                    TC2_TRACK_DESTINATION_COUNT ||
            request->destination_side < 0 ||
            request->destination_side >=
                    TC2_TRACK_DESTINATION_SIDE_COUNT ||
            request->speed < 1 || request->speed > 120 ||
            request->reversal_count < 0 ||
            route->node_count <= 0 ||
            route->node_count > TRACK_MAX ||
            route->distance_mm < 0 ||
            route->reversal_count != request->reversal_count) {
                return TC2_PREDICTION_INVALID;
        }

        const tc2_track_start_definition *start =
                Tc2TrackStartDefinition(request->start_index);
        const tc2_track_destination_side *side =
                Tc2TrackDestinationSide(
                        request->destination_index,
                        request->destination_side);
        int start_node = start ?
                TrackFindNodeByName(track, start->enter_node) : -1;
        int geometry_anchor = side ?
                TrackFindNodeByName(
                        track, side->anchor_sensor) : -1;
        if (!start || !side || start_node < 0 ||
            geometry_anchor < 0 ||
            !route_node_is_valid(route->nodes[0]) ||
            !route_node_is_valid(
                    route->nodes[route->node_count - 1]) ||
            route->nodes[0] != start_node ||
            route->nodes[route->node_count - 1] !=
                    geometry_anchor) {
                return TC2_PREDICTION_ROUTE_MISMATCH;
        }

        int measured_route_distance_mm;
        if (TrackRouteDistanceBetweenOffsets(
                    track, route, 0, route->node_count - 1,
                    &measured_route_distance_mm) < 0 ||
            measured_route_distance_mm != route->distance_mm) {
                return TC2_PREDICTION_ROUTE_MISMATCH;
        }

        int velocity =
                Tc2MotionProvisionalVelocityUmPerTick(
                        request->speed);
        int stop_distance =
                Tc2MotionProvisionalStopDistanceUm(
                        request->speed);
        int correction =
                Tc2MotionRouteEndpointCorrectionMm(
                        request->train, request->start_index,
                        request->destination_index, request->speed);
        int conservative_braking =
                Tc2MotionBrakingDistanceMm(request->speed);
        if (velocity <= 0 || stop_distance < 0 ||
            correction == INT_MIN ||
            conservative_braking < 0 ||
            side->base_offset_mm < 0) {
                return TC2_PREDICTION_RANGE;
        }

        int64_t physical_distance_um =
                (int64_t)route->distance_mm * 1000;
        if (checked_add_i64(
                    physical_distance_um,
                    (int64_t)side->base_offset_mm * 1000,
                    &physical_distance_um) < 0) {
                return TC2_PREDICTION_RANGE;
        }
        int64_t corrected_endpoint_um;
        if (checked_add_i64(
                    physical_distance_um,
                    (int64_t)correction * 1000,
                    &corrected_endpoint_um) < 0) {
                return TC2_PREDICTION_RANGE;
        }
        int64_t raw_command_um;
        if (checked_add_i64(
                    corrected_endpoint_um,
                    -(int64_t)stop_distance,
                    &raw_command_um) < 0) {
                return TC2_PREDICTION_RANGE;
        }
        int64_t reservation_ceiling_um;
        if (checked_add_i64(
                    physical_distance_um,
                    (int64_t)conservative_braking * 1000,
                    &reservation_ceiling_um) < 0) {
                return TC2_PREDICTION_RANGE;
        }

        plan->geometry_anchor_node = geometry_anchor;
        plan->geometry_anchor_route_offset =
                route->node_count - 1;
        plan->geometry_base_offset_mm =
                side->base_offset_mm;
        plan->geometry_evidence =
                side->geometry_evidence;
        plan->profile_evidence =
                profile_evidence_for(request);
        plan->exact_profile =
                plan->profile_evidence ==
                TC2_PREDICTION_PROFILE_EXACT_T14_A_D7;
        plan->velocity_um_per_tick = velocity;
        plan->point_stop_distance_um = stop_distance;
        plan->profile_correction_mm = correction;
        plan->physical_destination_distance_um =
                physical_distance_um;
        plan->corrected_endpoint_distance_um =
                corrected_endpoint_um;
        plan->endpoint_distance_um =
                corrected_endpoint_um > 0 ?
                corrected_endpoint_um : 0;
        plan->raw_command_distance_um = raw_command_um;
        plan->command_distance_um =
                raw_command_um > 0 ? raw_command_um : 0;
        plan->action =
                raw_command_um > 0 ?
                TC2_PREDICTION_TIMED :
                TC2_PREDICTION_IMMEDIATE_STOP;
        plan->conservative_braking_distance_mm =
                conservative_braking;
        plan->minimum_reservation_ceiling_distance_um =
                reservation_ceiling_um;

        int status = locate_scalar_on_route(
                track, route, plan->endpoint_distance_um,
                &plan->endpoint_anchor_node,
                &plan->endpoint_anchor_route_offset,
                &plan->endpoint_offset_um);
        if (status != TC2_PREDICTION_OK) return status;
        return locate_scalar_on_route(
                track, route, plan->command_distance_um,
                &plan->command_anchor_node,
                &plan->command_anchor_route_offset,
                &plan->command_offset_um);
}

const char *Tc2PredictionProfileEvidenceName(
        tc2_prediction_profile_evidence evidence) {
        if (evidence ==
            TC2_PREDICTION_PROFILE_EXACT_T14_A_D7) {
                return "EXACT_T14_A_D7";
        }
        if (evidence ==
            TC2_PREDICTION_PROFILE_TRANSFERRED_T14_A_D7) {
                return "TRANSFERRED_T14_A_D7";
        }
        return "NONE";
}

const char *Tc2PredictionActionName(
        tc2_prediction_action action) {
        return action == TC2_PREDICTION_IMMEDIATE_STOP ?
                "IMMEDIATE_STOP" : "TIMED";
}

#endif
