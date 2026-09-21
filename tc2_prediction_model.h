#ifndef _tc2_prediction_model_h_
#define _tc2_prediction_model_h_ 1

#include <stdint.h>

#include "tc2_track_model.h"
#include "track_route.h"

#ifdef MODE_TC2

/*
 * Geometry evidence describes where the operator endpoint is on Track D.
 * Profile evidence describes how confidently the measured T14/A/D7 motion
 * curve transfers to this train, origin, direction, and destination.
 * These concepts must never be collapsed into one confidence value.
 */
typedef enum {
        TC2_PREDICTION_PROFILE_NONE = 0,
        TC2_PREDICTION_PROFILE_EXACT_T14_A_D7,
        TC2_PREDICTION_PROFILE_TRANSFERRED_T14_A_D7
} tc2_prediction_profile_evidence;

typedef enum {
        TC2_PREDICTION_TIMED = 0,
        TC2_PREDICTION_IMMEDIATE_STOP
} tc2_prediction_action;

typedef enum {
        TC2_PREDICTION_OK = 0,
        TC2_PREDICTION_INVALID = -1,
        TC2_PREDICTION_ROUTE_MISMATCH = -2,
        TC2_PREDICTION_RANGE = -3
} tc2_prediction_status;

typedef struct {
        int train;
        int start_index;
        int destination_index;
        int destination_side;
        int speed;
        int reversal_count;
} tc2_prediction_request;

typedef struct {
        int geometry_anchor_node;
        int geometry_anchor_route_offset;
        int geometry_base_offset_mm;
        tc2_track_geometry_evidence geometry_evidence;

        tc2_prediction_profile_evidence profile_evidence;
        int exact_profile;
        int velocity_um_per_tick;
        int point_stop_distance_um;
        int profile_correction_mm;

        /*
         * The physical operator endpoint never includes the transferred
         * speed correction. corrected_endpoint_distance_um may be signed;
         * endpoint_distance_um is the executable, start-clamped scalar.
         */
        int64_t physical_destination_distance_um;
        int64_t corrected_endpoint_distance_um;
        int64_t endpoint_distance_um;
        int endpoint_anchor_node;
        int endpoint_anchor_route_offset;
        int64_t endpoint_offset_um;

        /*
         * The command point is the corrected endpoint minus the point-model
         * stopping distance. A point before route start becomes an explicit
         * immediate-stop plan rather than wrapping through unsigned math.
         */
        int64_t raw_command_distance_um;
        int64_t command_distance_um;
        int command_anchor_node;
        int command_anchor_route_offset;
        int64_t command_offset_um;
        tc2_prediction_action action;

        /*
         * This is an invariant/reporting lower bound only. The dispatcher
         * still builds the authoritative physical-pair/turnout/body safety
         * footprint. Crucially it is derived from the physical destination,
         * not the speed-corrected point prediction.
         */
        int conservative_braking_distance_mm;
        int64_t minimum_reservation_ceiling_distance_um;
} tc2_prediction_plan;

int Tc2PredictionBuildPlan(
        track_node track[TRACK_MAX],
        const track_route *route_to_geometry_anchor,
        const tc2_prediction_request *request,
        tc2_prediction_plan *plan);

const char *Tc2PredictionProfileEvidenceName(
        tc2_prediction_profile_evidence evidence);
const char *Tc2PredictionActionName(tc2_prediction_action action);

#endif

#endif
