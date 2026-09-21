#ifndef TC2_ROUTE_PROJECTION_H
#define TC2_ROUTE_PROJECTION_H

#include <stdint.h>

#include "tc2_prediction_model.h"
#include "tc2_track_d_layout.h"
#include "track_route.h"

#ifdef MODE_TC2

/*
 * A projection is an ordered, discrete description of one dispatcher
 * selection.  The selected motion route is retained in full for audit and
 * reservation provenance, including the braking/safety suffix after the
 * operator endpoint.  Semantic UI waypoints stop at the exact physical D
 * point; the suffix must never look like remaining train travel.
 */
#define TC2_ROUTE_PROJECTION_MAX_WAYPOINTS (TRACK_MAX + 2)

typedef enum {
        TC2_ROUTE_WAYPOINT_INVALID = 0,
        TC2_ROUTE_WAYPOINT_START,
        TC2_ROUTE_WAYPOINT_SENSOR,
        TC2_ROUTE_WAYPOINT_TURNOUT,
        TC2_ROUTE_WAYPOINT_REVERSAL,
        TC2_ROUTE_WAYPOINT_DESTINATION
} tc2_route_waypoint_kind;

typedef enum {
        TC2_ROUTE_PROJECTION_OK = 0,
        TC2_ROUTE_PROJECTION_INVALID = -1,
        TC2_ROUTE_PROJECTION_ROUTE_MISMATCH = -2,
        TC2_ROUTE_PROJECTION_LAYOUT_MISMATCH = -3,
        TC2_ROUTE_PROJECTION_CAPACITY = -4,
        TC2_ROUTE_PROJECTION_PREDICTION = -5
} tc2_route_projection_status;

/*
 * These are the four route facts already selected by the dispatcher.
 *
 * geometry_anchor_route_offset:
 *   job.target_route_offset, the directed sensor used by prediction.
 *
 * physical_destination_route_offset:
 *   job.destination_route_offset, the first route node whose cumulative
 *   scalar is at or beyond the exact physical operator endpoint.
 *
 * physical_destination_offset_mm:
 *   The ordinary A-F dispatch operator endpoint offset after the anchor
 *   (job.destination_base_offset_mm).  This is deliberately not a
 *   caldispatch speed-correction/control point.  The supplied ceiling must
 *   be recomputed for this same exact physical operator endpoint.
 *
 * The prediction-backed builders below accept only ordinary fixed A-F
 * dispatch geometry with an authoritative prediction.  CURRENT routes use
 * the separate geometry-only builder: it publishes the already-selected
 * shortest route for display, but can never become motion authority.
 */
typedef struct {
        const track_route *selected_motion_route;
        int geometry_anchor_route_offset;
        int physical_destination_route_offset;
        int physical_destination_offset_mm;
} tc2_route_projection_selection;

typedef struct {
        tc2_route_waypoint_kind kind;
        int route_offset;
        int graph_node;
        int sensor_index;
        int switch_number;
        int turnout_direction;
        int destination_index;
        int64_t distance_um;
        /*
         * Exact semantic label cells on the immutable dashboard, not the
         * surrounding decoration.  Starts are uppercase A-F; operator
         * destinations are lowercase d1-d8 so uppercase D-bank sensor names
         * cannot be mistaken for endpoints.
         */
        unsigned char ui_row;
        unsigned char ui_column;
        unsigned char ui_width;
} tc2_route_waypoint;

typedef struct {
        int valid;
        tc2_prediction_request request;

        /*
         * Deterministic copy of the complete selected motion route. Active
         * nodes and scalar fields are preserved exactly; unused node storage
         * is zero. UI waypoints below are a physical-endpoint-clipped view.
         */
        track_route selected_motion_route;
        tc2_prediction_plan prediction;

        int geometry_anchor_route_offset;
        int geometry_anchor_sensor_index;
        int physical_destination_route_offset;
        int physical_destination_offset_mm;
        int64_t physical_destination_distance_um;
        /*
         * Minimal physical ceiling node when the exact operator endpoint is
         * a graph node; otherwise its predecessor.  This intentionally hides
         * any zero-distance safety-suffix nodes and serves only as provenance
         * for the logical D waypoint.
         */
        int last_visible_route_offset;
        int operator_destination_waypoint_index;
        int waypoint_count;
        int sensor_count;
        int turnout_count;
        /* Reversals visible before the operator endpoint, not full-route. */
        int reversal_count;
        tc2_route_waypoint
                waypoints[TC2_ROUTE_PROJECTION_MAX_WAYPOINTS];
} tc2_route_projection;

/*
 * Produce the unique fail-closed empty value. Builders also leave exactly
 * this value after every non-alias failure.
 */
void Tc2RouteProjectionInitialize(tc2_route_projection *projection);

/*
 * Build derives prediction only from the route prefix through the geometry
 * anchor. BuildWithPrediction accepts the dispatcher's authoritative plan
 * and permits only the documented EXACT -> TRANSFERRED evidence downgrade.
 */
int Tc2RouteProjectionBuild(
        track_node track[TRACK_MAX],
        const tc2_route_projection_selection *selection,
        const tc2_prediction_request *request,
        tc2_route_projection *output);

int Tc2RouteProjectionBuildWithPrediction(
        track_node track[TRACK_MAX],
        const tc2_route_projection_selection *selection,
        const tc2_prediction_request *request,
        const tc2_prediction_plan *selected_prediction,
        tc2_route_projection *output);

/*
 * Build a display-only projection for an atomic CURRENT continuation.
 * request.start_index must be -1.  Unlike the prediction-backed builders,
 * this function does not derive timing or stopping commands; it only clips
 * the dispatcher's selected route at the exact physical destination and
 * maps its sensors/turnouts to immutable Track D UI cells.
 */
int Tc2RouteProjectionBuildCurrent(
        track_node track[TRACK_MAX],
        const tc2_route_projection_selection *selection,
        const tc2_prediction_request *request,
        tc2_route_projection *output);

int Tc2RouteProjectionValidate(
        track_node track[TRACK_MAX],
        const tc2_route_projection_selection *selection,
        const tc2_prediction_request *request,
        const tc2_route_projection *projection);

int Tc2RouteProjectionValidateWithPrediction(
        track_node track[TRACK_MAX],
        const tc2_route_projection_selection *selection,
        const tc2_prediction_request *request,
        const tc2_prediction_plan *selected_prediction,
        const tc2_route_projection *projection);

int Tc2RouteProjectionValidateCurrent(
        track_node track[TRACK_MAX],
        const tc2_route_projection_selection *selection,
        const tc2_prediction_request *request,
        const tc2_route_projection *projection);

/*
 * Returns the last stable waypoint whose physical scalar is <= distance_um.
 * Equal-distance destination events deliberately win over graph evidence.
 * A negative distance or invalid projection returns -1.
 */
int Tc2RouteProjectionWaypointAtOrBefore(
        const tc2_route_projection *projection,
        int64_t distance_um);

const char *Tc2RouteWaypointKindName(tc2_route_waypoint_kind kind);

#endif

#endif
