#ifndef TC2_OFFLINE_PLANNER_H
#define TC2_OFFLINE_PLANNER_H

#include "tc2_dispatch.h"
#include "tc2_prediction_model.h"
#include "tc2_route_projection.h"
#include "track_route.h"

#ifdef MODE_TC2

/*
 * The offline planner is deliberately a pure value transformer.  It shares
 * the dispatcher's immutable Track-D catalog, route, prediction, and
 * projection contracts, but it owns no service IDs and performs no IPC,
 * reservation, sensor, CAN, or train-control operation.
 */
#define TC2_OFFLINE_PLANNER_MAX_WAYPOINTS \
        TC2_ROUTE_PROJECTION_MAX_WAYPOINTS

typedef enum {
        TC2_OFFLINE_PLAN_OK = 0,
        TC2_OFFLINE_PLAN_INVALID_ARGUMENT = -1,
        TC2_OFFLINE_PLAN_CATALOG_INVALID = -2,
        TC2_OFFLINE_PLAN_UNREACHABLE = -3,
        TC2_OFFLINE_PLAN_ROUTE_GUARD = -4,
        TC2_OFFLINE_PLAN_PREDICTION = -5,
        TC2_OFFLINE_PLAN_PROJECTION = -6,
        TC2_OFFLINE_PLAN_INTERNAL = -7
} tc2_offline_plan_status;

typedef struct {
        int status;
        int valid;

        int selected_side;
        int target_node;
        int geometry_anchor_route_offset;
        int physical_destination_route_offset;
        int physical_destination_offset_mm;
        int path_distance_mm;
        int optimization_cost_mm;

        track_route selected_motion_route;
        tc2_prediction_request prediction_request;
        tc2_prediction_plan prediction_plan;

        tc2_dispatch_projection_header projection_header;
        tc2_dispatch_projection_waypoint
                waypoints[TC2_OFFLINE_PLANNER_MAX_WAYPOINTS];
} tc2_offline_plan;

/* Produce the unique fail-closed empty result. */
void Tc2OfflinePlannerInitialize(tc2_offline_plan *plan);

/*
 * Build one deterministic A-F -> d1-d8 plan.  Every speed in 1..120 is
 * accepted.  The output includes a complete immutable projection suitable
 * for an offline simulator/UI; it grants no authority to move a train.
 */
int Tc2OfflinePlannerBuild(
        track_node track[TRACK_MAX],
        int train,
        int start_index,
        int speed,
        int destination_index,
        tc2_offline_plan *plan);

/*
 * Rebuild and compare every published byte.  This checks the local
 * publication token as well as route/prediction/projection provenance.
 */
int Tc2OfflinePlannerValidate(
        track_node track[TRACK_MAX],
        int train,
        int start_index,
        int speed,
        int destination_index,
        const tc2_offline_plan *plan);

#endif

#endif
