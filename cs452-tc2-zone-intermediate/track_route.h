#ifndef _track_route_h_
#define _track_route_h_ 1

#include "track_data.h"

typedef struct {
        int nodes[TRACK_MAX];
        int node_count;
        int distance_mm;
        int optimization_cost_mm;
        int reversal_count;
} track_route;

typedef struct {
        int switch_number;
        int direction;
        int route_node_offset;
} track_turnout_action;

typedef struct {
        track_turnout_action actions[TRACK_MAX];
        int action_count;
} track_turnout_plan;

int TrackFindNodeByName(track_node *track, const char *name);
int TrackFindShortestRoute(track_node *track, int start_index, int destination_index,
                           track_route *route);
int TrackFindShortestRouteWithReversals(track_node *track, int start_index,
                                        int destination_index, int reversal_penalty_mm,
                                        track_route *route);
/*
 * Unavailable nodes are treated as unavailable physical landmarks: marking
 * either directed node also excludes its reverse node. A null availability
 * array means that every valid node is available.
 */
int TrackFindShortestRouteWithUnavailable(
        track_node *track, int start_index, int destination_index,
        const unsigned char unavailable_by_node[TRACK_MAX],
        track_route *route);
/*
 * This is the reversal-aware planner intended for automatic execution.
 * Reversal transitions are considered only at sensor nodes.
 */
int TrackFindShortestRouteWithSensorReversalsAndUnavailable(
        track_node *track, int start_index, int destination_index,
        int reversal_penalty_mm,
        const unsigned char unavailable_by_node[TRACK_MAX],
        track_route *route);
int TrackFindShortestRouteWithSensorReversals(
        track_node *track, int start_index, int destination_index,
        int reversal_penalty_mm, track_route *route);

/*
 * Recompute physical distance over route node offsets [from_offset,
 * to_offset]. A reversal transition contributes zero millimetres.
 */
int TrackRouteDistanceBetweenOffsets(
        track_node *track, const track_route *route,
        int from_offset, int to_offset, int *distance_mm);

int TrackExtendRouteAhead(track_node *track, track_route *route,
                          int additional_distance_mm);
int TrackBuildTurnoutPlan(track_node *track, const track_route *route,
                          track_turnout_plan *plan);
/*
 * A motion leg ends immediately before the next reversal transition. The
 * leg-only turnout plan never presets actions on the far side of a reversal.
 */
int TrackRouteMotionLegEnd(track_node *track, const track_route *route,
                           int leg_start_offset);
int TrackBuildTurnoutPlanForLeg(track_node *track, const track_route *route,
                                int leg_start_offset,
                                track_turnout_plan *plan);

// TrackExtendRouteAhead returns the represented added distance in mm, or -1
// for invalid input. It may return less than requested at an exit/buffer limit.
// 这个route result保存node indexes、physical distance和reversal count，不执行任何train movement。

#endif
