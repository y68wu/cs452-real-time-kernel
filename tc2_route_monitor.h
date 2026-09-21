#ifndef _tc2_route_monitor_h_
#define _tc2_route_monitor_h_ 1

#include "track_route.h"

typedef enum {
        TC2_ROUTE_SENSOR_NORMAL = 0,
        TC2_ROUTE_SENSOR_MISSING_ONE,
        TC2_ROUTE_SENSOR_DUPLICATE,
        TC2_ROUTE_SENSOR_SPURIOUS
} tc2_route_sensor_class;

typedef struct {
        int route_offset;
        int node_index;
        int distance_mm;
} tc2_route_sensor_point;

typedef struct {
        int origin_offset;
        int terminal_offset;
        int confirmed_offset;
        int confirmed_distance_mm;
        unsigned int last_sequence;
        int last_observed_node;
        int missing_sensor_count;
} tc2_route_monitor;

typedef struct {
        tc2_route_sensor_class classification;
        tc2_route_sensor_point expected;
        tc2_route_sensor_point expected_next;
        tc2_route_sensor_point matched;
        tc2_route_sensor_point missing;
        int advance_mm;
} tc2_route_observation;

/*
 * Return physical distance from route occurrence 0 through offset. Route
 * distance fields are not trusted; matching graph edges are summed directly.
 */
int Tc2RouteDistanceAtOffset(track_node *track, const track_route *route,
                             int offset, int *distance_mm);

/*
 * Find the first two sensor occurrences strictly after after_offset and no
 * later than terminal_offset. Return 0, 1, or 2; return -1 for invalid input.
 */
int Tc2RouteExpectedSensors(track_node *track, const track_route *route,
                            int after_offset, int terminal_offset,
                            tc2_route_sensor_point *next,
                            tc2_route_sensor_point *next_next);

/*
 * origin_offset is already confirmed. The inclusive terminal occurrence must
 * be captured before appending a braking guard. A monitor covers one motion
 * leg and therefore rejects a reversal transition in its range.
 *
 * baseline_sequence and every observed event_sequence must come from the same
 * wrapping uint32 sequence domain. Zero is reserved as the no-cursor sentinel;
 * nonzero values are ordered with modular half-range comparison, including
 * wrap within a motion leg.
 */
int Tc2RouteMonitorInit(track_node *track, const track_route *route,
                        int origin_offset, int terminal_offset,
                        unsigned int baseline_sequence,
                        tc2_route_monitor *monitor);

/*
 * A normal next sensor advances the confirmed occurrence. The next-next
 * sensor may advance once per monitor as MISSING_ONE. Older/equal sequence
 * tokens are DUPLICATE; all other out-of-order events are SPURIOUS. Duplicate
 * and spurious observations never advance route progress.
 */
int Tc2RouteMonitorObserve(track_node *track, const track_route *route,
                           tc2_route_monitor *monitor,
                           int sensor_node, unsigned int event_sequence,
                           tc2_route_observation *observation);

#endif
