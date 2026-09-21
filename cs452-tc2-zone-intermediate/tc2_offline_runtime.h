#ifndef TC2_OFFLINE_RUNTIME_H
#define TC2_OFFLINE_RUNTIME_H

#include <stdint.h>

#include "tc2_dispatch.h"
#include "tc2_route_projection.h"

#ifdef MODE_TC2

/*
 * Deterministic, prediction-only runtime for an offline Track-D overlay.
 *
 * This module consumes immutable Stage-26 projection publications.  It never
 * sends CAN traffic, changes turnouts, or acquires/releases the dispatcher's
 * real reservation service.  Its events are estimates for a simulator/UI and
 * must never be treated as live sensor or movement authority.
 */
#define TC2_OFFLINE_RUNTIME_MAX_TRAINS TC2_DISPATCH_MAX_JOBS
#define TC2_OFFLINE_RUNTIME_MAX_WAYPOINTS \
        TC2_ROUTE_PROJECTION_MAX_WAYPOINTS
#define TC2_OFFLINE_RUNTIME_MAX_STEP_TICKS 4096u
#define TC2_OFFLINE_RUNTIME_INVALID_INDEX (-1)
#define TC2_OFFLINE_RUNTIME_INVALID_TRAIN (-1)
#define TC2_OFFLINE_RUNTIME_UI_SCALE 1000

/*
 * A movement window contains waypoint vertex resources plus normalized
 * physical edges.  Three resources per semantic waypoint covers graph-node
 * identity, physical sensor/switch identity, and the preceding edge.
 */
#define TC2_OFFLINE_RUNTIME_MAX_CLAIMS \
        (3 * TC2_OFFLINE_RUNTIME_MAX_WAYPOINTS + 4)

typedef enum {
        TC2_OFFLINE_OK = 0,
        TC2_OFFLINE_INVALID_ARGUMENT = -1,
        TC2_OFFLINE_CAPACITY = -2,
        TC2_OFFLINE_DUPLICATE_TRAIN = -3,
        TC2_OFFLINE_NOT_FOUND = -4,
        TC2_OFFLINE_INVALID_STATE = -5,
        TC2_OFFLINE_STALE = -6,
        TC2_OFFLINE_BOUNDS = -7,
        TC2_OFFLINE_MALFORMED_PROJECTION = -8,
        TC2_OFFLINE_OVERFLOW = -9,
        TC2_OFFLINE_ALIAS = -10
} tc2_offline_status;

typedef enum {
        TC2_OFFLINE_TRAIN_EMPTY = 0,
        TC2_OFFLINE_TRAIN_LOADING,
        TC2_OFFLINE_TRAIN_READY,
        TC2_OFFLINE_TRAIN_RUNNING,
        TC2_OFFLINE_TRAIN_WAIT_CONFLICT,
        TC2_OFFLINE_TRAIN_ARRIVED,
        TC2_OFFLINE_TRAIN_FAILED
} tc2_offline_train_state;

typedef enum {
        TC2_OFFLINE_EVENT_NONE = 0,
        TC2_OFFLINE_EVENT_STARTED,
        TC2_OFFLINE_EVENT_CONFLICT_WAIT,
        TC2_OFFLINE_EVENT_CONFLICT_RESUMED,
        TC2_OFFLINE_EVENT_PREDICTED_SENSOR_PULSE,
        TC2_OFFLINE_EVENT_PREDICTED_TURNOUT,
        TC2_OFFLINE_EVENT_PREDICTED_REVERSAL,
        TC2_OFFLINE_EVENT_ARRIVED,
        TC2_OFFLINE_EVENT_FAILED
} tc2_offline_event_type;

typedef enum {
        TC2_OFFLINE_EVENT_SOURCE_INVALID = 0,
        TC2_OFFLINE_EVENT_SOURCE_PREDICTED_OFFLINE = 1
} tc2_offline_event_source;

typedef struct {
        tc2_offline_event_type type;
        tc2_offline_event_source source;
        uint32_t tick;
        uint32_t serial;
        int train;
        int conflict_train;
        int waypoint_index;
        int route_offset;
        int graph_node;
        int sensor_index;
        int switch_number;
        int turnout_direction;
        int destination_index;
        int64_t distance_um;
        int ui_row;
        int ui_column;
        int ui_width;
        /*
         * Always zero.  Kept explicit so a UI cannot accidentally present a
         * predicted pulse as a live hardware observation.
         */
        int is_live;
} tc2_offline_event;

typedef void (*tc2_offline_event_sink)(
        const tc2_offline_event *event, void *context);

typedef struct {
        uint64_t key;
} tc2_offline_resource_claim;

typedef struct {
        tc2_offline_train_state state;
        int last_error;
        int start_requested;
        int entered_route;
        int train;
        int conflict_train;
        uint32_t wait_ticks;
        uint32_t motion_revision;
        uint32_t last_event_tick;
        uint32_t last_sensor_tick;
        int last_sensor_index;

        tc2_dispatch_projection_header header;
        tc2_dispatch_projection_waypoint
                waypoints[TC2_OFFLINE_RUNTIME_MAX_WAYPOINTS];
        int loaded_waypoint_count;
        int next_waypoint_index;
        int64_t progress_um;
        int velocity_um_per_tick;

        tc2_offline_resource_claim
                claims[TC2_OFFLINE_RUNTIME_MAX_CLAIMS];
        int claim_count;
} tc2_offline_train;

typedef struct {
        uint32_t tick;
        uint32_t event_serial;
        int initialized;
        int in_step;
        tc2_offline_train trains[TC2_OFFLINE_RUNTIME_MAX_TRAINS];
        tc2_offline_resource_claim
                scratch_claims[TC2_OFFLINE_RUNTIME_MAX_CLAIMS];
} tc2_offline_runtime;

typedef struct {
        tc2_offline_train_state state;
        int last_error;
        int start_requested;
        int train;
        int speed;
        int start_index;
        int destination_index;
        int destination_side;
        int conflict_train;
        uint32_t wait_ticks;
        uint32_t motion_revision;
        uint32_t last_event_tick;
        uint32_t last_sensor_tick;
        int last_sensor_index;
        uint64_t publication_serial;
        uint32_t plan_generation;
        uint32_t launch_epoch;
        int64_t progress_um;
        int64_t destination_distance_um;
        int velocity_um_per_tick;
        int previous_waypoint_index;
        int next_waypoint_index;
        int route_offset;
        int graph_node;
        int next_graph_node;
        /*
         * Fixed-point dashboard coordinates.  1000 means one terminal cell.
         * Consumers can animate between semantic waypoints without floats.
         */
        int ui_row_milli;
        int ui_column_milli;
        int ui_width;
} tc2_offline_train_snapshot;

typedef struct {
        uint32_t tick;
        uint32_t event_serial;
        int train_count;
        int running_count;
        int waiting_count;
        int arrived_count;
        tc2_offline_train_snapshot
                trains[TC2_OFFLINE_RUNTIME_MAX_TRAINS];
} tc2_offline_snapshot;

/*
 * Initialize at a caller-supplied uint32 tick.  Subsequent Step calls use
 * modulo-2^32 elapsed arithmetic, so UINT32_MAX -> 0 is supported.
 */
int Tc2OfflineRuntimeInitialize(
        tc2_offline_runtime *runtime, uint32_t initial_tick);
void Tc2OfflineRuntimeReset(
        tc2_offline_runtime *runtime, uint32_t initial_tick);

/*
 * Atomic complete-projection ingestion.  `waypoints` must reconstruct the
 * complete header publication and waypoint_count must match exactly.
 */
int Tc2OfflineRuntimeAddTrip(
        tc2_offline_runtime *runtime,
        const tc2_dispatch_projection_header *header,
        const tc2_dispatch_projection_waypoint *waypoints,
        int waypoint_count);

/*
 * Equivalent bounded paged ingestion for direct Stage-26 clients.  Pages
 * must be contiguous, canonical, and carry the exact immutable token from
 * BeginTrip.  Any malformed/stale page fails the loading slot closed.
 */
int Tc2OfflineRuntimeBeginTrip(
        tc2_offline_runtime *runtime,
        const tc2_dispatch_projection_header *header);
int Tc2OfflineRuntimeAppendProjectionPage(
        tc2_offline_runtime *runtime,
        const tc2_dispatch_projection_page *page);
int Tc2OfflineRuntimeCommitTrip(
        tc2_offline_runtime *runtime, int train);
int Tc2OfflineRuntimeAbortTrip(
        tc2_offline_runtime *runtime, int train);

/*
 * Commit leaves a trip READY but motion is opt-in.  StartTrip/StartAll only
 * arm offline simulation; they do not send any physical command.
 */
int Tc2OfflineRuntimeStartTrip(
        tc2_offline_runtime *runtime, int train);
int Tc2OfflineRuntimeStartAll(tc2_offline_runtime *runtime);
int Tc2OfflineRuntimeRemoveTrip(
        tc2_offline_runtime *runtime, int train);

/*
 * Advance deterministically to now_tick.  A zero elapsed interval is a
 * no-op.  Gaps larger than MAX_STEP_TICKS or ambiguous backwards jumps are
 * rejected without changing runtime state.
 *
 * The sink is optional.  When supplied it is called synchronously and must
 * not mutate/re-enter the runtime.  Every emitted hardware-looking event has
 * source=PREDICTED_OFFLINE and is_live=0.
 */
int Tc2OfflineRuntimeStep(
        tc2_offline_runtime *runtime, uint32_t now_tick,
        tc2_offline_event_sink sink, void *context);

int Tc2OfflineRuntimeGetTrainSnapshot(
        const tc2_offline_runtime *runtime, int train,
        tc2_offline_train_snapshot *snapshot);
int Tc2OfflineRuntimeGetSnapshot(
        const tc2_offline_runtime *runtime,
        tc2_offline_snapshot *snapshot);

const char *Tc2OfflineRuntimeStateName(
        tc2_offline_train_state state);
const char *Tc2OfflineRuntimeEventName(
        tc2_offline_event_type type);

#endif

#endif
