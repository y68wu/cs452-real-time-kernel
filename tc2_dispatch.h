#ifndef _tc2_dispatch_h_
#define _tc2_dispatch_h_ 1

#include <stdint.h>

#include "tc2_track_model.h"

#define TC2_DISPATCH_SERVER_NAME "tc2-dispatch"
#define TC2_DISPATCH_MAX_JOBS 16
#define TC2_DISPATCH_START_COUNT TC2_TRACK_START_COUNT
#define TC2_DISPATCH_DESTINATION_COUNT TC2_TRACK_DESTINATION_COUNT
#define TC2_DISPATCH_START_CURRENT (-1)
#define TC2_DISPATCH_START_INVALID (-2)
#define TC2_DISPATCH_PROJECTION_PAGE_CAPACITY 8
#define TC2_DISPATCH_TURNOUT_ACTIONS_PER_STEP 4

/*
 * Projection IPC deliberately publishes a compact, fixed-width wire
 * representation.  The full route/projection object remains private to the
 * dispatcher, so a terminal consumer cannot accidentally treat UI state as
 * motion authority.
 */
typedef enum {
        TC2_DISPATCH_PROJECTION_OK = 0,
        TC2_DISPATCH_PROJECTION_INVALID_ARGUMENT = -1,
        TC2_DISPATCH_PROJECTION_NOT_FOUND = -2,
        TC2_DISPATCH_PROJECTION_NOT_READY = -3,
        TC2_DISPATCH_PROJECTION_STALE = -4,
        TC2_DISPATCH_PROJECTION_BOUNDS = -5,
        TC2_DISPATCH_PROJECTION_INTERNAL = -6
} tc2_dispatch_projection_status;

typedef struct {
        int64_t distance_um;
        int16_t route_offset;
        int16_t graph_node;
        int16_t sensor_index;
        int16_t switch_number;
        int8_t kind;
        int8_t turnout_direction;
        int8_t destination_index;
        uint8_t ui_row;
        uint8_t ui_column;
        uint8_t ui_width;
        uint16_t reserved;
} tc2_dispatch_projection_waypoint;

typedef struct {
        int32_t status;
        int32_t valid;
        int32_t train;
        int32_t job_state;
        uint64_t publication_serial;
        int64_t physical_destination_distance_um;
        uint32_t plan_generation;
        uint32_t launch_epoch;
        int32_t scheduler_healthy;
        int32_t start_index;
        int32_t destination_index;
        int32_t destination_side;
        int32_t speed;
        int32_t waypoint_count;
        int32_t sensor_count;
        int32_t turnout_count;
        int32_t reversal_count;
        int32_t geometry_anchor_route_offset;
        int32_t physical_destination_route_offset;
        int32_t last_visible_route_offset;
        int32_t operator_destination_waypoint_index;
        int32_t physical_destination_offset_mm;
} tc2_dispatch_projection_header;

typedef struct {
        int32_t status;
        int32_t train;
        uint64_t publication_serial;
        uint32_t plan_generation;
        uint32_t launch_epoch;
        int32_t first_waypoint;
        int32_t count;
        int32_t total_waypoints;
        int32_t has_more;
        tc2_dispatch_projection_waypoint
                waypoints[TC2_DISPATCH_PROJECTION_PAGE_CAPACITY];
} tc2_dispatch_projection_page;

typedef enum {
        TC2_JOB_EMPTY = 0,
        TC2_JOB_STAGED,
        TC2_JOB_WAITING,
        TC2_JOB_PREPARING,
        TC2_JOB_READY,
        TC2_JOB_LAUNCHING,
        TC2_JOB_RUNNING,
        TC2_JOB_BRAKING,
        TC2_JOB_STOP_UNCONFIRMED,
        TC2_JOB_ARRIVED,
        TC2_JOB_STOPPED,
        TC2_JOB_FAILED,
        TC2_JOB_REVERSING,
        TC2_JOB_CANCEL_BRAKING,
        /*
         * A traffic hold is distinct from an operator cancel and from an
         * endpoint arrival.  The train remains managed, keeps its route
         * reservation, and may resume only after the live safety policy and
         * the reservation generation both say that movement is safe.
         */
        TC2_JOB_TRAFFIC_HOLD
} tc2_job_state;

typedef enum {
        TC2_TRAFFIC_NONE = 0,
        TC2_TRAFFIC_FOLLOWING,
        TC2_TRAFFIC_HEAD_ON,
        /*
         * A topology/position sample was temporarily incoherent.  This is a
         * fail-closed stop, not evidence that the two trains are physically
         * head-on.  It is re-evaluated and may resume automatically once a
         * coherent directed-edge relation and movement authority exist.
         */
        TC2_TRAFFIC_AUDIT,
        /*
         * The train reached the end of its current single-owner rolling
         * authority.  It stays stopped until an atomic window extension and
         * every newly-authorized turnout command have completed.
         */
        TC2_TRAFFIC_AUTHORITY
} tc2_traffic_reason;

typedef enum {
        TC2_FAILURE_NONE = 0,
        TC2_FAILURE_ROUTE,
        TC2_FAILURE_RESERVATION,
        TC2_FAILURE_SENSOR_SERVICE,
        TC2_FAILURE_SENSOR_SEQUENCE,
        TC2_FAILURE_RESERVATION_SERVICE,
        TC2_FAILURE_CAN_SERVICE,
        TC2_FAILURE_CAN_HEALTH,
        TC2_FAILURE_TURNOUT,
        TC2_FAILURE_WATCHDOG,
        TC2_FAILURE_INTERNAL
} tc2_failure_reason;

typedef enum {
        TC2_SENSOR_HEALTH_OK = 0,
        TC2_SENSOR_HEALTH_SERVICE_UNAVAILABLE,
        TC2_SENSOR_HEALTH_HEARTBEAT_STALE,
        TC2_SENSOR_HEALTH_RECEIVE_FAILURE,
        TC2_SENSOR_HEALTH_TIME_FAILURE
} tc2_sensor_health_fault;

typedef enum {
        TC2_STOP_TRIGGER_NONE = 0,
        TC2_STOP_TRIGGER_TIMER,
        TC2_STOP_TRIGGER_SENSOR,
        TC2_STOP_TRIGGER_FAILSAFE
} tc2_stop_trigger;

typedef struct {
        int train;
        int start_index;
        int speed;
        /* Actual last acknowledged command; speed remains the trip request. */
        int command_speed;
        int destination_index;
        int state;
        int target_node;
        int conflict_train;
        int conflict_node;
        int route_distance_mm;
        unsigned int plan_generation;
        unsigned int launch_attributed_sequence;
        int ready_at_tick;
        int stop_sent_tick;
        int stop_request_tick;
        int stop_confirmed_tick;
        int watchdog_at_tick;
        int watchdog_margin_ticks;
        int needs_stop_retry;
        unsigned int launch_attribution_unavailable_count;
        int selected_destination_side;
        int current_node;
        int next_sensor_node;
        /*
         * The immutable per-train route stores every physical turnout and
         * its required S/C setting.  These public fields expose the current
         * next and next-next turnout steps; they are deliberately distinct
         * from next_sensor_node.
         */
        int turnout_plan_step_count;
        int turnout_plan_action_count;
        int next_turnout_action_count;
        int next_turnout_switch[
                TC2_DISPATCH_TURNOUT_ACTIONS_PER_STEP];
        char next_turnout_direction[
                TC2_DISPATCH_TURNOUT_ACTIONS_PER_STEP];
        int next_next_turnout_action_count;
        int next_next_turnout_switch[
                TC2_DISPATCH_TURNOUT_ACTIONS_PER_STEP];
        char next_next_turnout_direction[
                TC2_DISPATCH_TURNOUT_ACTIONS_PER_STEP];
        int current_route_offset;
        /*
         * target_* names the physical sensor used to localize the train.
         * The commanded d1-d8 destination is a directed point after that
         * anchor and may lie inside an edge, so its exact scalar distance
         * and the route-node ceiling are recorded separately.
         */
        int target_route_offset;
        int destination_base_offset_mm;
        int destination_speed_correction_mm;
        int destination_offset_mm;
        int destination_distance_mm;
        int destination_route_offset;
        int destination_offset_confirmed;
        int confirmed_distance_mm;
        int64_t estimated_distance_um;
        int remaining_distance_mm;
        int position_estimated;
        int route_reversal_count;
        int reversals_completed;
        int current_leg;
        int missing_sensor_count;
        int spurious_sensor_count;
        int duplicate_sensor_count;
        int journal_lost;
        int hold_active;
        int wait_age_ticks;
        unsigned int queue_sequence;
        unsigned int job_launch_epoch;
        int launch_tick;
        int launch_skew_ticks;
        int braking_at_tick;
        int provisional_prediction;
        int prediction_timing_valid;
        int prediction_velocity_um_per_tick;
        int prediction_stop_distance_um;
        int prediction_stop_distance_mm;
        int prediction_anchor_node;
        int prediction_anchor_tick;
        int prediction_anchor_distance_mm;
        int prediction_command_at_tick;
        /*
         * The generic route-relative predictor is deliberately published
         * separately from the legacy supervised caldispatch fields above.
         * Stage 20 exposes the selected plan for audit/UI/simulation while
         * prediction_execution_active remains false; a later stage may
         * explicitly opt ordinary dispatch into timer execution.
         */
        int prediction_plan_valid;
        int prediction_execution_active;
        int prediction_geometry_evidence;
        int prediction_profile_evidence;
        int prediction_exact_profile;
        int prediction_action;
        int prediction_geometry_anchor_node;
        int prediction_geometry_anchor_route_offset;
        int prediction_geometry_base_offset_mm;
        int prediction_endpoint_node;
        int prediction_endpoint_route_offset;
        int64_t prediction_endpoint_distance_um;
        int64_t prediction_endpoint_offset_um;
        int prediction_command_node;
        int prediction_command_route_offset;
        int64_t prediction_command_distance_um;
        int64_t prediction_command_offset_um;
        int64_t prediction_physical_destination_distance_um;
        int64_t prediction_corrected_endpoint_distance_um;
        int64_t prediction_minimum_reservation_ceiling_distance_um;
        int stop_timing_valid;
        int stop_trigger;
        int failure_reason;
        int sensor_health_fault;
        unsigned int can_tx_completed;
        unsigned int can_tx_failed;
        unsigned int can_tx_timeout;
        unsigned int can_rx_dropped;
        unsigned int can_rx_overflow;
        unsigned int can_turnout_changes;
        int projection_valid;
        uint64_t projection_publication_serial;
        unsigned int projection_plan_generation;
        unsigned int projection_launch_epoch;
        int projection_waypoint_count;
        int traffic_hold_active;
        int traffic_reason;
        int traffic_peer_train;
        int traffic_gap_mm;
        int traffic_stop_threshold_mm;
        int traffic_resume_threshold_mm;
        int64_t traffic_hold_distance_um;
} tc2_dispatch_job_snapshot;

typedef struct {
        tc2_dispatch_job_snapshot jobs[TC2_DISPATCH_MAX_JOBS];
        int job_count;
        unsigned int launch_epoch;
        unsigned char blocked_by_node[TRACK_MAX];
        int blocked_physical_count;
        int batch_ready_at_tick;
        int scheduler_healthy;
} tc2_dispatch_snapshot;

typedef struct {
        unsigned char blocked_by_node[TRACK_MAX];
        int blocked_physical_count;
} tc2_dispatch_blocked_snapshot;

int Tc2DispatchParseStart(const char *label);
int Tc2DispatchParseDestination(const char *label);
const char *Tc2DispatchStartLabel(int start_index);
const char *Tc2DispatchStartNodeName(int start_index);
const char *Tc2DispatchDestinationLabel(int destination_index);
const char *Tc2DispatchDestinationSensorA(int destination_index);
const char *Tc2DispatchDestinationSensorB(int destination_index);
const char *Tc2DispatchStateName(int state);

int Tc2DispatchStage(int tid, int train, int start_index, int speed,
                     int destination_index);
/*
 * Supervised first-pass physical prediction. This intentionally accepts only
 * T14 from A to d7 and cannot coexist with another managed job.
 */
int Tc2DispatchStageCalibration(int tid, int train, int start_index,
                                int speed, int destination_index);
int Tc2DispatchStageFromCurrent(int tid, int train, int speed,
                                int destination_index);
/* Safely stop the same managed train, then replace its route from CURRENT. */
int Tc2DispatchStageReroute(int tid, int train, int speed,
                            int destination_index);
int Tc2DispatchStartAll(int tid);
int Tc2DispatchCancel(int tid, int train);
int Tc2DispatchRemove(int tid, int train);
int Tc2DispatchBlockNode(int tid, int node_index);
int Tc2DispatchUnblockNode(int tid, int node_index);
int Tc2DispatchGetBlockedSnapshot(
        int tid, tc2_dispatch_blocked_snapshot *snapshot);
int Tc2DispatchIsTrainManaged(int tid, int train);
int Tc2DispatchGetSnapshot(int tid, tc2_dispatch_snapshot *snapshot);
int Tc2DispatchGetProjectionHeader(
        int tid, int train,
        tc2_dispatch_projection_header *header);
int Tc2DispatchGetProjectionPage(
        int tid, int train, uint64_t publication_serial,
        uint32_t plan_generation, uint32_t launch_epoch,
        int first_waypoint,
        tc2_dispatch_projection_page *page);

void Tc2DispatchServerTask(void);
void Tc2DispatchTickerTask(void);
void Tc2DispatchSafetyTask(void);

#endif
