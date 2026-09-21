#include "tc2_dispatch.h"

#include "can.h"
#include "clock.h"
#include "events.h"
#include "nameserver.h"
#include "syscall.h"
#include "tasks.h"
#include "tc2_conflict_zone.h"
#include "tc2_motion_model.h"
#include "tc2_prediction_model.h"
#include "tc2_route_projection.h"
#include "tc2_route_monitor.h"
#include "tc2_traffic_safety.h"
#include "track_reservation_server.h"
#include "track_route.h"
#include "train_sensor.h"
#include "timer.h"
#include "util.h"

#ifndef TC2_DISPATCH_CONFLICT_ZONE_REMOVE_TRAIN
#define TC2_DISPATCH_CONFLICT_ZONE_REMOVE_TRAIN \
        Tc2ConflictZoneRemoveTrain
#endif

#ifndef MODE_TC2

static int fallback_streq(const char *left, const char *right) {
        if (!left || !right) return 0;
        while (*left && *right) {
                char a = *left++;
                char b = *right++;
                if (a >= 'a' && a <= 'z') a = (char)(a - 'a' + 'A');
                if (b >= 'a' && b <= 'z') b = (char)(b - 'a' + 'A');
                if (a != b) return 0;
        }
        return *left == 0 && *right == 0;
}

int Tc2DispatchParseStart(const char *label) {
        if (fallback_streq(label, "CURRENT")) {
                return TC2_DISPATCH_START_CURRENT;
        }
        for (int index = 0; index < TC2_DISPATCH_START_COUNT; ++index) {
                const tc2_track_start_definition *definition =
                        Tc2TrackStartDefinition(index);
                if (definition &&
                    fallback_streq(label, definition->label)) {
                        return index;
                }
        }
        return TC2_DISPATCH_START_INVALID;
}

int Tc2DispatchParseDestination(const char *label) {
        if (!label || label[0] != 'd' ||
            label[1] < '1' || label[1] > '8' || label[2] != '\0') {
                return -1;
        }
        for (int index = 0;
             index < TC2_DISPATCH_DESTINATION_COUNT; ++index) {
                const tc2_track_destination_definition *definition =
                        Tc2TrackDestinationDefinition(index);
                if (definition &&
                    fallback_streq(label, definition->label)) {
                        return index;
                }
        }
        return -1;
}

const char *Tc2DispatchStartLabel(int index) {
        if (index == TC2_DISPATCH_START_CURRENT) return "CURRENT";
        const tc2_track_start_definition *definition =
                Tc2TrackStartDefinition(index);
        return definition ? definition->label : "?";
}

const char *Tc2DispatchStartNodeName(int index) {
        const tc2_track_start_definition *definition =
                Tc2TrackStartDefinition(index);
        return definition ? definition->enter_node : "?";
}

const char *Tc2DispatchDestinationLabel(int index) {
        const tc2_track_destination_definition *definition =
                Tc2TrackDestinationDefinition(index);
        return definition ? definition->label : "?";
}

const char *Tc2DispatchDestinationSensorA(int index) {
        const tc2_track_destination_side *side =
                Tc2TrackDestinationSide(index, 0);
        return side ? side->anchor_sensor : "?";
}

const char *Tc2DispatchDestinationSensorB(int index) {
        const tc2_track_destination_side *side =
                Tc2TrackDestinationSide(index, 1);
        return side ? side->anchor_sensor : "?";
}

const char *Tc2DispatchStateName(int state) {
        switch (state) {
        case TC2_JOB_STAGED: return "STAGED";
        case TC2_JOB_WAITING: return "WAIT_ROUTE";
        case TC2_JOB_PREPARING: return "PREPARING";
        case TC2_JOB_READY: return "READY";
        case TC2_JOB_LAUNCHING: return "LAUNCHING";
        case TC2_JOB_RUNNING: return "RUNNING";
        case TC2_JOB_BRAKING: return "BRAKING";
        case TC2_JOB_STOP_UNCONFIRMED: return "STOP_UNCONFIRMED";
        case TC2_JOB_ARRIVED: return "ARRIVED";
        case TC2_JOB_STOPPED: return "STOP_HOLD";
        case TC2_JOB_FAILED: return "FAILED";
        case TC2_JOB_REVERSING: return "REVERSING";
        case TC2_JOB_CANCEL_BRAKING: return "CANCEL_BRAKING";
        case TC2_JOB_TRAFFIC_HOLD: return "TRAFFIC_HOLD";
        default: return "EMPTY";
        }
}

#endif

#ifdef MODE_TC2

#define TC2_DISPATCH_MSG_STAGE 1
#define TC2_DISPATCH_MSG_START_ALL 2
#define TC2_DISPATCH_MSG_CANCEL 3
#define TC2_DISPATCH_MSG_SNAPSHOT 4
#define TC2_DISPATCH_MSG_TICK 5
#define TC2_DISPATCH_MSG_REMOVE 6
#define TC2_DISPATCH_MSG_BLOCK 7
#define TC2_DISPATCH_MSG_UNBLOCK 8
#define TC2_DISPATCH_MSG_BLOCKED_SNAPSHOT 9
#define TC2_DISPATCH_MSG_IS_MANAGED 10
#define TC2_DISPATCH_MSG_STAGE_CALIBRATION 11
#define TC2_DISPATCH_MSG_PROJECTION_HEADER 12
#define TC2_DISPATCH_MSG_PROJECTION_PAGE 13
#define TC2_DISPATCH_REQUEST_FORCE_REVERSE_FIRST 0x1u
#define TC2_DISPATCH_REQUEST_STAGE_FLAGS \
        TC2_DISPATCH_REQUEST_FORCE_REVERSE_FIRST
#define TC2_DISPATCH_TICK_TICKS 2
#define TC2_TURNOUT_SETTLE_TICKS 20
#define TC2_REVERSE_SETTLE_TICKS 20
#define TC2_REVERSAL_PENALTY_MM 600
/* Train 14 is 8 inches long; its pickup is at the geometric centre. */
#define TC2_TRAIN_BODY_MM 203
#define TC2_TRAIN_HALF_LENGTH_MM 102
#define TC2_MAX_SENSOR_SPURIOUS 1
#define TC2_MAX_SENSOR_MISSING 1
#define TC2_MAX_LOCALIZATION_HOPS 16
#define TC2_SUPERVISOR_STALL_EVENTS 200
#define TC2_SUPERVISOR_STALL_USEC 2000000u
#define TC2_SUPERVISOR_RETRY_USEC 250000u
#define TC2_SERVICE_HEARTBEAT_MAX_AGE 6
#define TC2_SERVICE_HEARTBEAT_MAX_FUTURE_SKEW 1
#define TC2_SENSOR_STARTUP_MAX_ATTEMPTS 8
#define TC2_PRECISION_APPROACH_SPEED 20
#define TC2_INTERSECTION_RELEASE_MARGIN_MM 200
#define TC2_AUTHORITY_CAN_BUDGET_TICKS 6

typedef struct {
        int type;
        int train;
        int start_index;
        int speed;
        int destination_index;
        int node_index;
        uint64_t projection_serial;
        uint32_t projection_generation;
        uint32_t projection_launch_epoch;
        int projection_first_waypoint;
        uint32_t reserved;
} tc2_dispatch_request;

typedef struct {
        int status;
        int affected;
} tc2_dispatch_reply;

_Static_assert(sizeof(tc2_dispatch_request) == 48,
               "TC2 dispatch request ABI changed");
_Static_assert(sizeof(tc2_dispatch_projection_waypoint) == 24,
               "TC2 projection waypoint ABI changed");
_Static_assert(sizeof(tc2_dispatch_projection_header) == 96,
               "TC2 projection header ABI changed");
_Static_assert(sizeof(tc2_dispatch_projection_page) == 232,
               "TC2 projection page ABI changed");

typedef enum {
        TC2_STOP_NONE = 0,
        TC2_STOP_DESTINATION,
        TC2_STOP_REVERSAL,
        TC2_STOP_TRAFFIC
} tc2_stop_purpose;

typedef enum {
        TC2_REVERSAL_PHASE_NONE = 0,
        TC2_REVERSAL_PHASE_TURNOUT,
        TC2_REVERSAL_PHASE_SETTLE
} tc2_reversal_phase;

typedef struct {
        int valid;
        tc2_dispatch_projection_header header;
        tc2_dispatch_projection_waypoint
                waypoints[TC2_ROUTE_PROJECTION_MAX_WAYPOINTS];
} tc2_dispatch_projection_publication;

typedef struct {
        track_route route;
        track_route safety_footprint;
        tc2_prediction_plan prediction_plan;
        int prediction_plan_valid;
        /*
         * Every CURRENT continuation carries the preceding physical hold
         * into its atomic plan replacement, protecting the stopped train's
         * tail. If the destination report was not corroborated, the same
         * owner set is also the position-ambiguity footprint and remains
         * until two ordered forward observations localize the new trip.
         */
        track_route carry_ambiguity_footprint;
        int carry_ambiguity_active;
        int carry_position_estimated;
        track_turnout_plan carry_confirmed_turnouts;
        int carry_confirmed_turnouts_valid;
        track_route carry_motion_route;
        int carry_motion_origin_offset;
        int carry_previous_speed;
        int planned_launch_speed;
        /* Explicit operator reroutes must begin with one physical reverse. */
        int force_reverse_first;
        /*
         * A timer/model-confirmed lowercase destination can lie downstream
         * of its last detector.  After a forced leading reverse, the new
         * route's directed origin detector is therefore still physically
         * ahead of the train, even though its graph distance is zero.  Keep
         * that pre-origin distance until the real detector pulse rebases the
         * new motion epoch; never silently treat the detector as observed.
         */
        int carry_preorigin_distance_mm;
        int preorigin_distance_mm;
        int preorigin_sensor_pending;
        int localization_only;
        int localization_hops;
        int zero_motion_arrival_pending;
        int zero_motion_arrival_ready_at_tick;
        int replace_plan_expected;
        unsigned int replace_expected_generation;
        int replace_expected_destination;
        unsigned int stage_attributed_sequence;
        int stage_sensor_baseline_valid;
        tc2_route_monitor monitor;
        int route_valid;
        int start_node;
        int leg_start_offset;
        int leg_end_offset;
        int monitor_terminal_offset;
        int stop_purpose;
        int settle_at_tick;
        int prelaunch_turnout_settle_at_tick;
        int prelaunch_reverse_ready_at_tick;
        int prelaunch_reverse_pending;
        int reverse_ready_at_tick;
        int reversal_phase;
        /*
         * Crossing a reversal is a stopped, two-window handoff.  The
         * incoming authority remains owned while the next short outbound
         * window is acquired and until ordered outbound motion puts the
         * train tail plus 200 mm beyond the reversal sensor.
         */
        track_route reversal_carry_footprint;
        int reversal_carry_active;
        int reversal_carry_origin_offset;
        int approach_sent;
        int command_speed;
        int motion_speed_ceiling;
        int speed_reduction_pending;
        int precision_approach_active;
        int64_t motion_anchor_distance_um;
        int motion_anchor_tick;
        int motion_anchor_from_traffic;
        int64_t traffic_hold_distance_um;
        /*
         * traffic_hold_distance_um is a public route scalar and therefore
         * cannot be negative.  A forced reverse from a virtual destination
         * can, however, stop before its first real detector while its private
         * physical scalar is still in the measured negative prefix.  Preserve
         * that coast endpoint separately until the train resumes or the
         * origin detector rebases the route at zero.
         */
        int64_t preorigin_traffic_hold_distance_um;
        int preorigin_traffic_hold_valid;
        int traffic_resume_speed;
        int traffic_peer_train;
        int traffic_reason;
        int traffic_hold_since_tick;
        /*
         * Fixed A-F trips use a single-owner rolling authority window.
         * The immutable route remains complete for monitoring/UI, while the
         * reservation owns only the contiguous section the train can safely
         * reach before stopping.
         */
        int rolling_authority_active;
        int authority_end_offset;
        int authority_center_ceiling_mm;
        int authority_request_tick;
        unsigned int authority_request_sequence;
        /*
         * FIFO is scoped to the exact next local window being requested.
         * Keeping this footprint separate from the immutable destination route
         * prevents a far-away intersection from holding a train at its origin.
         */
        track_route authority_requested_footprint;
        int authority_requested_footprint_valid;
        int authority_resume_pending;
        /*
         * A moving train may reserve the next short authority window before
         * reaching its present braking boundary.  Reservation ownership is
         * updated first, but the movement ceiling is promoted only after the
         * newly owned turnout suffix is confirmed and mechanically settled.
         */
        int authority_prefetch_active;
        int authority_prefetch_end_offset;
        int authority_prefetch_center_ceiling_mm;
        int authority_prefetch_settle_at_tick;
        int authority_prefetch_rearm_progress_mm;
        /*
         * A CURRENT command issued while collision control is holding the
         * train is staged without replacing the live job.  `go` arms this
         * request; the old job and reservation remain authoritative until a
         * generation-checked replacement reservation commits.
         */
        int retarget_pending;
        int retarget_armed;
        int retarget_speed;
        int retarget_destination_index;
        unsigned int retarget_expected_generation;
        int retarget_expected_destination;
        unsigned int retarget_queue_sequence;
        unsigned int retarget_launch_epoch;
        int retarget_force_reverse_first;
        int head_on_recovery_active;
        int head_on_recovery_cleared;
        /*
         * A rolling-authority continuation remains physically stopped while
         * its turnout suffix settles and its positive-speed batch awaits
         * confirmation.  Dynamic traffic can veto that launch without
         * discarding the newly acquired reservation.
         */
        int prelaunch_traffic_blocked;
        int authority_previous_end_offset;
        int target_seen;
        int target_corroborated;
        int unattributed_spurious_tolerated;
        int reservation_anchor_offset;
        int last_confirmed_tick;
        int observed_velocity_um_per_tick;
        int pending_missing_offset;
        int wait_since_tick;
        int ready_wave_tick;
        unsigned int last_journal_sequence;
        unsigned int launch_epoch;
        unsigned int queue_sequence;
        can_health_t can_baseline;
        int can_baseline_valid;
        unsigned int can_tx_heartbeat;
        unsigned int can_rx_heartbeat;
        int can_tx_heartbeat_tick;
        int can_rx_heartbeat_tick;
        unsigned int prepare_attributed_sequence;
        unsigned int prepare_attribution_unavailable_count;
        unsigned int prepare_unattributed_count;
        unsigned int launch_unattributed_count;
        unsigned int launch_train_attribution_failure;
        unsigned int launch_batch_token;
        int launch_batch_index;
        int launch_wave_tick;
        unsigned int turnout_batch_token;
        int turnout_queue_pending;
        /*
         * Physical turnouts are admitted one conflict zone at a time.  The
         * immutable route plan is separate from rolling track authority: the
         * next zone is a hard FIFO request and next-next is only a soft hint.
         */
        tc2_conflict_zone_route_plan conflict_zone_plan;
        int conflict_zone_plan_valid;
        int conflict_zone_next_step;
        int conflict_zone_hard_step;
        int conflict_zone_soft_step;
        unsigned int conflict_zone_hard_ticket;
        unsigned int conflict_zone_batch_token;
        /*
         * A physical conflict zone may contain several distinct turnout
         * motors (SW5/SW7 and SW153-SW156).  The zone ticket is shared, but
         * the motors must be commanded and confirmed one at a time in route
         * order.  Bind this cursor to both the immutable route step and the
         * FIFO owner ticket so a stale completion can never settle a newer
         * route.
         */
        int conflict_zone_action_step;
        int conflict_zone_action_index;
        unsigned int conflict_zone_action_ticket;
        unsigned int conflict_zone_soft_batch_token;
        int conflict_zone_soft_batch_step;
        int conflict_zone_soft_action_index;
        int conflict_zone_soft_settle_at_tick;
        int conflict_zone_soft_prefetched_step;
        /*
         * A next-next command is authority-free, but a fully confirmed
         * prefetch may be promoted atomically when it becomes the next hard
         * request.  Keep success separate from "already attempted": a
         * failed/cancelled CAN batch must never count as a proved setting.
         */
        int conflict_zone_soft_confirmed_step;
        int conflict_zone_queue_pending;
        int conflict_zone_settle_at_tick;
        int conflict_zone_hold_active;
        /*
         * A CURRENT reverse-first route may begin by leaving the exact
         * physical zone that the same train still occupies.  The old owner
         * ticket remains authority only for that one immediate egress; it
         * must never authorize a later re-entry or a different turnout
         * setting.
         */
        int conflict_zone_origin_egress_active;
        int conflict_zone_origin_egress_step;
        int conflict_zone_origin_egress_zone;
        unsigned int conflict_zone_origin_egress_ticket;
        /*
         * A trimmed CURRENT route can represent the already-occupied
         * origin zone as a zero-length step.  In that case a scalar value
         * of zero is not proof that the train left the zone.  Remember the
         * sensor journal generation at arm time and require a fresh,
         * ordered outbound observation before consuming that cursor.
         */
        unsigned int conflict_zone_origin_egress_sequence;
        int conflict_zone_carry_count;
        int conflict_zone_carry_zone[TC2_CONFLICT_ZONE_COUNT];
        /*
         * A CURRENT replacement can outlive an already-entered old zone.
         * Preserve the old physical boundary nodes: only a later confirmed
         * sensor whose graph distance proves the complete body clear may
         * start the three-second release delay.
         */
        int conflict_zone_carry_entry_node[TC2_CONFLICT_ZONE_COUNT];
        int conflict_zone_carry_exit_node[TC2_CONFLICT_ZONE_COUNT];
        /*
         * Route scalar for an entered zone.  Unlike current_node (which the
         * UI may extrapolate between sensors), this is compared only with
         * monitor.confirmed_distance_mm, so an estimated display position
         * can never release a physical turnout.
         */
        int conflict_zone_carry_exit_distance_mm[
                TC2_CONFLICT_ZONE_COUNT];
        unsigned char
                conflict_zone_carry_scalar_proof_valid[
                        TC2_CONFLICT_ZONE_COUNT];
        unsigned int
                conflict_zone_carry_ticket[TC2_CONFLICT_ZONE_COUNT];
        unsigned char
                conflict_zone_carry_mode[TC2_CONFLICT_ZONE_COUNT];
        unsigned int emergency_stop_token;
        unsigned int sensor_receive_failure_baseline;
        unsigned int sensor_time_failure_baseline;
        int sensor_health_baseline_valid;
        unsigned char
                prepare_sensor_state[TRAIN_SENSOR_COUNT];
} tc2_dispatch_runtime;

static track_node dispatch_track[TRACK_MAX];
static tc2_dispatch_job_snapshot dispatch_jobs[TC2_DISPATCH_MAX_JOBS];
static tc2_dispatch_runtime dispatch_runtime[TC2_DISPATCH_MAX_JOBS];
static tc2_dispatch_projection_publication
        dispatch_projection_publications[TC2_DISPATCH_MAX_JOBS];
static tc2_dispatch_job_snapshot dispatch_retarget_job_backup;
static tc2_dispatch_runtime dispatch_retarget_runtime_backup;
static tc2_dispatch_projection_publication
        dispatch_retarget_projection_backup;
/*
 * The conflict-zone table is dispatcher-task-owned.  A CURRENT replacement
 * is prepared synchronously by that single writer, so keeping one global
 * scratch copy makes its pre-CAS portion a real transaction without putting
 * the roughly multi-kilobyte table on the embedded task stack.  In
 * particular, fail_job() may remove the held train's queued hard/soft
 * requests while validating a candidate.  If the reservation generation did
 * not change, restoring this copy preserves the original immutable FIFO
 * tickets as well as the old job/runtime bytes.
 */
static tc2_conflict_zone_table
        dispatch_retarget_conflict_zone_backup;
/*
 * remove is a two-service commit: the dispatcher-owned conflict-zone table
 * and the reservation server must either both forget the physical train or
 * neither may do so.  Keep this task-owned scratch copy off the embedded
 * stack so a reservation-service failure can restore the exact zone owner,
 * waiter, and FIFO-ticket state before the operator retries remove.
 */
static tc2_conflict_zone_table
        dispatch_remove_conflict_zone_backup;
/* True only while a CURRENT candidate is being validated before its CAS. */
static int dispatch_retarget_candidate_pre_cas;
static tc2_route_projection dispatch_projection_scratch;
static uint64_t dispatch_projection_serial;
static unsigned char dispatch_blocked_by_node[TRACK_MAX];
static unsigned int dispatch_launch_epoch;
static unsigned int dispatch_queue_sequence;
static int dispatch_batch_ready_at_tick;
static int dispatch_scheduler_healthy;
static int dispatch_can_tid = -1;
static int dispatch_sensor_tid = -1;
static int dispatch_reservation_tid = -1;
static volatile unsigned int dispatch_heartbeat;

static int discard_future_conflict_zone_work(int slot);
static volatile int dispatch_last_logical_time;
static volatile int dispatch_supervisor_trip;
static volatile unsigned long long dispatch_last_progress_usec;
static tc2_conflict_zone_table dispatch_conflict_zones;
static int dispatch_conflict_zones_initialized;

static int resume_traffic_hold(
        int slot, int now, int expected_reason);
static int command_train_speed_with_retry(int train, int speed);
static int traffic_resume_fits_current_authority(
        int slot, int speed);
static int authority_resume_speed_for_remaining(
        int requested_speed, int remaining_mm);
static int snapshot_owned_conflict_zone_carries(int slot);
static int abandon_authority_prefetch(int slot);

static void initialize_conflict_zone_table(void) {
        Tc2ConflictZoneInit(&dispatch_conflict_zones);
        dispatch_conflict_zones_initialized = 1;
}

/*
 * Runtime initialization is explicit and monotonic.  In particular, an
 * empty/temporarily invalid route plan is not proof that every physical
 * owner, waiter, release delay, or CURRENT carry has cleared.
 */
static void ensure_conflict_zone_table(void) {
        if (!dispatch_conflict_zones_initialized) {
                initialize_conflict_zone_table();
        }
}

static int dispatch_streq(const char *left, const char *right) {
        if (!left || !right) return 0;
        while (*left && *right) {
                char a = *left++;
                char b = *right++;
                if (a >= 'a' && a <= 'z') a = (char)(a - 'a' + 'A');
                if (b >= 'a' && b <= 'z') b = (char)(b - 'a' + 'A');
                if (a != b) return 0;
        }
        return *left == 0 && *right == 0;
}

/*
 * Track D's physically commissioned fleet is deliberately closed.  Keeping
 * this check at the server boundary matters: startup atomically stops these
 * same four locomotives, so accepting any other address would allow a train
 * whose initial physical state was never established to enter managed
 * traffic.  The conflict-zone table itself remains generic and independently
 * testable; production dispatch is restricted to the installed fleet.
 */
static int dispatch_train_is_supported(int train) {
        return train == 14 || train == 15 ||
               train == 17 || train == 18;
}

int Tc2DispatchParseStart(const char *label) {
        if (dispatch_streq(label, "CURRENT")) {
                return TC2_DISPATCH_START_CURRENT;
        }
        for (int i = 0; i < TC2_DISPATCH_START_COUNT; ++i) {
                const tc2_track_start_definition *definition =
                        Tc2TrackStartDefinition(i);
                if (definition &&
                    dispatch_streq(label, definition->label)) {
                        return i;
                }
        }
        return TC2_DISPATCH_START_INVALID;
}

int Tc2DispatchParseDestination(const char *label) {
        if (!label || label[0] != 'd' ||
            label[1] < '1' || label[1] > '8' || label[2] != '\0') {
                return -1;
        }
        for (int i = 0; i < TC2_DISPATCH_DESTINATION_COUNT; ++i) {
                const tc2_track_destination_definition *definition =
                        Tc2TrackDestinationDefinition(i);
                if (definition &&
                    dispatch_streq(label, definition->label)) {
                        return i;
                }
        }
        return -1;
}

const char *Tc2DispatchStartLabel(int index) {
        if (index == TC2_DISPATCH_START_CURRENT) return "CURRENT";
        const tc2_track_start_definition *definition =
                Tc2TrackStartDefinition(index);
        return definition ? definition->label : "?";
}

const char *Tc2DispatchStartNodeName(int index) {
        const tc2_track_start_definition *definition =
                Tc2TrackStartDefinition(index);
        return definition ? definition->enter_node : "?";
}

const char *Tc2DispatchDestinationLabel(int index) {
        const tc2_track_destination_definition *definition =
                Tc2TrackDestinationDefinition(index);
        return definition ? definition->label : "?";
}

const char *Tc2DispatchDestinationSensorA(int index) {
        const tc2_track_destination_side *side =
                Tc2TrackDestinationSide(index, 0);
        return side ? side->anchor_sensor : "?";
}

const char *Tc2DispatchDestinationSensorB(int index) {
        const tc2_track_destination_side *side =
                Tc2TrackDestinationSide(index, 1);
        return side ? side->anchor_sensor : "?";
}

const char *Tc2DispatchStateName(int state) {
        switch (state) {
        case TC2_JOB_STAGED: return "STAGED";
        case TC2_JOB_WAITING: return "WAIT_ROUTE";
        case TC2_JOB_PREPARING: return "PREPARING";
        case TC2_JOB_READY: return "READY";
        case TC2_JOB_LAUNCHING: return "LAUNCHING";
        case TC2_JOB_RUNNING: return "RUNNING";
        case TC2_JOB_BRAKING: return "BRAKING";
        case TC2_JOB_STOP_UNCONFIRMED: return "STOP_UNCONFIRMED";
        case TC2_JOB_ARRIVED: return "ARRIVED";
        case TC2_JOB_STOPPED: return "STOP_HOLD";
        case TC2_JOB_FAILED: return "FAILED";
        case TC2_JOB_REVERSING: return "REVERSING";
        case TC2_JOB_CANCEL_BRAKING: return "CANCEL_BRAKING";
        case TC2_JOB_TRAFFIC_HOLD: return "TRAFFIC_HOLD";
        default: return "EMPTY";
        }
}

static int tick_reached(int now, int deadline) {
        return (unsigned int)now - (unsigned int)deadline <
                0x80000000u;
}

static int tick_after(int now, int delta) {
        return (int)((unsigned int)now + (unsigned int)delta);
}

static int tick_age(int now, int then) {
        unsigned int age = (unsigned int)now - (unsigned int)then;
        return age > 0x7fffffffu ? 0x7fffffff : (int)age;
}

/*
 * A same-priority sensor courier can timestamp an event one clock tick after
 * the dispatch tick's cached `now`.  That bounded future skew means zero
 * elapsed motion, not an almost-2^32-tick journey.  Larger backward/future
 * values are incoherent and fail closed.
 */
static int motion_tick_age(int now, int then) {
        unsigned int age =
                (unsigned int)now - (unsigned int)then;
        if (age <= 0x7fffffffu) return (int)age;
        unsigned int future =
                (unsigned int)then - (unsigned int)now;
        return future <= 1u ? 0 : -1;
}

static int service_heartbeat_is_fresh(int now, int heartbeat) {
        if (now < 0 || heartbeat < 0) return 0;

        unsigned int age =
                (unsigned int)now - (unsigned int)heartbeat;
        if (age <= TC2_SERVICE_HEARTBEAT_MAX_AGE) return 1;

        /*
         * The dispatch tick caches `now`, while an equal-priority courier
         * timestamps its heartbeat through a separate ClockServer request.
         * It can therefore publish N+1 before this tick finishes using N.
         * Treat only that bounded one-tick future skew as fresh. Larger
         * future values still fail closed as corrupt or incoherent time.
         */
        unsigned int future =
                (unsigned int)heartbeat - (unsigned int)now;
        return future <=
                TC2_SERVICE_HEARTBEAT_MAX_FUTURE_SKEW;
}

static int sequence_before(unsigned int left, unsigned int right) {
        return left != right &&
                right - left < 0x80000000u;
}

static unsigned int next_dispatch_queue_sequence(void) {
        ++dispatch_queue_sequence;
        if (dispatch_queue_sequence == 0) {
                ++dispatch_queue_sequence;
        }
        return dispatch_queue_sequence;
}

static int physical_reverse_index(int node) {
        if (node < 0 || node >= TRACK_MAX ||
            !dispatch_track[node].reverse) {
                return -1;
        }
        int reverse = (int)(dispatch_track[node].reverse - dispatch_track);
        return reverse >= 0 && reverse < TRACK_MAX ? reverse : -1;
}

static void clear_runtime(tc2_dispatch_runtime *runtime) {
        runtime->route.node_count = 0;
        runtime->route.distance_mm = 0;
        runtime->route.optimization_cost_mm = 0;
        runtime->route.reversal_count = 0;
        runtime->safety_footprint.node_count = 0;
        runtime->safety_footprint.distance_mm = 0;
        runtime->safety_footprint.optimization_cost_mm = 0;
        runtime->safety_footprint.reversal_count = 0;
        util_zero_bytes(
                (volatile unsigned char *)
                        &runtime->prediction_plan,
                (unsigned int)sizeof(runtime->prediction_plan));
        runtime->prediction_plan_valid = 0;
        runtime->carry_ambiguity_footprint.node_count = 0;
        runtime->carry_ambiguity_footprint.distance_mm = 0;
        runtime->carry_ambiguity_footprint.optimization_cost_mm = 0;
        runtime->carry_ambiguity_footprint.reversal_count = 0;
        runtime->carry_ambiguity_active = 0;
        runtime->carry_position_estimated = 0;
        runtime->carry_confirmed_turnouts.action_count = 0;
        runtime->carry_confirmed_turnouts_valid = 0;
        runtime->carry_motion_route.node_count = 0;
        runtime->carry_motion_route.distance_mm = 0;
        runtime->carry_motion_route.optimization_cost_mm = 0;
        runtime->carry_motion_route.reversal_count = 0;
        runtime->carry_motion_origin_offset = -1;
        runtime->carry_previous_speed = 0;
        runtime->planned_launch_speed = 0;
        runtime->force_reverse_first = 0;
        runtime->carry_preorigin_distance_mm = 0;
        runtime->preorigin_distance_mm = 0;
        runtime->preorigin_sensor_pending = 0;
        runtime->localization_only = 0;
        runtime->localization_hops = 0;
        runtime->zero_motion_arrival_pending = 0;
        runtime->zero_motion_arrival_ready_at_tick = -1;
        runtime->replace_plan_expected = 0;
        runtime->replace_expected_generation = 0;
        runtime->replace_expected_destination = -1;
        runtime->stage_attributed_sequence = 0;
        runtime->stage_sensor_baseline_valid = 0;
        runtime->route_valid = 0;
        runtime->start_node = -1;
        runtime->leg_start_offset = 0;
        runtime->leg_end_offset = 0;
        runtime->monitor_terminal_offset = 0;
        runtime->stop_purpose = TC2_STOP_NONE;
        runtime->settle_at_tick = -1;
        runtime->prelaunch_turnout_settle_at_tick = -1;
        runtime->prelaunch_reverse_ready_at_tick = -1;
        runtime->prelaunch_reverse_pending = 0;
        runtime->reverse_ready_at_tick = -1;
        runtime->reversal_phase = TC2_REVERSAL_PHASE_NONE;
        runtime->reversal_carry_footprint.node_count = 0;
        runtime->reversal_carry_footprint.distance_mm = 0;
        runtime->reversal_carry_footprint.optimization_cost_mm = 0;
        runtime->reversal_carry_footprint.reversal_count = 0;
        runtime->reversal_carry_active = 0;
        runtime->reversal_carry_origin_offset = -1;
        runtime->approach_sent = 0;
        runtime->command_speed = 0;
        runtime->motion_speed_ceiling = 0;
        runtime->speed_reduction_pending = 0;
        runtime->precision_approach_active = 0;
        runtime->motion_anchor_distance_um = 0;
        runtime->motion_anchor_tick = -1;
        runtime->motion_anchor_from_traffic = 0;
        runtime->traffic_hold_distance_um = -1;
        runtime->preorigin_traffic_hold_distance_um = 0;
        runtime->preorigin_traffic_hold_valid = 0;
        runtime->traffic_resume_speed = 0;
        runtime->traffic_peer_train = 0;
        runtime->traffic_reason = TC2_TRAFFIC_NONE;
        runtime->traffic_hold_since_tick = -1;
        runtime->rolling_authority_active = 0;
        runtime->authority_end_offset = -1;
        runtime->authority_center_ceiling_mm = -1;
        runtime->authority_request_tick = -1;
        runtime->authority_request_sequence = 0;
        runtime->authority_requested_footprint.node_count = 0;
        runtime->authority_requested_footprint.distance_mm = 0;
        runtime->authority_requested_footprint.optimization_cost_mm = 0;
        runtime->authority_requested_footprint.reversal_count = 0;
        runtime->authority_requested_footprint_valid = 0;
        runtime->authority_resume_pending = 0;
        runtime->authority_prefetch_active = 0;
        runtime->authority_prefetch_end_offset = -1;
        runtime->authority_prefetch_center_ceiling_mm = -1;
        runtime->authority_prefetch_settle_at_tick = -1;
        runtime->authority_prefetch_rearm_progress_mm = -1;
        runtime->retarget_pending = 0;
        runtime->retarget_armed = 0;
        runtime->retarget_speed = 0;
        runtime->retarget_destination_index = -1;
        runtime->retarget_expected_generation = 0;
        runtime->retarget_expected_destination = -1;
        runtime->retarget_queue_sequence = 0;
        runtime->retarget_launch_epoch = 0;
        runtime->retarget_force_reverse_first = 0;
        runtime->head_on_recovery_active = 0;
        runtime->head_on_recovery_cleared = 0;
        runtime->prelaunch_traffic_blocked = 0;
        runtime->authority_previous_end_offset = -1;
        runtime->target_seen = 0;
        runtime->target_corroborated = 0;
        runtime->unattributed_spurious_tolerated = 0;
        runtime->reservation_anchor_offset = 0;
        runtime->last_confirmed_tick = -1;
        runtime->observed_velocity_um_per_tick = -1;
        runtime->pending_missing_offset = -1;
        runtime->wait_since_tick = -1;
        runtime->ready_wave_tick = -1;
        runtime->last_journal_sequence = 0;
        runtime->launch_epoch = 0;
        runtime->queue_sequence = 0;
        runtime->can_baseline_valid = 0;
        runtime->can_tx_heartbeat = 0;
        runtime->can_rx_heartbeat = 0;
        runtime->can_tx_heartbeat_tick = -1;
        runtime->can_rx_heartbeat_tick = -1;
        runtime->prepare_attributed_sequence = 0;
        runtime->prepare_attribution_unavailable_count = 0;
        runtime->prepare_unattributed_count = 0;
        runtime->launch_unattributed_count = 0;
        runtime->launch_train_attribution_failure = 0;
        runtime->launch_batch_token = 0;
        runtime->launch_batch_index = -1;
        runtime->launch_wave_tick = -1;
        runtime->turnout_batch_token = 0;
        runtime->turnout_queue_pending = 0;
        runtime->conflict_zone_plan.step_count = 0;
        runtime->conflict_zone_plan_valid = 0;
        runtime->conflict_zone_next_step = 0;
        runtime->conflict_zone_hard_step = -1;
        runtime->conflict_zone_soft_step = -1;
        runtime->conflict_zone_hard_ticket = 0;
        runtime->conflict_zone_batch_token = 0;
        runtime->conflict_zone_action_step = -1;
        runtime->conflict_zone_action_index = 0;
        runtime->conflict_zone_action_ticket = 0;
        runtime->conflict_zone_soft_batch_token = 0;
        runtime->conflict_zone_soft_batch_step = -1;
        runtime->conflict_zone_soft_action_index = 0;
        runtime->conflict_zone_soft_settle_at_tick = -1;
        runtime->conflict_zone_soft_prefetched_step = -1;
        runtime->conflict_zone_soft_confirmed_step = -1;
        runtime->conflict_zone_queue_pending = 0;
        runtime->conflict_zone_settle_at_tick = -1;
        runtime->conflict_zone_hold_active = 0;
        runtime->conflict_zone_origin_egress_active = 0;
        runtime->conflict_zone_origin_egress_step = -1;
        runtime->conflict_zone_origin_egress_zone =
                TC2_CONFLICT_ZONE_NONE;
        runtime->conflict_zone_origin_egress_ticket = 0;
        runtime->conflict_zone_origin_egress_sequence = 0;
        runtime->conflict_zone_carry_count = 0;
        for (int zone = 0; zone < TC2_CONFLICT_ZONE_COUNT; ++zone) {
                runtime->conflict_zone_carry_zone[zone] =
                        TC2_CONFLICT_ZONE_NONE;
                runtime->conflict_zone_carry_entry_node[zone] = -1;
                runtime->conflict_zone_carry_exit_node[zone] = -1;
                runtime->conflict_zone_carry_exit_distance_mm[zone] = -1;
                runtime->conflict_zone_carry_scalar_proof_valid[zone] = 0;
                runtime->conflict_zone_carry_ticket[zone] = 0;
                runtime->conflict_zone_carry_mode[zone] = 0;
        }
        runtime->emergency_stop_token = 0;
        runtime->sensor_receive_failure_baseline = 0;
        runtime->sensor_time_failure_baseline = 0;
        runtime->sensor_health_baseline_valid = 0;
        /*
         * Keep this bounded clear in a separate translation unit.  GCC 12's
         * AArch64 -O3 bounds analysis otherwise emits a false
         * -Wstringop-overflow diagnostic for this valid 80-byte array.
         */
        util_zero_bytes(runtime->prepare_sensor_state,
                        (unsigned int)sizeof(
                                runtime->prepare_sensor_state));
}

static void clear_authority_request(
        tc2_dispatch_runtime *runtime) {
        if (!runtime) return;
        runtime->authority_request_tick = -1;
        runtime->authority_request_sequence = 0;
        runtime->authority_requested_footprint.node_count = 0;
        runtime->authority_requested_footprint.distance_mm = 0;
        runtime->authority_requested_footprint.optimization_cost_mm = 0;
        runtime->authority_requested_footprint.reversal_count = 0;
        runtime->authority_requested_footprint_valid = 0;
}

static int begin_authority_request(
        int slot, int now,
        const track_route *requested_footprint) {
        if (slot < 0 || slot >= TC2_DISPATCH_MAX_JOBS ||
            now < 0) {
                return -1;
        }
        tc2_dispatch_runtime *runtime =
                &dispatch_runtime[slot];
        if (runtime->authority_request_tick < 0) {
                runtime->authority_request_tick = now;
                runtime->authority_request_sequence =
                        next_dispatch_queue_sequence();
        }
        if (requested_footprint) {
                if (requested_footprint->node_count < 1 ||
                    requested_footprint->node_count > TRACK_MAX) {
                        return -1;
                }
                runtime->authority_requested_footprint =
                        *requested_footprint;
                runtime->authority_requested_footprint_valid = 1;
        }
        return 0;
}

static void clear_job(tc2_dispatch_job_snapshot *job) {
        job->train = 0;
        job->start_index = TC2_DISPATCH_START_INVALID;
        job->speed = 0;
        job->command_speed = 0;
        job->destination_index = -1;
        job->state = TC2_JOB_EMPTY;
        job->target_node = -1;
        job->conflict_train = 0;
        job->conflict_node = -1;
        job->route_distance_mm = 0;
        job->plan_generation = 0;
        job->launch_attributed_sequence = 0;
        job->ready_at_tick = -1;
        job->stop_sent_tick = -1;
        job->stop_request_tick = -1;
        job->stop_confirmed_tick = -1;
        job->watchdog_at_tick = -1;
        job->watchdog_margin_ticks = 0;
        job->needs_stop_retry = 0;
        job->launch_attribution_unavailable_count = 0;
        job->selected_destination_side = -1;
        job->current_node = -1;
        job->next_sensor_node = -1;
        job->turnout_plan_step_count = 0;
        job->turnout_plan_action_count = 0;
        job->next_turnout_action_count = 0;
        job->next_next_turnout_action_count = 0;
        for (int action = 0;
             action < TC2_DISPATCH_TURNOUT_ACTIONS_PER_STEP; ++action) {
                job->next_turnout_switch[action] = -1;
                job->next_turnout_direction[action] = 0;
                job->next_next_turnout_switch[action] = -1;
                job->next_next_turnout_direction[action] = 0;
        }
        job->current_route_offset = -1;
        job->target_route_offset = -1;
        job->destination_base_offset_mm = 0;
        job->destination_speed_correction_mm = 0;
        job->destination_offset_mm = 0;
        job->destination_distance_mm = 0;
        job->destination_route_offset = -1;
        job->destination_offset_confirmed = 0;
        job->confirmed_distance_mm = 0;
        job->estimated_distance_um = -1;
        job->remaining_distance_mm = 0;
        job->position_estimated = 0;
        job->route_reversal_count = 0;
        job->reversals_completed = 0;
        job->current_leg = 0;
        job->missing_sensor_count = 0;
        job->spurious_sensor_count = 0;
        job->duplicate_sensor_count = 0;
        job->journal_lost = 0;
        job->hold_active = 0;
        job->wait_age_ticks = 0;
        job->queue_sequence = 0;
        job->job_launch_epoch = 0;
        job->launch_tick = -1;
        job->launch_skew_ticks = -1;
        job->braking_at_tick = -1;
        job->provisional_prediction = 0;
        job->prediction_timing_valid = 0;
        job->prediction_velocity_um_per_tick = -1;
        job->prediction_stop_distance_um = -1;
        job->prediction_stop_distance_mm = -1;
        job->prediction_anchor_node = -1;
        job->prediction_anchor_tick = -1;
        job->prediction_anchor_distance_mm = -1;
        job->prediction_command_at_tick = -1;
        job->prediction_plan_valid = 0;
        job->prediction_execution_active = 0;
        job->prediction_geometry_evidence = -1;
        job->prediction_profile_evidence = -1;
        job->prediction_exact_profile = 0;
        job->prediction_action = -1;
        job->prediction_geometry_anchor_node = -1;
        job->prediction_geometry_anchor_route_offset = -1;
        job->prediction_geometry_base_offset_mm = 0;
        job->prediction_endpoint_node = -1;
        job->prediction_endpoint_route_offset = -1;
        job->prediction_endpoint_distance_um = -1;
        job->prediction_endpoint_offset_um = -1;
        job->prediction_command_node = -1;
        job->prediction_command_route_offset = -1;
        job->prediction_command_distance_um = -1;
        job->prediction_command_offset_um = -1;
        job->prediction_physical_destination_distance_um = -1;
        job->prediction_corrected_endpoint_distance_um = -1;
        job->prediction_minimum_reservation_ceiling_distance_um = -1;
        job->stop_timing_valid = 0;
        job->stop_trigger = TC2_STOP_TRIGGER_NONE;
        job->failure_reason = TC2_FAILURE_NONE;
        job->sensor_health_fault = TC2_SENSOR_HEALTH_OK;
        job->can_tx_completed = 0;
        job->can_tx_failed = 0;
        job->can_tx_timeout = 0;
        job->can_rx_dropped = 0;
        job->can_rx_overflow = 0;
        job->can_turnout_changes = 0;
        job->projection_valid = 0;
        job->projection_publication_serial = 0;
        job->projection_plan_generation = 0;
        job->projection_launch_epoch = 0;
        job->projection_waypoint_count = 0;
        job->traffic_hold_active = 0;
        job->traffic_reason = TC2_TRAFFIC_NONE;
        job->traffic_peer_train = 0;
        job->traffic_gap_mm = -1;
        job->traffic_stop_threshold_mm = -1;
        job->traffic_resume_threshold_mm = -1;
        job->traffic_hold_distance_um = -1;
}

static void clear_prediction_snapshot(
        tc2_dispatch_job_snapshot *job) {
        if (!job) return;
        job->prediction_plan_valid = 0;
        job->prediction_execution_active = 0;
        job->prediction_geometry_evidence = -1;
        job->prediction_profile_evidence = -1;
        job->prediction_exact_profile = 0;
        job->prediction_action = -1;
        job->prediction_geometry_anchor_node = -1;
        job->prediction_geometry_anchor_route_offset = -1;
        job->prediction_geometry_base_offset_mm = 0;
        job->prediction_endpoint_node = -1;
        job->prediction_endpoint_route_offset = -1;
        job->prediction_endpoint_distance_um = -1;
        job->prediction_endpoint_offset_um = -1;
        job->prediction_command_node = -1;
        job->prediction_command_route_offset = -1;
        job->prediction_command_distance_um = -1;
        job->prediction_command_offset_um = -1;
        job->prediction_physical_destination_distance_um = -1;
        job->prediction_corrected_endpoint_distance_um = -1;
        job->prediction_minimum_reservation_ceiling_distance_um = -1;
}

static void publish_prediction_snapshot(
        tc2_dispatch_job_snapshot *job,
        const tc2_dispatch_runtime *runtime) {
        clear_prediction_snapshot(job);
        if (!job || !runtime ||
            !runtime->prediction_plan_valid) {
                return;
        }
        const tc2_prediction_plan *plan =
                &runtime->prediction_plan;
        job->prediction_plan_valid = 1;
        /*
         * The point predictor controls only the final speed-zero deadline.
         * The independently larger conservative footprint remains the
         * reservation/collision authority.
         */
        job->prediction_execution_active = 1;
        job->prediction_geometry_evidence =
                (int)plan->geometry_evidence;
        job->prediction_profile_evidence =
                (int)plan->profile_evidence;
        job->prediction_exact_profile =
                plan->exact_profile;
        job->prediction_action = (int)plan->action;
        job->prediction_geometry_anchor_node =
                plan->geometry_anchor_node;
        job->prediction_geometry_anchor_route_offset =
                plan->geometry_anchor_route_offset;
        job->prediction_geometry_base_offset_mm =
                plan->geometry_base_offset_mm;
        job->prediction_endpoint_node =
                plan->endpoint_anchor_node;
        job->prediction_endpoint_route_offset =
                plan->endpoint_anchor_route_offset;
        job->prediction_endpoint_distance_um =
                plan->endpoint_distance_um;
        job->prediction_endpoint_offset_um =
                plan->endpoint_offset_um;
        job->prediction_command_node =
                plan->command_anchor_node;
        job->prediction_command_route_offset =
                plan->command_anchor_route_offset;
        job->prediction_command_distance_um =
                plan->command_distance_um;
        job->prediction_command_offset_um =
                plan->command_offset_um;
        job->prediction_physical_destination_distance_um =
                plan->physical_destination_distance_um;
        job->prediction_corrected_endpoint_distance_um =
                plan->corrected_endpoint_distance_um;
        job->prediction_minimum_reservation_ceiling_distance_um =
                plan->minimum_reservation_ceiling_distance_um;
}

static int find_train_job(int train) {
        for (int slot = 0; slot < TC2_DISPATCH_MAX_JOBS; ++slot) {
                if (dispatch_jobs[slot].state != TC2_JOB_EMPTY &&
                    dispatch_jobs[slot].train == train) {
                        return slot;
                }
        }
        return -1;
}

static void clear_projection_tokens(
        tc2_dispatch_job_snapshot *job) {
        if (!job) return;
        job->projection_valid = 0;
        job->projection_publication_serial = 0;
        job->projection_plan_generation = 0;
        job->projection_launch_epoch = 0;
        job->projection_waypoint_count = 0;
}

static void initialize_projection_publication(int slot) {
        if (slot < 0 || slot >= TC2_DISPATCH_MAX_JOBS) return;
        util_zero_bytes(
                (volatile unsigned char *)
                        &dispatch_projection_publications[slot],
                (unsigned int)sizeof(
                        dispatch_projection_publications[slot]));
}

static void invalidate_projection(int slot) {
        if (slot < 0 || slot >= TC2_DISPATCH_MAX_JOBS) return;
        /*
         * This valid bit is the publication commit marker.  Clear it before
         * touching the job's summary token so a reader can never assemble a
         * route across cancel/failure/re-stage boundaries.
         */
        dispatch_projection_publications[slot].valid = 0;
        dispatch_projection_publications[slot].header.valid = 0;
        clear_projection_tokens(&dispatch_jobs[slot]);
}

static uint64_t next_projection_serial(void) {
        ++dispatch_projection_serial;
        if (dispatch_projection_serial == 0) {
                ++dispatch_projection_serial;
        }
        return dispatch_projection_serial;
}

static int signed_fits_16(int value) {
        return value >= -32768 && value <= 32767;
}

static int signed_fits_8(int value) {
        return value >= -128 && value <= 127;
}

static int copy_projection_waypoint(
        const tc2_route_waypoint *source,
        tc2_dispatch_projection_waypoint *destination) {
        if (!source || !destination ||
            !signed_fits_16(source->route_offset) ||
            !signed_fits_16(source->graph_node) ||
            !signed_fits_16(source->sensor_index) ||
            !signed_fits_16(source->switch_number) ||
            !signed_fits_8((int)source->kind) ||
            !signed_fits_8(source->turnout_direction) ||
            !signed_fits_8(source->destination_index)) {
                return -1;
        }
        destination->distance_um = source->distance_um;
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
        destination->ui_column = source->ui_column;
        destination->ui_width = source->ui_width;
        destination->reserved = 0;
        return 0;
}

/*
 * Tc2RouteProjectionBuildCurrent() expresses distances in the selected
 * graph route's scalar domain.  A forced reverse from an estimated virtual
 * endpoint has one additional physical prefix before graph route[1] is
 * reached.  Keep validation in the canonical graph domain, then translate
 * only the immutable UI publication into the physical domain.  The route,
 * reservation and monitor continue to use their ordinary unshifted scalar.
 */
static int shift_current_projection_for_preorigin(
        tc2_route_projection *projection, int prefix_mm) {
        if (!projection || !projection->valid || prefix_mm <= 0 ||
            projection->waypoint_count < 1 ||
            projection->waypoint_count >
                    TC2_ROUTE_PROJECTION_MAX_WAYPOINTS) {
                return -1;
        }
        int64_t shift_um = (int64_t)prefix_mm * 1000;
        if (projection->physical_destination_distance_um < 0 ||
            projection->physical_destination_distance_um >
                    INT64_MAX - shift_um) {
                return -1;
        }
        for (int waypoint = 0;
             waypoint < projection->waypoint_count; ++waypoint) {
                if (projection->waypoints[waypoint].distance_um < 0 ||
                    projection->waypoints[waypoint].distance_um >
                            INT64_MAX - shift_um) {
                        return -1;
                }
        }
        projection->physical_destination_distance_um += shift_um;
        for (int waypoint = 0;
             waypoint < projection->waypoint_count; ++waypoint) {
                projection->waypoints[waypoint].distance_um += shift_um;
        }
        return 0;
}

static int publish_projection(
        int slot, const tc2_route_projection *projection) {
        if (slot < 0 || slot >= TC2_DISPATCH_MAX_JOBS ||
            !projection || !projection->valid ||
            projection->waypoint_count < 1 ||
            projection->waypoint_count >
                    TC2_ROUTE_PROJECTION_MAX_WAYPOINTS) {
                return -1;
        }
        tc2_dispatch_job_snapshot *job = &dispatch_jobs[slot];
        tc2_dispatch_projection_publication *publication =
                &dispatch_projection_publications[slot];
        uint64_t serial = next_projection_serial();

        initialize_projection_publication(slot);
        publication->header.status =
                TC2_DISPATCH_PROJECTION_OK;
        publication->header.valid = 0;
        publication->header.train = job->train;
        publication->header.job_state = job->state;
        publication->header.publication_serial = serial;
        publication->header.physical_destination_distance_um =
                projection->physical_destination_distance_um;
        publication->header.plan_generation =
                job->plan_generation;
        publication->header.launch_epoch =
                job->job_launch_epoch;
        publication->header.scheduler_healthy =
                dispatch_scheduler_healthy;
        publication->header.start_index =
                projection->request.start_index;
        publication->header.destination_index =
                projection->request.destination_index;
        publication->header.destination_side =
                projection->request.destination_side;
        publication->header.speed =
                projection->request.speed;
        publication->header.waypoint_count =
                projection->waypoint_count;
        publication->header.sensor_count =
                projection->sensor_count;
        publication->header.turnout_count =
                projection->turnout_count;
        publication->header.reversal_count =
                projection->reversal_count;
        publication->header.geometry_anchor_route_offset =
                projection->geometry_anchor_route_offset;
        publication->header.physical_destination_route_offset =
                projection->physical_destination_route_offset;
        publication->header.last_visible_route_offset =
                projection->last_visible_route_offset;
        publication->header.operator_destination_waypoint_index =
                projection->operator_destination_waypoint_index;
        publication->header.physical_destination_offset_mm =
                projection->physical_destination_offset_mm;

        for (int waypoint = 0;
             waypoint < projection->waypoint_count; ++waypoint) {
                if (copy_projection_waypoint(
                            &projection->waypoints[waypoint],
                            &publication->waypoints[waypoint]) < 0) {
                        initialize_projection_publication(slot);
                        clear_projection_tokens(job);
                        return -1;
                }
        }

        /*
         * Commit order is deliberate: immutable payload first, then the
         * public header bit, then the publication bit, and finally the job's
         * summary token.  The dispatcher is the sole writer.
         */
        publication->header.valid = 1;
        publication->valid = 1;
        job->projection_publication_serial = serial;
        job->projection_plan_generation =
                job->plan_generation;
        job->projection_launch_epoch =
                job->job_launch_epoch;
        job->projection_waypoint_count =
                projection->waypoint_count;
        job->projection_valid = 1;
        return 0;
}

static int projection_header_for_train(
        int train, tc2_dispatch_projection_header *header) {
        if (!header) {
                return TC2_DISPATCH_PROJECTION_INVALID_ARGUMENT;
        }
        util_zero_bytes(
                (volatile unsigned char *)header,
                (unsigned int)sizeof(*header));
        header->status =
                TC2_DISPATCH_PROJECTION_INVALID_ARGUMENT;
        if (train < 1 || train > 255) {
                return header->status;
        }
        int slot = find_train_job(train);
        if (slot < 0) {
                header->status =
                        TC2_DISPATCH_PROJECTION_NOT_FOUND;
                return header->status;
        }
        tc2_dispatch_projection_publication *publication =
                &dispatch_projection_publications[slot];
        tc2_dispatch_job_snapshot *job =
                &dispatch_jobs[slot];
        if (!publication->valid ||
            !publication->header.valid ||
            !job->projection_valid) {
                header->status =
                        TC2_DISPATCH_PROJECTION_NOT_READY;
                return header->status;
        }
        if (publication->header.train != job->train ||
            publication->header.publication_serial !=
                    job->projection_publication_serial ||
            publication->header.plan_generation !=
                    job->projection_plan_generation ||
            publication->header.launch_epoch !=
                    job->projection_launch_epoch ||
            publication->header.plan_generation !=
                    job->plan_generation ||
            publication->header.launch_epoch !=
                    job->job_launch_epoch ||
            publication->header.waypoint_count !=
                    job->projection_waypoint_count ||
            publication->header.waypoint_count < 1 ||
            publication->header.waypoint_count >
                    TC2_ROUTE_PROJECTION_MAX_WAYPOINTS) {
                header->status =
                        TC2_DISPATCH_PROJECTION_INTERNAL;
                return header->status;
        }
        *header = publication->header;
        header->status = TC2_DISPATCH_PROJECTION_OK;
        header->valid = 1;
        header->job_state = job->state;
        header->scheduler_healthy =
                dispatch_scheduler_healthy;
        return header->status;
}

static int projection_page_for_train(
        int train, uint64_t publication_serial,
        uint32_t plan_generation, uint32_t launch_epoch,
        int first_waypoint,
        tc2_dispatch_projection_page *page) {
        if (!page) {
                return TC2_DISPATCH_PROJECTION_INVALID_ARGUMENT;
        }
        util_zero_bytes(
                (volatile unsigned char *)page,
                (unsigned int)sizeof(*page));
        page->status =
                TC2_DISPATCH_PROJECTION_INVALID_ARGUMENT;
        if (train < 1 || train > 255 ||
            publication_serial == 0) {
                return page->status;
        }
        int slot = find_train_job(train);
        if (slot < 0) {
                page->status =
                        TC2_DISPATCH_PROJECTION_NOT_FOUND;
                return page->status;
        }
        tc2_dispatch_projection_publication *publication =
                &dispatch_projection_publications[slot];
        tc2_dispatch_job_snapshot *job =
                &dispatch_jobs[slot];
        const tc2_dispatch_projection_header *header =
                &publication->header;
        if (!publication->valid || !header->valid ||
            !job->projection_valid) {
                page->status =
                        header->publication_serial == 0 ?
                        TC2_DISPATCH_PROJECTION_NOT_READY :
                        TC2_DISPATCH_PROJECTION_STALE;
                return page->status;
        }
        if (header->train != job->train ||
            header->publication_serial !=
                    job->projection_publication_serial ||
            header->plan_generation !=
                    job->projection_plan_generation ||
            header->launch_epoch !=
                    job->projection_launch_epoch ||
            header->plan_generation !=
                    job->plan_generation ||
            header->launch_epoch !=
                    job->job_launch_epoch ||
            header->waypoint_count !=
                    job->projection_waypoint_count) {
                page->status =
                        TC2_DISPATCH_PROJECTION_INTERNAL;
                return page->status;
        }
        if (header->train != train ||
            header->publication_serial != publication_serial ||
            header->plan_generation != plan_generation ||
            header->launch_epoch != launch_epoch) {
                page->status = TC2_DISPATCH_PROJECTION_STALE;
                return page->status;
        }
        if (header->waypoint_count < 1 ||
            header->waypoint_count >
                    TC2_ROUTE_PROJECTION_MAX_WAYPOINTS) {
                page->status =
                        TC2_DISPATCH_PROJECTION_INTERNAL;
                return page->status;
        }
        if (first_waypoint < 0 ||
            first_waypoint >= header->waypoint_count) {
                page->status = TC2_DISPATCH_PROJECTION_BOUNDS;
                return page->status;
        }

        int remaining =
                header->waypoint_count - first_waypoint;
        int count = remaining >
                            TC2_DISPATCH_PROJECTION_PAGE_CAPACITY ?
                TC2_DISPATCH_PROJECTION_PAGE_CAPACITY :
                remaining;
        page->status = TC2_DISPATCH_PROJECTION_OK;
        page->train = train;
        page->publication_serial = publication_serial;
        page->plan_generation = plan_generation;
        page->launch_epoch = launch_epoch;
        page->first_waypoint = first_waypoint;
        page->count = count;
        page->total_waypoints = header->waypoint_count;
        page->has_more =
                first_waypoint + count <
                header->waypoint_count;
        for (int waypoint = 0; waypoint < count; ++waypoint) {
                page->waypoints[waypoint] =
                        publication->waypoints[
                                first_waypoint + waypoint];
        }
        return page->status;
}

static int find_empty_job(void) {
        for (int slot = 0; slot < TC2_DISPATCH_MAX_JOBS; ++slot) {
                if (dispatch_jobs[slot].state == TC2_JOB_EMPTY) return slot;
        }
        return -1;
}

static int has_any_job(void) {
        for (int slot = 0; slot < TC2_DISPATCH_MAX_JOBS; ++slot) {
                if (dispatch_jobs[slot].state != TC2_JOB_EMPTY) {
                        return 1;
                }
        }
        return 0;
}

static int has_provisional_job(void) {
        for (int slot = 0; slot < TC2_DISPATCH_MAX_JOBS; ++slot) {
                if (dispatch_jobs[slot].state != TC2_JOB_EMPTY &&
                    dispatch_jobs[slot].provisional_prediction) {
                        return 1;
                }
        }
        return 0;
}

static int route_contains_node(const track_route *route, int node) {
        if (!route) return 0;
        for (int offset = 0; offset < route->node_count; ++offset) {
                if (route->nodes[offset] == node) return 1;
        }
        return 0;
}

static void mark_physical(unsigned char unavailable[TRACK_MAX], int node) {
        if (node < 0 || node >= TRACK_MAX) return;
        unavailable[node] = 1;
        int reverse = physical_reverse_index(node);
        if (reverse >= 0) unavailable[reverse] = 1;
}

static int count_blocked_physical(void) {
        int count = 0;
        for (int node = 0; node < TRACK_MAX; ++node) {
                if (!dispatch_blocked_by_node[node]) continue;
                int reverse = physical_reverse_index(node);
                if (reverse < 0 || node < reverse) ++count;
        }
        return count;
}

static int scheduler_dependencies_healthy(
        int register_status, int can_tid, int sensor_tid,
        int reservation_tid, int ticker_tid,
        int safety_tid) {
        return register_status >= 0 && can_tid >= 0 &&
                sensor_tid >= 0 && reservation_tid >= 0 &&
                ticker_tid >= 0 && safety_tid >= 0;
}

static int sensor_snapshot_startup_ready(
        const train_sensor_snapshot_t *snapshot) {
        return snapshot &&
                snapshot->sensor_server_registered &&
                snapshot->courier_created &&
                snapshot->courier_ready;
}

static int wait_for_sensor_startup(
        int sensor_tid, train_sensor_snapshot_t *snapshot,
        int max_attempts) {
        if (sensor_tid < 0 || !snapshot || max_attempts < 1) {
                return 0;
        }
        for (int attempt = 0; attempt < max_attempts; ++attempt) {
                if (TrainSensorGetLatest(sensor_tid, snapshot) == 0 &&
                    sensor_snapshot_startup_ready(snapshot)) {
                        return 1;
                }
                /*
                 * The sensor courier shares priority 4 with this server.
                 * Block until the next hardware timer event so the courier
                 * and sensor server can complete their STARTED/READY
                 * handshake.  The attempt bound keeps startup fail-closed
                 * even if that handshake never arrives.
                 */
                if (attempt + 1 < max_attempts) {
                        if (AwaitEvent(EVENT_TIMER) < 0) {
                                return 0;
                        }
                }
        }
        return 0;
}

static void clear_request(tc2_dispatch_request *request, int type) {
        request->type = type;
        request->train = 0;
        request->start_index = TC2_DISPATCH_START_INVALID;
        request->speed = 0;
        request->destination_index = -1;
        request->node_index = -1;
        request->projection_serial = 0;
        request->projection_generation = 0;
        request->projection_launch_epoch = 0;
        request->projection_first_waypoint = -1;
        request->reserved = 0;
}

static int send_request(int tid, tc2_dispatch_request *request) {
        tc2_dispatch_reply reply;
        int ret = Send(tid, (const char *)request, sizeof(*request),
                       (char *)&reply, sizeof(reply));
        return ret == (int)sizeof(reply) ? reply.status : -1;
}

int Tc2DispatchStage(int tid, int train, int start_index, int speed,
                     int destination_index) {
        tc2_dispatch_request request;
        clear_request(&request, TC2_DISPATCH_MSG_STAGE);
        request.train = train;
        request.start_index = start_index;
        request.speed = speed;
        request.destination_index = destination_index;
        return send_request(tid, &request);
}

int Tc2DispatchStageCalibration(
        int tid, int train, int start_index, int speed,
        int destination_index) {
        tc2_dispatch_request request;
        clear_request(&request, TC2_DISPATCH_MSG_STAGE_CALIBRATION);
        request.train = train;
        request.start_index = start_index;
        request.speed = speed;
        request.destination_index = destination_index;
        return send_request(tid, &request);
}

int Tc2DispatchStageFromCurrent(int tid, int train, int speed,
                                int destination_index) {
        return Tc2DispatchStage(tid, train, TC2_DISPATCH_START_CURRENT,
                                speed, destination_index);
}

int Tc2DispatchStageReroute(int tid, int train, int speed,
                            int destination_index) {
        tc2_dispatch_request request;
        clear_request(&request, TC2_DISPATCH_MSG_STAGE);
        request.train = train;
        request.start_index = TC2_DISPATCH_START_CURRENT;
        request.speed = speed;
        request.destination_index = destination_index;
        request.reserved =
                TC2_DISPATCH_REQUEST_FORCE_REVERSE_FIRST;
        return send_request(tid, &request);
}

static int send_train_request(int tid, int type, int train) {
        tc2_dispatch_request request;
        clear_request(&request, type);
        request.train = train;
        return send_request(tid, &request);
}

int Tc2DispatchStartAll(int tid) {
        return send_train_request(tid, TC2_DISPATCH_MSG_START_ALL, 0);
}

int Tc2DispatchCancel(int tid, int train) {
        return send_train_request(tid, TC2_DISPATCH_MSG_CANCEL, train);
}

int Tc2DispatchRemove(int tid, int train) {
        return send_train_request(tid, TC2_DISPATCH_MSG_REMOVE, train);
}

int Tc2DispatchIsTrainManaged(int tid, int train) {
        return send_train_request(tid, TC2_DISPATCH_MSG_IS_MANAGED, train);
}

static int send_node_request(int tid, int type, int node_index) {
        tc2_dispatch_request request;
        clear_request(&request, type);
        request.node_index = node_index;
        return send_request(tid, &request);
}

int Tc2DispatchBlockNode(int tid, int node_index) {
        return send_node_request(tid, TC2_DISPATCH_MSG_BLOCK, node_index);
}

int Tc2DispatchUnblockNode(int tid, int node_index) {
        return send_node_request(tid, TC2_DISPATCH_MSG_UNBLOCK, node_index);
}

int Tc2DispatchGetSnapshot(int tid, tc2_dispatch_snapshot *snapshot) {
        tc2_dispatch_request request;
        if (!snapshot) return -1;
        clear_request(&request, TC2_DISPATCH_MSG_SNAPSHOT);
        int ret = Send(tid, (const char *)&request, sizeof(request),
                       (char *)snapshot, sizeof(*snapshot));
        return ret == (int)sizeof(*snapshot) ? 0 : -1;
}

int Tc2DispatchGetBlockedSnapshot(
        int tid, tc2_dispatch_blocked_snapshot *snapshot) {
        tc2_dispatch_request request;
        if (!snapshot) return -1;
        clear_request(&request, TC2_DISPATCH_MSG_BLOCKED_SNAPSHOT);
        int ret = Send(tid, (const char *)&request, sizeof(request),
                       (char *)snapshot, sizeof(*snapshot));
        return ret == (int)sizeof(*snapshot) ? 0 : -1;
}

int Tc2DispatchGetProjectionHeader(
        int tid, int train,
        tc2_dispatch_projection_header *header) {
        tc2_dispatch_request request;
        if (!header) return TC2_DISPATCH_PROJECTION_INVALID_ARGUMENT;
        util_zero_bytes(
                (volatile unsigned char *)header,
                (unsigned int)sizeof(*header));
        header->status = TC2_DISPATCH_PROJECTION_INTERNAL;
        clear_request(
                &request,
                TC2_DISPATCH_MSG_PROJECTION_HEADER);
        request.train = train;
        int ret = Send(
                tid, (const char *)&request, sizeof(request),
                (char *)header, sizeof(*header));
        if (ret != (int)sizeof(*header)) {
                util_zero_bytes(
                        (volatile unsigned char *)header,
                        (unsigned int)sizeof(*header));
                header->status =
                        TC2_DISPATCH_PROJECTION_INTERNAL;
                return TC2_DISPATCH_PROJECTION_INTERNAL;
        }
        return header->status;
}

int Tc2DispatchGetProjectionPage(
        int tid, int train, uint64_t publication_serial,
        uint32_t plan_generation, uint32_t launch_epoch,
        int first_waypoint,
        tc2_dispatch_projection_page *page) {
        tc2_dispatch_request request;
        if (!page) return TC2_DISPATCH_PROJECTION_INVALID_ARGUMENT;
        util_zero_bytes(
                (volatile unsigned char *)page,
                (unsigned int)sizeof(*page));
        page->status = TC2_DISPATCH_PROJECTION_INTERNAL;
        clear_request(
                &request,
                TC2_DISPATCH_MSG_PROJECTION_PAGE);
        request.train = train;
        request.projection_serial = publication_serial;
        request.projection_generation = plan_generation;
        request.projection_launch_epoch = launch_epoch;
        request.projection_first_waypoint = first_waypoint;
        int ret = Send(
                tid, (const char *)&request, sizeof(request),
                (char *)page, sizeof(*page));
        if (ret != (int)sizeof(*page)) {
                util_zero_bytes(
                        (volatile unsigned char *)page,
                        (unsigned int)sizeof(*page));
                page->status =
                        TC2_DISPATCH_PROJECTION_INTERNAL;
                return TC2_DISPATCH_PROJECTION_INTERNAL;
        }
        return page->status;
}

static void copy_can_health(tc2_dispatch_job_snapshot *job,
                            const can_health_t *health) {
        job->can_tx_completed = health->tx_completed;
        job->can_tx_failed = health->tx_failed;
        job->can_tx_timeout = health->tx_timeout;
        job->can_rx_dropped = health->rx_dropped;
        job->can_rx_overflow = health->rx_overflow;
        job->can_turnout_changes = health->turnout_changes;
}

static int can_health_is_safe(
        int slot, int establish_baseline, int now) {
        can_health_t health;
        tc2_dispatch_runtime *runtime = &dispatch_runtime[slot];
        tc2_dispatch_job_snapshot *job = &dispatch_jobs[slot];
        if (now < 0 ||
            CanGetHealth(dispatch_can_tid, &health) < 0 ||
            !health.hw_ready) {
                return -1;
        }
        copy_can_health(job, &health);
        if (establish_baseline || !runtime->can_baseline_valid) {
                if (health.tx_notifier_heartbeat == 0 ||
                    health.rx_notifier_heartbeat == 0) {
                        return -1;
                }
                runtime->can_baseline = health;
                runtime->can_baseline_valid = 1;
                runtime->can_tx_heartbeat =
                        health.tx_notifier_heartbeat;
                runtime->can_rx_heartbeat =
                        health.rx_notifier_heartbeat;
                runtime->can_tx_heartbeat_tick = now;
                runtime->can_rx_heartbeat_tick = now;
                return 0;
        }
        /*
         * CAN health counters are controller-wide lifetime totals.  They are
         * useful diagnostics, but a normal turnout change by another train
         * (or one old recovered error) is not a fault in this job.  Per-job
         * command safety is decided by the exact Can* return/batch status;
         * service liveness remains guarded by the two heartbeat deadlines.
         */
        if (health.tx_notifier_heartbeat !=
            runtime->can_tx_heartbeat) {
                runtime->can_tx_heartbeat =
                        health.tx_notifier_heartbeat;
                runtime->can_tx_heartbeat_tick = now;
        }
        if (health.rx_notifier_heartbeat !=
            runtime->can_rx_heartbeat) {
                runtime->can_rx_heartbeat =
                        health.rx_notifier_heartbeat;
                runtime->can_rx_heartbeat_tick = now;
        }
        if (runtime->can_tx_heartbeat_tick < 0 ||
            runtime->can_rx_heartbeat_tick < 0 ||
            tick_age(now, runtime->can_tx_heartbeat_tick) >
                    TC2_SERVICE_HEARTBEAT_MAX_AGE ||
            tick_age(now, runtime->can_rx_heartbeat_tick) >
                    TC2_SERVICE_HEARTBEAT_MAX_AGE) {
                return -1;
        }
        return 0;
}

static int sensor_health_is_safe(
        int slot, const train_sensor_snapshot_t *sensor,
        int establish_baseline, int now) {
        tc2_dispatch_runtime *runtime = &dispatch_runtime[slot];
        if (!sensor || now < 0 ||
            !sensor->sensor_server_registered ||
            !sensor->courier_created || !sensor->courier_ready) {
                dispatch_jobs[slot].sensor_health_fault =
                        TC2_SENSOR_HEALTH_SERVICE_UNAVAILABLE;
                return -1;
        }
        if (!service_heartbeat_is_fresh(
                    now, sensor->courier_last_heartbeat_tick)) {
                dispatch_jobs[slot].sensor_health_fault =
                        TC2_SENSOR_HEALTH_HEARTBEAT_STALE;
                return -1;
        }
        if (establish_baseline ||
            !runtime->sensor_health_baseline_valid) {
                runtime->sensor_receive_failure_baseline =
                        sensor->courier_receive_failure_count;
                runtime->sensor_time_failure_baseline =
                        sensor->time_failure_count;
                runtime->sensor_health_baseline_valid = 1;
                dispatch_jobs[slot].sensor_health_fault =
                        TC2_SENSOR_HEALTH_OK;
                return 0;
        }
        if (sensor->courier_receive_failure_count !=
                    runtime->sensor_receive_failure_baseline) {
                dispatch_jobs[slot].sensor_health_fault =
                        TC2_SENSOR_HEALTH_RECEIVE_FAILURE;
                return -1;
        }
        if (sensor->time_failure_count !=
                    runtime->sensor_time_failure_baseline) {
                dispatch_jobs[slot].sensor_health_fault =
                        TC2_SENSOR_HEALTH_TIME_FAILURE;
                return -1;
        }
        dispatch_jobs[slot].sensor_health_fault =
                TC2_SENSOR_HEALTH_OK;
        return 0;
}

static void fail_job(int slot, int reason) {
        /*
         * A failed control epoch must disappear from every UI reader before
         * any further state is exposed.  The physical reservation is
         * deliberately retained below until the normal cancel/remove
         * recovery completes.
         */
        invalidate_projection(slot);
        tc2_dispatch_job_snapshot *job = &dispatch_jobs[slot];
        tc2_dispatch_runtime *runtime = &dispatch_runtime[slot];
        /*
         * CURRENT replacement validation is transactional until the
         * reservation generation changes.  Its temporary job carries the
         * live hold's CAN tokens and conflict-zone ownership metadata;
         * canceling either here would be an external side effect that no
         * byte-for-byte rollback can undo.  Record only the candidate result.
         * process_pending_retargets() restores the authoritative live hold.
         */
        if (dispatch_retarget_candidate_pre_cas) {
                job->state = TC2_JOB_FAILED;
                job->stop_trigger = TC2_STOP_TRIGGER_FAILSAFE;
                job->failure_reason = reason;
                job->ready_at_tick = -1;
                job->braking_at_tick = -1;
                return;
        }
        int conflict_cleanup_status =
                discard_future_conflict_zone_work(slot);
        if (conflict_cleanup_status < 0) {
                dispatch_scheduler_healthy = 0;
        }
        if (job->state == TC2_JOB_LAUNCHING &&
            runtime->launch_batch_token != 0) {
                (void)CanCancelBatch(
                        dispatch_can_tid,
                        runtime->launch_batch_token);
        }
        if (runtime->turnout_batch_token != 0) {
                (void)CanCancelBatch(
                        dispatch_can_tid,
                        runtime->turnout_batch_token);
        }
        job->state = TC2_JOB_FAILED;
        job->stop_trigger = TC2_STOP_TRIGGER_FAILSAFE;
        job->failure_reason = reason;
        job->ready_at_tick = -1;
        job->braking_at_tick = -1;
        runtime->launch_batch_token = 0;
        runtime->turnout_batch_token = 0;
        runtime->turnout_queue_pending = 0;
        if (conflict_cleanup_status == 0) {
                runtime->conflict_zone_batch_token = 0;
                runtime->conflict_zone_soft_batch_token = 0;
                runtime->conflict_zone_soft_batch_step = -1;
                runtime->conflict_zone_soft_action_index = 0;
                runtime->conflict_zone_soft_settle_at_tick = -1;
                runtime->conflict_zone_soft_prefetched_step = -1;
                runtime->conflict_zone_soft_confirmed_step = -1;
                runtime->conflict_zone_queue_pending = 0;
                runtime->conflict_zone_settle_at_tick = -1;
        }
        int train = job->train;
        int token = CanTrainEmergencyStopBatch(
                dispatch_can_tid, &train, 1);
        runtime->emergency_stop_token =
                token > 0 ? (unsigned int)token : 0;
        /*
         * Keep the retry flag raised until the CAN server reports that the
         * stop itself was acknowledged. Queue admission is not motion proof.
         */
        job->needs_stop_retry = 1;
}

static int build_route_slice(const track_route *source, int first, int last,
                             track_route *slice) {
        if (!source || !slice || first < 0 || last < first ||
            last >= source->node_count) {
                return -1;
        }
        slice->node_count = 0;
        slice->distance_mm = 0;
        slice->optimization_cost_mm = 0;
        slice->reversal_count = 0;
        for (int offset = first; offset <= last; ++offset) {
                if (slice->node_count >= TRACK_MAX) return -1;
                slice->nodes[slice->node_count++] = source->nodes[offset];
                if (offset < last &&
                    dispatch_track[source->nodes[offset]].reverse ==
                            &dispatch_track[source->nodes[offset + 1]]) {
                        ++slice->reversal_count;
                }
        }
        if (TrackRouteDistanceBetweenOffsets(
                    dispatch_track, source, first, last,
                    &slice->distance_mm) < 0) {
                return -1;
        }
        slice->optimization_cost_mm = slice->distance_mm;
        return 0;
}

static int append_unique_node(track_route *route, int node) {
        if (!route || node < 0 || node >= TRACK_MAX) return -1;
        if (route_contains_node(route, node)) return 0;
        if (route->node_count >= TRACK_MAX) return -1;
        route->nodes[route->node_count++] = node;
        return 0;
}

static int append_route_nodes(track_route *destination,
                              const track_route *source) {
        if (!destination || !source ||
            source->node_count < 0 ||
            source->node_count > TRACK_MAX) {
                return -1;
        }
        for (int offset = 0; offset < source->node_count; ++offset) {
                if (append_unique_node(
                            destination,
                            source->nodes[offset]) < 0) {
                        return -1;
                }
        }
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

/*
 * Switches 153--156 are four motors around one compact physical crossing.
 * A train body in any one of them overlaps the complete assembly.  The
 * conflict-zone scheduler still records only the pair that the selected
 * route must set, but the reservation footprint must serialize all four
 * motors as one physical resource.
 */
static int append_paired_turnout_resources(track_route *footprint) {
        unsigned char protect_central_assembly = 0;
        if (!footprint) return -1;

        for (int offset = 0; offset < footprint->node_count; ++offset) {
                int node = footprint->nodes[offset];
                int number = dispatch_track[node].num;
                if (number >= 153 && number <= 156) {
                        protect_central_assembly = 1;
                }
        }
        if (!protect_central_assembly) return 0;
        for (int node = 0; node < TRACK_MAX; ++node) {
                int number = dispatch_track[node].num;
                if (number >= 153 && number <= 156) {
                        if (append_unique_node(footprint, node) < 0) {
                                return -1;
                        }
                }
        }
        return 0;
}

static int append_forward_guard_reachable(track_route *footprint,
                                          int start_node,
                                          int guard_distance_mm) {
        int distance[TRACK_MAX];
        unsigned char visited[TRACK_MAX];
        for (int node = 0; node < TRACK_MAX; ++node) {
                distance[node] = 0x3fffffff;
                visited[node] = 0;
        }
        if (start_node < 0 || start_node >= TRACK_MAX ||
            guard_distance_mm < 0) {
                return -1;
        }
        distance[start_node] = 0;
        for (int step = 0; step < TRACK_MAX; ++step) {
                int current = -1;
                for (int node = 0; node < TRACK_MAX; ++node) {
                        if (!visited[node] &&
                            distance[node] != 0x3fffffff &&
                            (current < 0 ||
                             distance[node] < distance[current])) {
                                current = node;
                        }
                }
                if (current < 0) break;
                visited[current] = 1;
                if (append_unique_node(footprint, current) < 0) return -1;
                if (distance[current] >= guard_distance_mm) continue;
                int edge_count =
                        dispatch_track[current].type == NODE_BRANCH ? 2 :
                        (dispatch_track[current].type == NODE_EXIT ||
                         dispatch_track[current].type == NODE_NONE ? 0 : 1);
                for (int direction = 0;
                     direction < edge_count; ++direction) {
                        track_edge *edge =
                                &dispatch_track[current].edge[direction];
                        if (!edge->dest || edge->dist < 0) continue;
                        int next = (int)(edge->dest - dispatch_track);
                        if (next < 0 || next >= TRACK_MAX) return -1;
                        int candidate =
                                distance[current] + edge->dist;
                        if (candidate < distance[next]) {
                                distance[next] = candidate;
                        }
                }
        }
        return 0;
}

static int build_safety_footprint_for_route(
        const track_route *motion_route, int first_motion_offset,
        int anchor_offset, int destination_route_offset,
        int destination_offset_mm_value, int speed,
        track_route *footprint) {
        if (build_route_slice(
                    motion_route, first_motion_offset,
                    motion_route->node_count - 1, footprint) < 0) {
                return -1;
        }
        int guard = Tc2MotionBrakingDistanceMm(speed);
        if (guard < 0 || destination_offset_mm_value < 0 ||
            destination_offset_mm_value > 0x7fffffff - guard) {
                return -1;
        }
        int virtual_guard = destination_offset_mm_value + guard;
        for (int offset = first_motion_offset;
             offset < destination_route_offset; ++offset) {
                int node = motion_route->nodes[offset];
                int next = motion_route->nodes[offset + 1];
                if (dispatch_track[node].reverse ==
                    &dispatch_track[next]) {
                        if (append_forward_guard_reachable(
                                    footprint, node,
                                    offset >= anchor_offset ?
                                            virtual_guard : guard) < 0) {
                                return -1;
                        }
                }
        }
        /*
         * A turnout can be changed by another operator/controller after the
         * route was planned.  Reserve a full braking-distance escape guard
         * down both forward branches of every turnout the train can reach,
         * so a wrong turnout state still cannot send it into another train's
         * unprotected space.
         */
        for (int offset = first_motion_offset;
             offset < motion_route->node_count; ++offset) {
                int node = motion_route->nodes[offset];
                if (dispatch_track[node].type == NODE_BRANCH &&
                    append_forward_guard_reachable(
                            footprint, node,
                            offset >= anchor_offset &&
                                    offset <=
                                            destination_route_offset ?
                                    virtual_guard : guard) < 0) {
                        return -1;
                }
        }
        /*
         * The anchor report localizes the train but does not complete the
         * trip. Protect every reachable continuation through the exact
         * virtual offset plus its braking guard, including a provisional
         * turnout direction immediately after the anchor.
         */
        if (anchor_offset >= first_motion_offset &&
            append_forward_guard_reachable(
                    footprint, motion_route->nodes[anchor_offset],
                    virtual_guard) < 0) {
                return -1;
        }
        if (append_paired_turnout_resources(footprint) < 0) {
                return -1;
        }
        footprint->distance_mm = motion_route->distance_mm;
        footprint->optimization_cost_mm =
                motion_route->optimization_cost_mm;
        return 0;
}

/*
 * A rolling authority is deliberately node-granular.  It owns one contiguous
 * route slice, both physical directions of those nodes (in the reservation
 * server), and paired 153-156 interlock resources.  Wrong branches are not
 * part of movement authority: including both branches turns one local
 * turnout into a distant route-wide lock and prevents another train from
 * safely approaching its own 20 mm gate.  The conflict-zone arbiter owns and
 * settles the selected physical turnout before this exact route slice can be
 * promoted.
 */
static int build_rolling_authority_footprint(
        const track_route *motion_route,
        int first_motion_offset, int last_motion_offset,
        int speed, track_route *footprint) {
        if (!motion_route || !footprint ||
            first_motion_offset < 0 ||
            last_motion_offset < first_motion_offset ||
            last_motion_offset >= motion_route->node_count ||
            build_route_slice(
                    motion_route, first_motion_offset,
                    last_motion_offset, footprint) < 0) {
                return -1;
        }
        if (Tc2MotionBrakingDistanceMm(speed) < 0) return -1;
        if (append_paired_turnout_resources(footprint) < 0) {
                return -1;
        }
        footprint->distance_mm = motion_route->distance_mm;
        footprint->optimization_cost_mm =
                motion_route->optimization_cost_mm;
        return 0;
}

/*
 * Rebuild one local rolling-authority window exactly as it was selected.
 * A window ending at an in-route reversal sensor needs a larger forward
 * overrun guard: the train must stop, keep its complete body plus the delayed
 * 200 mm release margin clear, and only then perform the stopped handoff.
 * This helper is used both at selection and after every sensor/tail rebase so
 * that a footprint can never silently shrink at a reversal boundary.
 */
static int build_local_authority_footprint(
        const track_route *motion_route,
        int first_motion_offset, int last_motion_offset,
        int motion_leg_start_offset,
        int destination_route_offset, int speed,
        track_route *footprint) {
        if (!motion_route || !footprint ||
            motion_leg_start_offset < 0 ||
            motion_leg_start_offset >= motion_route->node_count ||
            destination_route_offset < motion_leg_start_offset ||
            destination_route_offset >= motion_route->node_count ||
            build_rolling_authority_footprint(
                    motion_route, first_motion_offset,
                    last_motion_offset, speed,
                    footprint) < 0) {
                return -1;
        }
        int motion_leg_end =
                TrackRouteMotionLegEnd(
                        dispatch_track, motion_route,
                        motion_leg_start_offset);
        if (motion_leg_end < motion_leg_start_offset ||
            motion_leg_end >= motion_route->node_count) {
                return -1;
        }
        if (last_motion_offset == motion_leg_end &&
            motion_leg_end < destination_route_offset) {
                int guard =
                        Tc2MotionBrakingDistanceMm(speed);
                if (guard < 0 ||
                    guard > 0x7fffffff -
                                    TC2_TRAIN_HALF_LENGTH_MM ||
                    guard + TC2_TRAIN_HALF_LENGTH_MM >
                                    0x7fffffff -
                                    TC2_TRAFFIC_FOLLOWING_STOP_CLEARANCE_MM) {
                        return -1;
                }
                guard +=
                        TC2_TRAIN_HALF_LENGTH_MM +
                        TC2_TRAFFIC_FOLLOWING_STOP_CLEARANCE_MM;
                if (append_forward_guard_reachable(
                            footprint,
                            motion_route->nodes[motion_leg_end],
                            guard) < 0 ||
                    append_paired_turnout_resources(
                            footprint) < 0) {
                        return -1;
                }
        }
        footprint->distance_mm =
                motion_route->distance_mm;
        footprint->optimization_cost_mm =
                motion_route->optimization_cost_mm;
        return 0;
}

static int authority_footprint_available(
        const track_route *footprint, int train,
        const track_reservation_snapshot *reservation,
        const train_sensor_snapshot_t *sensors,
        track_reservation_conflict *conflict) {
        if (!footprint || !reservation || !sensors ||
            train < 1 || train > 255) {
                return -1;
        }
        if (conflict) {
                conflict->node_index = -1;
                conflict->owner_train = 0;
        }
        for (int offset = 0;
             offset < footprint->node_count; ++offset) {
                int node = footprint->nodes[offset];
                int physical[2] = {
                        node, physical_reverse_index(node)
                };
                for (int side = 0; side < 2; ++side) {
                        int candidate = physical[side];
                        if (candidate < 0 ||
                            candidate >= TRACK_MAX) {
                                continue;
                        }
                        int owner =
                                reservation
                                        ->owner_by_node[candidate];
                        if (owner != 0 && owner != train) {
                                if (conflict) {
                                        conflict->node_index =
                                                candidate;
                                        conflict->owner_train =
                                                owner;
                                }
                                return 1;
                        }
                        if (candidate <
                                    TRAIN_SENSOR_COUNT &&
                            sensors->sensor_state[candidate] &&
                            owner != train) {
                                if (conflict) {
                                        conflict->node_index =
                                                candidate;
                                        conflict->owner_train =
                                                owner;
                                }
                                return 1;
                        }
                }
        }
        return 0;
}

static int route_distance_to_offset(
        const track_route *route, int offset,
        int *distance_mm) {
        if (!route || !distance_mm || offset < 0 ||
            offset >= route->node_count) {
                return -1;
        }
        if (offset == 0) {
                *distance_mm = 0;
                return 0;
        }
        return TrackRouteDistanceBetweenOffsets(
                dispatch_track, route, 0, offset,
                distance_mm);
}

static int footprint_tail_offset_for_anchor(
        const track_route *route, int current,
        int speed) {
        if (!route || route->node_count < 1 ||
            speed < 1 || speed > 120) {
                return -1;
        }
        if (current < 0) current = 0;
        if (current >= route->node_count) return -1;
        /*
         * A detector localizes the pickup at the end of its incoming edge.
         * Even when that edge is longer than the numeric release margin, the
         * train body can still straddle it at the instant of the report.
         * Retain the immediately preceding route node unconditionally, then
         * extend farther back by the body-plus-delay distance.
         */
        int tail = current > 0 ? current - 1 : current;
        int release_margin =
                Tc2MotionUncertaintyMm(speed);
        if (release_margin < 0) return -1;
        if (release_margin <
            TC2_INTERSECTION_RELEASE_MARGIN_MM) {
                release_margin =
                        TC2_INTERSECTION_RELEASE_MARGIN_MM;
        }
        int protected_behind =
                TC2_TRAIN_HALF_LENGTH_MM +
                release_margin;
        while (tail > 0) {
                int distance;
                if (TrackRouteDistanceBetweenOffsets(
                            dispatch_track, route,
                            tail - 1, current, &distance) < 0 ||
                    distance > protected_behind) {
                        break;
                }
                --tail;
        }
        return tail;
}

/*
 * A HEAD_ON CURRENT replacement cannot release its inherited corridor from a
 * continuously predicted position.  update_remaining_footprint() releases it
 * only after an ordered sensor observation advances reservation_anchor_offset
 * far enough that footprint_tail_offset() places the complete train plus the
 * delayed 200 mm margin beyond the new route origin.
 *
 * reservation_anchor_offset deliberately trails ordered detector evidence by
 * one observation.  The first downstream event therefore still retains the
 * old origin.  When the immediately following event arrives, the first
 * downstream sensor becomes the corroborated reservation anchor.  That anchor
 * must put the computed tail past the origin, while the train's physical
 * position at the second event must still leave a complete conservative stop
 * inside the same active authority ceiling.
 */
static int head_on_recovery_window_has_release_witness(
        const track_route *route, int origin_offset,
        int authority_end_offset,
        int authority_center_ceiling_mm, int speed) {
        if (!route || origin_offset < 0 ||
            origin_offset >= route->node_count ||
            authority_end_offset <= origin_offset ||
            authority_end_offset >= route->node_count ||
            authority_center_ceiling_mm < 0 ||
            speed < 1 || speed > 120) {
                return 0;
        }
        int braking = Tc2MotionBrakingDistanceMm(speed);
        if (braking < 0) return 0;

        int first_sensor = -1;
        for (int anchor = origin_offset + 1;
             anchor <= authority_end_offset; ++anchor) {
                int anchor_node = route->nodes[anchor];
                if (dispatch_track[anchor_node].type !=
                            NODE_SENSOR) {
                        continue;
                }
                int tail =
                        footprint_tail_offset_for_anchor(
                                route, anchor, speed);
                if (tail < 0) return 0;
                if (first_sensor < 0) {
                        /*
                         * This anchor is applied only when the next ordered
                         * sensor event corroborates it.  It must then prove
                         * that the complete delayed-release footprint is
                         * beyond the inherited CURRENT origin.
                         */
                        if (tail <= origin_offset) return 0;
                        first_sensor = anchor;
                        continue;
                }
                int second_event_distance;
                if (route_distance_to_offset(
                            route, anchor,
                            &second_event_distance) < 0) {
                        return 0;
                }
                if (second_event_distance <=
                            authority_center_ceiling_mm &&
                    authority_center_ceiling_mm -
                                    second_event_distance >=
                                    braking) {
                        return 1;
                }
                return 0;
        }
        return 0;
}

static int authority_prefetch_lead_mm(int speed) {
        int fast_tenths =
                Tc2MotionFastVelocityTenthsMmPerTick(
                        speed);
        int latency_ticks =
                TC2_TURNOUT_SETTLE_TICKS +
                TC2_AUTHORITY_CAN_BUDGET_TICKS;
        if (fast_tenths < 0 ||
            fast_tenths >
                    (0x7fffffff - 9) /
                            latency_ticks) {
                return -1;
        }
        /*
         * The authority ceiling already stays one half train plus 200 mm
         * behind the last owned node.  Add only the distance travelled while
         * CAN confirmation and mechanical turnout settling are pending; adding
         * another 200 mm here would count the following clearance twice.
         */
        return (fast_tenths * latency_ticks + 9) / 10;
}

/*
 * Select the shortest currently conflict-free movement-authority window that
 * can contain one complete conservative stop.  Reserving the longest prefix
 * (or the complete remaining route) makes a distant merge exclusive long
 * before the train can reach it and prevents a safe same-direction follower
 * from using the 200/400 mm traffic policy.
 *
 * A partial window leaves its center-line ceiling one train half-length plus
 * 200 mm before the last owned route node.  Only when the physical destination
 * itself is the first reachable safe boundary do we install the complete
 * endpoint/braking guard.
 */
static int select_rolling_authority_window_for_leg(
        const track_route *route, int first_motion_offset,
        int motion_leg_start_offset,
        int minimum_end_offset, int target_offset,
        int destination_route_offset,
        int destination_offset_mm_value,
        int destination_distance_mm, int speed,
        int current_progress_mm, int train,
        const track_reservation_snapshot *reservation,
        const train_sensor_snapshot_t *sensors,
        track_route *selected, int *selected_end_offset,
        int *selected_center_ceiling_mm,
        track_reservation_conflict *conflict) {
        if (!route || !selected || !selected_end_offset ||
            !selected_center_ceiling_mm || !reservation ||
            !sensors || route->node_count < 2 ||
            first_motion_offset < 0 ||
            motion_leg_start_offset < 0 ||
            motion_leg_start_offset >= route->node_count ||
            minimum_end_offset < first_motion_offset ||
            target_offset < 0 ||
            destination_route_offset < target_offset ||
            destination_route_offset >= route->node_count ||
            destination_distance_mm < 0 ||
            current_progress_mm < 0) {
                return -1;
        }
        int braking_envelope =
                Tc2MotionBrakingDistanceMm(speed);
        int measured_stop =
                Tc2MotionStopDistanceMm(speed);
        if (braking_envelope < 0 || measured_stop < 0) {
                return -1;
        }
        /*
         * A granted window only has to contain one complete conservative stop.
         * CAN/turnout latency controls how early the running prefetch starts;
         * charging that distance again here can reject an otherwise safe local
         * prefix and unnecessarily leave the train at its origin.
         */
        int partial_headroom = braking_envelope;
        track_reservation_conflict first_conflict;
        first_conflict.node_index = -1;
        first_conflict.owner_train = 0;

        int motion_leg_end =
                TrackRouteMotionLegEnd(
                        dispatch_track, route,
                        motion_leg_start_offset);
        if (motion_leg_end < motion_leg_start_offset ||
            motion_leg_end >= route->node_count) {
                return -1;
        }
        int final_motion_leg =
                motion_leg_end >= destination_route_offset;
        int full_end = final_motion_leg ?
                route->node_count - 1 : motion_leg_end;
        track_route candidate;
        int boundary_margin =
                (TC2_TRAIN_BODY_MM + 1) / 2 +
                TC2_TRAFFIC_FOLLOWING_STOP_CLEARANCE_MM;
        for (int end = minimum_end_offset;
             end <= full_end; ++end) {
                /*
                 * Nodes after the physical destination are braking/wrong-turn
                 * guards, not another movement block.  Once a window reaches
                 * the endpoint, select the complete destination footprint and
                 * its exact center stop; never authorize the train center to a
                 * synthetic ceiling beyond d1-d8.
                 */
                if (end >= destination_route_offset &&
                    end != full_end) {
                        continue;
                }
                int complete =
                        final_motion_leg &&
                        end == full_end;
                int reversal_terminal =
                        !final_motion_leg &&
                        end == full_end;
                int ceiling;
                if (complete) {
                        ceiling = destination_distance_mm;
                        int zero_motion =
                                ceiling == current_progress_mm;
                        if (ceiling < current_progress_mm ||
                            (!zero_motion &&
                             ceiling - current_progress_mm <
                                     measured_stop) ||
                            build_safety_footprint_for_route(
                                    route, first_motion_offset,
                                    target_offset,
                                    destination_route_offset,
                                    destination_offset_mm_value,
                                    speed, &candidate) < 0) {
                                continue;
                        }
                } else if (reversal_terminal) {
                        int end_distance;
                        if (route_distance_to_offset(
                                    route, end,
                                    &end_distance) < 0) {
                                return -1;
                        }
                        ceiling = end_distance;
                        if (ceiling < current_progress_mm ||
                            ceiling - current_progress_mm <
                                    measured_stop ||
                            build_local_authority_footprint(
                                    route,
                                    first_motion_offset,
                                    end,
                                    motion_leg_start_offset,
                                    destination_route_offset,
                                    speed, &candidate) < 0) {
                                continue;
                        }
                } else {
                        int end_distance;
                        if (route_distance_to_offset(
                                    route, end,
                                    &end_distance) < 0 ||
                            end_distance < boundary_margin) {
                                continue;
                        }
                        ceiling =
                                end_distance - boundary_margin;
                        if (ceiling < current_progress_mm ||
                            ceiling - current_progress_mm <
                                    partial_headroom) {
                                continue;
                        }
                        if (build_local_authority_footprint(
                                    route, first_motion_offset,
                                    end,
                                    motion_leg_start_offset,
                                    destination_route_offset,
                                    speed,
                                    &candidate) < 0) {
                                return -1;
                        }
                }
                track_reservation_conflict candidate_conflict;
                int available =
                        authority_footprint_available(
                                &candidate, train,
                                reservation, sensors,
                                &candidate_conflict);
                if (available < 0) return -1;
                if (available != 0) {
                        if (first_conflict.node_index < 0) {
                                first_conflict =
                                        candidate_conflict;
                                /*
                                 * Publish the exact blocked local request for
                                 * resource-scoped FIFO.  The caller must not
                                 * treat it as granted, but an older waiter can
                                 * now arbitrate this block without consulting
                                 * either train's distant destination route.
                                 */
                                *selected = candidate;
                                *selected_end_offset = end;
                                *selected_center_ceiling_mm =
                                        ceiling;
                        }
                        /*
                         * Every farther window contains this same route
                         * prefix.  Continuing could only grow the monopoly;
                         * wait for the conflicting owner to release it.
                         */
                        break;
                }
                *selected = candidate;
                *selected_end_offset = end;
                *selected_center_ceiling_mm = ceiling;
                if (conflict) {
                        conflict->node_index = -1;
                        conflict->owner_train = 0;
                }
                return 0;
        }
        if (conflict) *conflict = first_conflict;
        return 1;
}

/*
 * Prefer one bounded local authority at the highest usable speed.
 *
 * A complete destination footprint is local only when the physical
 * destination is already inside the next requested block.  Otherwise a full
 * result would reserve every remaining intersection merely because the
 * requested stopping envelope does not fit an earlier boundary.  Retry at
 * lower speeds instead.  If even speed one has no bounded prefix, return WAIT
 * with no full-route request: the caller must remain stopped (or, while
 * running, stop under its already-owned ceiling) and must not monopolize the
 * remaining route.
 *
 * A blocked bounded candidate is retained for resource-scoped FIFO.  A blocked
 * remote complete candidate is deliberately discarded because it is not a
 * local resource request.  When partial-speed fallback is forbidden, however,
 * retain a free complete candidate at the immutable requested speed as the
 * last resort.  Otherwise a short CURRENT continuation can have no stoppable
 * requested-speed boundary, reject every lower-speed partial window, and wait
 * forever even though its whole remaining route is free.
 */
static int select_bounded_rolling_authority_window_for_leg(
        const track_route *route, int first_motion_offset,
        int motion_leg_start_offset, int minimum_end_offset,
        int target_offset, int destination_route_offset,
        int destination_offset_mm_value,
        int destination_distance_mm, int requested_speed,
        int allow_partial_speed_fallback,
        int current_progress_mm, int train,
        const track_reservation_snapshot *reservation,
        const train_sensor_snapshot_t *sensors,
        track_route *selected, int *selected_end_offset,
        int *selected_center_ceiling_mm, int *selected_speed,
        track_reservation_conflict *conflict) {
        if (!route || !selected || !selected_end_offset ||
            !selected_center_ceiling_mm || !selected_speed ||
            !reservation || !sensors ||
            (allow_partial_speed_fallback != 0 &&
             allow_partial_speed_fallback != 1) ||
            requested_speed < 1 || requested_speed > 120) {
                return -1;
        }

        selected->node_count = 0;
        selected->distance_mm = 0;
        selected->optimization_cost_mm = 0;
        selected->reversal_count = 0;
        *selected_end_offset = -1;
        *selected_center_ceiling_mm = -1;
        *selected_speed = -1;
        if (conflict) {
                conflict->node_index = -1;
                conflict->owner_train = 0;
        }

        track_route blocked;
        blocked.node_count = 0;
        blocked.distance_mm = 0;
        blocked.optimization_cost_mm = 0;
        blocked.reversal_count = 0;
        int blocked_end = -1;
        int blocked_ceiling = -1;
        int blocked_speed = -1;
        track_reservation_conflict blocked_conflict;
        blocked_conflict.node_index = -1;
        blocked_conflict.owner_train = 0;

        track_route requested_complete;
        requested_complete.node_count = 0;
        requested_complete.distance_mm = 0;
        requested_complete.optimization_cost_mm = 0;
        requested_complete.reversal_count = 0;
        int requested_complete_end = -1;
        int requested_complete_ceiling = -1;

        int minimum_end_distance = -1;
        int local_destination_limit = -1;
        int boundary_margin =
                (TC2_TRAIN_BODY_MM + 1) / 2 +
                TC2_TRAFFIC_FOLLOWING_STOP_CLEARANCE_MM;
        if (route_distance_to_offset(
                    route, minimum_end_offset,
                    &minimum_end_distance) < 0 ||
            minimum_end_distance >
                    0x7fffffff - boundary_margin) {
                return -1;
        }
        local_destination_limit =
                minimum_end_distance + boundary_margin;
        int destination_is_local =
                destination_distance_mm <=
                        local_destination_limit;

        for (int speed = requested_speed; speed >= 1; --speed) {
                track_route candidate;
                candidate.node_count = 0;
                candidate.distance_mm = 0;
                candidate.optimization_cost_mm = 0;
                candidate.reversal_count = 0;
                int candidate_end = -1;
                int candidate_ceiling = -1;
                track_reservation_conflict candidate_conflict;
                candidate_conflict.node_index = -1;
                candidate_conflict.owner_train = 0;
                int status =
                        select_rolling_authority_window_for_leg(
                                route, first_motion_offset,
                                motion_leg_start_offset,
                                minimum_end_offset, target_offset,
                                destination_route_offset,
                                destination_offset_mm_value,
                                destination_distance_mm, speed,
                                current_progress_mm, train,
                                reservation, sensors, &candidate,
                                &candidate_end, &candidate_ceiling,
                                &candidate_conflict);
                if (status < 0) return -1;

                int remote_complete =
                        candidate.node_count > 0 &&
                        candidate_end == route->node_count - 1 &&
                        !destination_is_local;
                int terminal_precision_partial =
                        candidate_end >= target_offset;
                int forbidden_lower_speed_partial =
                        !allow_partial_speed_fallback &&
                        speed < requested_speed &&
                        candidate_end < route->node_count - 1 &&
                        !terminal_precision_partial;
                if (!allow_partial_speed_fallback &&
                    speed == requested_speed &&
                    status == 0 && remote_complete) {
                        requested_complete = candidate;
                        requested_complete_end = candidate_end;
                        requested_complete_ceiling =
                                candidate_ceiling;
                }
                if (status == 0 && !remote_complete &&
                    !forbidden_lower_speed_partial) {
                        *selected = candidate;
                        *selected_end_offset = candidate_end;
                        *selected_center_ceiling_mm =
                                candidate_ceiling;
                        *selected_speed = speed;
                        if (conflict) *conflict = candidate_conflict;
                        return 0;
                }
                if (status > 0 && candidate.node_count > 0 &&
                    !remote_complete &&
                    !forbidden_lower_speed_partial) {
                        /*
                         * Lower speeds can expose a shorter free prefix, so
                         * retain this exact local conflict but keep searching.
                         * The last retained request is the shortest one.
                         */
                        blocked = candidate;
                        blocked_end = candidate_end;
                        blocked_ceiling = candidate_ceiling;
                        blocked_speed = speed;
                        blocked_conflict = candidate_conflict;
                }
        }

        if (requested_complete.node_count > 0) {
                *selected = requested_complete;
                *selected_end_offset = requested_complete_end;
                *selected_center_ceiling_mm =
                        requested_complete_ceiling;
                *selected_speed = requested_speed;
                if (conflict) {
                        conflict->node_index = -1;
                        conflict->owner_train = 0;
                }
                return 0;
        }

        if (blocked.node_count > 0) {
                *selected = blocked;
                *selected_end_offset = blocked_end;
                *selected_center_ceiling_mm = blocked_ceiling;
                *selected_speed = blocked_speed;
                if (conflict) *conflict = blocked_conflict;
        }
        return 1;
}

/*
 * Select the first HEAD_ON CURRENT recovery window.
 *
 * The ordinary bounded selector deliberately returns the shortest stoppable
 * prefix.  That is insufficient for a recovery if the prefix ends before
 * ordered detector evidence can retire the inherited CURRENT footprint.  Keep
 * extending the local candidate at a given speed until it contains the
 * two-sensor release witness described above; if necessary retry at a lower
 * speed.  A remote complete candidate remains forbidden exactly as in the
 * ordinary bounded selector.
 */
static int select_head_on_recovery_initial_authority(
        const track_route *route, int first_motion_offset,
        int minimum_end_offset, int target_offset,
        int destination_route_offset,
        int destination_offset_mm_value,
        int destination_distance_mm, int requested_speed,
        int allow_partial_speed_fallback,
        int current_progress_mm, int train,
        const track_reservation_snapshot *reservation,
        const train_sensor_snapshot_t *sensors,
        track_route *selected, int *selected_end_offset,
        int *selected_center_ceiling_mm, int *selected_speed,
        track_reservation_conflict *conflict) {
        if (!route || !selected || !selected_end_offset ||
            !selected_center_ceiling_mm || !selected_speed ||
            !reservation || !sensors ||
            first_motion_offset < 0 ||
            minimum_end_offset < first_motion_offset ||
            minimum_end_offset >= route->node_count ||
            (allow_partial_speed_fallback != 0 &&
             allow_partial_speed_fallback != 1) ||
            requested_speed < 1 || requested_speed > 120) {
                return -1;
        }
        selected->node_count = 0;
        selected->distance_mm = 0;
        selected->optimization_cost_mm = 0;
        selected->reversal_count = 0;
        *selected_end_offset = -1;
        *selected_center_ceiling_mm = -1;
        *selected_speed = -1;
        if (conflict) {
                conflict->node_index = -1;
                conflict->owner_train = 0;
        }

        int minimum_end_distance;
        int boundary_margin =
                (TC2_TRAIN_BODY_MM + 1) / 2 +
                TC2_TRAFFIC_FOLLOWING_STOP_CLEARANCE_MM;
        if (route_distance_to_offset(
                    route, minimum_end_offset,
                    &minimum_end_distance) < 0 ||
            minimum_end_distance >
                    0x7fffffff - boundary_margin) {
                return -1;
        }
        int destination_is_local =
                destination_distance_mm <=
                        minimum_end_distance +
                                boundary_margin;

        track_route blocked = {0};
        int blocked_end = -1;
        int blocked_ceiling = -1;
        int blocked_speed = -1;
        track_reservation_conflict blocked_conflict;
        blocked_conflict.node_index = -1;
        blocked_conflict.owner_train = 0;

        track_route requested_complete = {0};
        int requested_complete_end = -1;
        int requested_complete_ceiling = -1;

        for (int speed = requested_speed; speed >= 1; --speed) {
                int candidate_minimum_end =
                        minimum_end_offset;
                while (candidate_minimum_end <
                       route->node_count) {
                        track_route candidate = {0};
                        int candidate_end = -1;
                        int candidate_ceiling = -1;
                        track_reservation_conflict
                                candidate_conflict;
                        candidate_conflict.node_index = -1;
                        candidate_conflict.owner_train = 0;
                        int status =
                                select_rolling_authority_window_for_leg(
                                        route,
                                        first_motion_offset,
                                        first_motion_offset,
                                        candidate_minimum_end,
                                        target_offset,
                                        destination_route_offset,
                                        destination_offset_mm_value,
                                        destination_distance_mm,
                                        speed,
                                        current_progress_mm,
                                        train, reservation, sensors,
                                        &candidate,
                                        &candidate_end,
                                        &candidate_ceiling,
                                        &candidate_conflict);
                        if (status < 0) return -1;
                        if (candidate.node_count < 1 ||
                            candidate_end <
                                    candidate_minimum_end) {
                                break;
                        }

                        int remote_complete =
                                candidate_end ==
                                        route->node_count - 1 &&
                                !destination_is_local;
                        int terminal_precision_partial =
                                candidate_end >= target_offset;
                        int forbidden_lower_speed_partial =
                                !allow_partial_speed_fallback &&
                                speed < requested_speed &&
                                candidate_end <
                                        route->node_count - 1 &&
                                !terminal_precision_partial;
                        int has_release_witness =
                                head_on_recovery_window_has_release_witness(
                                        route,
                                        first_motion_offset,
                                        candidate_end,
                                        candidate_ceiling,
                                        speed);
                        if (!allow_partial_speed_fallback &&
                            speed == requested_speed &&
                            status == 0 && remote_complete &&
                            has_release_witness) {
                                requested_complete = candidate;
                                requested_complete_end =
                                        candidate_end;
                                requested_complete_ceiling =
                                        candidate_ceiling;
                        }
                        if (has_release_witness &&
                            !remote_complete) {
                                if (status == 0 &&
                                    !forbidden_lower_speed_partial) {
                                        *selected = candidate;
                                        *selected_end_offset =
                                                candidate_end;
                                        *selected_center_ceiling_mm =
                                                candidate_ceiling;
                                        *selected_speed = speed;
                                        if (conflict) {
                                                *conflict =
                                                        candidate_conflict;
                                        }
                                        return 0;
                                }
                                if (status > 0 &&
                                    !forbidden_lower_speed_partial) {
                                        blocked = candidate;
                                        blocked_end = candidate_end;
                                        blocked_ceiling =
                                                candidate_ceiling;
                                        blocked_speed = speed;
                                        blocked_conflict =
                                                candidate_conflict;
                                }
                                /*
                                 * Every farther candidate contains this same
                                 * blocked prefix.  Wait on this exact local
                                 * resource instead of growing the request.
                                 */
                                if (status > 0) break;
                        }
                        if (status > 0 ||
                            candidate_end >=
                                    route->node_count - 1) {
                                break;
                        }
                        candidate_minimum_end =
                                candidate_end + 1;
                }
        }

        if (requested_complete.node_count > 0) {
                *selected = requested_complete;
                *selected_end_offset = requested_complete_end;
                *selected_center_ceiling_mm =
                        requested_complete_ceiling;
                *selected_speed = requested_speed;
                if (conflict) {
                        conflict->node_index = -1;
                        conflict->owner_train = 0;
                }
                return 0;
        }

        if (blocked.node_count > 0) {
                *selected = blocked;
                *selected_end_offset = blocked_end;
                *selected_center_ceiling_mm =
                        blocked_ceiling;
                *selected_speed = blocked_speed;
                if (conflict) *conflict = blocked_conflict;
        }
        return 1;
}

static int select_bounded_rolling_authority_window(
        const track_route *route, int first_motion_offset,
        int minimum_end_offset, int target_offset,
        int destination_route_offset,
        int destination_offset_mm_value,
        int destination_distance_mm, int requested_speed,
        int allow_partial_speed_fallback,
        int current_progress_mm, int train,
        const track_reservation_snapshot *reservation,
        const train_sensor_snapshot_t *sensors,
        track_route *selected, int *selected_end_offset,
        int *selected_center_ceiling_mm, int *selected_speed,
        track_reservation_conflict *conflict) {
        return select_bounded_rolling_authority_window_for_leg(
                route, first_motion_offset, first_motion_offset,
                minimum_end_offset, target_offset,
                destination_route_offset,
                destination_offset_mm_value,
                destination_distance_mm, requested_speed,
                allow_partial_speed_fallback,
                current_progress_mm, train, reservation, sensors,
                selected, selected_end_offset,
                selected_center_ceiling_mm, selected_speed,
                conflict);
}

static int turnout_plan_is_consistent(
        const track_turnout_plan *plan) {
        if (!plan || plan->action_count < 0 ||
            plan->action_count > TRACK_MAX) {
                return 0;
        }
        for (int left = 0; left < plan->action_count; ++left) {
                for (int right = left + 1;
                     right < plan->action_count; ++right) {
                        if (plan->actions[left].switch_number ==
                                    plan->actions[right].switch_number &&
                            plan->actions[left].direction !=
                                    plan->actions[right].direction) {
                                return 0;
                        }
                        if (paired_turnout_mate(
                                    plan->actions[left].switch_number) ==
                                    plan->actions[right].switch_number &&
                            plan->actions[left].direction == DIR_CURVED &&
                            plan->actions[right].direction == DIR_CURVED) {
                                return 0;
                        }
                }
        }
        return 1;
}

static int build_turnout_plan_range_checked(
        const track_route *route, int first, int last,
        track_turnout_plan *plan) {
        track_route leg;
        if (build_route_slice(route, first, last, &leg) < 0 ||
            TrackBuildTurnoutPlan(dispatch_track, &leg, plan) < 0) {
                return -1;
        }
        return turnout_plan_is_consistent(plan) ? 0 : -1;
}

static int footprint_tail_offset(const tc2_dispatch_runtime *runtime,
                                 const tc2_dispatch_job_snapshot *job) {
        if (!runtime || !job) return -1;
        return footprint_tail_offset_for_anchor(
                &runtime->route,
                runtime->reservation_anchor_offset,
                job->speed);
}

static int update_remaining_footprint(int slot) {
        tc2_dispatch_runtime *runtime = &dispatch_runtime[slot];
        tc2_dispatch_job_snapshot *job = &dispatch_jobs[slot];
        track_route footprint;
        track_reservation_conflict conflict;
        unsigned int generation;
        int footprint_speed =
                runtime->command_speed > 0 ?
                        runtime->command_speed :
                runtime->traffic_resume_speed > 0 ?
                        runtime->traffic_resume_speed :
                        job->speed;
        int tail = footprint_tail_offset(runtime, job);
        /*
         * A CURRENT replacement carries the old collision/arrival footprint.
         * Do not drop it after merely one downstream anchor advance.  The same
         * delayed-release calculation used for intersections must put the
         * train tail plus 200 mm beyond the new route origin first.
         */
        int retain_carry =
                runtime->carry_ambiguity_active &&
                tail <= runtime->leg_start_offset;
        int retain_reversal_carry =
                runtime->reversal_carry_active &&
                tail <=
                        runtime
                                ->reversal_carry_origin_offset;
        int reserved_authority_end =
                runtime->authority_prefetch_active ?
                runtime->authority_prefetch_end_offset :
                runtime->authority_end_offset;
        int full_authority =
                !runtime->rolling_authority_active ||
                reserved_authority_end ==
                        runtime->route.node_count - 1;
        int footprint_status =
                full_authority ?
                build_safety_footprint_for_route(
                        &runtime->route, tail,
                        job->target_route_offset,
                        job->destination_route_offset,
                        job->destination_offset_mm,
                        footprint_speed,
                        &footprint) :
                build_local_authority_footprint(
                        &runtime->route, tail,
                        reserved_authority_end,
                        runtime->leg_start_offset,
                        job->destination_route_offset,
                        footprint_speed, &footprint);
        if (footprint_status < 0 ||
            (retain_carry &&
             append_route_nodes(
                     &footprint,
                     &runtime->carry_ambiguity_footprint) < 0) ||
            (retain_reversal_carry &&
             append_route_nodes(
                     &footprint,
                     &runtime
                              ->reversal_carry_footprint) < 0) ||
            (full_authority &&
             !route_contains_node(
                     &footprint, job->target_node))) {
                return -1;
        }
        int status = runtime->rolling_authority_active ?
                TrackReservationServerUpdateWindow(
                        dispatch_reservation_tid,
                        &footprint, job->train,
                        job->target_node,
                        job->plan_generation, &conflict,
                        &generation) :
                TrackReservationServerUpdateFootprint(
                        dispatch_reservation_tid,
                        &footprint, job->train,
                        job->target_node,
                        job->plan_generation, &conflict,
                        &generation);
        if (status != 0 || generation != job->plan_generation) {
                job->conflict_train = conflict.owner_train;
                job->conflict_node = conflict.node_index;
                return -1;
        }
        runtime->safety_footprint = footprint;
        if (runtime->carry_ambiguity_active && !retain_carry) {
                runtime->carry_ambiguity_active = 0;
                runtime->carry_ambiguity_footprint.node_count = 0;
                /*
                 * This bit is a release witness for the train left behind
                 * by a HEAD_ON recovery.  It is set only after the
                 * generation-checked reservation update above has removed
                 * the old corridor and footprint_tail_offset() has proved
                 * that the complete train plus the delayed-release margin
                 * is beyond the CURRENT origin.  A mere cancel/STOPPED
                 * transition can therefore never wake the opposing train.
                 */
                if (runtime->head_on_recovery_active) {
                        runtime->head_on_recovery_cleared = 1;
                }
                if (runtime->carry_position_estimated) {
                        job->position_estimated = 0;
                }
                runtime->carry_position_estimated = 0;
        }
        if (runtime->reversal_carry_active &&
            !retain_reversal_carry) {
                runtime->reversal_carry_active = 0;
                runtime->reversal_carry_footprint.node_count = 0;
                runtime->reversal_carry_origin_offset = -1;
        }
        return 0;
}

static int update_arrival_hold(int slot) {
        tc2_dispatch_runtime *runtime = &dispatch_runtime[slot];
        tc2_dispatch_job_snapshot *job = &dispatch_jobs[slot];
        track_route hold;
        track_reservation_conflict conflict;
        unsigned int generation;
        int first = job->target_route_offset;
        int last = job->current_route_offset;
        if (last < first) last = first;
        int behind =
                TC2_TRAIN_HALF_LENGTH_MM +
                Tc2MotionUncertaintyMm(job->speed);
        if (first > 0) --first;
        while (first > 0) {
                int distance;
                if (TrackRouteDistanceBetweenOffsets(
                            dispatch_track, &runtime->route,
                            first - 1, job->target_route_offset,
                            &distance) < 0 ||
                    distance > behind) {
                        break;
                }
                --first;
        }
        int ahead = Tc2MotionBrakingDistanceMm(job->speed);
        if (ahead < 0) return -1;
        while (last + 1 < runtime->route.node_count) {
                int distance;
                if (TrackRouteDistanceBetweenOffsets(
                            dispatch_track, &runtime->route,
                            job->target_route_offset, last + 1,
                            &distance) < 0 ||
                    distance > ahead) {
                        break;
                }
                ++last;
        }
        if (build_route_slice(&runtime->route, first, last, &hold) < 0 ||
            !route_contains_node(&hold, job->target_node)) {
                return -1;
        }
        if (append_forward_guard_reachable(
                    &hold, job->target_node, ahead) < 0) {
                return -1;
        }
        if ((runtime->carry_ambiguity_active &&
             append_route_nodes(
                     &hold,
                     &runtime
                              ->carry_ambiguity_footprint) < 0) ||
            (runtime->reversal_carry_active &&
             append_route_nodes(
                     &hold,
                     &runtime
                              ->reversal_carry_footprint) < 0)) {
                return -1;
        }
        if (append_paired_turnout_resources(&hold) < 0) {
                return -1;
        }
        int status = TrackReservationServerUpdateFootprint(
                dispatch_reservation_tid, &hold, job->train,
                job->target_node, job->plan_generation, &conflict,
                &generation);
        if (status != 0 || generation != job->plan_generation) return -1;
        runtime->safety_footprint = hold;
        job->hold_active = 1;
        return 0;
}

static void build_unavailable(
        int slot, const track_reservation_snapshot *reservation,
        const train_sensor_snapshot_t *sensors,
        unsigned char unavailable[TRACK_MAX],
        int ignore_managed_traffic) {
        int train = dispatch_jobs[slot].train;
        for (int node = 0; node < TRACK_MAX; ++node) {
                unavailable[node] = dispatch_blocked_by_node[node];
                if (!ignore_managed_traffic &&
                    reservation->owner_by_node[node] != 0 &&
                    reservation->owner_by_node[node] != train) {
                        unavailable[node] = 1;
                }
                if (node < TRAIN_SENSOR_COUNT &&
                    sensors->sensor_state[node] &&
                    reservation->owner_by_node[node] != train &&
                    (!ignore_managed_traffic ||
                     reservation->owner_by_node[node] == 0)) {
                        mark_physical(unavailable, node);
                }
        }
        if (ignore_managed_traffic) return;
        for (int other = 0; other < TC2_DISPATCH_MAX_JOBS; ++other) {
                if (other == slot ||
                    dispatch_jobs[other].state == TC2_JOB_EMPTY) {
                        continue;
                }
                int occupied = dispatch_jobs[other].current_node;
                if (occupied < 0 &&
                    dispatch_jobs[other].start_index >= 0) {
                        occupied = TrackFindNodeByName(
                                dispatch_track,
                                Tc2DispatchStartNodeName(
                                        dispatch_jobs[other].start_index));
                }
                mark_physical(unavailable, occupied);
        }
}

static int reversal_turnout_clearance_is_safe(
        const track_route *route, int target_offset, int speed) {
        int uncertainty = Tc2MotionUncertaintyMm(speed);
        if (!route || target_offset < 0 ||
            target_offset >= route->node_count ||
            uncertainty < 0) {
                return 0;
        }
        int required_clearance =
                TC2_TRAIN_HALF_LENGTH_MM +
                uncertainty;
        for (int offset = 0;
             offset < target_offset; ++offset) {
                int node = route->nodes[offset];
                int next = route->nodes[offset + 1];
                if (dispatch_track[node].reverse !=
                    &dispatch_track[next]) {
                        continue;
                }
                for (int candidate = offset + 1;
                     candidate <= target_offset; ++candidate) {
                        int candidate_node =
                                route->nodes[candidate];
                        if (dispatch_track[candidate_node].type !=
                            NODE_BRANCH) {
                                continue;
                        }
                        int distance;
                        if (TrackRouteDistanceBetweenOffsets(
                                    dispatch_track, route,
                                    offset + 1, candidate,
                                    &distance) < 0 ||
                            distance < required_clearance) {
                                return 0;
                        }
                        break;
                }
        }
        return 1;
}

static int turnout_action_overlaps_footprint(
        const track_turnout_action *action,
        const track_route *footprint) {
        if (!action || !footprint) return 0;
        int controlled = action->switch_number;
        int controlled_mate =
                paired_turnout_mate(controlled);
        int controlled_is_central =
                controlled >= 153 && controlled <= 156;
        for (int offset = 0;
             offset < footprint->node_count; ++offset) {
                int node = footprint->nodes[offset];
                int number = dispatch_track[node].num;
                if ((dispatch_track[node].type == NODE_BRANCH ||
                     dispatch_track[node].type == NODE_MERGE) &&
                    ((controlled_is_central &&
                      number >= 153 && number <= 156) ||
                     number == controlled ||
                     number == controlled_mate ||
                     paired_turnout_mate(number) ==
                             controlled)) {
                        return 1;
                }
        }
        return 0;
}

static int turnout_plan_direction_for_switch(
        const track_turnout_plan *plan, int switch_number,
        int *direction) {
        int found = 0;
        int selected = DIR_STRAIGHT;
        if (!turnout_plan_is_consistent(plan) ||
            switch_number < 1 || !direction) {
                return -1;
        }
        for (int action = 0;
             action < plan->action_count; ++action) {
                const track_turnout_action *candidate =
                        &plan->actions[action];
                int candidate_direction;
                if (candidate->switch_number ==
                    switch_number) {
                        candidate_direction =
                                candidate->direction;
                } else if (candidate->direction ==
                                   DIR_CURVED &&
                           paired_turnout_mate(
                                   candidate
                                           ->switch_number) ==
                                   switch_number) {
                        candidate_direction = DIR_STRAIGHT;
                } else {
                        continue;
                }
                if (found &&
                    candidate_direction != selected) {
                        return -1;
                }
                selected = candidate_direction;
                found = 1;
        }
        if (!found) return 1;
        *direction = selected;
        return 0;
}

static int turnout_action_matches_confirmed_plan(
        const track_turnout_action *action,
        const track_turnout_plan *confirmed) {
        int direction;
        if (!action || !confirmed ||
            turnout_plan_direction_for_switch(
                    confirmed, action->switch_number,
                    &direction) != 0 ||
            direction != action->direction) {
                return 0;
        }
        int mate = paired_turnout_mate(
                action->switch_number);
        if (action->direction == DIR_CURVED &&
            mate != 0 &&
            (turnout_plan_direction_for_switch(
                     confirmed, mate, &direction) != 0 ||
             direction != DIR_STRAIGHT)) {
                return 0;
        }
        return 1;
}

static int turnout_plan_safe_for_carried_footprint(
        const track_turnout_plan *plan,
        const track_route *footprint,
        const track_turnout_plan *confirmed) {
        if (!turnout_plan_is_consistent(plan) || !footprint ||
            footprint->node_count < 0 ||
            footprint->node_count > TRACK_MAX) {
                return 0;
        }
        for (int action = 0; action < plan->action_count; ++action) {
                if (turnout_action_overlaps_footprint(
                            &plan->actions[action],
                            footprint) &&
                    !turnout_action_matches_confirmed_plan(
                            &plan->actions[action],
                            confirmed)) {
                        return 0;
                }
        }
        return 1;
}

static int initial_turnouts_clear_of_carried_ambiguity(
        const track_route *route, int target_offset,
        const track_route *ambiguity,
        const track_turnout_plan *confirmed) {
        track_turnout_plan plan;
        int leg_start = 0;
        int leg_end;
        if (!route || !ambiguity || route->node_count < 1 ||
            target_offset < 0 ||
            target_offset >= route->node_count) {
                return 0;
        }
        /*
         * A stopped CURRENT route may deliberately begin with a sensor
         * reversal.  That transition has no turnout action, so validate the
         * first physical-motion leg that will actually be queued after the
         * confirmed direction change.  Otherwise an empty offset-zero plan
         * could hide a conflicting turnout immediately beyond the reverse.
         */
        if (target_offset > 0 &&
            dispatch_track[route->nodes[0]].reverse ==
                    &dispatch_track[route->nodes[1]]) {
                leg_start = 1;
        }
        leg_end = TrackRouteMotionLegEnd(
                dispatch_track, route, leg_start);
        if (leg_end >= target_offset) {
                leg_end = route->node_count - 1;
        }
        if (leg_end < leg_start ||
            build_turnout_plan_range_checked(
                    route, leg_start, leg_end, &plan) < 0) {
                return 0;
        }
        return turnout_plan_safe_for_carried_footprint(
                &plan, ambiguity, confirmed);
}

static int route_has_consecutive_reversals(
        const track_route *route, int target_offset) {
        int previous_was_reversal = 0;
        if (!route || route->node_count < 1 ||
            route->node_count > TRACK_MAX ||
            target_offset < 0 ||
            target_offset >= route->node_count) {
                return 1;
        }
        for (int offset = 0;
             offset < target_offset; ++offset) {
                int node = route->nodes[offset];
                int next = route->nodes[offset + 1];
                if (node < 0 || node >= TRACK_MAX ||
                    next < 0 || next >= TRACK_MAX) {
                        return 1;
                }
                int is_reversal =
                        dispatch_track[node].reverse ==
                        &dispatch_track[next];
                if (is_reversal && previous_was_reversal) {
                        /*
                         * Two zero-distance reversals return to the original
                         * orientation, add no reachability, and can otherwise
                         * make preparation launch into another immediate
                         * reversal. Reject the non-canonical route.
                         */
                        return 1;
                }
                previous_was_reversal = is_reversal;
        }
        return 0;
}

static int route_does_not_reenter_before_localization(
        const track_route *route,
        const track_route *ambiguity) {
        int left_ambiguity = 0;
        if (!route || !ambiguity || route->node_count < 1 ||
            route->node_count > TRACK_MAX) {
                return 0;
        }
        for (int offset = 0; offset < route->node_count; ++offset) {
                int in_ambiguity =
                        route_contains_node(
                                ambiguity,
                                route->nodes[offset]);
                if (!in_ambiguity) {
                        left_ambiguity = 1;
                } else if (left_ambiguity) {
                        /*
                         * The first expected report can be the one false
                         * pulse and the next report can legitimately skip
                         * one missing sensor.  Therefore no static count of
                         * intervening sensors proves that the train left the
                         * carried footprint.  A route may not leave and then
                         * re-enter it while the CURRENT origin is estimated.
                         */
                        return 0;
                }
        }
        return 1;
}

static int route_has_named_node_after(
        const track_route *route, int after_offset,
        const char *name, int through_offset) {
        if (!name) return 1;
        int node = TrackFindNodeByName(dispatch_track, name);
        if (!route || node < 0 || after_offset < -1 ||
            through_offset >= route->node_count) {
                return 0;
        }
        for (int offset = after_offset + 1;
             offset <= through_offset; ++offset) {
                if (route->nodes[offset] == node) return 1;
        }
        return 0;
}

/*
 * A virtual destination can lie inside an edge. Return the first route-node
 * offset at or beyond that exact scalar point; callers use the scalar
 * distance for cost/timing and the ceiling only for graph traversal,
 * turnout planning, and conservative reservation.
 */
static int route_ceiling_after_offset(
        const track_route *route, int anchor_offset,
        int additional_mm, int *ceiling_offset) {
        if (!route || !ceiling_offset || additional_mm < 0 ||
            anchor_offset < 0 ||
            anchor_offset >= route->node_count) {
                return -1;
        }
        if (additional_mm == 0) {
                *ceiling_offset = anchor_offset;
                return 0;
        }
        for (int offset = anchor_offset + 1;
             offset < route->node_count; ++offset) {
                int distance;
                if (TrackRouteDistanceBetweenOffsets(
                            dispatch_track, route,
                            anchor_offset, offset,
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

static int validate_and_guard_candidate(
        const unsigned char unavailable[TRACK_MAX], int speed,
        int destination_offset_mm_value,
        const char *straight_guide,
        track_route *route, int *target_offset,
        int *destination_route_offset, int *path_distance,
        int *path_cost) {
        if (!route || route->node_count < 1 ||
            destination_offset_mm_value < 0 ||
            route->distance_mm >
                    0x7fffffff - destination_offset_mm_value ||
            route->optimization_cost_mm >
                    0x7fffffff - destination_offset_mm_value) {
                return -1;
        }
        *target_offset = route->node_count - 1;
        *path_distance =
                route->distance_mm + destination_offset_mm_value;
        *path_cost =
                route->optimization_cost_mm +
                destination_offset_mm_value;
        if (route_has_consecutive_reversals(
                    route, *target_offset)) {
                return -1;
        }
        int braking = Tc2MotionBrakingDistanceMm(speed);
        if (braking < 0 ||
            destination_offset_mm_value >
                    0x7fffffff - braking) {
                return -1;
        }
        int represented = TrackExtendRouteAhead(
                dispatch_track, route,
                destination_offset_mm_value + braking);
        if (represented >=
                    destination_offset_mm_value + braking &&
            route->node_count < TRACK_MAX &&
            dispatch_track[
                    route->nodes[
                            route->node_count - 1]].type ==
                    NODE_BRANCH) {
                /*
                 * Do not leave a braking extension ending on an
                 * unconfigured branch. Add its selected outgoing edge so
                 * the prelaunch turnout batch establishes the direction a
                 * possible overrun (and later relocalization) will follow.
                 */
                int before = route->node_count;
                int extra = TrackExtendRouteAhead(
                        dispatch_track, route, 1);
                if (extra <= 0 ||
                    route->node_count <= before) {
                        return -1;
                }
        }
        if (represented <
                    destination_offset_mm_value + braking ||
            route_ceiling_after_offset(
                    route, *target_offset,
                    destination_offset_mm_value,
                    destination_route_offset) < 0 ||
            !route_has_named_node_after(
                    route, *target_offset, straight_guide,
                    *destination_route_offset) ||
            !reversal_turnout_clearance_is_safe(
                    route, *destination_route_offset, speed)) {
                return -1;
        }
        int leg_start = 0;
        while (leg_start <= *destination_route_offset) {
                track_turnout_plan plan;
                int leg_end = TrackRouteMotionLegEnd(
                        dispatch_track, route, leg_start);
                if (leg_end >= *destination_route_offset) {
                        leg_end = route->node_count - 1;
                }
                if (leg_end < leg_start ||
                    build_turnout_plan_range_checked(
                            route, leg_start, leg_end,
                            &plan) < 0) {
                        return -1;
                }
                if (leg_end >= *destination_route_offset) break;
                leg_start = leg_end + 1;
        }
        track_route safety;
        if (build_safety_footprint_for_route(
                    route, 0, *target_offset,
                    *destination_route_offset,
                    destination_offset_mm_value,
                    speed, &safety) < 0) {
                return -1;
        }
        for (int offset = 0; offset < safety.node_count; ++offset) {
                int node = safety.nodes[offset];
                if (unavailable[node]) return -1;
                int reverse = physical_reverse_index(node);
                if (reverse >= 0 && unavailable[reverse]) return -1;
        }
        return 0;
}

static int candidate_route(int start, int target,
                           const unsigned char unavailable[TRACK_MAX],
                           int speed, int allow_reversals,
                           int prefer_forward,
                           int destination_offset_mm_value,
                           const char *straight_guide,
                           track_route *route,
                           int *target_offset,
                           int *destination_route_offset,
                           int *path_distance,
                           int *path_cost) {
        track_route forward;
        track_route reversal;
        int forward_target = -1;
        int forward_destination = -1;
        int forward_distance = 0;
        int forward_cost = 0;
        int reversal_target = -1;
        int reversal_destination = -1;
        int reversal_distance = 0;
        int reversal_cost = 0;
        int have_forward =
                TrackFindShortestRouteWithUnavailable(
                        dispatch_track, start, target,
                        unavailable, &forward) == 0 &&
                validate_and_guard_candidate(
                        unavailable, speed,
                        destination_offset_mm_value,
                        straight_guide, &forward,
                        &forward_target, &forward_destination,
                        &forward_distance,
                        &forward_cost) == 0;
        int have_reversal =
                allow_reversals &&
                TrackFindShortestRouteWithSensorReversalsAndUnavailable(
                        dispatch_track, start, target,
                        TC2_REVERSAL_PENALTY_MM, unavailable,
                        &reversal) == 0 &&
                validate_and_guard_candidate(
                        unavailable, speed,
                        destination_offset_mm_value,
                        straight_guide, &reversal,
                        &reversal_target, &reversal_destination,
                        &reversal_distance,
                        &reversal_cost) == 0;
        if (!have_forward && !have_reversal) return -1;
        /*
         * A catalogued A-F dispatch owns a known departure direction.
         * Preserve that direction whenever a safe forward route exists;
         * changing direction is a fallback for a topology/blocking case,
         * not a distance shortcut.  This keeps live control identical to
         * the offline 48-route oracle and prevents an unexpected reversal
         * from replacing the operator's shortest forward turnout plan.
         */
        if (have_forward &&
            (prefer_forward ||
             !have_reversal ||
             forward_cost <= reversal_cost)) {
                *route = forward;
                *target_offset = forward_target;
                *destination_route_offset =
                        forward_destination;
                *path_distance = forward_distance;
                *path_cost = forward_cost;
        } else {
                *route = reversal;
                *target_offset = reversal_target;
                *destination_route_offset =
                        reversal_destination;
                *path_distance = reversal_distance;
                *path_cost = reversal_cost;
        }
        return 0;
}

static int head_on_recovery_candidate_has_initial_authority(
        const track_route *route, int target_offset,
        int destination_route_offset,
        int destination_offset_mm_value,
        int destination_distance_mm, int speed, int train,
        const track_reservation_snapshot *reservation,
        const train_sensor_snapshot_t *sensors) {
        if (!route || !reservation || !sensors ||
            route->node_count < 2) {
                return 0;
        }
        int first_motion_offset = 0;
        int minimum_end_offset = 1;
        int initial_leg_end =
                TrackRouteMotionLegEnd(
                        dispatch_track, route, 0);
        if (initial_leg_end < 0 ||
            initial_leg_end >= route->node_count) {
                return 0;
        }
        if (initial_leg_end == 0 &&
            destination_route_offset > 0) {
                if (route->node_count < 3 ||
                    dispatch_track[route->nodes[0]].reverse !=
                            &dispatch_track[
                                    route->nodes[1]]) {
                        return 0;
                }
                first_motion_offset = 1;
                minimum_end_offset = 2;
        }

        track_route authority;
        int authority_end = -1;
        int authority_ceiling = -1;
        int authority_speed = -1;
        track_reservation_conflict conflict;
        int status =
                select_head_on_recovery_initial_authority(
                        route, first_motion_offset,
                        minimum_end_offset, target_offset,
                        destination_route_offset,
                        destination_offset_mm_value,
                        destination_distance_mm, speed,
                        first_motion_offset == 1 ? 1 : 0,
                        0, train, reservation, sensors,
                        &authority, &authority_end,
                        &authority_ceiling, &authority_speed,
                        &conflict);
        /*
         * A free local window is immediately usable.  A blocked local window is
         * still a valid route candidate and must be retained for FIFO; an empty
         * WAIT means no bounded sensor-witness authority exists on this route.
         */
        return status == 0 ||
                (status > 0 &&
                 authority.node_count > 0);
}

static int candidate_route_for_current_hold(
        int start, int target,
        const unsigned char unavailable[TRACK_MAX],
        int speed, int destination_offset_mm_value,
        const char *straight_guide,
        int force_reverse_first,
        const track_route *carried,
        const track_turnout_plan *confirmed,
        int train,
        const track_reservation_snapshot *reservation,
        const train_sensor_snapshot_t *sensors,
        int require_head_on_release_witness,
        track_route *route,
        int *target_offset, int *destination_route_offset,
        int *path_distance,
        int *path_cost, int *hold_blocked) {
        track_route forward;
        track_route reversal;
        int forward_target = -1;
        int forward_destination = -1;
        int forward_distance = 0;
        int forward_cost = 0;
        int reversal_target = -1;
        int reversal_destination = -1;
        int reversal_distance = 0;
        int reversal_cost = 0;
        track_route leading;
        int leading_target = -1;
        int leading_destination = -1;
        int leading_distance = 0;
        int leading_cost = 0;
        if (!hold_blocked) return -1;
        *hold_blocked = 0;
        int forward_graph =
                TrackFindShortestRouteWithUnavailable(
                        dispatch_track, start, target,
                        unavailable, &forward) == 0 &&
                validate_and_guard_candidate(
                        unavailable, speed,
                        destination_offset_mm_value,
                        straight_guide, &forward,
                        &forward_target, &forward_destination,
                        &forward_distance,
                        &forward_cost) == 0;
        int have_forward =
                forward_graph &&
                initial_turnouts_clear_of_carried_ambiguity(
                        &forward, forward_destination, carried,
                        confirmed);
        if (forward_graph && !have_forward) {
                *hold_blocked = 1;
        }
        if (have_forward &&
            require_head_on_release_witness &&
            !head_on_recovery_candidate_has_initial_authority(
                    &forward, forward_target,
                    forward_destination,
                    destination_offset_mm_value,
                    forward_distance, speed, train,
                    reservation, sensors)) {
                have_forward = 0;
        }
        int reversal_graph =
                TrackFindShortestRouteWithSensorReversalsAndUnavailable(
                        dispatch_track, start, target,
                        TC2_REVERSAL_PENALTY_MM, unavailable,
                        &reversal) == 0 &&
                validate_and_guard_candidate(
                        unavailable, speed,
                        destination_offset_mm_value,
                        straight_guide, &reversal,
                        &reversal_target, &reversal_destination,
                        &reversal_distance,
                        &reversal_cost) == 0;
        int have_reversal =
                reversal_graph &&
                initial_turnouts_clear_of_carried_ambiguity(
                        &reversal, reversal_destination, carried,
                        confirmed);
        if (reversal_graph && !have_reversal) {
                *hold_blocked = 1;
        }
        if (have_reversal &&
            require_head_on_release_witness &&
            !head_on_recovery_candidate_has_initial_authority(
                    &reversal, reversal_target,
                    reversal_destination,
                    destination_offset_mm_value,
                    reversal_distance, speed, train,
                    reservation, sensors)) {
                have_reversal = 0;
        }
        int have_leading = 0;
        int reverse_start = physical_reverse_index(start);
        if (reverse_start >= 0 &&
            dispatch_track[start].type == NODE_SENSOR &&
            dispatch_track[reverse_start].type == NODE_SENSOR &&
            /*
             * This is the explicit "reverse first" alternative.  Search
             * forward-only after that one intentional reversal; allowing a
             * second generalized reversal here can produce the useless
             * start -> reverse(start) -> start prefix and hide the actual
             * opposite-direction route.
             */
            TrackFindShortestRouteWithUnavailable(
                    dispatch_track, reverse_start, target,
                    unavailable,
                    &leading) == 0 &&
            leading.node_count < TRACK_MAX &&
            leading.optimization_cost_mm <=
                    0x7fffffff -
                            TC2_REVERSAL_PENALTY_MM) {
                for (int offset = leading.node_count;
                     offset > 0; --offset) {
                        leading.nodes[offset] =
                                leading.nodes[offset - 1];
                }
                leading.nodes[0] = start;
                ++leading.node_count;
                ++leading.reversal_count;
                leading.optimization_cost_mm +=
                        TC2_REVERSAL_PENALTY_MM;
                int leading_graph =
                        validate_and_guard_candidate(
                                unavailable, speed,
                                destination_offset_mm_value,
                                straight_guide, &leading,
                                &leading_target,
                                &leading_destination,
                                &leading_distance,
                                &leading_cost) == 0;
                have_leading =
                        leading_graph &&
                        initial_turnouts_clear_of_carried_ambiguity(
                                &leading, leading_destination,
                                carried, confirmed);
                if (leading_graph && !have_leading) {
                        *hold_blocked = 1;
                }
                if (have_leading &&
                    require_head_on_release_witness &&
                    !head_on_recovery_candidate_has_initial_authority(
                            &leading, leading_target,
                            leading_destination,
                            destination_offset_mm_value,
                            leading_distance, speed, train,
                            reservation, sensors)) {
                        have_leading = 0;
                }
        }
        if ((force_reverse_first && !have_leading) ||
            (!force_reverse_first &&
             !have_forward && !have_reversal &&
             !have_leading)) {
                return -1;
        }
        if (force_reverse_first) {
                *route = leading;
                *target_offset = leading_target;
                *destination_route_offset =
                        leading_destination;
                *path_distance = leading_distance;
                *path_cost = leading_cost;
        } else if (have_forward) {
                *route = forward;
                *target_offset = forward_target;
                *destination_route_offset =
                        forward_destination;
                *path_distance = forward_distance;
                *path_cost = forward_cost;
        } else if (have_reversal) {
                *route = reversal;
                *target_offset = reversal_target;
                *destination_route_offset =
                        reversal_destination;
                *path_distance = reversal_distance;
                *path_cost = reversal_cost;
        } else {
                *route = leading;
                *target_offset = leading_target;
                *destination_route_offset =
                        leading_destination;
                *path_distance = leading_distance;
                *path_cost = leading_cost;
        }
        return 0;
}

static int routes_have_identical_nodes(
        const track_route *left,
        const track_route *right) {
        if (!left || !right ||
            left->node_count != right->node_count) {
                return 0;
        }
        for (int offset = 0;
             offset < left->node_count; ++offset) {
                if (left->nodes[offset] !=
                    right->nodes[offset]) {
                        return 0;
                }
        }
        return 1;
}

/*
 * Build prediction metadata only for ordinary, catalogued A-F dispatches.
 * CURRENT continuations depend on a live localization anchor, while the
 * supervised calibration command has its own deliberately provisional
 * execution path. Neither may inherit fixed-start transfer evidence.
 *
 * The route passed to the predictor ends at the geometry anchor. The
 * dispatch candidate itself remains extended through the physical endpoint,
 * conservative braking distance, train body, uncertainty, and turnout
 * branches by the existing authoritative safety-footprint logic.
 */
static int build_candidate_prediction_plan(
        const tc2_dispatch_job_snapshot *job,
        const track_route *candidate,
        int geometry_anchor_offset,
        int destination_side,
        tc2_prediction_plan *plan) {
        if (!job || !candidate || !plan ||
            job->provisional_prediction ||
            job->start_index < 0 ||
            job->start_index >= TC2_TRACK_START_COUNT ||
            destination_side < 0 ||
            destination_side >=
                    TC2_TRACK_DESTINATION_SIDE_COUNT) {
                return -1;
        }

        track_route geometry_route;
        if (build_route_slice(
                    candidate, 0, geometry_anchor_offset,
                    &geometry_route) < 0) {
                return -1;
        }
        tc2_prediction_request request;
        request.train = job->train;
        request.start_index = job->start_index;
        request.destination_index =
                job->destination_index;
        request.destination_side = destination_side;
        request.speed = job->speed;
        request.reversal_count =
                geometry_route.reversal_count;
        if (Tc2PredictionBuildPlan(
                    dispatch_track, &geometry_route,
                    &request, plan) != TC2_PREDICTION_OK) {
                return -1;
        }

        /*
         * The only exact motion profile is the measured canonical
         * T14/A/D7/no-reversal path. If a live block makes the dispatcher
         * choose another node sequence, retain the plan as transferred
         * inference rather than mislabelling it exact.
         */
        if (plan->exact_profile) {
                track_route canonical;
                int canonical_ok =
                        TrackFindShortestRoute(
                                dispatch_track,
                                geometry_route.nodes[0],
                                geometry_route.nodes[
                                        geometry_route.node_count - 1],
                                &canonical) == 0 &&
                        routes_have_identical_nodes(
                                &canonical,
                                &geometry_route);
                if (!canonical_ok) {
                        plan->exact_profile = 0;
                        plan->profile_evidence =
                                TC2_PREDICTION_PROFILE_TRANSFERRED_T14_A_D7;
                }
        }

        /*
         * This is only a consistency assertion. It can reject incomplete
         * graph coverage, but it can never reduce the physical reservation
         * that validate_and_guard_candidate already required.
         */
        if ((int64_t)candidate->distance_mm * 1000 <
            plan->minimum_reservation_ceiling_distance_um) {
                return -1;
        }
        return 0;
}

static int choose_shortest_available_route(
        int slot, const track_reservation_snapshot *reservation,
        const train_sensor_snapshot_t *sensors, track_route *selected,
        int *target_node, int *target_offset,
        int *destination_route_offset,
        int *destination_offset_value,
        int *destination_confirmed,
        int *path_distance,
        int *selected_side, int *estimated_ambiguity_blocked,
        int ignore_managed_traffic) {
        tc2_dispatch_job_snapshot *job = &dispatch_jobs[slot];
        tc2_dispatch_runtime *runtime = &dispatch_runtime[slot];
        util_zero_bytes(
                (volatile unsigned char *)
                        &runtime->prediction_plan,
                (unsigned int)sizeof(
                        runtime->prediction_plan));
        runtime->prediction_plan_valid = 0;
        clear_prediction_snapshot(job);
        unsigned char unavailable[TRACK_MAX];
        build_unavailable(
                slot, reservation, sensors, unavailable,
                ignore_managed_traffic);
        if (runtime->start_node < 0 || runtime->start_node >= TRACK_MAX ||
            unavailable[runtime->start_node] ||
            !estimated_ambiguity_blocked) {
                return -1;
        }
        *estimated_ambiguity_blocked = 0;

        int found = 0;
        int selected_cost = 0;
        for (int side = 0; side < 2; ++side) {
                const tc2_track_destination_side *side_definition =
                        Tc2TrackDestinationSide(
                                job->destination_index, side);
                if (!side_definition) continue;
                const char *name =
                        side_definition->anchor_sensor;
                int target = TrackFindNodeByName(dispatch_track, name);
                track_route candidate;
                int candidate_target_offset;
                int candidate_destination_offset;
                int candidate_distance;
                int candidate_cost;
                int candidate_hold_blocked = 0;
                tc2_prediction_plan candidate_prediction;
                int candidate_prediction_valid = 0;
                int virtual_offset =
                        side_definition->base_offset_mm;
                if (job->provisional_prediction &&
                    job->destination_index == 6 && side == 0) {
                        int correction =
                                Tc2MotionD7EndpointCorrectionMm(
                                        job->speed);
                        if (correction > 0 &&
                            virtual_offset >
                                    0x7fffffff - correction) {
                                continue;
                        }
                        virtual_offset += correction;
                        if (virtual_offset < 0) {
                                if (!side_definition
                                             ->previous_anchor_sensor ||
                                    side_definition
                                             ->previous_anchor_distance_mm <=
                                             0 ||
                                    virtual_offset <
                                    -side_definition
                                             ->previous_anchor_distance_mm) {
                                        continue;
                                }
                                name =
                                        side_definition
                                                ->previous_anchor_sensor;
                                target = TrackFindNodeByName(
                                        dispatch_track, name);
                                virtual_offset +=
                                        side_definition
                                                ->previous_anchor_distance_mm;
                        }
                }
                const char *straight_guide =
                        side_definition->straight_guide;
                int candidate_status =
                        job->start_index ==
                                        TC2_DISPATCH_START_CURRENT ?
                        candidate_route_for_current_hold(
                                runtime->start_node, target,
                                unavailable, job->speed,
                                virtual_offset,
                                straight_guide,
                                runtime->force_reverse_first,
                                &runtime
                                         ->carry_ambiguity_footprint,
                                runtime
                                        ->carry_confirmed_turnouts_valid ?
                                &runtime
                                         ->carry_confirmed_turnouts :
                                0,
                                job->train,
                                reservation, sensors,
                                runtime
                                        ->head_on_recovery_active &&
                                runtime
                                        ->carry_ambiguity_active,
                                &candidate,
                                &candidate_target_offset,
                                &candidate_destination_offset,
                                &candidate_distance,
                                &candidate_cost,
                                &candidate_hold_blocked) :
                        candidate_route(
                                runtime->start_node, target,
                                unavailable, job->speed,
                                !runtime
                                         ->carry_position_estimated,
                                !job->provisional_prediction,
                                virtual_offset,
                                straight_guide,
                                &candidate,
                                &candidate_target_offset,
                                &candidate_destination_offset,
                                &candidate_distance,
                                &candidate_cost);
                if (target < 0 || unavailable[target] ||
                    candidate_status < 0) {
                        if (candidate_hold_blocked) {
                                *estimated_ambiguity_blocked = 1;
                        } else if (
                                target >= 0 &&
                                !unavailable[target] &&
                                runtime
                                        ->carry_position_estimated) {
                                /*
                                 * Distinguish a direction/topology limit
                                 * from a temporary foreign reservation or
                                 * operator block.  Only the former may start
                                 * another confirmed-direction localization
                                 * phase; if the forward candidate exists on
                                 * an otherwise clear graph, the live
                                 * unavailability is the reason and the train
                                 * must remain stopped in WAIT_ROUTE.
                                 */
                                unsigned char clear[TRACK_MAX];
                                for (int node = 0;
                                     node < TRACK_MAX; ++node) {
                                        clear[node] = 0;
                                }
                                track_route clear_candidate;
                                int clear_target_offset;
                                int clear_destination_offset;
                                int clear_distance;
                                int clear_cost;
                                if (candidate_route(
                                            runtime->start_node,
                                            target, clear,
                                            job->speed, 0, 1,
                                            virtual_offset,
                                            straight_guide,
                                            &clear_candidate,
                                            &clear_target_offset,
                                            &clear_destination_offset,
                                            &clear_distance,
                                            &clear_cost) < 0) {
                                        *estimated_ambiguity_blocked = 1;
                                }
                        }
                        continue;
                }
                int forced_leading_reversal =
                        runtime->force_reverse_first &&
                        candidate.node_count >= 2 &&
                        candidate.reversal_count >= 1 &&
                        dispatch_track[candidate.nodes[0]].reverse ==
                                &dispatch_track[
                                        candidate.nodes[1]];
                if (runtime->carry_position_estimated &&
                    ((!forced_leading_reversal &&
                      candidate.reversal_count != 0) ||
                     !route_does_not_reenter_before_localization(
                              &candidate,
                              &runtime
                                       ->carry_ambiguity_footprint) ||
                     !initial_turnouts_clear_of_carried_ambiguity(
                              &candidate,
                              candidate_destination_offset,
                              &runtime
                                       ->carry_ambiguity_footprint,
                              runtime
                                      ->carry_confirmed_turnouts_valid ?
                              &runtime
                                       ->carry_confirmed_turnouts :
                              0))) {
                        *estimated_ambiguity_blocked = 1;
                        continue;
                }
                if (!job->provisional_prediction &&
                    job->start_index >= 0 &&
                    job->start_index <
                            TC2_TRACK_START_COUNT) {
                        if (build_candidate_prediction_plan(
                                    job, &candidate,
                                    candidate_target_offset,
                                    side,
                                    &candidate_prediction) < 0) {
                                continue;
                        }
                        candidate_prediction_valid = 1;
                }
                if (!found ||
                    candidate_distance < *path_distance ||
                    (candidate_distance == *path_distance &&
                     candidate_cost < selected_cost)) {
                        *selected = candidate;
                        *target_node = target;
                        *target_offset = candidate_target_offset;
                        *destination_route_offset =
                                candidate_destination_offset;
                        *destination_offset_value =
                                virtual_offset;
                        *destination_confirmed =
                                side_definition
                                        ->geometry_evidence ==
                                TC2_TRACK_GEOMETRY_CONFIRMED;
                        *path_distance = candidate_distance;
                        *selected_side = side;
                        selected_cost = candidate_cost;
                        if (candidate_prediction_valid) {
                                runtime->prediction_plan =
                                        candidate_prediction;
                                runtime->prediction_plan_valid =
                                        1;
                        } else {
                                util_zero_bytes(
                                        (volatile unsigned char *)
                                                &runtime
                                                        ->prediction_plan,
                                        (unsigned int)sizeof(
                                                runtime
                                                        ->prediction_plan));
                                runtime->prediction_plan_valid =
                                        0;
                        }
                        found = 1;
                }
        }
        return found ? 0 : -1;
}

static int second_sensor_after_origin(
        const track_route *route) {
        int count = 0;
        if (!route) return -1;
        for (int offset = 1;
             offset < route->node_count; ++offset) {
                if (dispatch_track[
                            route->nodes[offset]].type !=
                    NODE_SENSOR) {
                        continue;
                }
                if (++count == 2) return offset;
        }
        return -1;
}

static int choose_safe_localization_route(
        int slot,
        const unsigned char unavailable[TRACK_MAX],
        track_route *selected, int *target_node,
        int *target_offset, int *path_distance) {
        tc2_dispatch_job_snapshot *job =
                &dispatch_jobs[slot];
        tc2_dispatch_runtime *runtime =
                &dispatch_runtime[slot];
        track_route extended;
        int second;
        if (!runtime->carry_ambiguity_active ||
            !runtime->carry_confirmed_turnouts_valid ||
            runtime->carry_motion_origin_offset < 0 ||
            runtime->carry_motion_origin_offset >=
                    runtime->carry_motion_route.node_count ||
            build_route_slice(
                    &runtime->carry_motion_route,
                    runtime->carry_motion_origin_offset,
                    runtime->carry_motion_route.node_count - 1,
                    &extended) < 0 ||
            extended.reversal_count != 0) {
                return -1;
        }

        second = second_sensor_after_origin(&extended);
        for (int extension = 0;
             second < 0 && extension < 8; ++extension) {
                int before = extended.node_count;
                if (TrackExtendRouteAhead(
                            dispatch_track, &extended,
                            1000) < 0 ||
                    extended.node_count <= before) {
                        return -1;
                }
                second =
                        second_sensor_after_origin(&extended);
        }
        if (second < 0 ||
            build_route_slice(
                    &extended, 0, second, selected) < 0) {
                return -1;
        }

        int path_cost;
        int destination_route_offset;
        if (validate_and_guard_candidate(
                    unavailable, job->speed, 0, 0, selected,
                    target_offset, &destination_route_offset,
                    path_distance,
                    &path_cost) < 0 ||
            selected->reversal_count != 0 ||
            *target_offset < 1 ||
            destination_route_offset != *target_offset ||
            !route_does_not_reenter_before_localization(
                    selected,
                    &runtime->carry_ambiguity_footprint) ||
            !initial_turnouts_clear_of_carried_ambiguity(
                    selected, *target_offset,
                    &runtime->carry_ambiguity_footprint,
                    &runtime->carry_confirmed_turnouts)) {
                return -1;
        }
        (void)path_cost;
        *target_node =
                selected->nodes[*target_offset];
        return dispatch_track[*target_node].type ==
                       NODE_SENSOR ?
                0 : -1;
}

static int start_is_in_use(int start_index, int excluding_train) {
        if (start_index < 0) return 0;
        for (int slot = 0; slot < TC2_DISPATCH_MAX_JOBS; ++slot) {
                tc2_dispatch_job_snapshot *job = &dispatch_jobs[slot];
                if (job->state == TC2_JOB_EMPTY ||
                    job->state == TC2_JOB_ARRIVED ||
                    job->train == excluding_train) {
                        continue;
                }
                if (job->start_index == start_index) return 1;
        }
        return 0;
}

static int snapshot_owned_footprint(
        int train, unsigned int expected_generation,
        int expected_destination, int required_node,
        track_route *owned) {
        track_reservation_snapshot reservation;
        if (!owned || train < 1 || train > 255 ||
            expected_generation == 0 ||
            required_node < 0 || required_node >= TRACK_MAX ||
            TrackReservationServerSnapshot(
                    dispatch_reservation_tid, &reservation) < 0 ||
            reservation.generation_by_train[train] !=
                    expected_generation ||
            reservation.destination_by_train[train] !=
                    expected_destination) {
                return -1;
        }
        owned->node_count = 0;
        owned->distance_mm = 0;
        owned->optimization_cost_mm = 0;
        owned->reversal_count = 0;
        for (int node = 0; node < TRACK_MAX; ++node) {
                if (reservation.owner_by_node[node] == train &&
                    append_unique_node(owned, node) < 0) {
                        return -1;
                }
        }
        return owned->node_count > 0 &&
                       route_contains_node(owned, required_node) ?
                0 : -1;
}

static int stage_job(const tc2_dispatch_request *request) {
        int provisional =
                request->type ==
                TC2_DISPATCH_MSG_STAGE_CALIBRATION;
        int force_reverse_first =
                (request->reserved &
                 TC2_DISPATCH_REQUEST_FORCE_REVERSE_FIRST) != 0;
        if (request->train < 1 || request->train > 255 ||
            request->speed < 1 || request->speed > 120 ||
            request->destination_index < 0 ||
            request->destination_index >= TC2_DISPATCH_DESTINATION_COUNT ||
            (request->type != TC2_DISPATCH_MSG_STAGE &&
             !provisional) ||
            (request->reserved &
             ~TC2_DISPATCH_REQUEST_STAGE_FLAGS) != 0 ||
            (force_reverse_first &&
             (provisional ||
              request->start_index !=
                      TC2_DISPATCH_START_CURRENT)) ||
            (request->start_index != TC2_DISPATCH_START_CURRENT &&
             (request->start_index < 0 ||
              request->start_index >= TC2_DISPATCH_START_COUNT))) {
                return -1;
        }
        /*
         * The physically fitted calibration profile is still restricted to
         * one supervised T14/A/D7 run.  Do not let it share reservations or
         * transfer its point fit to an ordinary concurrently managed trip.
         */
        if ((provisional &&
             (request->train != 14 ||
              request->start_index != 0 ||
              request->destination_index != 6 ||
              has_any_job())) ||
            (!provisional && has_provisional_job())) {
                return -1;
        }
        int slot = find_train_job(request->train);
        int current_node = -1;
        unsigned int current_generation = 0;
        int current_destination = -1;
        int current_speed = 0;
        train_sensor_snapshot_t stage_sensors;
        unsigned int stage_attributed_sequence = 0;
        int stage_sensor_baseline_valid = 0;
        track_route carry_ambiguity;
        int carry_ambiguity_active = 0;
        int carry_position_estimated = 0;
        track_turnout_plan carry_confirmed_turnouts;
        int carry_confirmed_turnouts_valid = 0;
        track_route carry_motion_route;
        int carry_motion_origin_offset = -1;
        int carry_preorigin_distance_mm = 0;
        int carry_head_on_recovery = 0;
        int carry_conflict_zone_count = 0;
        int carry_conflict_zone[TC2_CONFLICT_ZONE_COUNT];
        int carry_conflict_zone_entry[TC2_CONFLICT_ZONE_COUNT];
        int carry_conflict_zone_exit[TC2_CONFLICT_ZONE_COUNT];
        int carry_conflict_zone_exit_distance[
                TC2_CONFLICT_ZONE_COUNT];
        unsigned int carry_conflict_zone_ticket[TC2_CONFLICT_ZONE_COUNT];
        unsigned char carry_conflict_zone_mode[TC2_CONFLICT_ZONE_COUNT];
        if (request->start_index ==
                    TC2_DISPATCH_START_CURRENT &&
            slot >= 0 &&
            dispatch_jobs[slot].state ==
                    TC2_JOB_TRAFFIC_HOLD) {
                tc2_dispatch_job_snapshot *held_job =
                        &dispatch_jobs[slot];
                tc2_dispatch_runtime *held_runtime =
                        &dispatch_runtime[slot];
                int now = Time();
                dispatch_retarget_runtime_backup =
                        *held_runtime;
                int health_status =
                        now >= 0 ?
                        can_health_is_safe(
                                slot, 0, now) : -1;
                /*
                 * Health sampling updates heartbeat baselines for ordinary
                 * state-machine transitions.  Staging a pending recovery is
                 * deliberately read-only, so restore those bookkeeping
                 * fields before recording only the pending intent.
                 */
                *held_runtime =
                        dispatch_retarget_runtime_backup;
                if (!held_job->hold_active ||
                    !held_job->traffic_hold_active ||
                    held_job->plan_generation == 0 ||
                    held_job->target_node < 0 ||
                    held_job->current_node < 0 ||
                    held_runtime->stop_purpose !=
                            TC2_STOP_TRAFFIC ||
                    held_runtime->traffic_hold_distance_um < 0 ||
                    held_runtime->command_speed != 0 ||
                    held_runtime->retarget_armed ||
                    now < 0 ||
                    health_status < 0) {
                        return -1;
                }
                /*
                 * Only record intent.  In particular, do not invalidate the
                 * live projection, clear the HEAD_ON/FOLLOWING latch, send a
                 * turnout command, or alter one reservation owner here.
                 */
                held_runtime->retarget_pending = 1;
                held_runtime->retarget_speed =
                        request->speed;
                held_runtime->retarget_destination_index =
                        request->destination_index;
                held_runtime
                        ->retarget_expected_generation =
                        held_job->plan_generation;
                held_runtime
                        ->retarget_expected_destination =
                        held_job->target_node;
                held_runtime->retarget_queue_sequence = 0;
                held_runtime->retarget_launch_epoch = 0;
                held_runtime->retarget_force_reverse_first =
                        force_reverse_first;
                return 0;
        }
        if (request->start_index == TC2_DISPATCH_START_CURRENT) {
                if (slot < 0 ||
                    dispatch_jobs[slot].state != TC2_JOB_ARRIVED ||
                    !dispatch_jobs[slot].hold_active ||
                    dispatch_jobs[slot].current_node < 0) {
                        return -1;
                }
                current_node = dispatch_jobs[slot].current_node;
                current_generation = dispatch_jobs[slot].plan_generation;
                current_destination =
                        dispatch_jobs[slot].target_node;
                current_speed = dispatch_jobs[slot].speed;
                carry_head_on_recovery =
                        dispatch_runtime[slot].stop_purpose ==
                                TC2_STOP_TRAFFIC &&
                        dispatch_runtime[slot].traffic_reason ==
                                TC2_TRAFFIC_HEAD_ON;
                if (current_generation == 0 ||
                    current_destination < 0 ||
                    can_health_is_safe(
                            slot, 0, Time()) < 0 ||
                    TrainSensorGetLatest(
                            dispatch_sensor_tid,
                            &stage_sensors) < 0) {
                        return -1;
                }
                stage_attributed_sequence =
                        stage_sensors.attributed_count;
                stage_sensor_baseline_valid = 1;
                carry_position_estimated =
                        dispatch_jobs[slot]
                                .position_estimated;
                if (snapshot_owned_footprint(
                            request->train,
                            current_generation,
                            current_destination,
                            current_node,
                            &carry_ambiguity) < 0) {
                        return -1;
                }
                carry_ambiguity_active = 1;
                tc2_dispatch_runtime *old_runtime =
                        &dispatch_runtime[slot];
                /*
                 * Preserve physical ownership before clear_runtime() erases
                 * the old route geometry.  This is bookkeeping only: no
                 * owner or waiter changes until the replacement reservation
                 * commits below.
                 */
                if (snapshot_owned_conflict_zone_carries(slot) < 0) {
                        return -1;
                }
                carry_conflict_zone_count =
                        old_runtime->conflict_zone_carry_count;
                for (int index = 0;
                     index < carry_conflict_zone_count; ++index) {
                        carry_conflict_zone[index] =
                                old_runtime->conflict_zone_carry_zone[index];
                        carry_conflict_zone_entry[index] =
                                old_runtime
                                        ->conflict_zone_carry_entry_node[index];
                        carry_conflict_zone_exit[index] =
                                old_runtime
                                        ->conflict_zone_carry_exit_node[index];
                        carry_conflict_zone_exit_distance[index] =
                                old_runtime
                                        ->conflict_zone_carry_exit_distance_mm[
                                                index];
                        carry_conflict_zone_ticket[index] =
                                old_runtime
                                        ->conflict_zone_carry_ticket[index];
                        carry_conflict_zone_mode[index] =
                                old_runtime
                                        ->conflict_zone_carry_mode[index];
                }
                if (!old_runtime->route_valid ||
                    dispatch_jobs[slot]
                            .current_route_offset < 0 ||
                    dispatch_jobs[slot]
                            .current_route_offset >=
                            old_runtime->route.node_count ||
                    build_turnout_plan_range_checked(
                            &old_runtime->route,
                            old_runtime->leg_start_offset,
                            old_runtime->route.node_count - 1,
                            &carry_confirmed_turnouts) < 0) {
                        return -1;
                }
                if (force_reverse_first &&
                    carry_position_estimated &&
                    dispatch_jobs[slot].destination_offset_mm > 0 &&
                    dispatch_jobs[slot].current_route_offset ==
                            dispatch_jobs[slot].target_route_offset) {
                        int raw_current_distance_mm;
                        if (Tc2RouteDistanceAtOffset(
                                    dispatch_track,
                                    &old_runtime->route,
                                    dispatch_jobs[slot]
                                            .current_route_offset,
                                    &raw_current_distance_mm) < 0) {
                                return -1;
                        }
                        int virtual_prefix_mm =
                                dispatch_jobs[slot]
                                        .destination_distance_mm -
                                raw_current_distance_mm;
                        /*
                         * This narrow carry is valid only for the modeled
                         * interior endpoint immediately downstream of its
                         * anchor detector.  Any other estimated hold keeps the
                         * existing ambiguity/localization path instead of
                         * inventing a negative route scalar.
                         */
                        if (virtual_prefix_mm > 0 &&
                            virtual_prefix_mm ==
                                    dispatch_jobs[slot]
                                            .destination_offset_mm) {
                                carry_preorigin_distance_mm =
                                        virtual_prefix_mm;
                        }
                }
                if (carry_position_estimated &&
                    old_runtime->stop_purpose !=
                            TC2_STOP_TRAFFIC) {
                        /*
                         * The reservation also owns braking guards and
                         * paired-turnout resources that the train cannot
                         * physically occupy.  Keep that complete reservation
                         * live until localization, but use only the travelled
                         * corridor from the last trusted sensor through the
                         * virtual endpoint as the position ambiguity set.
                         * Otherwise a virtual endpoint that crosses a turnout
                         * can falsely place the train on an adjacent protected
                         * branch and make every safe CURRENT route appear
                         * blocked.
                         */
                        int ambiguity_first =
                                old_runtime
                                        ->reservation_anchor_offset;
                        int ambiguity_last =
                                dispatch_jobs[slot]
                                        .destination_route_offset;
                        if (ambiguity_first < 0 ||
                            ambiguity_first >
                                    dispatch_jobs[slot]
                                            .current_route_offset) {
                                ambiguity_first =
                                        dispatch_jobs[slot]
                                                .current_route_offset;
                        }
                        if (ambiguity_last <
                                    dispatch_jobs[slot]
                                            .current_route_offset ||
                            ambiguity_last >=
                                    old_runtime->route.node_count ||
                            build_route_slice(
                                    &old_runtime->route,
                                    ambiguity_first,
                                    ambiguity_last,
                                    &carry_ambiguity) < 0 ||
                            !route_contains_node(
                                    &carry_ambiguity,
                                    current_node)) {
                                return -1;
                        }
                }
                carry_confirmed_turnouts_valid = 1;
                carry_motion_route =
                        old_runtime->route;
                carry_motion_origin_offset =
                        dispatch_jobs[slot]
                                .current_route_offset;
        } else {
                if (start_is_in_use(request->start_index, request->train)) {
                        return -1;
                }
                if (slot >= 0) {
                        /*
                         * A CURRENT trip carries a live destination hold.
                         * Never let a later fixed-bay command overwrite that
                         * managed position and eventually replace the
                         * reservation under the physical train.
                         */
                        if (dispatch_jobs[slot].plan_generation != 0 ||
                            dispatch_jobs[slot].hold_active ||
                            dispatch_jobs[slot].start_index ==
                                    TC2_DISPATCH_START_CURRENT ||
                            (dispatch_jobs[slot].state !=
                                             TC2_JOB_STAGED &&
                             dispatch_jobs[slot].state !=
                                             TC2_JOB_WAITING)) {
                                return -1;
                        }
                }
        }
        if (slot < 0) slot = find_empty_job();
        if (slot < 0 ||
            CanTrainSetSpeed(dispatch_can_tid, request->train, 0) < 0) {
                return -1;
        }

        /*
         * A successful re-stage begins a new logical route epoch.  Invalidate
         * the prior immutable publication before clearing and rebuilding the
         * slot so stale page tokens cannot be spliced into the new route.
         */
        invalidate_projection(slot);
        clear_job(&dispatch_jobs[slot]);
        clear_runtime(&dispatch_runtime[slot]);
        dispatch_runtime[slot].conflict_zone_carry_count =
                carry_conflict_zone_count;
        for (int index = 0;
             index < carry_conflict_zone_count; ++index) {
                dispatch_runtime[slot].conflict_zone_carry_zone[index] =
                        carry_conflict_zone[index];
                dispatch_runtime[slot]
                        .conflict_zone_carry_entry_node[index] =
                        carry_conflict_zone_entry[index];
                dispatch_runtime[slot]
                        .conflict_zone_carry_exit_node[index] =
                        carry_conflict_zone_exit[index];
                dispatch_runtime[slot]
                        .conflict_zone_carry_exit_distance_mm[index] =
                        carry_conflict_zone_exit_distance[index];
                /*
                 * A CURRENT plan has a new route-distance origin.  The old
                 * physical owner remains, but its old scalar must never be
                 * compared with the new monitor.  Sensor-confirmed graph
                 * geometry will retire it safely instead.
                 */
                dispatch_runtime[slot]
                        .conflict_zone_carry_scalar_proof_valid[index] = 0;
                dispatch_runtime[slot]
                        .conflict_zone_carry_ticket[index] =
                        carry_conflict_zone_ticket[index];
                dispatch_runtime[slot]
                        .conflict_zone_carry_mode[index] =
                        carry_conflict_zone_mode[index];
        }
        /*
         * A CURRENT route may carry only proved physical occupancy.  CAN
         * batch tokens and per-action cursors belong to the old immutable
         * route generation and must never be resurrected after clear_runtime.
         */
        /* Every CURRENT/reroute plan starts a fresh CAN health epoch. */
        dispatch_jobs[slot].train = request->train;
        dispatch_jobs[slot].start_index = request->start_index;
        dispatch_jobs[slot].speed = request->speed;
        dispatch_jobs[slot].destination_index =
                request->destination_index;
        dispatch_jobs[slot].state = TC2_JOB_STAGED;
        dispatch_jobs[slot].provisional_prediction =
                provisional;
        if (provisional) {
                dispatch_jobs[slot]
                        .prediction_velocity_um_per_tick =
                        Tc2MotionProvisionalVelocityUmPerTick(
                                request->speed);
                dispatch_jobs[slot]
                        .prediction_stop_distance_um =
                        Tc2MotionProvisionalStopDistanceUm(
                                request->speed);
                dispatch_jobs[slot]
                        .prediction_stop_distance_mm =
                        Tc2MotionProvisionalStopDistanceMm(
                                request->speed);
                if (dispatch_jobs[slot]
                                .prediction_velocity_um_per_tick < 1 ||
                    dispatch_jobs[slot]
                                .prediction_stop_distance_um < 0 ||
                    dispatch_jobs[slot]
                                .prediction_stop_distance_mm < 0) {
                        clear_job(&dispatch_jobs[slot]);
                        clear_runtime(&dispatch_runtime[slot]);
                        return -1;
                }
        }
        dispatch_jobs[slot].current_node = current_node;
        dispatch_jobs[slot].plan_generation = current_generation;
        dispatch_jobs[slot].hold_active =
                request->start_index == TC2_DISPATCH_START_CURRENT;
        dispatch_runtime[slot].start_node =
                request->start_index == TC2_DISPATCH_START_CURRENT ?
                        current_node :
                        TrackFindNodeByName(
                                dispatch_track,
                                Tc2DispatchStartNodeName(
                                        request->start_index));
        if (request->start_index ==
            TC2_DISPATCH_START_CURRENT) {
                dispatch_runtime[slot].replace_plan_expected = 1;
                dispatch_runtime[slot]
                        .replace_expected_generation =
                        current_generation;
                dispatch_runtime[slot]
                        .replace_expected_destination =
                        current_destination;
                dispatch_runtime[slot]
                        .stage_attributed_sequence =
                        stage_attributed_sequence;
                dispatch_runtime[slot]
                        .stage_sensor_baseline_valid =
                        stage_sensor_baseline_valid;
        }
        if (carry_ambiguity_active) {
                dispatch_runtime[slot].carry_ambiguity_footprint =
                        carry_ambiguity;
                dispatch_runtime[slot].carry_ambiguity_active = 1;
                dispatch_runtime[slot].carry_position_estimated =
                        carry_position_estimated;
                dispatch_runtime[slot].carry_previous_speed =
                        current_speed;
                dispatch_runtime[slot]
                        .carry_preorigin_distance_mm =
                        carry_preorigin_distance_mm;
                if (carry_confirmed_turnouts_valid) {
                        dispatch_runtime[slot]
                                .carry_confirmed_turnouts =
                                carry_confirmed_turnouts;
                        dispatch_runtime[slot]
                                .carry_confirmed_turnouts_valid = 1;
                        dispatch_runtime[slot]
                                .carry_motion_route =
                                carry_motion_route;
                        dispatch_runtime[slot]
                                .carry_motion_origin_offset =
                                carry_motion_origin_offset;
                }
                dispatch_jobs[slot].position_estimated =
                        carry_position_estimated;
        }
        dispatch_runtime[slot].head_on_recovery_active =
                carry_head_on_recovery;
        dispatch_runtime[slot].head_on_recovery_cleared = 0;
        dispatch_runtime[slot].force_reverse_first =
                force_reverse_first;
        dispatch_runtime[slot].planned_launch_speed =
                carry_position_estimated &&
                        current_speed < request->speed ?
                current_speed : request->speed;
        return dispatch_runtime[slot].start_node >= 0 ? 0 : -1;
}

/*
 * Production-only admission boundary.  Keep stage_job generic so the route,
 * reservation, and recovery core can be exercised with synthetic train
 * addresses, but never let an uncommissioned locomotive reach that core from
 * the live server.  This wrapper is intentionally small and directly
 * testable: a rejected request cannot allocate a job, reserve track, or emit
 * speed/turnout CAN traffic because stage_job is never entered.
 */
static int production_stage_job(const tc2_dispatch_request *request) {
        if (!request ||
            !dispatch_train_is_supported(request->train)) {
                return -1;
        }
        return stage_job(request);
}

static int block_node(int node, int blocked) {
        if (node < 0 || node >= TRACK_MAX ||
            dispatch_track[node].type == NODE_NONE) {
                return -1;
        }
        int reverse = physical_reverse_index(node);
        if (blocked) {
                track_reservation_snapshot snapshot;
                train_sensor_snapshot_t sensors;
                if (TrackReservationServerSnapshot(
                            dispatch_reservation_tid, &snapshot) < 0 ||
                    TrainSensorGetLatest(
                            dispatch_sensor_tid, &sensors) < 0 ||
                    snapshot.owner_by_node[node] != 0 ||
                    (reverse >= 0 &&
                     snapshot.owner_by_node[reverse] != 0) ||
                    (node < TRAIN_SENSOR_COUNT &&
                     sensors.sensor_state[node]) ||
                    (reverse >= 0 &&
                     reverse < TRAIN_SENSOR_COUNT &&
                     sensors.sensor_state[reverse])) {
                        return -1;
                }
        }
        dispatch_blocked_by_node[node] = blocked ? 1 : 0;
        if (reverse >= 0) {
                dispatch_blocked_by_node[reverse] = blocked ? 1 : 0;
        }
        return 0;
}

static int last_sensor_after(const track_route *route, int offset) {
        int last = -1;
        for (int next = offset + 1;
             next < route->node_count; ++next) {
                if (dispatch_track[route->nodes[next]].type ==
                    NODE_SENSOR) {
                        last = next;
                }
        }
        return last;
}

static int sensor_before_offset(
        const track_route *route, int offset) {
        if (!route || offset < 1 ||
            offset > route->node_count) {
                return -1;
        }
        for (int prior = offset - 1;
             prior >= 0; --prior) {
                if (dispatch_track[
                            route->nodes[prior]].type ==
                    NODE_SENSOR) {
                        return prior;
                }
        }
        return -1;
}

static int init_leg_monitor(int slot, unsigned int baseline_sequence) {
        tc2_dispatch_job_snapshot *job = &dispatch_jobs[slot];
        tc2_dispatch_runtime *runtime = &dispatch_runtime[slot];
        runtime->leg_end_offset = TrackRouteMotionLegEnd(
                dispatch_track, &runtime->route,
                runtime->leg_start_offset);
        if (runtime->leg_end_offset >
            job->destination_route_offset) {
                runtime->leg_end_offset =
                        job->destination_route_offset;
        }
        if (runtime->leg_end_offset < runtime->leg_start_offset ||
            runtime->leg_end_offset >
                    job->destination_route_offset) {
                return -1;
        }
        runtime->monitor_terminal_offset = runtime->leg_end_offset;
        if (runtime->leg_end_offset ==
            job->destination_route_offset) {
                int guard_sensor = last_sensor_after(
                        &runtime->route,
                        job->destination_route_offset);
                if (guard_sensor >= 0) {
                        runtime->monitor_terminal_offset = guard_sensor;
                }
        }
        if (Tc2RouteMonitorInit(
                    dispatch_track, &runtime->route,
                    runtime->leg_start_offset,
                    runtime->monitor_terminal_offset,
                    baseline_sequence, &runtime->monitor) < 0) {
                return -1;
        }
        runtime->last_journal_sequence = baseline_sequence;
        runtime->target_seen = 0;
        runtime->target_corroborated = 0;
        runtime->reservation_anchor_offset =
                runtime->leg_start_offset;
        runtime->pending_missing_offset = -1;
        job->current_route_offset = runtime->leg_start_offset;
        job->current_node =
                runtime->route.nodes[runtime->leg_start_offset];
        if (Tc2RouteDistanceAtOffset(
                    dispatch_track, &runtime->route,
                    runtime->leg_start_offset,
                    &job->confirmed_distance_mm) < 0) {
                return -1;
        }
        runtime->motion_anchor_distance_um =
                (int64_t)job->confirmed_distance_mm * 1000;
        runtime->motion_anchor_tick = -1;
        runtime->motion_anchor_from_traffic = 0;
        return 0;
}

static int effective_confirmed_distance_mm(
        const tc2_dispatch_runtime *runtime,
        int *distance_mm) {
        if (!runtime || !distance_mm) return -1;
        if (runtime->preorigin_sensor_pending) {
                if (runtime->preorigin_distance_mm <= 0) return -1;
                *distance_mm = -runtime->preorigin_distance_mm;
                return 0;
        }
        if (runtime->monitor.confirmed_distance_mm < 0) return -1;
        *distance_mm = runtime->monitor.confirmed_distance_mm;
        return 0;
}

static void update_next_sensor(int slot) {
        tc2_route_sensor_point next;
        tc2_route_sensor_point next_next;
        tc2_dispatch_runtime *runtime = &dispatch_runtime[slot];
        tc2_dispatch_job_snapshot *job = &dispatch_jobs[slot];
        if (runtime->preorigin_sensor_pending) {
                int origin = runtime->leg_start_offset;
                job->next_sensor_node =
                        origin >= 0 &&
                        origin < runtime->route.node_count &&
                        dispatch_track[
                                runtime->route.nodes[origin]].type ==
                                NODE_SENSOR ?
                        runtime->route.nodes[origin] : -1;
                return;
        }
        if (!runtime->localization_only &&
            job->destination_offset_mm > 0 &&
            runtime->target_seen) {
                job->next_sensor_node = -1;
                return;
        }
        int count = Tc2RouteExpectedSensors(
                dispatch_track, &runtime->route,
                runtime->monitor.confirmed_offset,
                runtime->monitor_terminal_offset, &next, &next_next);
        (void)next_next;
        job->next_sensor_node = count > 0 ? next.node_index : -1;
}

/*
 * The physically accepted train-14 stopping curve is independent of the
 * departure siding. Every catalogued A-F -> d1-d8 trip therefore keeps the
 * requested command speed and issues zero exactly one measured
 * command-to-rest distance before the endpoint. The same curve is the
 * provisional seed for T15/T17/T18 until each locomotive is measured;
 * routing, sensor rebasing, and turnout preparation are already train
 * independent and remain specific to the selected shortest forward route.
 */
static int uses_measured_direct_stop(
        const tc2_dispatch_job_snapshot *job) {
        return job &&
                job->start_index >= 0 &&
                job->start_index < TC2_DISPATCH_START_COUNT &&
                job->destination_index >= 0 &&
                job->destination_index <
                        TC2_DISPATCH_DESTINATION_COUNT;
}

/*
 * Keep physical endpoint calibration separate from the conservative traffic
 * envelope.  After rolling-authority/traffic scheduling was enabled, the
 * latest Track-D runs of T14 from fixed bay A established a common command-100
 * correction of 120 mm. B selects the identical directed destination side,
 * final sensor, and turnout suffix for every d1-d8 route (its start prefix is
 * only 11 mm longer), so it shares the same physical endpoint calibration.
 * C/D/E share the same physical suffix from C7 onward and approach every
 * endpoint from the same catalogued side. Their direction-specific anchor
 * offset is already included in route distance, while the locomotive's
 * measured command-to-rest curve remains unchanged.
 * F approaches d1-d5/d7/d8 from the same directed sides as the accepted A/B
 * runs. Its d6 anchor is the co-located reverse sensor, so the zero endpoint
 * offset is unchanged. F therefore inherits the common T14 endpoint
 * correction but not the C/D/E-only d5 residual.
 * At command 120, d1 still overshot by 100 mm with the
 * previous 230 mm early-command correction, then the common command-120
 * calibration was advanced by another 45 mm, so d1 now commands zero 375 mm
 * earlier. Conversely d3 and d8 stopped before their final preceding sensors:
 * their short downstream offsets exposed that the extra 230 mm was wrong for
 * those endpoints; after the same latest 45 mm advance they use a 45 mm
 * correction. Other command-120 endpoints now use the provisional 275 mm
 * correction. Keep these physical corrections out of collision gaps,
 * reservations, other trains, and other start bays.
 */
static int measured_direct_stop_distance_um(
        const tc2_dispatch_job_snapshot *job, int speed) {
        int stop_um =
                Tc2MotionMeasuredStopDistanceUm(speed);
        if (stop_um < 0) return -1;
        int correction_um = 0;
        if (job && job->train == 14 &&
            job->start_index >= 0 &&
            job->start_index <= 5) {
                if (speed == 100) {
                        correction_um = 120000;
                        /*
                         * The C->d5 physical run first commanded zero at
                         * D5.  d5 is the E5/E6 detector centre, only 376 mm
                         * beyond D5 on this directed route, while T14 coasts
                         * 875 mm from command 100.  The observed settle point
                         * was therefore 500 mm beyond d5 (875 - 376, rounded
                         * by the operator). D and E share this exact suffix,
                         * so transfer the same residual only to C/D/E d5 at
                         * speed 100. A/B and every other endpoint retain
                         * their already accepted correction.
                         */
                        if (job->start_index >= 2 &&
                            job->start_index <= 4 &&
                            job->destination_index == 4) {
                                correction_um += 500000;
                        }
                } else if (speed == 120) {
                        if (job->destination_index == 0) {
                                correction_um = 375000;
                        } else if (job->destination_index != 2 &&
                                   job->destination_index != 7) {
                                correction_um = 275000;
                        } else {
                                correction_um = 45000;
                        }
                }
        }
        if (correction_um > 0) {
                if (stop_um > 0x7fffffff - correction_um) {
                        return -1;
                }
                stop_um += correction_um;
        }
        return stop_um;
}

static int measured_direct_stop_distance_mm(
        const tc2_dispatch_job_snapshot *job, int speed) {
        int stop_um =
                measured_direct_stop_distance_um(
                        job, speed);
        return stop_um < 0 ? -1 :
                (stop_um + 999) / 1000;
}

static int measured_direct_velocity_um_per_tick(
        const tc2_dispatch_job_snapshot *job, int speed) {
        (void)job;
        return Tc2MotionProvisionalVelocityUmPerTick(speed);
}

/*
 * Direct dispatch always commands zero from remaining route distance and the
 * measured coast curve. A detector may confirm the coast passed the final
 * physical anchor, but it never changes speed or starts a second crawl.
 */
static int uses_final_sensor_crawl(
        const tc2_dispatch_job_snapshot *job) {
        (void)job;
        return 0;
}

static int uses_precision_approach(
        const tc2_dispatch_job_snapshot *job) {
        return job && job->destination_index >= 0 &&
                job->destination_index <
                        TC2_DISPATCH_DESTINATION_COUNT &&
                !uses_measured_direct_stop(job);
}

static int reduce_for_anchor_stage(
        int slot, int requested_speed) {
        tc2_dispatch_job_snapshot *job = &dispatch_jobs[slot];
        tc2_dispatch_runtime *runtime = &dispatch_runtime[slot];
        int reduced = requested_speed;

        if (reduced > job->speed) reduced = job->speed;
        if (reduced < 1) return -1;
        if (runtime->command_speed == reduced) {
                job->command_speed = reduced;
                return 0;
        }
        if (CanTrainSetSpeedPriority(
                    dispatch_can_tid, job->train, reduced) < 0) {
                return -1;
        }
        if (runtime->motion_speed_ceiling <
            runtime->command_speed) {
                runtime->motion_speed_ceiling =
                        runtime->command_speed;
        }
        runtime->command_speed = reduced;
        runtime->speed_reduction_pending = 1;
        runtime->precision_approach_active = 1;
        runtime->observed_velocity_um_per_tick = -1;
        job->command_speed = reduced;
        return 0;
}

static int schedule_motion_deadlines(int slot, int now) {
        tc2_dispatch_job_snapshot *job = &dispatch_jobs[slot];
        tc2_dispatch_runtime *runtime = &dispatch_runtime[slot];
        int confirmed_distance_mm;
        if (effective_confirmed_distance_mm(
                    runtime, &confirmed_distance_mm) < 0) {
                return -1;
        }
        int final_leg =
                runtime->leg_end_offset ==
                job->destination_route_offset;
        int point_prediction =
                final_leg &&
                !uses_measured_direct_stop(job) &&
                !(runtime->target_seen &&
                  job->destination_offset_mm > 0) &&
                (job->provisional_prediction ||
                 runtime->prediction_plan_valid);
        int braking =
                final_leg && uses_measured_direct_stop(job) ?
                        measured_direct_stop_distance_mm(
                                job,
                                runtime->command_speed) :
                point_prediction ?
                        Tc2MotionProvisionalStopDistanceMm(
                                runtime->command_speed) :
                !final_leg ?
                        Tc2MotionStopDistanceMm(
                                runtime->command_speed) :
                        Tc2MotionBrakingDistanceMm(
                                runtime->command_speed);
        int stop_distance;
        int terminal_distance;
        int stop_remaining;
        int watchdog_remaining;
        int prediction_stop_um = -1;
        int prediction_velocity_um_per_tick = -1;
        if (braking < 0 ||
            Tc2RouteDistanceAtOffset(
                    dispatch_track, &runtime->route,
                    runtime->monitor_terminal_offset,
                    &terminal_distance) < 0) {
                return -1;
        }
        if (final_leg) {
                stop_distance =
                        job->destination_distance_mm;
        } else if (Tc2RouteDistanceAtOffset(
                           dispatch_track, &runtime->route,
                           runtime->leg_end_offset,
                           &stop_distance) < 0) {
                return -1;
        }
        int64_t controller_progress_um =
                (int64_t)confirmed_distance_mm * 1000;
        if (uses_measured_direct_stop(job) &&
            runtime->motion_anchor_from_traffic &&
            runtime->motion_anchor_tick >= 0 &&
            runtime->motion_anchor_distance_um >
                    controller_progress_um) {
                controller_progress_um =
                        runtime->motion_anchor_distance_um;
        }
        int controller_progress_mm = confirmed_distance_mm;
        if (controller_progress_um >= 0) {
                controller_progress_mm =
                        (int)((controller_progress_um + 999) / 1000);
        }
        if (point_prediction &&
            runtime->prediction_plan_valid) {
                int64_t point_remaining_um =
                        runtime->prediction_plan
                                .corrected_endpoint_distance_um -
                        (int64_t)confirmed_distance_mm * 1000;
                if (point_remaining_um < 0 ||
                    point_remaining_um >
                            (int64_t)0x7fffffff * 1000) {
                        return -1;
                }
                stop_remaining =
                        (int)((point_remaining_um + 999) / 1000);
        } else if (uses_measured_direct_stop(job)) {
                stop_remaining =
                        stop_distance - controller_progress_mm;
        } else {
                stop_remaining = stop_distance -
                        confirmed_distance_mm;
        }
        watchdog_remaining = terminal_distance -
                confirmed_distance_mm;
        if (stop_remaining < 0 || watchdog_remaining < 0) return -1;
        int before_braking = stop_remaining > braking ?
                stop_remaining - braking : 0;
        int braking_ticks;
        if (point_prediction) {
                prediction_velocity_um_per_tick =
                        runtime->observed_velocity_um_per_tick > 0 ?
                        runtime->observed_velocity_um_per_tick :
                        Tc2MotionProvisionalVelocityUmPerTick(
                                runtime->command_speed);
                prediction_stop_um =
                        Tc2MotionStopDistanceForVelocityUm(
                                prediction_velocity_um_per_tick);
                if (prediction_stop_um < 0 ||
                    stop_remaining > 0x7fffffff / 1000) {
                        return -1;
                }
                int64_t remaining_um =
                        runtime->prediction_plan_valid ?
                        runtime->prediction_plan
                                .corrected_endpoint_distance_um -
                                (int64_t)confirmed_distance_mm * 1000 :
                        (int64_t)stop_remaining * 1000;
                int before_braking_um =
                        remaining_um >
                                        prediction_stop_um ?
                        (int)(remaining_um -
                              prediction_stop_um) :
                        0;
                braking_ticks =
                        Tc2MotionTravelTicksUmAtVelocity(
                                prediction_velocity_um_per_tick,
                                before_braking_um);
        } else if (uses_measured_direct_stop(job)) {
                /*
                 * Direct physical dispatch is a distance-domain controller.
                 * A raw interval between two detectors is not allowed to
                 * replace the calibrated command-speed velocity: curved
                 * sections, detector latency, and batched reports otherwise
                 * move the spatial zero-command point. Sensors rebase only
                 * the known route distance; the currently acknowledged
                 * command speed selects the stable velocity and
                 * command-to-rest curves.  This differs from job->speed only
                 * when a short final authority safely resumes at a lower
                 * speed whose stopping distance fits the remaining track.
                 */
                prediction_velocity_um_per_tick =
                        measured_direct_velocity_um_per_tick(
                                job, runtime->command_speed);
                prediction_stop_um =
                        measured_direct_stop_distance_um(
                                job,
                                runtime->command_speed);
                if (prediction_velocity_um_per_tick < 1 ||
                    prediction_stop_um < 0 ||
                    before_braking > 0x7fffffff / 1000) {
                        return -1;
                }
                braking_ticks =
                        Tc2MotionTravelTicksUmAtVelocity(
                                prediction_velocity_um_per_tick,
                                before_braking * 1000);
        } else {
                braking_ticks =
                        Tc2MotionFastTravelTicks(
                                runtime->command_speed,
                                before_braking);
        }
        int watchdog_ticks =
                point_prediction ||
                uses_measured_direct_stop(job) ?
                        Tc2MotionProvisionalSlowTravelTicks(
                                runtime->command_speed,
                                watchdog_remaining) :
                        Tc2MotionSlowTravelTicks(
                                runtime->command_speed,
                                watchdog_remaining);
        if (braking_ticks < 0 || watchdog_ticks < 0) return -1;
        job->braking_at_tick =
                tick_after(now, braking_ticks);
        if (point_prediction ||
            uses_measured_direct_stop(job)) {
                job->prediction_velocity_um_per_tick =
                        prediction_velocity_um_per_tick;
                job->prediction_stop_distance_um =
                        prediction_stop_um;
                job->prediction_stop_distance_mm = braking;
                job->prediction_anchor_node = job->current_node;
                job->prediction_anchor_tick = now;
                job->prediction_anchor_distance_mm =
                        uses_measured_direct_stop(job) ?
                        controller_progress_mm :
                        confirmed_distance_mm;
                job->prediction_command_at_tick =
                        job->braking_at_tick;
                job->prediction_timing_valid = 1;
                if (uses_measured_direct_stop(job)) {
                        job->estimated_distance_um =
                                controller_progress_um;
                }
        }
        job->watchdog_margin_ticks =
                Tc2MotionWatchdogMarginTicks(runtime->command_speed);
        if (runtime->leg_end_offset <
                    job->destination_route_offset &&
            runtime->approach_sent) {
                job->watchdog_margin_ticks = 5;
        }
        job->watchdog_at_tick =
                tick_after(
                        now, watchdog_ticks +
                                job->watchdog_margin_ticks);
        job->remaining_distance_mm =
                job->destination_distance_mm -
                (uses_measured_direct_stop(job) ?
                 controller_progress_mm :
                 confirmed_distance_mm);
        if (job->remaining_distance_mm < 0) {
                job->remaining_distance_mm = 0;
        }
        update_next_sensor(slot);
        return 0;
}

/*
 * Estimate physical progress on the selected route from the latest confirmed
 * route distance. The detector contributes position only. It cannot select a
 * stopping distance, directly stop a final-leg train, or replace the stable
 * command-speed velocity with one noisy sensor interval.
 */
static int measured_direct_progress_um(
        int slot, int now, int64_t *progress_um,
        int64_t *zero_command_distance_um) {
        if (slot < 0 || slot >= TC2_DISPATCH_MAX_JOBS ||
            !progress_um || !zero_command_distance_um) {
                return -1;
        }
        tc2_dispatch_job_snapshot *job = &dispatch_jobs[slot];
        tc2_dispatch_runtime *runtime = &dispatch_runtime[slot];
        if (!uses_measured_direct_stop(job) ||
            runtime->leg_end_offset !=
                    job->destination_route_offset ||
            job->destination_distance_mm < 0) {
                return -1;
        }
        int anchor_tick =
                runtime->motion_anchor_from_traffic ?
                runtime->motion_anchor_tick :
                runtime->last_confirmed_tick;
        int64_t anchor_distance_um =
                runtime->motion_anchor_from_traffic ?
                runtime->motion_anchor_distance_um :
                (int64_t)runtime->monitor
                        .confirmed_distance_mm * 1000;
        if (anchor_tick == -1 ||
            anchor_distance_um < 0) {
                return -1;
        }
        int velocity =
                measured_direct_velocity_um_per_tick(
                        job, runtime->command_speed);
        int stop_um =
                measured_direct_stop_distance_um(
                        job,
                        runtime->command_speed);
        if (velocity < 1 || stop_um < 0) return -1;

        int age = motion_tick_age(
                now, anchor_tick);
        if (age < 0) return -1;
        int64_t confirmed_um =
                anchor_distance_um;
        int64_t destination_um =
                (int64_t)job->destination_distance_mm * 1000;
        int64_t estimated_um =
                confirmed_um + (int64_t)age * velocity;
        if (estimated_um < confirmed_um) return -1;
        if (estimated_um > destination_um) {
                estimated_um = destination_um;
        }
        int64_t command_um = destination_um - stop_um;
        if (command_um < 0) command_um = 0;

        *progress_um = estimated_um;
        *zero_command_distance_um = command_um;
        return 0;
}

/*
 * Return a monotonic route scalar for live separation checks.  This is still
 * a calibrated model rather than an encoder, so collision thresholds use the
 * larger braking envelope and the reservation layer remains authoritative.
 * A traffic stop owns a finite coast endpoint; after settle the scalar is
 * frozen there instead of being recomputed from an old sensor on resume.
 */
static int runtime_private_motion_progress_um(
        int slot, int now, int64_t *progress_um) {
        if (slot < 0 || slot >= TC2_DISPATCH_MAX_JOBS ||
            !progress_um) {
                return -1;
        }
        tc2_dispatch_job_snapshot *job =
                &dispatch_jobs[slot];
        tc2_dispatch_runtime *runtime =
                &dispatch_runtime[slot];
        int64_t minimum_preorigin_um =
                -(int64_t)runtime->preorigin_distance_mm * 1000;
        if (!runtime->route_valid ||
            runtime->motion_anchor_tick < 0 ||
            (runtime->motion_anchor_distance_um < 0 &&
             (!runtime->preorigin_sensor_pending ||
              runtime->preorigin_distance_mm <= 0 ||
              runtime->motion_anchor_distance_um <
                      minimum_preorigin_um))) {
                return -1;
        }
        if (job->state == TC2_JOB_TRAFFIC_HOLD ||
            job->state == TC2_JOB_REVERSING) {
                int64_t held_progress_um =
                        job->state == TC2_JOB_TRAFFIC_HOLD &&
                                runtime->preorigin_sensor_pending &&
                                runtime->preorigin_traffic_hold_valid ?
                        runtime->preorigin_traffic_hold_distance_um :
                        job->state == TC2_JOB_TRAFFIC_HOLD &&
                                runtime->traffic_hold_distance_um >= 0 ?
                        runtime->traffic_hold_distance_um :
                        runtime->motion_anchor_distance_um;
                *progress_um = held_progress_um;
                return 0;
        }

        int speed = runtime->command_speed;
        if (runtime->motion_speed_ceiling > speed) {
                speed = runtime->motion_speed_ceiling;
        }
        if (speed <= 0) speed = job->speed;
        int velocity =
                Tc2MotionProvisionalVelocityUmPerTick(speed);
        int age = motion_tick_age(
                now, runtime->motion_anchor_tick);
        if (velocity < 1 || age < 0) return -1;
        int64_t estimated =
                runtime->motion_anchor_distance_um +
                (int64_t)age * velocity;
        if (estimated <
            runtime->motion_anchor_distance_um) {
                return -1;
        }
        /*
         * Model time may predict arrival at route[0], but it is not physical
         * evidence that the pickup crossed that detector.  Until the fresh
         * origin pulse arrives, the private scalar may approach zero but
         * must not enter the ordinary positive route domain.
         */
        if (runtime->preorigin_sensor_pending && estimated > 0) {
                estimated = 0;
        }
        if ((runtime->stop_purpose == TC2_STOP_TRAFFIC ||
             job->state == TC2_JOB_CANCEL_BRAKING) &&
            ((runtime->preorigin_sensor_pending &&
              runtime->preorigin_traffic_hold_valid) ||
             runtime->traffic_hold_distance_um >= 0)) {
                int64_t hold_um =
                        runtime->preorigin_sensor_pending &&
                                runtime->preorigin_traffic_hold_valid ?
                        runtime->preorigin_traffic_hold_distance_um :
                        runtime->traffic_hold_distance_um;
                if (estimated > hold_um) estimated = hold_um;
        }
        int64_t destination_um =
                (int64_t)job->destination_distance_mm * 1000;
        if (destination_um >= 0 &&
            estimated > destination_um) {
                estimated = destination_um;
        }
        *progress_um = estimated;
        return 0;
}

static int runtime_motion_progress_um(
        int slot, int now, int64_t *progress_um) {
        if (runtime_private_motion_progress_um(
                    slot, now, progress_um) < 0) {
                return -1;
        }
        /*
         * All consumers outside the motion/deadline controller operate in
         * the selected route's nonnegative scalar domain.  Until the first
         * real reverse-origin detector fires, model age is not evidence that
         * the train crossed route[0]; keep UI, authority, conflict zones and
         * pairwise traffic pinned at that origin.  The private helper above
         * retains the measured negative prefix for stop/resume bookkeeping.
         */
        if (dispatch_runtime[slot].preorigin_sensor_pending) {
                *progress_um = 0;
        }
        return 0;
}

/*
 * A turnout may sit between two detectors, so conflict-zone progression
 * cannot wait exclusively for the next sensor.  It also must not treat the
 * nominal clock projection as proof that the train has already entered the
 * turnout: doing so advances next/next-next early and can switch the wrong
 * motor while the train is still approaching the current one.
 *
 * Use the sensor-confirmed scalar as the hard lower bound and only the
 * uncertainty-discounted portion of the calibrated projection beyond it.
 * This preserves geometry-based prediction between sensors while every
 * ordered sensor observation continuously rebases the estimate.
 */
static int runtime_conflict_zone_progress_lower_bound_um(
        int slot, int now, int64_t *progress_um) {
        if (slot < 0 || slot >= TC2_DISPATCH_MAX_JOBS ||
            !progress_um) {
                return -1;
        }
        int64_t projected_um = 0;
        if (runtime_motion_progress_um(
                    slot, now, &projected_um) < 0) {
                return -1;
        }
        tc2_dispatch_job_snapshot *job = &dispatch_jobs[slot];
        tc2_dispatch_runtime *runtime = &dispatch_runtime[slot];
        int speed = runtime->command_speed;
        if (runtime->motion_speed_ceiling > speed) {
                speed = runtime->motion_speed_ceiling;
        }
        if (speed <= 0) speed = job->speed;
        int uncertainty_mm = Tc2MotionUncertaintyMm(speed);
        if (uncertainty_mm < 0) return -1;

        int64_t lower_um = projected_um -
                (int64_t)uncertainty_mm * 1000;
        int confirmed_mm = -1;
        if (effective_confirmed_distance_mm(
                    runtime, &confirmed_mm) == 0) {
                int64_t confirmed_um = (int64_t)confirmed_mm * 1000;
                if (lower_um < confirmed_um) lower_um = confirmed_um;
        }
        if (lower_um < 0 || runtime->preorigin_sensor_pending) {
                lower_um = 0;
        }
        *progress_um = lower_um;
        return 0;
}

static int conflict_zone_settings_equal(
        const tc2_conflict_zone_setting *left,
        const tc2_conflict_zone_setting *right) {
        if (!left || !right ||
            left->action_count != right->action_count) {
                return 0;
        }
        for (int action = 0; action < left->action_count; ++action) {
                if (left->actions[action].switch_number !=
                            right->actions[action].switch_number ||
                    left->actions[action].direction !=
                            right->actions[action].direction) {
                        return 0;
                }
        }
        return 1;
}

static int conflict_zone_same_physical_node(int left, int right) {
        if (left < 0 || left >= TRACK_MAX ||
            right < 0 || right >= TRACK_MAX) {
                return 0;
        }
        return left == right || physical_reverse_index(left) == right;
}

static void clear_origin_zone_egress(
        tc2_dispatch_runtime *runtime) {
        if (!runtime) return;
        runtime->conflict_zone_origin_egress_active = 0;
        runtime->conflict_zone_origin_egress_step = -1;
        runtime->conflict_zone_origin_egress_zone =
                TC2_CONFLICT_ZONE_NONE;
        runtime->conflict_zone_origin_egress_ticket = 0;
        runtime->conflict_zone_origin_egress_sequence = 0;
}

/*
 * Validate the one exceptional use of an old owner ticket: after CURRENT
 * reverses, the train may leave the exact zone that still contains its
 * body.  The owner setting is the hard current-step setting; a next-next
 * prefetch is deliberately irrelevant here.
 */
static int origin_zone_egress_ready(int slot, int step_index) {
        if (slot < 0 || slot >= TC2_DISPATCH_MAX_JOBS) return 0;
        tc2_dispatch_runtime *runtime = &dispatch_runtime[slot];
        if (!runtime->conflict_zone_origin_egress_active ||
            step_index != runtime->conflict_zone_origin_egress_step ||
            step_index != runtime->conflict_zone_next_step ||
            !runtime->conflict_zone_plan_valid || step_index < 0 ||
            step_index >= runtime->conflict_zone_plan.step_count) {
                return 0;
        }
        const tc2_conflict_zone_route_step *step =
                &runtime->conflict_zone_plan.steps[step_index];
        tc2_conflict_zone_grant grant;
        return step->zone ==
                            runtime->conflict_zone_origin_egress_zone &&
                runtime->conflict_zone_origin_egress_ticket != 0 &&
                Tc2ConflictZoneQueryGrant(
                        &dispatch_conflict_zones, step->zone,
                        &grant) > 0 &&
                grant.owner_train == dispatch_jobs[slot].train &&
                grant.owner_ticket ==
                        runtime->conflict_zone_origin_egress_ticket &&
                grant.state == TC2_CONFLICT_ZONE_OCCUPIED &&
                conflict_zone_settings_equal(
                        &grant.setting, &step->setting);
}

static int arm_origin_zone_egress(int slot) {
        if (slot < 0 || slot >= TC2_DISPATCH_MAX_JOBS) return -1;
        tc2_dispatch_runtime *runtime = &dispatch_runtime[slot];
        tc2_dispatch_job_snapshot *job = &dispatch_jobs[slot];
        clear_origin_zone_egress(runtime);
        if (job->start_index != TC2_DISPATCH_START_CURRENT ||
            !runtime->force_reverse_first ||
            !runtime->conflict_zone_plan_valid ||
            runtime->conflict_zone_plan.step_count < 1) {
                return 0;
        }

        const int step_index = 0;
        const tc2_conflict_zone_route_step *step =
                &runtime->conflict_zone_plan.steps[step_index];
        if (step->first_route_offset < 0 ||
            step->first_route_offset >= runtime->route.node_count ||
            step->last_route_offset < 0 ||
            step->last_route_offset >= runtime->route.node_count) {
                return -1;
        }
        int new_entry =
                runtime->route.nodes[step->first_route_offset];
        int new_exit =
                runtime->route.nodes[step->last_route_offset];
        int leading_reverse =
                runtime->route.node_count > 1 &&
                dispatch_track[runtime->route.nodes[0]].reverse ==
                        &dispatch_track[runtime->route.nodes[1]];

        for (int carry = 0;
             carry < runtime->conflict_zone_carry_count; ++carry) {
                int carry_entry =
                        runtime->conflict_zone_carry_entry_node[carry];
                int carry_exit =
                        runtime->conflict_zone_carry_exit_node[carry];
                int exact_reversed_span =
                        conflict_zone_same_physical_node(
                                carry_entry, new_exit) &&
                        conflict_zone_same_physical_node(
                                carry_exit, new_entry);
                /*
                 * A route that starts at the detector on a zone's far edge
                 * is trimmed after its mandatory leading reverse.  The new
                 * zone step can therefore collapse to the reverse turnout
                 * node (for example A12,A11,MR1 gives MR1..MR1), rather than
                 * repeating the detector-to-turnout span BR1..A12 recorded
                 * by the live owner.  This is still the same physical
                 * egress only when the carry ends at route[0], its old entry
                 * is the collapsed reverse turnout, and the new step has no
                 * second physical endpoint.  Do not relax any ticket,
                 * OCCUPIED-state, or hard-setting checks below.
                 */
                int origin_trimmed_span =
                        leading_reverse &&
                        conflict_zone_same_physical_node(
                                new_entry, new_exit) &&
                        conflict_zone_same_physical_node(
                                carry_entry, new_entry) &&
                        conflict_zone_same_physical_node(
                                carry_exit,
                                runtime->route.nodes[0]);
                if (runtime->conflict_zone_carry_zone[carry] !=
                            step->zone ||
                    runtime->conflict_zone_carry_ticket[carry] == 0 ||
                    (!exact_reversed_span &&
                     !origin_trimmed_span)) {
                        continue;
                }
                tc2_conflict_zone_grant grant;
                if (Tc2ConflictZoneQueryGrant(
                            &dispatch_conflict_zones, step->zone,
                            &grant) <= 0 ||
                    grant.owner_train != job->train ||
                    grant.owner_ticket !=
                            runtime->conflict_zone_carry_ticket[carry] ||
                    grant.state != TC2_CONFLICT_ZONE_OCCUPIED ||
                    !conflict_zone_settings_equal(
                            &grant.setting, &step->setting)) {
                        continue;
                }
                runtime->conflict_zone_origin_egress_active = 1;
                runtime->conflict_zone_origin_egress_step = step_index;
                runtime->conflict_zone_origin_egress_zone = step->zone;
                runtime->conflict_zone_origin_egress_ticket =
                        grant.owner_ticket;
                runtime->conflict_zone_origin_egress_sequence =
                        runtime->last_journal_sequence;
                return 1;
        }
        return 0;
}

static int conflict_zone_step_ready(int slot, int step_index) {
        if (slot < 0 || slot >= TC2_DISPATCH_MAX_JOBS) return 0;
        tc2_dispatch_runtime *runtime = &dispatch_runtime[slot];
        if (!runtime->conflict_zone_plan_valid) {
                return 1;
        }
        /*
         * `step_count` is the one valid exhausted cursor: every physical
         * turnout on this route has already been consumed.  Do not treat a
         * negative cursor, or a cursor beyond the end, as ready.  Those are
         * corrupted scheduler state and must remain fail-closed rather than
         * silently granting movement authority.
         */
        if (step_index == runtime->conflict_zone_plan.step_count) return 1;
        if (step_index < 0 ||
            step_index > runtime->conflict_zone_plan.step_count) {
                return 0;
        }
        if (origin_zone_egress_ready(slot, step_index)) return 1;
        tc2_conflict_zone_grant grant;
        int zone = runtime->conflict_zone_plan.steps[step_index].zone;
        if (Tc2ConflictZoneQueryGrant(
                    &dispatch_conflict_zones, zone, &grant) <= 0 ||
            grant.owner_train != dispatch_jobs[slot].train ||
            runtime->conflict_zone_hard_step != step_index ||
            runtime->conflict_zone_hard_ticket == 0 ||
            grant.owner_ticket != runtime->conflict_zone_hard_ticket ||
            !conflict_zone_settings_equal(
                    &grant.setting,
                    &runtime->conflict_zone_plan.steps[step_index]
                             .setting)) {
                return 0;
        }
        return grant.state == TC2_CONFLICT_ZONE_SETTLED ||
               grant.state == TC2_CONFLICT_ZONE_OCCUPIED;
}

static void fill_conflict_zone_request(
        int slot, int step_index,
        tc2_conflict_zone_request *request) {
        tc2_dispatch_job_snapshot *job = &dispatch_jobs[slot];
        tc2_dispatch_runtime *runtime = &dispatch_runtime[slot];
        const tc2_conflict_zone_route_step *step =
                &runtime->conflict_zone_plan.steps[step_index];
        request->train = job->train;
        request->moving =
                runtime->command_speed > 0 &&
                job->state == TC2_JOB_RUNNING;
        request->track_key = step->approach_track_key;
        request->travel_direction = step->approach_direction;
        int centre_mm = job->estimated_distance_um >= 0 ?
                (int)(job->estimated_distance_um / 1000) :
                job->confirmed_distance_mm;
        request->front_rank_mm = centre_mm - step->entry_distance_mm;
        request->setting = step->setting;
}

/*
 * A carry entry is an old physical traversal whose lifecycle has not yet
 * been proved complete.  Even when the new route asks for the same zone and
 * setting, its old owner ticket is not authority for the new geometry.
 */
static int conflict_zone_has_unresolved_carry(
        const tc2_dispatch_runtime *runtime, int zone) {
        if (!runtime || zone < 0 || zone >= TC2_CONFLICT_ZONE_COUNT) {
                return 0;
        }
        for (int index = 0;
             index < runtime->conflict_zone_carry_count; ++index) {
                if (runtime->conflict_zone_carry_zone[index] == zone) {
                        return 1;
                }
        }
        return 0;
}

/* Phase one only enqueues. Grant selection happens after every slot queued. */
static int refresh_conflict_zone_requests(int slot) {
        tc2_dispatch_runtime *runtime = &dispatch_runtime[slot];
        tc2_dispatch_job_snapshot *job = &dispatch_jobs[slot];
        if (!runtime->conflict_zone_plan_valid ||
            job->state == TC2_JOB_EMPTY ||
            runtime->conflict_zone_next_step >=
                    runtime->conflict_zone_plan.step_count) {
                return 0;
        }
        int hard_step = runtime->conflict_zone_next_step;
        const tc2_conflict_zone_route_step *hard =
                &runtime->conflict_zone_plan.steps[hard_step];
        int origin_egress =
                runtime->conflict_zone_origin_egress_active &&
                runtime->conflict_zone_origin_egress_step == hard_step;
        if (origin_egress &&
            origin_zone_egress_ready(slot, hard_step)) {
                runtime->conflict_zone_hard_step = hard_step;
                runtime->conflict_zone_hard_ticket =
                        runtime->conflict_zone_origin_egress_ticket;
        } else if (origin_egress) {
                /*
                 * Never move under a stale or differently-set old owner.
                 * If that owner disappeared normally, fall back to a fresh
                 * FIFO request; otherwise fail closed.
                 */
                tc2_conflict_zone_grant grant;
                int still_same_owner =
                        Tc2ConflictZoneQueryGrant(
                                &dispatch_conflict_zones,
                                hard->zone, &grant) > 0 &&
                        grant.owner_train == job->train &&
                        grant.owner_ticket ==
                                runtime
                                  ->conflict_zone_origin_egress_ticket;
                if (still_same_owner &&
                    grant.state == TC2_CONFLICT_ZONE_OCCUPIED) {
                        return -1;
                }
                clear_origin_zone_egress(runtime);
                origin_egress = 0;
        }
        if (!origin_egress &&
            conflict_zone_has_unresolved_carry(runtime, hard->zone)) {
                runtime->conflict_zone_hard_step = -1;
                runtime->conflict_zone_hard_ticket = 0;
                return 0;
        }
        tc2_conflict_zone_request request;
        if (!origin_egress) {
                tc2_conflict_zone_grant existing;
                int owns_current_traversal =
                        runtime->conflict_zone_hard_step == hard_step &&
                        runtime->conflict_zone_hard_ticket != 0 &&
                        Tc2ConflictZoneQueryGrant(
                                &dispatch_conflict_zones, hard->zone,
                                &existing) > 0 &&
                        existing.owner_train == job->train &&
                        existing.owner_ticket ==
                                runtime->conflict_zone_hard_ticket &&
                        conflict_zone_settings_equal(
                                &existing.setting, &hard->setting);
                if (!owns_current_traversal) {
                        fill_conflict_zone_request(slot, hard_step, &request);
                        uint32_t hard_ticket = 0;
                        int hard_status = Tc2ConflictZoneRequestHard(
                                    &dispatch_conflict_zones, hard->zone,
                                    &request, &hard_ticket);
                        if (hard_status == 0 && hard_ticket != 0) {
                                runtime->conflict_zone_hard_step = hard_step;
                                runtime->conflict_zone_hard_ticket = hard_ticket;
                        } else if (hard_status < 0) {
                                return -1;
                        } else {
                                /*
                                 * A different, uncleared OCCUPIED/RELEASE
                                 * traversal cannot authorize this route
                                 * step.  The exact current ticket above is
                                 * deliberately retained until centre-exit;
                                 * otherwise the per-tick request refresh
                                 * makes a train lose its own authority while
                                 * it is physically inside the turnout.
                                 */
                                runtime->conflict_zone_hard_step = -1;
                                runtime->conflict_zone_hard_ticket = 0;
                        }
                }
        }

        int soft_step = hard_step + 1;
        if (soft_step < runtime->conflict_zone_plan.step_count) {
                const tc2_conflict_zone_route_step *soft =
                        &runtime->conflict_zone_plan.steps[soft_step];
                fill_conflict_zone_request(slot, soft_step, &request);
                if (Tc2ConflictZoneRequestSoft(
                            &dispatch_conflict_zones, soft->zone,
                            &request, 0) < 0) {
                        return -1;
                }
                runtime->conflict_zone_soft_step = soft_step;
        } else {
                (void)Tc2ConflictZoneCancelSoft(
                        &dispatch_conflict_zones, job->train);
                runtime->conflict_zone_soft_step = -1;
        }
        return 0;
}

static void reset_conflict_zone_action_cursor(
        tc2_dispatch_runtime *runtime) {
        if (!runtime) return;
        runtime->conflict_zone_action_step = -1;
        runtime->conflict_zone_action_index = 0;
        runtime->conflict_zone_action_ticket = 0;
}

static void reset_soft_conflict_zone_cursor(
        tc2_dispatch_runtime *runtime, int clear_confirmation) {
        if (!runtime) return;
        runtime->conflict_zone_soft_batch_token = 0;
        runtime->conflict_zone_soft_batch_step = -1;
        runtime->conflict_zone_soft_action_index = 0;
        runtime->conflict_zone_soft_settle_at_tick = -1;
        runtime->conflict_zone_soft_prefetched_step = -1;
        if (clear_confirmation) {
                runtime->conflict_zone_soft_confirmed_step = -1;
        }
}

/*
 * A soft request is not movement authority.  It only permits the unique best
 * next-next waiter to prepare an otherwise FREE physical zone.  Compound
 * zones still command their real motors one at a time in route order; the
 * shared zone is never treated as one synthetic turnout.
 */
static int cancel_soft_conflict_zone_prefetch(int slot,
                                               int clear_confirmation) {
        if (slot < 0 || slot >= TC2_DISPATCH_MAX_JOBS) return -1;
        tc2_dispatch_runtime *runtime = &dispatch_runtime[slot];
        if (runtime->conflict_zone_soft_batch_token != 0) {
                can_batch_status_t status;
                unsigned int token =
                        runtime->conflict_zone_soft_batch_token;
                if (CanGetBatchStatus(dispatch_can_tid, token, &status) < 0 ||
                    status.token != token) {
                        return -1;
                }
                if (status.state == CAN_BATCH_PENDING &&
                    CanCancelBatch(dispatch_can_tid, token) < 0) {
                        return -1;
                }
        }
        reset_soft_conflict_zone_cursor(runtime, clear_confirmation);
        return 0;
}

static int soft_conflict_zone_still_eligible(int slot, int step_index) {
        if (slot < 0 || slot >= TC2_DISPATCH_MAX_JOBS) return 0;
        tc2_dispatch_runtime *runtime = &dispatch_runtime[slot];
        if (!runtime->conflict_zone_plan_valid || step_index < 0 ||
            step_index >= runtime->conflict_zone_plan.step_count ||
            step_index != runtime->conflict_zone_soft_step) {
                return 0;
        }
        return Tc2ConflictZoneCanPrefetch(
                       &dispatch_conflict_zones,
                       runtime->conflict_zone_plan.steps[step_index].zone,
                       dispatch_jobs[slot].train) == 1;
}

static int validate_soft_conflict_zone_prefetch(int slot) {
        tc2_dispatch_runtime *runtime = &dispatch_runtime[slot];
        int active_step = runtime->conflict_zone_soft_batch_step;
        if ((runtime->conflict_zone_soft_batch_token != 0 ||
            runtime->conflict_zone_soft_settle_at_tick >= 0) &&
            !soft_conflict_zone_still_eligible(slot, active_step)) {
                return cancel_soft_conflict_zone_prefetch(slot, 1);
        }
        int confirmed = runtime->conflict_zone_soft_confirmed_step;
        if (confirmed < 0) return 0;
        if (confirmed == runtime->conflict_zone_soft_step) {
                if (!soft_conflict_zone_still_eligible(slot, confirmed)) {
                        runtime->conflict_zone_soft_confirmed_step = -1;
                }
                return 0;
        }
        /*
         * A just-promoted confirmation may survive until global hard
         * arbitration.  No other cursor value is meaningful.
         */
        if (confirmed != runtime->conflict_zone_next_step ||
            runtime->conflict_zone_hard_step != confirmed ||
            runtime->conflict_zone_hard_ticket == 0) {
                runtime->conflict_zone_soft_confirmed_step = -1;
        }
        return 0;
}

static int queue_soft_conflict_zone_prefetch(int slot, int now) {
        tc2_dispatch_runtime *runtime = &dispatch_runtime[slot];
        int step_index = runtime->conflict_zone_soft_step;
        if (!soft_conflict_zone_still_eligible(slot, step_index) ||
            runtime->conflict_zone_batch_token != 0 ||
            runtime->conflict_zone_settle_at_tick >= 0 ||
            !conflict_zone_step_ready(
                    slot, runtime->conflict_zone_next_step)) {
                return 0;
        }
        const tc2_conflict_zone_route_step *step =
                &runtime->conflict_zone_plan.steps[step_index];
        if (runtime->conflict_zone_soft_confirmed_step == step_index) {
                return 0;
        }
        if (runtime->conflict_zone_soft_batch_step != step_index) {
                if (runtime->conflict_zone_soft_batch_token != 0 ||
                    runtime->conflict_zone_soft_settle_at_tick >= 0) {
                        return -1;
                }
                runtime->conflict_zone_soft_batch_step = step_index;
                runtime->conflict_zone_soft_action_index = 0;
        }
        if (runtime->conflict_zone_soft_batch_token != 0 ||
            runtime->conflict_zone_soft_settle_at_tick >= 0) {
                return 0;
        }
        int action = runtime->conflict_zone_soft_action_index;
        if (action < 0 || action >= step->setting.action_count) {
                return -1;
        }
        int switch_number =
                step->setting.actions[action].switch_number;
        char direction = Tc2TrackPhysicalTurnoutDirection(
                step->setting.actions[action].direction ==
                        TC2_CONFLICT_TURNOUT_CURVED ?
                        DIR_CURVED : DIR_STRAIGHT);
        if (!direction) return -1;
        int token = CanSwitchBatch(
                dispatch_can_tid, &switch_number, &direction, 1);
        if (token == CAN_SEND_BUSY) return 0;
        if (token < 0) return -1;
        runtime->conflict_zone_soft_batch_token =
                token > 0 ? (unsigned int)token : 0;
        runtime->conflict_zone_soft_settle_at_tick =
                token == 0 ?
                tick_after(now, TC2_TURNOUT_SETTLE_TICKS) : -1;
        return 0;
}

static int poll_soft_conflict_zone_prefetch(int slot, int now) {
        tc2_dispatch_runtime *runtime = &dispatch_runtime[slot];
        int step_index = runtime->conflict_zone_soft_batch_step;
        if (step_index < 0) return 0;
        if (!soft_conflict_zone_still_eligible(slot, step_index)) {
                return cancel_soft_conflict_zone_prefetch(slot, 1);
        }
        if (runtime->conflict_zone_soft_batch_token != 0) {
                can_batch_status_t status;
                unsigned int token =
                        runtime->conflict_zone_soft_batch_token;
                if (CanGetBatchStatus(
                            dispatch_can_tid, token, &status) < 0 ||
                    status.token != token) {
                        (void)cancel_soft_conflict_zone_prefetch(slot, 1);
                        return -1;
                }
                if (status.state == CAN_BATCH_PENDING) return 0;
                if (status.state != CAN_BATCH_COMPLETE ||
                    status.confirmed != status.total) {
                        reset_soft_conflict_zone_cursor(runtime, 1);
                        return 0;
                }
                runtime->conflict_zone_soft_batch_token = 0;
                runtime->conflict_zone_soft_settle_at_tick =
                        tick_after(now, TC2_TURNOUT_SETTLE_TICKS);
                return 0;
        }
        if (runtime->conflict_zone_soft_settle_at_tick < 0 ||
            !tick_reached(now,
                    runtime->conflict_zone_soft_settle_at_tick)) {
                return 0;
        }
        ++runtime->conflict_zone_soft_action_index;
        runtime->conflict_zone_soft_settle_at_tick = -1;
        const tc2_conflict_zone_route_step *step =
                &runtime->conflict_zone_plan.steps[step_index];
        if (runtime->conflict_zone_soft_action_index <
                step->setting.action_count) {
                return queue_soft_conflict_zone_prefetch(slot, now);
        }
        runtime->conflict_zone_soft_prefetched_step = step_index;
        runtime->conflict_zone_soft_confirmed_step = step_index;
        runtime->conflict_zone_soft_batch_step = -1;
        runtime->conflict_zone_soft_action_index = 0;
        return 0;
}

static int queue_owned_conflict_zone_setting(int slot) {
        tc2_dispatch_runtime *runtime = &dispatch_runtime[slot];
        tc2_dispatch_job_snapshot *job = &dispatch_jobs[slot];
        int step_index = runtime->conflict_zone_next_step;
        if (!runtime->conflict_zone_plan_valid || step_index < 0 ||
            step_index >= runtime->conflict_zone_plan.step_count) {
                return 0;
        }
        const tc2_conflict_zone_route_step *step =
                &runtime->conflict_zone_plan.steps[step_index];
        if (runtime->conflict_zone_origin_egress_active &&
            runtime->conflict_zone_origin_egress_step == step_index) {
                return origin_zone_egress_ready(slot, step_index) ? 0 : -1;
        }
        tc2_conflict_zone_grant grant;
        if (Tc2ConflictZoneQueryGrant(
                    &dispatch_conflict_zones, step->zone,
                    &grant) <= 0 ||
            grant.owner_train != job->train ||
            runtime->conflict_zone_hard_ticket == 0 ||
            grant.owner_ticket != runtime->conflict_zone_hard_ticket) {
                /*
                 * Another hard contender won after this train had safely
                 * prepared the formerly-free zone.  Its setting can now be
                 * changed by that owner, so the old advisory proof must not
                 * survive until a later grant.
                 */
                if (runtime->conflict_zone_soft_confirmed_step ==
                    step_index) {
                        runtime->conflict_zone_soft_confirmed_step = -1;
                        runtime->conflict_zone_soft_prefetched_step = -1;
                }
                return 0;
        }
        if (grant.state == TC2_CONFLICT_ZONE_SETTLED ||
            grant.state == TC2_CONFLICT_ZONE_OCCUPIED ||
            grant.state == TC2_CONFLICT_ZONE_RELEASE_DELAY) {
                return 0;
        }
        if (grant.state != TC2_CONFLICT_ZONE_GRANTED &&
            grant.state != TC2_CONFLICT_ZONE_SETTING) {
                return 0;
        }
        if (grant.state == TC2_CONFLICT_ZONE_GRANTED &&
            runtime->conflict_zone_soft_confirmed_step == step_index &&
            conflict_zone_settings_equal(
                    &grant.setting, &step->setting)) {
                /*
                 * This exact vector was confirmed while the zone was FREE,
                 * remained the unique eligible soft request, and has now
                 * won the real FIFO hard ticket.  Promote the proof under
                 * that ticket without sending the motors a second time.
                 */
                if (Tc2ConflictZoneMarkSetting(
                            &dispatch_conflict_zones, step->zone,
                            job->train) < 0 ||
                    Tc2ConflictZoneMarkSettled(
                            &dispatch_conflict_zones, step->zone,
                            job->train) < 0) {
                        return -1;
                }
                runtime->conflict_zone_soft_confirmed_step = -1;
                runtime->conflict_zone_soft_prefetched_step = -1;
                reset_conflict_zone_action_cursor(runtime);
                return 0;
        }
        if (grant.setting.action_count < 1 ||
            grant.setting.action_count >
                    TC2_CONFLICT_ZONE_MAX_ACTIONS) {
                return -1;
        }
        if (runtime->conflict_zone_action_step != step_index ||
            runtime->conflict_zone_action_ticket !=
                    runtime->conflict_zone_hard_ticket) {
                if (runtime->conflict_zone_batch_token != 0 ||
                    runtime->conflict_zone_settle_at_tick >= 0) {
                        return -1;
                }
                runtime->conflict_zone_action_step = step_index;
                runtime->conflict_zone_action_index = 0;
                runtime->conflict_zone_action_ticket =
                        runtime->conflict_zone_hard_ticket;
        }
        int action = runtime->conflict_zone_action_index;
        if (action < 0 || action >= grant.setting.action_count) {
                return action == grant.setting.action_count ? 0 : -1;
        }
        int switch_numbers[1];
        char directions[1];
        switch_numbers[0] =
                grant.setting.actions[action].switch_number;
        directions[0] =
                Tc2TrackPhysicalTurnoutDirection(
                    grant.setting.actions[action].direction ==
                            TC2_CONFLICT_TURNOUT_CURVED ?
                            DIR_CURVED : DIR_STRAIGHT);
        if (!directions[0]) return -1;
        int token = CanSwitchBatch(
                dispatch_can_tid, switch_numbers, directions,
                1);
        if (token == CAN_SEND_BUSY) {
                runtime->conflict_zone_queue_pending = 1;
                return 0;
        }
        if (token < 0) {
                return -1;
        }
        if (grant.state == TC2_CONFLICT_ZONE_GRANTED &&
            Tc2ConflictZoneMarkSetting(
                    &dispatch_conflict_zones, step->zone,
                    job->train) < 0) {
                if (token > 0) {
                        (void)CanCancelBatch(
                                dispatch_can_tid,
                                (unsigned int)token);
                }
                return -1;
        }
        runtime->conflict_zone_queue_pending = 0;
        runtime->conflict_zone_batch_token =
                token > 0 ? (unsigned int)token : 0;
        runtime->conflict_zone_settle_at_tick =
                token == 0 ?
                tick_after(Time(), TC2_TURNOUT_SETTLE_TICKS) : -1;
        return 0;
}

/*
 * Move the immutable look-ahead cursor only.  The next global arbitration
 * pass will grant and then physically set the new exact hard turnout.
 */
static int promote_next_conflict_zone_after_entry(int slot) {
        if (slot < 0 || slot >= TC2_DISPATCH_MAX_JOBS) return -1;
        tc2_dispatch_runtime *runtime = &dispatch_runtime[slot];
        tc2_dispatch_job_snapshot *job = &dispatch_jobs[slot];
        int step_index = runtime->conflict_zone_next_step;

        if (!runtime->conflict_zone_plan_valid ||
            step_index < 0 ||
            step_index >= runtime->conflict_zone_plan.step_count) {
                (void)Tc2ConflictZoneCancelSoft(
                        &dispatch_conflict_zones, job->train);
                runtime->conflict_zone_soft_step = -1;
                runtime->conflict_zone_soft_batch_step = -1;
                runtime->conflict_zone_soft_action_index = 0;
                runtime->conflict_zone_soft_settle_at_tick = -1;
                runtime->conflict_zone_soft_prefetched_step = -1;
                runtime->conflict_zone_soft_confirmed_step = -1;
                return 0;
        }

        const tc2_conflict_zone_route_step *step =
                &runtime->conflict_zone_plan.steps[step_index];
        int promoted_confirmation =
                runtime->conflict_zone_soft_confirmed_step == step_index &&
                soft_conflict_zone_still_eligible(slot, step_index);
        if (runtime->conflict_zone_soft_batch_token != 0) {
                unsigned int token =
                        runtime->conflict_zone_soft_batch_token;
                can_batch_status_t status;
                if (CanGetBatchStatus(
                            dispatch_can_tid, token, &status) < 0 ||
                    status.token != token) {
                        return -1;
                }
                /*
                 * Promotion can occur in the same scheduler pass that
                 * queued the advisory batch.  A fast CAN service may have
                 * completed it before the hard cursor advances, in which
                 * case cancellation is neither necessary nor legal.  Only
                 * a still-pending advisory must be synchronously removed;
                 * completed/failed advisory state is discarded and the
                 * exact hard setting is re-sent below in every case.
                 */
                if (status.state == CAN_BATCH_PENDING &&
                    CanCancelBatch(dispatch_can_tid, token) < 0) {
                        return -1;
                }
                reset_soft_conflict_zone_cursor(runtime, 1);
        }
        (void)Tc2ConflictZoneCancelSoft(
                &dispatch_conflict_zones, job->train);
        runtime->conflict_zone_soft_step = -1;
        runtime->conflict_zone_soft_batch_step = -1;
        runtime->conflict_zone_soft_action_index = 0;
        runtime->conflict_zone_soft_settle_at_tick = -1;
        runtime->conflict_zone_soft_prefetched_step = -1;
        runtime->conflict_zone_soft_confirmed_step =
                promoted_confirmation ? step_index : -1;

        /*
         * Promotion happens during phase three, after this scheduler pass's
         * global arbitration.  Queue the new hard request now, but never
         * grant it from the current slot: another train can promote to the
         * same physical zone later in this same pass.  The next pass first
         * refreshes every contender and only then performs the single global
         * grant phase, so slot order cannot choose the turnout owner.
         */
        if (refresh_conflict_zone_requests(slot) < 0) {
                return -1;
        }

        tc2_conflict_zone_grant grant;
        int owns_exact_ticket =
                Tc2ConflictZoneQueryGrant(
                        &dispatch_conflict_zones, step->zone,
                        &grant) > 0 &&
                grant.owner_train == job->train &&
                runtime->conflict_zone_hard_step == step_index &&
                runtime->conflict_zone_hard_ticket != 0 &&
                grant.owner_ticket == runtime->conflict_zone_hard_ticket &&
                conflict_zone_settings_equal(
                        &grant.setting, &step->setting);
        if (!owns_exact_ticket) return 0;

        /* A grant can only exist from a prior global pass; never fabricate it. */
        if (queue_owned_conflict_zone_setting(slot) < 0) {
                return -1;
        }
        return 0;
}

static int conflict_zone_job_can_request(
        const tc2_dispatch_job_snapshot *job) {
        return job && job->state != TC2_JOB_EMPTY &&
               job->state != TC2_JOB_FAILED &&
               job->state != TC2_JOB_CANCEL_BRAKING &&
               job->state != TC2_JOB_STOPPED &&
               job->state != TC2_JOB_STOP_UNCONFIRMED &&
               job->state != TC2_JOB_ARRIVED;
}

enum {
        TC2_CONFLICT_CARRY_UNKNOWN = 0,
        TC2_CONFLICT_CARRY_UNENTERED = 1,
        TC2_CONFLICT_CARRY_OCCUPIED = 2,
        TC2_CONFLICT_CARRY_TAIL_CLEAR = 3
};

static void remove_conflict_zone_carry(
        tc2_dispatch_runtime *runtime, int index) {
        if (!runtime || index < 0 ||
            index >= runtime->conflict_zone_carry_count) {
                return;
        }
        if (runtime->conflict_zone_origin_egress_active &&
            runtime->conflict_zone_carry_zone[index] ==
                    runtime->conflict_zone_origin_egress_zone &&
            runtime->conflict_zone_carry_ticket[index] ==
                    runtime->conflict_zone_origin_egress_ticket) {
                clear_origin_zone_egress(runtime);
        }
        int last = --runtime->conflict_zone_carry_count;
        if (index != last) {
                runtime->conflict_zone_carry_zone[index] =
                        runtime->conflict_zone_carry_zone[last];
                runtime->conflict_zone_carry_entry_node[index] =
                        runtime->conflict_zone_carry_entry_node[last];
                runtime->conflict_zone_carry_exit_node[index] =
                        runtime->conflict_zone_carry_exit_node[last];
                runtime->conflict_zone_carry_exit_distance_mm[index] =
                        runtime->conflict_zone_carry_exit_distance_mm[last];
                runtime->conflict_zone_carry_scalar_proof_valid[index] =
                        runtime->conflict_zone_carry_scalar_proof_valid[last];
                runtime->conflict_zone_carry_ticket[index] =
                        runtime->conflict_zone_carry_ticket[last];
                runtime->conflict_zone_carry_mode[index] =
                        runtime->conflict_zone_carry_mode[last];
        }
        runtime->conflict_zone_carry_zone[last] =
                TC2_CONFLICT_ZONE_NONE;
        runtime->conflict_zone_carry_entry_node[last] = -1;
        runtime->conflict_zone_carry_exit_node[last] = -1;
        runtime->conflict_zone_carry_exit_distance_mm[last] = -1;
        runtime->conflict_zone_carry_scalar_proof_valid[last] = 0;
        runtime->conflict_zone_carry_ticket[last] = 0;
        runtime->conflict_zone_carry_mode[last] =
                TC2_CONFLICT_CARRY_UNKNOWN;
}

static int remember_conflict_zone_carry(
        tc2_dispatch_runtime *runtime, int zone,
        int entry_node, int exit_node, int exit_distance_mm,
        int scalar_proof_valid, unsigned int ticket,
        int mode) {
        if (!runtime || zone < 0 ||
            zone >= TC2_CONFLICT_ZONE_COUNT || ticket == 0) {
                return -1;
        }
        for (int index = 0;
             index < runtime->conflict_zone_carry_count; ++index) {
                if (runtime->conflict_zone_carry_zone[index] != zone) {
                        continue;
                }
                /* Preserve older geometry while the same traversal lives. */
                if (runtime->conflict_zone_carry_ticket[index] == ticket) {
                        return 0;
                }
                /*
                 * One physical zone cannot have two live owner tickets.
                 * Replacing this entry would lose the exact geometry needed
                 * to release the older traversal, so fail closed instead.
                 */
                return -1;
        }
        if (runtime->conflict_zone_carry_count >=
            TC2_CONFLICT_ZONE_COUNT) {
                return -1;
        }
        int index = runtime->conflict_zone_carry_count++;
        runtime->conflict_zone_carry_zone[index] = zone;
        runtime->conflict_zone_carry_entry_node[index] = entry_node;
        runtime->conflict_zone_carry_exit_node[index] = exit_node;
        runtime->conflict_zone_carry_exit_distance_mm[index] =
                exit_distance_mm;
        runtime->conflict_zone_carry_scalar_proof_valid[index] =
                scalar_proof_valid ? 1 : 0;
        runtime->conflict_zone_carry_ticket[index] = ticket;
        runtime->conflict_zone_carry_mode[index] = (unsigned char)mode;
        return 0;
}

/*
 * Snapshot, but do not release, every physical owner before a CURRENT route
 * replaces its route storage.  This pure bookkeeping step is safe to copy
 * across stage_job() and safe to roll back if the reservation CAS fails.
 */
static int snapshot_owned_conflict_zone_carries(int slot) {
        if (slot < 0 || slot >= TC2_DISPATCH_MAX_JOBS) return -1;
        tc2_dispatch_job_snapshot *job = &dispatch_jobs[slot];
        tc2_dispatch_runtime *runtime = &dispatch_runtime[slot];
        for (int zone = 0; zone < TC2_CONFLICT_ZONE_COUNT; ++zone) {
                tc2_conflict_zone_grant grant;
                if (Tc2ConflictZoneQueryGrant(
                            &dispatch_conflict_zones, zone, &grant) <= 0 ||
                    grant.owner_train != job->train ||
                    grant.owner_ticket == 0 ||
                    grant.state == TC2_CONFLICT_ZONE_RELEASE_DELAY) {
                        continue;
                }

                int already_recorded = 0;
                for (int carry = 0;
                     carry < runtime->conflict_zone_carry_count; ++carry) {
                        if (runtime->conflict_zone_carry_zone[carry] != zone) {
                                continue;
                        }
                        if (runtime->conflict_zone_carry_ticket[carry] !=
                            grant.owner_ticket) {
                                return -1;
                        }
                        already_recorded = 1;
                        break;
                }
                if (already_recorded) continue;

                /*
                 * The hard grant records its exact immutable route step at
                 * ticket creation.  Never recover it by nearest-distance or
                 * zone/setting matching: a route may visit the same turnout
                 * more than once with the same S/C setting.
                 */
                int old_step = runtime->conflict_zone_hard_step;
                if (runtime->conflict_zone_hard_ticket !=
                            grant.owner_ticket ||
                    old_step < 0 ||
                    !runtime->conflict_zone_plan_valid ||
                    !runtime->route_valid ||
                    old_step >= runtime->conflict_zone_plan.step_count ||
                    runtime->conflict_zone_plan.steps[old_step].zone != zone) {
                        return -1;
                }
                int entry_node = -1;
                int exit_node = -1;
                int mode = TC2_CONFLICT_CARRY_UNKNOWN;
                const tc2_conflict_zone_route_step *step =
                        &runtime->conflict_zone_plan.steps[old_step];
                if (step->first_route_offset < 0 ||
                    step->first_route_offset >= runtime->route.node_count ||
                    step->last_route_offset < 0 ||
                    step->last_route_offset >= runtime->route.node_count) {
                        return -1;
                }
                entry_node = runtime->route.nodes[
                        step->first_route_offset];
                exit_node = runtime->route.nodes[
                        step->last_route_offset];
                /*
                 * monitor.confirmed_distance_mm is advanced only by an
                 * accepted physical sensor.  job->position_estimated merely
                 * describes the continuously rendered centre position and
                 * must not suppress this hard route-scalar proof.
                 */
                int confirmed = step &&
                        runtime->monitor.confirmed_distance_mm >= 0;
                int before_entry = confirmed &&
                        runtime->monitor.confirmed_distance_mm +
                                        TC2_TRAIN_HALF_LENGTH_MM <
                                step->entry_distance_mm - 20;
                int after_exit = confirmed &&
                        runtime->monitor.confirmed_distance_mm -
                                        TC2_TRAIN_HALF_LENGTH_MM >
                                step->exit_distance_mm + 20;
                if (grant.state == TC2_CONFLICT_ZONE_GRANTED) {
                        mode = TC2_CONFLICT_CARRY_UNENTERED;
                } else if (grant.state == TC2_CONFLICT_ZONE_OCCUPIED) {
                        mode = after_exit ?
                                TC2_CONFLICT_CARRY_TAIL_CLEAR :
                                TC2_CONFLICT_CARRY_OCCUPIED;
                } else if ((grant.state ==
                                    TC2_CONFLICT_ZONE_SETTING ||
                            grant.state ==
                                    TC2_CONFLICT_ZONE_SETTLED) &&
                           before_entry) {
                        mode = TC2_CONFLICT_CARRY_UNENTERED;
                }
                if (remember_conflict_zone_carry(
                        runtime, zone, entry_node, exit_node,
                        step->exit_distance_mm, 1,
                        grant.owner_ticket, mode) < 0) {
                        return -1;
                }
        }
        return 0;
}

static int cancel_old_conflict_zone_batch(int slot) {
        tc2_dispatch_runtime *runtime = &dispatch_runtime[slot];
        unsigned int tokens[2] = {
                runtime->conflict_zone_batch_token,
                runtime->conflict_zone_soft_batch_token
        };
        for (int index = 0; index < 2; ++index) {
                unsigned int token = tokens[index];
                if (token == 0) continue;
                can_batch_status_t status;
                if (CanGetBatchStatus(
                            dispatch_can_tid, token, &status) < 0 ||
                    status.token != token) {
                        return -1;
                }
                if (status.state == CAN_BATCH_PENDING &&
                    CanCancelBatch(dispatch_can_tid, token) < 0) {
                        return -1;
                }
        }
        runtime->conflict_zone_batch_token = 0;
        reset_soft_conflict_zone_cursor(runtime, 1);
        runtime->conflict_zone_queue_pending = 0;
        runtime->conflict_zone_settle_at_tick = -1;
        reset_conflict_zone_action_cursor(runtime);
        return 0;
}

/*
 * Delete the train's immutable future turnout program without pretending
 * that its physical body disappeared.  Every already-owned turnout is first
 * copied into the carry ledger with its exact ticket and route geometry.
 * Queued hard/soft work and in-flight switch commands are then canceled;
 * OCCUPIED or otherwise unproved owners remain fail-closed until a later
 * physical sensor proves clearance, or remove performs an explicit physical
 * release.
 */
static int discard_future_conflict_zone_work(int slot) {
        if (slot < 0 || slot >= TC2_DISPATCH_MAX_JOBS) return -1;
        tc2_dispatch_job_snapshot *job = &dispatch_jobs[slot];
        tc2_dispatch_runtime *runtime = &dispatch_runtime[slot];
        if (job->state == TC2_JOB_EMPTY) return 0;

        if (snapshot_owned_conflict_zone_carries(slot) < 0 ||
            cancel_old_conflict_zone_batch(slot) < 0) {
                return -1;
        }

        /* A canceled CAN batch cannot leave the software zone in SETTING. */
        for (int zone = 0; zone < TC2_CONFLICT_ZONE_COUNT; ++zone) {
                tc2_conflict_zone_grant grant;
                if (Tc2ConflictZoneQueryGrant(
                            &dispatch_conflict_zones, zone,
                            &grant) > 0 &&
                    grant.owner_train == job->train &&
                    grant.state == TC2_CONFLICT_ZONE_SETTING) {
                        if (Tc2ConflictZoneRetrySetting(
                                    &dispatch_conflict_zones,
                                    zone, job->train) < 0) {
                                return -1;
                        }
                }
        }
        if (Tc2ConflictZoneCancelTrain(
                    &dispatch_conflict_zones, job->train) < 0) {
                return -1;
        }

        runtime->conflict_zone_plan.step_count = 0;
        runtime->conflict_zone_plan_valid = 0;
        runtime->conflict_zone_next_step = 0;
        runtime->conflict_zone_hard_step = -1;
        runtime->conflict_zone_soft_step = -1;
        runtime->conflict_zone_hard_ticket = 0;
        runtime->conflict_zone_hold_active = 0;
        clear_origin_zone_egress(runtime);
        return 0;
}

/* Apply only lifecycle transitions proved by the pre-replacement snapshot. */
static void finalize_old_conflict_zone_carries(int slot, int now) {
        tc2_dispatch_job_snapshot *job = &dispatch_jobs[slot];
        tc2_dispatch_runtime *runtime = &dispatch_runtime[slot];
        int index = 0;
        while (index < runtime->conflict_zone_carry_count) {
                int zone = runtime->conflict_zone_carry_zone[index];
                tc2_conflict_zone_grant grant;
                if (Tc2ConflictZoneQueryGrant(
                            &dispatch_conflict_zones, zone, &grant) <= 0 ||
                    grant.owner_train != job->train ||
                    grant.owner_ticket !=
                            runtime->conflict_zone_carry_ticket[index]) {
                        remove_conflict_zone_carry(runtime, index);
                        continue;
                }
                if (grant.state == TC2_CONFLICT_ZONE_SETTING) {
                        (void)Tc2ConflictZoneRetrySetting(
                                &dispatch_conflict_zones,
                                zone, job->train);
                        grant.state = TC2_CONFLICT_ZONE_GRANTED;
                }
                int completed = 0;
                if (runtime->conflict_zone_carry_mode[index] ==
                            TC2_CONFLICT_CARRY_UNENTERED &&
                    (grant.state == TC2_CONFLICT_ZONE_GRANTED ||
                     grant.state == TC2_CONFLICT_ZONE_SETTLED)) {
                        completed = Tc2ConflictZoneYieldUnentered(
                                &dispatch_conflict_zones,
                                zone, job->train) == 0;
                } else if (runtime->conflict_zone_carry_mode[index] ==
                                   TC2_CONFLICT_CARRY_TAIL_CLEAR &&
                           grant.state ==
                                   TC2_CONFLICT_ZONE_OCCUPIED) {
                        completed = Tc2ConflictZoneMarkTailClear(
                                &dispatch_conflict_zones, zone,
                                job->train, (uint32_t)now) == 0;
                }
                if (completed ||
                    grant.state == TC2_CONFLICT_ZONE_RELEASE_DELAY) {
                        remove_conflict_zone_carry(runtime, index);
                        continue;
                }
                ++index;
        }
}

static int conflict_zone_physical_distances_from(
        int start_node, int distance[TRACK_MAX]) {
        unsigned char visited[TRACK_MAX];
        if (!distance || start_node < 0 || start_node >= TRACK_MAX ||
            dispatch_track[start_node].type == NODE_NONE) {
                return -1;
        }
        for (int node = 0; node < TRACK_MAX; ++node) {
                distance[node] = 0x3fffffff;
                visited[node] = 0;
        }
        distance[start_node] = 0;
        for (int pass = 0; pass < TRACK_MAX; ++pass) {
                int current = -1;
                for (int node = 0; node < TRACK_MAX; ++node) {
                        if (!visited[node] &&
                            distance[node] != 0x3fffffff &&
                            (current < 0 ||
                             distance[node] < distance[current])) {
                                current = node;
                        }
                }
                if (current < 0) break;
                visited[current] = 1;
                if (dispatch_track[current].reverse) {
                        int reverse = (int)(dispatch_track[current].reverse -
                                            dispatch_track);
                        if (reverse < 0 || reverse >= TRACK_MAX) return -1;
                        if (distance[current] < distance[reverse]) {
                                distance[reverse] = distance[current];
                        }
                }
                int edges = dispatch_track[current].type == NODE_BRANCH ? 2 :
                        (dispatch_track[current].type == NODE_EXIT ||
                         dispatch_track[current].type == NODE_NONE ? 0 : 1);
                for (int direction = 0; direction < edges; ++direction) {
                        track_edge *edge =
                                &dispatch_track[current].edge[direction];
                        if (!edge->dest || edge->dist < 0) continue;
                        int next = (int)(edge->dest - dispatch_track);
                        if (next < 0 || next >= TRACK_MAX ||
                            distance[current] >
                                    0x3fffffff - edge->dist) {
                                return -1;
                        }
                        int candidate = distance[current] + edge->dist;
                        if (candidate < distance[next]) {
                                distance[next] = candidate;
                        }
                }
        }
        return 0;
}

static void process_conflict_zone_carries(int slot, int now) {
        tc2_dispatch_job_snapshot *job = &dispatch_jobs[slot];
        tc2_dispatch_runtime *runtime = &dispatch_runtime[slot];
        int distance[TRACK_MAX];
        int have_physical_proof =
                !job->position_estimated &&
                job->current_node >= 0 &&
                job->current_node < TRACK_MAX &&
                conflict_zone_physical_distances_from(
                        job->current_node, distance) == 0;
        int index = 0;
        while (index < runtime->conflict_zone_carry_count) {
                int zone = runtime->conflict_zone_carry_zone[index];
                tc2_conflict_zone_grant grant;
                if (Tc2ConflictZoneQueryGrant(
                            &dispatch_conflict_zones, zone, &grant) <= 0 ||
                    grant.owner_train != job->train ||
                    grant.owner_ticket !=
                            runtime->conflict_zone_carry_ticket[index]) {
                        remove_conflict_zone_carry(runtime, index);
                        continue;
                }
                if (grant.state == TC2_CONFLICT_ZONE_RELEASE_DELAY) {
                        remove_conflict_zone_carry(runtime, index);
                        continue;
                }
                int entry = runtime->conflict_zone_carry_entry_node[index];
                int exit = runtime->conflict_zone_carry_exit_node[index];
                int exit_distance_mm =
                        runtime->conflict_zone_carry_exit_distance_mm[index];
                int scalar_clear = exit_distance_mm >= 0 &&
                        runtime->conflict_zone_carry_scalar_proof_valid[
                                index] &&
                        runtime->monitor.confirmed_distance_mm >= 0 &&
                        runtime->monitor.confirmed_distance_mm -
                                        TC2_TRAIN_HALF_LENGTH_MM >
                                exit_distance_mm + 20;
                int graph_clear = have_physical_proof &&
                        entry >= 0 && entry < TRACK_MAX &&
                        exit >= 0 && exit < TRACK_MAX &&
                        distance[entry] != 0x3fffffff &&
                        distance[exit] != 0x3fffffff &&
                        distance[entry] > TC2_TRAIN_HALF_LENGTH_MM + 20 &&
                        distance[exit] > TC2_TRAIN_HALF_LENGTH_MM + 20;
                int clear = scalar_clear || graph_clear;
                int completed = 0;
                int carry_mode =
                        runtime->conflict_zone_carry_mode[index];

                /*
                 * UNKNOWN is intentionally fail-closed at replacement time:
                 * no estimated position may release an old physical owner.
                 * It must not, however, become an irrevocable deadlock.  A
                 * later sensor-confirmed position farther than the complete
                 * half-body plus the 20 mm margin from both old boundaries
                 * proves the train is outside the zone.  At that point the
                 * old traversal can be retired according to its real state.
                 */
                if (clear && carry_mode == TC2_CONFLICT_CARRY_UNKNOWN &&
                    grant.state == TC2_CONFLICT_ZONE_SETTING &&
                    Tc2ConflictZoneRetrySetting(
                            &dispatch_conflict_zones,
                            zone, job->train) == 0) {
                        grant.state = TC2_CONFLICT_ZONE_GRANTED;
                }
                if (clear &&
                    (carry_mode == TC2_CONFLICT_CARRY_UNENTERED ||
                     carry_mode == TC2_CONFLICT_CARRY_UNKNOWN) &&
                    (grant.state == TC2_CONFLICT_ZONE_GRANTED ||
                     grant.state == TC2_CONFLICT_ZONE_SETTLED)) {
                        completed = Tc2ConflictZoneYieldUnentered(
                                &dispatch_conflict_zones,
                                zone, job->train) == 0;
                } else if (clear &&
                           (carry_mode ==
                                    TC2_CONFLICT_CARRY_OCCUPIED ||
                            carry_mode ==
                                    TC2_CONFLICT_CARRY_UNKNOWN) &&
                           grant.state ==
                                   TC2_CONFLICT_ZONE_OCCUPIED) {
                        completed = Tc2ConflictZoneMarkTailClear(
                                &dispatch_conflict_zones, zone,
                                job->train, (uint32_t)now) == 0;
                }
                if (completed) {
                        remove_conflict_zone_carry(runtime, index);
                        continue;
                }
                ++index;
        }
}

static void process_conflict_zones(int now) {
        Tc2ConflictZoneTick(&dispatch_conflict_zones, (uint32_t)now);

        /* Phase 1: take an immutable request snapshot from every train. */
        for (int slot = 0; slot < TC2_DISPATCH_MAX_JOBS; ++slot) {
                if (!dispatch_runtime[slot].conflict_zone_plan_valid ||
                    !conflict_zone_job_can_request(&dispatch_jobs[slot])) {
                        continue;
                }
                if (refresh_conflict_zone_requests(slot) < 0) {
                        fail_job(slot, TC2_FAILURE_RESERVATION);
                }
        }

        /*
         * A newly queued hard contender immediately revokes any advisory
         * command for that physical zone.  Completed confirmations survive
         * only when they are the exact step being promoted to a hard ticket.
         */
        for (int slot = 0; slot < TC2_DISPATCH_MAX_JOBS; ++slot) {
                if (dispatch_jobs[slot].state == TC2_JOB_EMPTY) continue;
                if (validate_soft_conflict_zone_prefetch(slot) < 0) {
                        fail_job(slot, TC2_FAILURE_TURNOUT);
                        continue;
                }
                if (poll_soft_conflict_zone_prefetch(slot, now) < 0) {
                        fail_job(slot, TC2_FAILURE_TURNOUT);
                }
        }

        /* Phase 2: arbitrate only after all eligible contenders are queued. */
        for (int zone = 0; zone < TC2_CONFLICT_ZONE_COUNT; ++zone) {
                if (Tc2ConflictZoneTryGrant(
                            &dispatch_conflict_zones, zone) < 0) {
                        dispatch_scheduler_healthy = 0;
                }
        }

        /* Phase 3: only the ticketed hard owner may move a switch or enter. */
        for (int slot = 0; slot < TC2_DISPATCH_MAX_JOBS; ++slot) {
                tc2_dispatch_job_snapshot *job = &dispatch_jobs[slot];
                tc2_dispatch_runtime *runtime = &dispatch_runtime[slot];
                if (job->state == TC2_JOB_EMPTY) {
                        continue;
                }
                int discard_failed = 0;
                if (runtime->conflict_zone_plan_valid &&
                    (job->state == TC2_JOB_ARRIVED ||
                     job->state == TC2_JOB_FAILED ||
                     job->state == TC2_JOB_CANCEL_BRAKING ||
                     job->state == TC2_JOB_STOP_UNCONFIRMED ||
                     job->state == TC2_JOB_STOPPED)) {
                        if (discard_future_conflict_zone_work(slot) < 0) {
                                dispatch_scheduler_healthy = 0;
                                discard_failed = 1;
                        }
                }
                process_conflict_zone_carries(slot, now);
                if (discard_failed) {
                        continue;
                }
                if (!runtime->conflict_zone_plan_valid) {
                        continue;
                }
                int active = conflict_zone_job_can_request(job);
                if (active && runtime->conflict_zone_queue_pending &&
                    queue_owned_conflict_zone_setting(slot) < 0) {
                        fail_job(slot, TC2_FAILURE_TURNOUT);
                        continue;
                }
                if (active && runtime->conflict_zone_batch_token != 0) {
                        int action_step =
                                runtime->conflict_zone_action_step;
                        tc2_conflict_zone_grant action_grant;
                        int action_ticket_valid =
                                action_step ==
                                        runtime->conflict_zone_next_step &&
                                action_step >= 0 &&
                                action_step < runtime
                                        ->conflict_zone_plan.step_count &&
                                runtime->conflict_zone_action_ticket != 0 &&
                                runtime->conflict_zone_action_ticket ==
                                        runtime->conflict_zone_hard_ticket &&
                                Tc2ConflictZoneQueryGrant(
                                        &dispatch_conflict_zones,
                                        runtime->conflict_zone_plan
                                                .steps[action_step].zone,
                                        &action_grant) > 0 &&
                                action_grant.owner_train == job->train &&
                                action_grant.owner_ticket ==
                                        runtime
                                          ->conflict_zone_action_ticket &&
                                action_grant.state ==
                                        TC2_CONFLICT_ZONE_SETTING;
                        if (!action_ticket_valid) {
                                (void)CanCancelBatch(
                                        dispatch_can_tid,
                                        runtime->conflict_zone_batch_token);
                                runtime->conflict_zone_batch_token = 0;
                                runtime->conflict_zone_settle_at_tick = -1;
                                reset_conflict_zone_action_cursor(runtime);
                                fail_job(slot, TC2_FAILURE_TURNOUT);
                                continue;
                        }
                        can_batch_status_t status;
                        if (CanGetBatchStatus(
                                    dispatch_can_tid,
                                    runtime->conflict_zone_batch_token,
                                    &status) < 0 ||
                            status.token !=
                                    runtime->conflict_zone_batch_token) {
                                fail_job(slot, TC2_FAILURE_TURNOUT);
                                continue;
                        }
                        if (status.state == CAN_BATCH_FAILED) {
                                int step_index =
                                        runtime->conflict_zone_next_step;
                                if (step_index >= 0 &&
                                    step_index < runtime
                                            ->conflict_zone_plan
                                            .step_count) {
                                        int zone = runtime
                                                ->conflict_zone_plan
                                                .steps[step_index].zone;
                                        (void)Tc2ConflictZoneRetrySetting(
                                                &dispatch_conflict_zones,
                                                zone, job->train);
                                }
                                runtime->conflict_zone_batch_token = 0;
                                runtime->conflict_zone_settle_at_tick = -1;
                                runtime->conflict_zone_queue_pending = 1;
                                continue;
                        }
                        if (status.state != CAN_BATCH_COMPLETE ||
                            status.confirmed != status.total) {
                                continue;
                        }
                        runtime->conflict_zone_batch_token = 0;
                        runtime->conflict_zone_settle_at_tick =
                                tick_after(now,
                                           TC2_TURNOUT_SETTLE_TICKS);
                }
                if (active &&
                    runtime->conflict_zone_settle_at_tick >= 0 &&
                    tick_reached(now,
                        runtime->conflict_zone_settle_at_tick)) {
                        int step_index =
                                runtime->conflict_zone_next_step;
                        if (step_index >= 0 && step_index <
                            runtime->conflict_zone_plan.step_count) {
                                const tc2_conflict_zone_route_step *setting_step =
                                        &runtime->conflict_zone_plan
                                                 .steps[step_index];
                                if (runtime->conflict_zone_action_step !=
                                            step_index ||
                                    runtime->conflict_zone_action_ticket !=
                                            runtime
                                              ->conflict_zone_hard_ticket ||
                                    runtime->conflict_zone_action_index < 0 ||
                                    runtime->conflict_zone_action_index >=
                                            setting_step->setting
                                                    .action_count) {
                                        fail_job(slot,
                                                 TC2_FAILURE_TURNOUT);
                                        continue;
                                }
                                ++runtime->conflict_zone_action_index;
                                if (runtime->conflict_zone_action_index <
                                        setting_step->setting.action_count) {
                                        /*
                                         * Keep the shared zone in SETTING.
                                         * The next scheduler pass sends only
                                         * the next real turnout on the route;
                                         * no paired motor is synthesized.
                                         */
                                        runtime->conflict_zone_queue_pending =
                                                1;
                                        runtime->conflict_zone_settle_at_tick =
                                                -1;
                                        continue;
                                }
                                if (Tc2ConflictZoneMarkSettled(
                                            &dispatch_conflict_zones,
                                            setting_step->zone,
                                            job->train) < 0) {
                                        fail_job(slot,
                                                 TC2_FAILURE_TURNOUT);
                                        continue;
                                }
                                reset_conflict_zone_action_cursor(runtime);
                        }
                        runtime->conflict_zone_settle_at_tick = -1;
                }
                if (active &&
                    runtime->conflict_zone_batch_token == 0 &&
                    runtime->conflict_zone_settle_at_tick < 0 &&
                    queue_owned_conflict_zone_setting(slot) < 0) {
                        fail_job(slot, TC2_FAILURE_TURNOUT);
                        continue;
                }
                if (active &&
                    queue_soft_conflict_zone_prefetch(slot, now) < 0) {
                        fail_job(slot, TC2_FAILURE_TURNOUT);
                        continue;
                }
                int step_index = runtime->conflict_zone_next_step;
                int physical_zone_progress_due = 0;
                if (active && step_index >= 0 &&
                    step_index <
                            runtime->conflict_zone_plan.step_count &&
                    runtime->conflict_zone_hard_ticket != 0) {
                        const tc2_conflict_zone_route_step *progress_step =
                                &runtime->conflict_zone_plan
                                         .steps[step_index];
                        tc2_conflict_zone_grant progress_grant;
                        int64_t progress_um = 0;
                        if (Tc2ConflictZoneQueryGrant(
                                    &dispatch_conflict_zones,
                                    progress_step->zone,
                                    &progress_grant) > 0 &&
                            progress_grant.owner_train == job->train &&
                            progress_grant.owner_ticket ==
                                    runtime->conflict_zone_hard_ticket &&
                            runtime_conflict_zone_progress_lower_bound_um(
                                    slot, now, &progress_um) == 0) {
                                physical_zone_progress_due =
                                        (progress_grant.state ==
                                                 TC2_CONFLICT_ZONE_SETTLED &&
                                         progress_um +
                                                 (int64_t)
                                                         TC2_TRAIN_HALF_LENGTH_MM *
                                                         1000 >=
                                                 (int64_t)progress_step
                                                         ->entry_distance_mm *
                                                         1000) ||
                                        (progress_grant.state ==
                                                 TC2_CONFLICT_ZONE_OCCUPIED &&
                                         progress_um >=
                                                 (int64_t)progress_step
                                                         ->exit_distance_mm *
                                                         1000);
                        }
                }
                /*
                 * A turnout hold can outlive the final route-zone cursor.
                 * In that normal case next_step == step_count and there is
                 * no hard ticket left to inspect.  Resolve the hold before
                 * the range check below; otherwise the train is stranded in
                 * AUTHORITY hold forever after consuming its last turnout.
                 * conflict_zone_step_ready() accepts exactly the exhausted
                 * cursor and rejects every other invalid value.
                 */
                if (active && !physical_zone_progress_due &&
                    runtime->conflict_zone_hold_active &&
                    conflict_zone_step_ready(slot, step_index) &&
                    job->state == TC2_JOB_TRAFFIC_HOLD) {
                        /*
                         * Settling the exact current turnout only proves
                         * switch authority.  It must not be mistaken for a
                         * long enough rolling reservation.  If the frozen
                         * train cannot brake inside its present window,
                         * consume the zone latch and hand the same stopped
                         * position to the ordinary FIFO authority extender.
                         * No positive-speed command is sent until that
                         * extension is committed and its launch path is
                         * confirmed.
                         */
                        if (!traffic_resume_fits_current_authority(
                                    slot,
                                    runtime->traffic_resume_speed)) {
                                int safe_resume_speed = -1;
                                if (runtime->traffic_hold_distance_um >= 0 &&
                                    job->destination_distance_mm >= 0) {
                                        int current_progress_mm =
                                                (int)((runtime
                                                                      ->traffic_hold_distance_um +
                                                              999) /
                                                      1000);
                                        int remaining_mm =
                                                job->destination_distance_mm -
                                                current_progress_mm;
                                        safe_resume_speed =
                                                authority_resume_speed_for_remaining(
                                                        runtime
                                                                ->traffic_resume_speed,
                                                        remaining_mm);
                                }
                                int full_authority =
                                        !runtime->rolling_authority_active ||
                                        (runtime->route_valid &&
                                         runtime->authority_end_offset ==
                                                 runtime->route.node_count -
                                                         1);
                                if (full_authority &&
                                    safe_resume_speed > 0) {
                                        /*
                                         * Near the destination there may be
                                         * no route left to reserve, while the
                                         * original command speed no longer
                                         * fits its measured coast distance.
                                         * Resume at the highest generally
                                         * calibrated speed that can still
                                         * stop inside the remaining route.
                                         */
                                        runtime->traffic_resume_speed =
                                                safe_resume_speed;
                                } else if (
                                    runtime->rolling_authority_active &&
                                    runtime->authority_end_offset >= 0 &&
                                    runtime->authority_end_offset <
                                            runtime->route.node_count - 1) {
                                        runtime->conflict_zone_hold_active =
                                                0;
                                        runtime->traffic_reason =
                                                TC2_TRAFFIC_AUTHORITY;
                                        runtime->traffic_peer_train = 0;
                                        job->traffic_reason =
                                                TC2_TRAFFIC_AUTHORITY;
                                        job->traffic_peer_train = 0;
                                        if (begin_authority_request(
                                                    slot, now, 0) < 0) {
                                                fail_job(
                                                        slot,
                                                        TC2_FAILURE_INTERNAL);
                                        }
                                        continue;
                                } else {
                                        continue;
                                }
                        }
                        if (resume_traffic_hold(
                                    slot, now,
                                    TC2_TRAFFIC_AUTHORITY) == 0) {
                                runtime->conflict_zone_hold_active = 0;
                        }
                }
                if (step_index < 0 || step_index >=
                    runtime->conflict_zone_plan.step_count) {
                        continue;
                }
                const tc2_conflict_zone_route_step *step =
                        &runtime->conflict_zone_plan.steps[step_index];
                tc2_conflict_zone_grant grant;
                if (Tc2ConflictZoneQueryGrant(
                            &dispatch_conflict_zones, step->zone,
                            &grant) <= 0 ||
                    grant.owner_train != job->train ||
                    runtime->conflict_zone_hard_ticket == 0 ||
                    grant.owner_ticket !=
                            runtime->conflict_zone_hard_ticket) {
                        continue;
                }

                int64_t progress_um = 0;
                int entered_this_tick = 0;
                int origin_egress_proved = 0;
                if (origin_zone_egress_ready(slot, step_index)) {
                        if (step->exit_distance_mm >
                                    step->entry_distance_mm) {
                                origin_egress_proved =
                                        runtime_conflict_zone_progress_lower_bound_um(
                                                slot, now,
                                                &progress_um) == 0 &&
                                        progress_um >=
                                                (int64_t)step
                                                        ->exit_distance_mm *
                                                        1000;
                        } else {
                                /*
                                 * A trimmed reverse-origin step may collapse
                                 * to MR1..MR1 (entry == exit == 0).  The
                                 * projected scalar is then true at arm time,
                                 * before any movement.  Require a later
                                 * ordered sensor that advances the confirmed
                                 * route scalar instead of consuming the step
                                 * immediately.
                                 */
                                origin_egress_proved =
                                        runtime->last_journal_sequence !=
                                                runtime
                                                  ->conflict_zone_origin_egress_sequence &&
                                        runtime->monitor
                                                        .confirmed_distance_mm >
                                                step->exit_distance_mm;
                        }
                }
                if (origin_egress_proved) {
                        /*
                         * The body already owns this physical zone.  Consume
                         * the new route cursor only after its calibrated
                         * centre reaches the route-relative exit.  Advancing
                         * at front-edge entry promoted next/next-next while
                         * the train was still traversing a compound zone and
                         * could move a later turnout underneath the train.
                         * The old carry keeps the exact ticket until a later
                         * real sensor proves tail-clear.
                         */
                        ++runtime->conflict_zone_next_step;
                        runtime->conflict_zone_hard_step = -1;
                        runtime->conflict_zone_hard_ticket = 0;
                        reset_conflict_zone_action_cursor(runtime);
                        clear_origin_zone_egress(runtime);
                        if (promote_next_conflict_zone_after_entry(slot) < 0) {
                                fail_job(slot, TC2_FAILURE_TURNOUT);
                        }
                        continue;
                }
                if (grant.state == TC2_CONFLICT_ZONE_SETTLED &&
                    runtime_conflict_zone_progress_lower_bound_um(
                            slot, now, &progress_um) == 0 &&
                    progress_um +
                            (int64_t)TC2_TRAIN_HALF_LENGTH_MM * 1000 >=
                            (int64_t)step->entry_distance_mm * 1000) {
                        if (Tc2ConflictZoneMarkOccupied(
                                    &dispatch_conflict_zones, step->zone,
                                    job->train) == 0) {
                                grant.state = TC2_CONFLICT_ZONE_OCCUPIED;
                                /*
                                 * Physical ownership and forward preparation
                                 * have different lifetimes.  Once the front
                                 * enters this zone, retain its exact OCCUPIED
                                 * ticket as a carry until the tail is clear,
                                 * but allow next/next-next preparation to move
                                 * on immediately when the following route step
                                 * is a different physical zone.  Waiting for
                                 * the centre to cross the old exit made an
                                 * uncontended single train stop before every
                                 * closely-spaced turnout because the following
                                 * motor could not settle in time.
                                 *
                                 * Consecutive actions in the same compound
                                 * physical zone remain fail-closed: advancing
                                 * there could still move a turnout underneath
                                 * the train.
                                 */
                                entered_this_tick = 1;

                                int next_step = step_index + 1;
                                int next_is_distinct_zone =
                                        next_step >= runtime
                                                ->conflict_zone_plan.step_count ||
                                        runtime->conflict_zone_plan
                                                        .steps[next_step].zone !=
                                                step->zone;
                                if (next_is_distinct_zone) {
                                        int entry_node = -1;
                                        int exit_node = -1;
                                        if (step->first_route_offset >= 0 &&
                                            step->first_route_offset <
                                                    runtime->route.node_count) {
                                                entry_node = runtime->route.nodes[
                                                        step->first_route_offset];
                                        }
                                        if (step->last_route_offset >= 0 &&
                                            step->last_route_offset <
                                                    runtime->route.node_count) {
                                                exit_node = runtime->route.nodes[
                                                        step->last_route_offset];
                                        }
                                        if (remember_conflict_zone_carry(
                                                    runtime, step->zone,
                                                    entry_node, exit_node,
                                                    step->exit_distance_mm,
                                                    1, grant.owner_ticket,
                                                    TC2_CONFLICT_CARRY_OCCUPIED) <
                                            0) {
                                                fail_job(
                                                        slot,
                                                        TC2_FAILURE_INTERNAL);
                                                continue;
                                        }
                                        runtime->conflict_zone_next_step =
                                                next_step;
                                        runtime->conflict_zone_hard_step = -1;
                                        runtime->conflict_zone_hard_ticket = 0;
                                        reset_conflict_zone_action_cursor(
                                                runtime);
                                        if (promote_next_conflict_zone_after_entry(
                                                    slot) < 0) {
                                                fail_job(
                                                        slot,
                                                        TC2_FAILURE_TURNOUT);
                                        }
                                }
                        }
                }
                if (entered_this_tick) {
                        continue;
                }
                if (grant.state == TC2_CONFLICT_ZONE_OCCUPIED &&
                    runtime_conflict_zone_progress_lower_bound_um(
                            slot, now, &progress_um) == 0 &&
                    progress_um >=
                            (int64_t)step->exit_distance_mm * 1000) {
                        int entry_node = -1;
                        int exit_node = -1;
                        if (step->first_route_offset >= 0 &&
                            step->first_route_offset <
                                    runtime->route.node_count) {
                                entry_node = runtime->route.nodes[
                                        step->first_route_offset];
                        }
                        if (step->last_route_offset >= 0 &&
                            step->last_route_offset <
                                    runtime->route.node_count) {
                                exit_node = runtime->route.nodes[
                                        step->last_route_offset];
                        }
                        /*
                         * The acquire cursor and the physical ownership have
                         * different lifetimes.  The cursor may advance at the
                         * exit, while the exact OCCUPIED ticket remains as a
                         * carry until sensor-confirmed tail clear plus the
                         * release delay.
                         */
                        if (remember_conflict_zone_carry(
                                    runtime, step->zone,
                                    entry_node, exit_node,
                                    step->exit_distance_mm,
                                    1, grant.owner_ticket,
                                    TC2_CONFLICT_CARRY_OCCUPIED) < 0) {
                                fail_job(slot, TC2_FAILURE_INTERNAL);
                                continue;
                        }
                        ++runtime->conflict_zone_next_step;
                        runtime->conflict_zone_hard_step = -1;
                        runtime->conflict_zone_hard_ticket = 0;
                        reset_conflict_zone_action_cursor(runtime);
                        if (promote_next_conflict_zone_after_entry(slot) < 0) {
                                fail_job(slot, TC2_FAILURE_TURNOUT);
                        }
                }
        }
}

static int queue_turnout_plan_range(
        int slot, int plan_start, int plan_end) {
        track_turnout_plan plan;
        int switch_numbers[CAN_COMMAND_BATCH_MAX];
        char directions[CAN_COMMAND_BATCH_MAX];
        int count = 0;
        if (slot < 0 || slot >= TC2_DISPATCH_MAX_JOBS) {
                return -1;
        }
        tc2_dispatch_runtime *runtime = &dispatch_runtime[slot];
        if (!runtime->route_valid ||
            plan_start < 0 || plan_end < plan_start ||
            plan_end >= runtime->route.node_count) {
                return -1;
        }
        /*
         * Once the physical-zone plan exists, only its current owner is
         * allowed to put a switch command on CAN.  The zone scheduler below
         * sends one canonical setting after a hard grant; the legacy
         * whole-leg batch must therefore become a no-op for these jobs.
         */
        if (runtime->conflict_zone_plan_valid) return 0;
        if (build_turnout_plan_range_checked(
                    &runtime->route, plan_start,
                    plan_end, &plan) < 0) {
                return -1;
        }
        for (int action = 0; action < plan.action_count; ++action) {
                if (runtime->carry_ambiguity_active &&
                    turnout_action_overlaps_footprint(
                            &plan.actions[action],
                            &runtime
                                     ->carry_ambiguity_footprint)) {
                        /*
                         * This physical resource may be beneath any one of
                         * the carried position hypotheses. The preceding
                         * final-leg batch already received its CS3
                         * confirmation, so an identical setting is reused
                         * without putting another command on the wire.
                         */
                        if (!runtime
                                     ->carry_confirmed_turnouts_valid ||
                            !turnout_action_matches_confirmed_plan(
                                    &plan.actions[action],
                                    &runtime
                                             ->carry_confirmed_turnouts)) {
                                return -1;
                        }
                        continue;
                }
                char direction =
                        Tc2TrackPhysicalTurnoutDirection(
                                plan.actions[action].direction);
                if (!direction) return -1;
                int mate = paired_turnout_mate(
                        plan.actions[action].switch_number);
                if (plan.actions[action].direction ==
                            DIR_CURVED &&
                    mate != 0) {
                        if (count >= CAN_COMMAND_BATCH_MAX) return -1;
                        switch_numbers[count] = mate;
                        directions[count++] =
                                Tc2TrackPhysicalTurnoutDirection(
                                        DIR_STRAIGHT);
                }
                if (count >= CAN_COMMAND_BATCH_MAX) return -1;
                switch_numbers[count] =
                        plan.actions[action].switch_number;
                directions[count++] = direction;
        }
        if (count == 0) return 0;
        return CanSwitchBatch(
                dispatch_can_tid, switch_numbers, directions, count);
}

static int queue_turnout_plan_for_leg(int slot) {
        tc2_dispatch_runtime *runtime = &dispatch_runtime[slot];
        tc2_dispatch_job_snapshot *job = &dispatch_jobs[slot];
        int plan_start = runtime->leg_start_offset;
        int plan_end = runtime->leg_end_offset;
        if (plan_end == job->destination_route_offset) {
                plan_end = runtime->route.node_count - 1;
        }
        if (runtime->rolling_authority_active &&
            runtime->authority_end_offset >=
                    runtime->leg_start_offset &&
            runtime->authority_end_offset < plan_end) {
                plan_end = runtime->authority_end_offset;
        }
        /*
         * An authority extension may run after the train has released track
         * behind it.  Reissuing the complete leg plan would then command
         * turnouts that this train no longer owns.  Include the old boundary
         * node (the connecting edge can itself be a turnout), but command
         * only the newly authorized suffix.
         */
        if (runtime->rolling_authority_active &&
            runtime->authority_resume_pending &&
            runtime->authority_previous_end_offset >= plan_start &&
            runtime->authority_previous_end_offset <= plan_end) {
                plan_start =
                        runtime->authority_previous_end_offset;
        }
        return queue_turnout_plan_range(
                slot, plan_start, plan_end);
}

static int next_leg_turnouts_clear_of_uncertain_train(int slot) {
        tc2_dispatch_job_snapshot *job = &dispatch_jobs[slot];
        tc2_dispatch_runtime *runtime = &dispatch_runtime[slot];
        track_turnout_plan plan;
        track_route ambiguity;
        int next_leg = runtime->leg_end_offset + 1;
        int next_end;
        int corridor_first;
        if (next_leg >= runtime->route.node_count) return 0;
        next_end = TrackRouteMotionLegEnd(
                dispatch_track, &runtime->route, next_leg);
        if (next_end >= job->destination_route_offset) {
                next_end =
                        runtime->route.node_count - 1;
        }
        if (next_end < next_leg ||
            build_turnout_plan_range_checked(
                    &runtime->route, next_leg, next_end, &plan) < 0) {
                return 0;
        }

        /*
         * One false report can name the exact reversal sensor.  In that
         * case the stopped train may actually be anywhere from the body-tail
         * behind the last independently confirmed point through the reported
         * sensor.  Reversing inside owned track is safe, but changing any
         * turnout in that ambiguity corridor is not.  Treat each paired
         * middle turnout as one physical resource as well.
         */
        corridor_first = footprint_tail_offset(runtime, job);
        int braking_speed =
                runtime->motion_speed_ceiling > job->speed ?
                        runtime->motion_speed_ceiling :
                        job->speed;
        int forward_overrun =
                Tc2MotionBrakingDistanceMm(braking_speed);
        if (forward_overrun < 0 ||
            build_route_slice(
                    &runtime->route, corridor_first,
                    runtime->leg_end_offset, &ambiguity) < 0 ||
            append_forward_guard_reachable(
                    &ambiguity,
                    runtime->route.nodes[
                            runtime->leg_end_offset],
                    forward_overrun) < 0 ||
            append_paired_turnout_resources(&ambiguity) < 0) {
                return 0;
        }
        for (int action = 0; action < plan.action_count; ++action) {
                int controlled =
                        plan.actions[action].switch_number;
                int controlled_mate =
                        paired_turnout_mate(controlled);
                for (int offset = 0;
                     offset < ambiguity.node_count; ++offset) {
                        int node = ambiguity.nodes[offset];
                        int number = dispatch_track[node].num;
                        if ((dispatch_track[node].type == NODE_BRANCH ||
                             dispatch_track[node].type == NODE_MERGE) &&
                            (number == controlled ||
                             number == controlled_mate ||
                             paired_turnout_mate(number) ==
                                     controlled)) {
                                return 0;
                        }
                }
        }
        return 1;
}

static int route_has_foreign_active_sensor(
        int train, const track_route *route,
        const train_sensor_snapshot_t *sensor,
        const track_reservation_snapshot *reservation) {
        for (int offset = 0; offset < route->node_count; ++offset) {
                int node = route->nodes[offset];
                if (node < TRAIN_SENSOR_COUNT &&
                    sensor->sensor_state[node] &&
                    reservation->owner_by_node[node] != train) {
                        return node;
                }
        }
        return -1;
}

static int prelaunch_sensor_changed(
        const tc2_dispatch_runtime *runtime,
        const train_sensor_snapshot_t *sensor) {
        for (int offset = 0;
             offset < runtime->safety_footprint.node_count; ++offset) {
                int node = runtime->safety_footprint.nodes[offset];
                int physical[2] = {node, physical_reverse_index(node)};
                for (int side = 0; side < 2; ++side) {
                        int candidate = physical[side];
                        if (candidate < 0 ||
                            candidate >= TRAIN_SENSOR_COUNT ||
                            candidate == runtime->start_node ||
                            candidate ==
                                    physical_reverse_index(
                                            runtime->start_node)) {
                                continue;
                        }
                        if (!runtime->prepare_sensor_state[candidate] &&
                            sensor->sensor_state[candidate]) {
                                return candidate;
                        }
                }
        }
        return -1;
}

static int journal_is_clean_after(
        int train, unsigned int after) {
        for (int page = 0; page < 8; ++page) {
                train_sensor_event_batch_t batch;
                if (TrainSensorGetAttributedEvents(
                            dispatch_sensor_tid, train,
                            after, &batch) < 0 ||
                    batch.lost || batch.count < 0 ||
                    batch.count >
                            TRAIN_SENSOR_EVENT_BATCH_CAPACITY) {
                        return 0;
                }
                if (batch.count > 0) return 0;
                if (!batch.has_more) return 1;
                after = batch.newest_sequence;
        }
        return 0;
}

static int prelaunch_journal_is_clean(
        const tc2_dispatch_job_snapshot *job,
        const tc2_dispatch_runtime *runtime) {
        return journal_is_clean_after(
                job->train,
                runtime->prepare_attributed_sequence);
}

static int has_other_moving_or_queued_job(int slot) {
        for (int other = 0;
             other < TC2_DISPATCH_MAX_JOBS; ++other) {
                if (other == slot) continue;
                if (conflict_zone_job_can_request(
                            &dispatch_jobs[other])) {
                        return 1;
                }
        }
        return 0;
}

static int prepare_waiting_job(int slot, int now) {
        tc2_dispatch_job_snapshot *job = &dispatch_jobs[slot];
        tc2_dispatch_runtime *runtime = &dispatch_runtime[slot];
        track_reservation_snapshot reservation;
        train_sensor_snapshot_t sensor;
        track_route route;
        track_route safety_footprint;
        tc2_conflict_zone_route_plan candidate_zone_plan;
        track_reservation_conflict conflict;
        unsigned int generation;
        int target = -1;
        int target_offset = -1;
        int destination_route_offset = -1;
        int destination_offset_value = 0;
        int destination_confirmed = 0;
        int path_distance = -1;
        int side = -1;
        int localization_only = 0;
        int estimated_ambiguity_blocked = 0;
        int projection_ready = 0;
        tc2_route_projection_selection projection_selection;
        tc2_prediction_request projection_request;
        track_route projection_geometry_route;
        tc2_prediction_plan projection_prediction;
        const tc2_prediction_plan *projection_prediction_source = 0;
        int projection_anchor_offset = -1;
        int projection_destination_offset = -1;

        ensure_conflict_zone_table();
        candidate_zone_plan.step_count = 0;
        int rolling_authority = 0;
        int authority_end_offset = -1;
        int authority_center_ceiling_mm = -1;
        int authority_launch_speed = job->speed;
        int initial_authority_first_offset = 0;
        int initial_authority_minimum_end = 1;
        int initial_authority_progress_mm = 0;
        int leading_reversal = 0;
        track_route leading_reversal_carry;
        leading_reversal_carry.node_count = 0;
        leading_reversal_carry.distance_mm = 0;
        leading_reversal_carry.optimization_cost_mm = 0;
        leading_reversal_carry.reversal_count = 0;
        const tc2_track_destination_side *selected_side_definition = 0;
        int rolling_authority_candidate =
                (job->start_index >= 0 &&
                 job->start_index <
                         TC2_DISPATCH_START_COUNT) ||
                job->start_index ==
                        TC2_DISPATCH_START_CURRENT;
        /*
         * Route selection for every rolling-authority job ignores distant
         * managed reservations.  The exact first local window is still checked
         * atomically below, so CURRENT recovery can choose a safe shortest path
         * without a far-away train making the whole graph unavailable.
         */
        int rolling_route_selection =
                rolling_authority_candidate;
        candidate_zone_plan.step_count = 0;

        if (TrackReservationServerSnapshot(
                    dispatch_reservation_tid, &reservation) < 0) {
                fail_job(slot, TC2_FAILURE_RESERVATION_SERVICE);
                return -1;
        }
        if (TrainSensorGetLatest(dispatch_sensor_tid, &sensor) < 0) {
                fail_job(slot, TC2_FAILURE_SENSOR_SERVICE);
                return -1;
        }
        if (runtime->stage_sensor_baseline_valid &&
            !journal_is_clean_after(
                    job->train,
                    runtime->stage_attributed_sequence)) {
                fail_job(slot, TC2_FAILURE_SENSOR_SEQUENCE);
                return -1;
        }
        /*
         * CURRENT staging preserves the ARRIVED job's CAN baseline.  A
         * turnout change between staging and planning must not become the
         * new trusted baseline, especially when an identical prior setting
         * would otherwise be reused without retransmission.
         */
        if (can_health_is_safe(slot, 0, now) < 0) {
                fail_job(slot, TC2_FAILURE_CAN_HEALTH);
                return -1;
        }
        if (choose_shortest_available_route(
                    slot, &reservation, &sensor, &route, &target,
                    &target_offset, &destination_route_offset,
                    &destination_offset_value,
                    &destination_confirmed,
                    &path_distance, &side,
                    &estimated_ambiguity_blocked,
                    rolling_route_selection) < 0) {
                unsigned char unavailable[TRACK_MAX];
                if (!runtime->carry_ambiguity_active ||
                    !estimated_ambiguity_blocked ||
                    runtime->localization_hops >=
                            TC2_MAX_LOCALIZATION_HOPS) {
                        job->conflict_train = 0;
                        job->conflict_node = -1;
                        return 1;
                }
                build_unavailable(
                        slot, &reservation, &sensor,
                        unavailable, 0);
                if (choose_safe_localization_route(
                            slot, unavailable, &route,
                            &target, &target_offset,
                            &path_distance) < 0) {
                        job->conflict_train = 0;
                        job->conflict_node = -1;
                        return 1;
                }
                side = -1;
                localization_only = 1;
                util_zero_bytes(
                        (volatile unsigned char *)
                                &runtime->prediction_plan,
                        (unsigned int)sizeof(
                                runtime->prediction_plan));
                runtime->prediction_plan_valid = 0;
                clear_prediction_snapshot(job);
                destination_route_offset = target_offset;
                destination_offset_value = 0;
                destination_confirmed = 1;
        }
        rolling_authority =
                rolling_authority_candidate &&
                !job->provisional_prediction &&
                !localization_only;
        if (rolling_authority) {
                /*
                 * With no second moving/queued train there is no FIFO,
                 * next-turnout, next-next-turnout, following, or head-on
                 * contender.  A bounded rolling window would only create an
                 * artificial authority stop while CS3 changes the next
                 * turnout (the observed E11/E12 stop on A->d1).  Acquire the
                 * complete single-train motion leg at admission so every
                 * required turnout is confirmed before launch and the train
                 * runs continuously.  This does not weaken multi-train
                 * arbitration: as soon as another live job exists both jobs
                 * retain bounded local authority and FIFO conflict zones.
                 */
                /*
                 * The first local authority must cover both one complete
                 * conservative stop and one CAN-confirmation/turnout-settle
                 * cycle.  Otherwise a route whose first discrete boundary is
                 * only just stoppable can launch correctly, request its next
                 * suffix immediately, and still hit the old hard ceiling
                 * before that suffix becomes usable.  This was first exposed
                 * by command 120 from A and later by command 100 from C.
                 *
                 * Seed the bounded selector one prefetch lead ahead for the
                 * calibrated A/B high-speed case and for C/D/E/F, whose
                 * longer sensor intervals expose the same race at ordinary
                 * speeds. It still
                 * chooses the shortest conflict-free local prefix, so this
                 * does not restore whole-route reservation
                 * at the origin.
                 */
                if (job->start_index >= 0 &&
                    job->start_index <
                            TC2_DISPATCH_START_COUNT &&
                    uses_measured_direct_stop(job) &&
                    (job->speed == 120 ||
                     job->start_index >= 2)) {
                        initial_authority_progress_mm =
                                authority_prefetch_lead_mm(
                                        job->speed);
                        if (initial_authority_progress_mm < 0) {
                                fail_job(
                                        slot,
                                        TC2_FAILURE_ROUTE);
                                return -1;
                        }
                }
                /*
                 * F->d1/d2 at command 100/120 has no intermediate boundary
                 * that contains the requested-speed conservative stop plus
                 * the turnout-confirmation lead. Treating the complete short
                 * trip as "remote" merely downshifts launch speed (d1 was
                 * previously 94 at command 100 and 92 at command 120). For
                 * these two short high-speed trips the endpoint is the first
                 * local authority block; lower speeds and longer F routes
                 * continue to use rolling windows.
                 */
                if (job->start_index == 5 &&
                    job->destination_index <= 1 &&
                    job->speed >= 100) {
                        initial_authority_minimum_end =
                                route.node_count - 1;
                }
                int initial_leg_end =
                        TrackRouteMotionLegEnd(
                                dispatch_track, &route, 0);
                if (initial_leg_end < 0 ||
                    initial_leg_end >= route.node_count) {
                        fail_job(slot, TC2_FAILURE_ROUTE);
                        return -1;
                }
                if (!has_other_moving_or_queued_job(slot)) {
                        initial_authority_minimum_end =
                                initial_leg_end;
                }
                if (initial_leg_end == 0 &&
                    destination_route_offset > 0) {
                        if (route.node_count < 2 ||
                            dispatch_track[route.nodes[0]].reverse !=
                                    &dispatch_track[
                                            route.nodes[1]]) {
                                fail_job(slot, TC2_FAILURE_ROUTE);
                                return -1;
                        }
                        leading_reversal = 1;
                        initial_authority_first_offset = 1;
                        initial_authority_minimum_end = 2;
                }
                int require_head_on_release_witness =
                        job->start_index ==
                                TC2_DISPATCH_START_CURRENT &&
                        runtime->head_on_recovery_active &&
                        runtime->carry_ambiguity_active;
                int requested_stop_distance =
                        Tc2MotionStopDistanceMm(job->speed);
                int allow_precision_speed_fallback =
                        requested_stop_distance >= 0 &&
                        path_distance <= requested_stop_distance;
                int window_status =
                        require_head_on_release_witness ?
                        select_head_on_recovery_initial_authority(
                                &route,
                                initial_authority_first_offset,
                                initial_authority_minimum_end,
                                target_offset,
                                destination_route_offset,
                                destination_offset_value,
                                path_distance, job->speed,
                                leading_reversal ||
                                        allow_precision_speed_fallback,
                                initial_authority_progress_mm,
                                job->train,
                                &reservation, &sensor,
                                &safety_footprint,
                                &authority_end_offset,
                                &authority_center_ceiling_mm,
                                &authority_launch_speed,
                                &conflict) :
                        select_bounded_rolling_authority_window(
                                &route,
                                initial_authority_first_offset,
                                initial_authority_minimum_end,
                                target_offset,
                                destination_route_offset,
                                destination_offset_value,
                                path_distance, job->speed,
                                leading_reversal ||
                                        allow_precision_speed_fallback,
                                initial_authority_progress_mm,
                                job->train,
                                &reservation, &sensor,
                                &safety_footprint,
                                &authority_end_offset,
                                &authority_center_ceiling_mm,
                                &authority_launch_speed,
                                &conflict);
                if (window_status == 0 &&
                    leading_reversal) {
                        int stopped_guard =
                                TC2_TRAIN_HALF_LENGTH_MM +
                                TC2_TRAFFIC_FOLLOWING_STOP_CLEARANCE_MM;
                        if (build_route_slice(
                                    &route, 0, 0,
                                    &leading_reversal_carry) < 0 ||
                            append_forward_guard_reachable(
                                    &leading_reversal_carry,
                                    route.nodes[0],
                                    stopped_guard) < 0 ||
                            append_paired_turnout_resources(
                                    &leading_reversal_carry) < 0 ||
                            append_route_nodes(
                                    &safety_footprint,
                                    &leading_reversal_carry) < 0) {
                                fail_job(slot, TC2_FAILURE_ROUTE);
                                return -1;
                        }
                        track_reservation_conflict
                                carry_conflict;
                        int carry_available =
                                authority_footprint_available(
                                        &safety_footprint,
                                        job->train,
                                        &reservation, &sensor,
                                        &carry_conflict);
                        if (carry_available > 0) {
                                job->conflict_train =
                                        carry_conflict
                                                .owner_train;
                                job->conflict_node =
                                        carry_conflict
                                                .node_index;
                                return 1;
                        }
                        if (carry_available < 0) {
                                fail_job(
                                        slot,
                                        TC2_FAILURE_ROUTE);
                                return -1;
                        }
                }
                if (window_status > 0) {
                        job->conflict_train =
                                conflict.owner_train;
                        job->conflict_node =
                                conflict.node_index;
                        return 1;
                }
                if (window_status < 0) {
                        fail_job(slot, TC2_FAILURE_ROUTE);
                        return -1;
                }
        } else {
                int active = route_has_foreign_active_sensor(
                        job->train, &route, &sensor,
                        &reservation);
                if (active >= 0) {
                        job->conflict_train = 0;
                        job->conflict_node = active;
                        return 1;
                }
                if (build_safety_footprint_for_route(
                            &route, 0, target_offset,
                            destination_route_offset,
                            destination_offset_value,
                            job->speed,
                            &safety_footprint) < 0) {
                        fail_job(slot, TC2_FAILURE_ROUTE);
                        return -1;
                }
        }
        if ((runtime->carry_ambiguity_active &&
             append_route_nodes(
                     &safety_footprint,
                     &runtime->carry_ambiguity_footprint) < 0)) {
                fail_job(slot, TC2_FAILURE_ROUTE);
                return -1;
        }
        if (destination_route_offset > 0 &&
            Tc2ConflictZoneBuildRoutePlan(
                    dispatch_track, &route, 0,
                    destination_route_offset,
                    &candidate_zone_plan) < 0) {
                fail_job(slot, TC2_FAILURE_ROUTE);
                return -1;
        }
        /*
         * Recheck immediately before the reservation CAS. A failure here
         * leaves the preceding hold and its generation byte-for-byte
         * unchanged.
         */
        if (can_health_is_safe(slot, 0, now) < 0) {
                fail_job(slot, TC2_FAILURE_CAN_HEALTH);
                return -1;
        }

        selected_side_definition =
                side >= 0 && side < 2 ?
                Tc2TrackDestinationSide(
                        job->destination_index, side) : 0;
        /*
         * Build the complete immutable UI projection before changing
         * reservation ownership. Ordinary fixed-bay A-F jobs retain their
         * authoritative prediction-backed contract. An exact CURRENT
         * continuation publishes the same selected shortest-route geometry
         * without inventing a second timing controller.
         */
        int projection_required =
                (job->start_index ==
                         TC2_DISPATCH_START_CURRENT ||
                 (job->start_index >= 0 &&
                  job->start_index <
                          TC2_DISPATCH_START_COUNT)) &&
                !localization_only;
        if (projection_required) {
                if (!selected_side_definition) {
                        fail_job(slot, TC2_FAILURE_ROUTE);
                        return -1;
                }
                projection_anchor_offset = target_offset;
                projection_destination_offset =
                        destination_route_offset;
                if (job->start_index ==
                            TC2_DISPATCH_START_CURRENT) {
                        projection_request.train = job->train;
                        projection_request.start_index =
                                job->start_index;
                        projection_request.destination_index =
                                job->destination_index;
                        projection_request.destination_side = side;
                        projection_request.speed = job->speed;
                        projection_request.reversal_count =
                                route.reversal_count;
                        projection_selection.selected_motion_route =
                                &route;
                        projection_selection
                                .geometry_anchor_route_offset =
                                projection_anchor_offset;
                        projection_selection
                                .physical_destination_route_offset =
                                projection_destination_offset;
                        projection_selection
                                .physical_destination_offset_mm =
                                selected_side_definition
                                        ->base_offset_mm;
                        Tc2RouteProjectionInitialize(
                                &dispatch_projection_scratch);
                        int projection_status =
                                Tc2RouteProjectionBuildCurrent(
                                    dispatch_track,
                                    &projection_selection,
                                    &projection_request,
                                    &dispatch_projection_scratch);
                        if (projection_status ==
                                    TC2_ROUTE_PROJECTION_OK) {
                                projection_status =
                                        Tc2RouteProjectionValidateCurrent(
                                            dispatch_track,
                                            &projection_selection,
                                            &projection_request,
                                            &dispatch_projection_scratch);
                        }
                        if (projection_status !=
                                    TC2_ROUTE_PROJECTION_OK) {
                                fail_job(slot, TC2_FAILURE_ROUTE);
                                return -1;
                        }
                        if (runtime->force_reverse_first &&
                            runtime->carry_preorigin_distance_mm > 0 &&
                            shift_current_projection_for_preorigin(
                                    &dispatch_projection_scratch,
                                    runtime
                                            ->carry_preorigin_distance_mm) <
                                    0) {
                                fail_job(slot, TC2_FAILURE_ROUTE);
                                return -1;
                        }
                        projection_ready = 1;
                } else {
                if (job->provisional_prediction) {
                        int physical_anchor =
                                TrackFindNodeByName(
                                        dispatch_track,
                                        selected_side_definition
                                                ->anchor_sensor);
                        projection_anchor_offset = -1;
                        for (int offset = 0;
                             offset < route.node_count; ++offset) {
                                if (route.nodes[offset] ==
                                    physical_anchor) {
                                        projection_anchor_offset =
                                                offset;
                                        break;
                                }
                        }
                        if (projection_anchor_offset < 0 ||
                            route_ceiling_after_offset(
                                    &route,
                                    projection_anchor_offset,
                                    selected_side_definition
                                            ->base_offset_mm,
                                    &projection_destination_offset) < 0) {
                                fail_job(slot, TC2_FAILURE_ROUTE);
                                return -1;
                        }
                } else if (!runtime->prediction_plan_valid) {
                        fail_job(slot, TC2_FAILURE_ROUTE);
                        return -1;
                }
                if (build_route_slice(
                            &route, 0,
                            projection_anchor_offset,
                            &projection_geometry_route) < 0) {
                        fail_job(slot, TC2_FAILURE_ROUTE);
                        return -1;
                }
                projection_request.train = job->train;
                projection_request.start_index =
                        job->start_index;
                projection_request.destination_index =
                        job->destination_index;
                projection_request.destination_side = side;
                projection_request.speed = job->speed;
                projection_request.reversal_count =
                        projection_geometry_route
                                .reversal_count;
                if (job->provisional_prediction) {
                        if (Tc2PredictionBuildPlan(
                                    dispatch_track,
                                    &projection_geometry_route,
                                    &projection_request,
                                    &projection_prediction) !=
                                        TC2_PREDICTION_OK) {
                                fail_job(slot, TC2_FAILURE_ROUTE);
                                return -1;
                        }
                        projection_prediction_source =
                                &projection_prediction;
                } else {
                        projection_prediction_source =
                                &runtime->prediction_plan;
                }
                projection_selection.selected_motion_route =
                        &route;
                projection_selection
                        .geometry_anchor_route_offset =
                        projection_anchor_offset;
                projection_selection
                        .physical_destination_route_offset =
                        projection_destination_offset;
                projection_selection
                        .physical_destination_offset_mm =
                        selected_side_definition
                                ->base_offset_mm;
                Tc2RouteProjectionInitialize(
                        &dispatch_projection_scratch);
                int projection_status =
                        Tc2RouteProjectionBuildWithPrediction(
                            dispatch_track,
                            &projection_selection,
                            &projection_request,
                            projection_prediction_source,
                            &dispatch_projection_scratch);
                if (projection_status == TC2_ROUTE_PROJECTION_OK) {
                        projection_status =
                                Tc2RouteProjectionValidateWithPrediction(
                            dispatch_track,
                            &projection_selection,
                            &projection_request,
                            projection_prediction_source,
                            &dispatch_projection_scratch);
                }
                if (projection_status !=
                            TC2_ROUTE_PROJECTION_OK) {
                        if (!job->provisional_prediction) {
                                fail_job(slot, TC2_FAILURE_ROUTE);
                                return -1;
                        }
                        /*
                         * A negative supervised correction can make the
                         * physical-OP projection require a longer audit
                         * suffix than the already conservative control-point
                         * reservation.  Visualization must never reject an
                         * otherwise valid physical calibration run; the live
                         * UI falls back to start/OP plus exact CAN flashes.
                         */
                        projection_ready = 0;
                } else {
                        projection_ready = 1;
                }
                }
        }

        int reserve = runtime->replace_plan_expected &&
                              rolling_authority ?
                TrackReservationServerReplaceWindowExpected(
                        dispatch_reservation_tid,
                        &safety_footprint, job->train,
                        runtime->replace_expected_destination,
                        runtime->replace_expected_generation,
                        target, &conflict, &generation) :
                runtime->replace_plan_expected ?
                TrackReservationServerReplacePlanExpected(
                        dispatch_reservation_tid,
                        &safety_footprint, job->train,
                        runtime->replace_expected_destination,
                        runtime->replace_expected_generation,
                        target, &conflict, &generation) :
                rolling_authority ?
                TrackReservationServerReserveWindowWithGeneration(
                        dispatch_reservation_tid,
                        &safety_footprint, job->train,
                        target, &conflict, &generation) :
                TrackReservationServerReserveWithGeneration(
                        dispatch_reservation_tid,
                        &safety_footprint, job->train, target,
                        &conflict, &generation);
        if (reserve == 1) {
                job->conflict_train = conflict.owner_train;
                job->conflict_node = conflict.node_index;
                return 1;
        }
        if (reserve < 0 || generation == 0) {
                fail_job(slot, TC2_FAILURE_RESERVATION);
                return -1;
        }

        /* The replacement reservation is authoritative from this point on. */
        dispatch_retarget_candidate_pre_cas = 0;

        /*
         * The route-reservation CAS is now committed.  Cancel any old
         * in-flight switch batch before the candidate plan can issue a CAN
         * command, then apply only old-zone transitions whose entry/exit
         * geometry was proved before stage_job() cleared that route.
         */
        if (cancel_old_conflict_zone_batch(slot) < 0) {
                fail_job(slot, TC2_FAILURE_TURNOUT);
                return -1;
        }
        finalize_old_conflict_zone_carries(slot, now);
        (void)Tc2ConflictZoneCancelTrain(
                &dispatch_conflict_zones, job->train);
        runtime->route = route;
        runtime->safety_footprint = safety_footprint;
        runtime->route_valid = 1;
        runtime->conflict_zone_plan = candidate_zone_plan;
        runtime->conflict_zone_plan_valid = 1;
        runtime->conflict_zone_next_step = 0;
        runtime->conflict_zone_hard_step = -1;
        runtime->conflict_zone_soft_step = -1;
        runtime->conflict_zone_hard_ticket = 0;
        runtime->conflict_zone_batch_token = 0;
        reset_soft_conflict_zone_cursor(runtime, 1);
        runtime->conflict_zone_queue_pending = 0;
        runtime->conflict_zone_settle_at_tick = -1;
        runtime->conflict_zone_hold_active = 0;
        if (Tc2ConflictZoneRegisterTrain(
                    &dispatch_conflict_zones,
                    job->train) < 0) {
                fail_job(slot, TC2_FAILURE_RESERVATION);
                return -1;
        }
        /*
         * This is evaluated only after the replacement CAS committed and
         * the old logical CAN batches were cancelled.  Therefore an origin
         * continuation is backed solely by the live physical owner ticket
         * and its exact hard setting, never by a stale batch or soft hint.
         */
        if (arm_origin_zone_egress(slot) < 0) {
                fail_job(slot, TC2_FAILURE_RESERVATION);
                return -1;
        }
        if (refresh_conflict_zone_requests(slot) < 0) {
                fail_job(slot, TC2_FAILURE_RESERVATION);
                return -1;
        }
        runtime->rolling_authority_active =
                rolling_authority;
        runtime->authority_end_offset =
                rolling_authority ?
                authority_end_offset :
                route.node_count - 1;
        runtime->authority_center_ceiling_mm =
                rolling_authority ?
                authority_center_ceiling_mm :
                path_distance;
        clear_authority_request(runtime);
        runtime->authority_resume_pending = 0;
        runtime->authority_previous_end_offset = -1;
        if (leading_reversal) {
                runtime->reversal_carry_footprint =
                        leading_reversal_carry;
                runtime->reversal_carry_active = 1;
                runtime->reversal_carry_origin_offset = 1;
        }
        runtime->localization_only =
                localization_only;
        runtime->leg_start_offset = 0;
        if (runtime->planned_launch_speed <= 0 ||
            (rolling_authority &&
             authority_launch_speed <
                     runtime->planned_launch_speed)) {
                runtime->planned_launch_speed =
                        authority_launch_speed;
        }
        runtime->command_speed =
                runtime->planned_launch_speed;
        runtime->motion_speed_ceiling =
                runtime->planned_launch_speed;
        runtime->speed_reduction_pending = 0;
        runtime->precision_approach_active = 0;
        runtime->approach_sent = 0;
        runtime->target_seen = 0;
        runtime->reservation_anchor_offset = 0;
        runtime->pending_missing_offset = -1;
        job->target_node = target;
        job->target_route_offset = target_offset;
        job->destination_base_offset_mm =
                selected_side_definition ?
                selected_side_definition->base_offset_mm : 0;
        if (job->provisional_prediction &&
            job->destination_index == 6 && side == 0 &&
            selected_side_definition &&
            selected_side_definition->previous_anchor_sensor &&
            target == TrackFindNodeByName(
                    dispatch_track,
                    selected_side_definition
                            ->previous_anchor_sensor)) {
                job->destination_base_offset_mm +=
                        selected_side_definition
                                ->previous_anchor_distance_mm;
        }
        job->destination_speed_correction_mm =
                destination_offset_value -
                job->destination_base_offset_mm;
        job->destination_offset_mm =
                destination_offset_value;
        job->destination_distance_mm = path_distance;
        job->destination_route_offset =
                destination_route_offset;
        job->destination_offset_confirmed =
                destination_confirmed;
        job->selected_destination_side = side;
        job->route_distance_mm = path_distance;
        job->remaining_distance_mm = path_distance;
        job->route_reversal_count = route.reversal_count;
        job->plan_generation = generation;
        publish_prediction_snapshot(job, runtime);
        if (projection_ready) {
                if (publish_projection(
                            slot,
                            &dispatch_projection_scratch) < 0) {
                        fail_job(slot, TC2_FAILURE_INTERNAL);
                        return -1;
                }
        } else {
                invalidate_projection(slot);
        }
        job->conflict_train = 0;
        job->conflict_node = -1;
        job->hold_active = 0;
        job->position_estimated =
                runtime->carry_position_estimated ? 1 : 0;
        if (init_leg_monitor(
                    slot, sensor.attributed_count) < 0) {
                fail_job(slot, TC2_FAILURE_TURNOUT);
                return -1;
        }
        if (sensor_health_is_safe(
                    slot, &sensor, 1, now) < 0) {
                fail_job(slot, TC2_FAILURE_SENSOR_SERVICE);
                return -1;
        }
        if (can_health_is_safe(slot, 0, now) < 0) {
                fail_job(slot, TC2_FAILURE_CAN_HEALTH);
                return -1;
        }
        runtime->prepare_attributed_sequence =
                sensor.attributed_count;
        runtime->prepare_attribution_unavailable_count =
                sensor.attribution_unavailable_count;
        runtime->prepare_unattributed_count =
                sensor.unattributed_count;
        for (int sensor_index = 0;
             sensor_index < TRAIN_SENSOR_COUNT; ++sensor_index) {
                runtime->prepare_sensor_state[sensor_index] =
                        sensor.sensor_state[sensor_index];
        }
        /*
         * A generalized shortest path may begin with a zero-distance sensor
         * reversal. Perform that direction change while the train is still
         * stopped, then initialize the first actual motion leg before it can
         * join a launch wave.
         */
        if (runtime->leg_end_offset ==
                    runtime->leg_start_offset &&
            runtime->leg_end_offset <
                    job->destination_route_offset) {
                int next = runtime->leg_start_offset + 1;
                if (next >= runtime->route.node_count ||
                    dispatch_track[
                            runtime->route.nodes[
                                    runtime->leg_start_offset]]
                                    .reverse !=
                            &dispatch_track[
                                    runtime->route.nodes[next]]) {
                        fail_job(slot, TC2_FAILURE_ROUTE);
                        return -1;
                }
                /*
                 * The bounded outbound window is already reserved.  Queue
                 * and settle its turnouts first; process_preparing_jobs()
                 * sends the reverse command only after that barrier.
                 */
                runtime->prelaunch_reverse_pending = 1;
                runtime->prelaunch_turnout_settle_at_tick = -1;
                runtime->prelaunch_reverse_ready_at_tick = -1;
                runtime->leg_start_offset = next;
                ++job->reversals_completed;
                ++job->current_leg;
                if (init_leg_monitor(
                            slot, sensor.attributed_count) < 0) {
                        fail_job(slot, TC2_FAILURE_ROUTE);
                        return -1;
                }
                if (runtime->force_reverse_first &&
                    runtime->carry_position_estimated &&
                    runtime->carry_preorigin_distance_mm > 0) {
                        runtime->preorigin_distance_mm =
                                runtime->carry_preorigin_distance_mm;
                        runtime->preorigin_sensor_pending = 1;
                        job->confirmed_distance_mm =
                                -runtime->preorigin_distance_mm;
                        runtime->motion_anchor_distance_um =
                                -(int64_t)runtime
                                        ->preorigin_distance_mm * 1000;
                        /*
                         * The reverse command changes direction, not physical
                         * position.  Keep the published anchor on the old
                         * directed occurrence (route[0]) until the reverse
                         * origin detector is actually observed.  Publishing
                         * route[1] here made the UI claim A16 before the train
                         * had traversed the virtual d1-to-A16 prefix.
                         */
                        job->current_route_offset = 0;
                        job->current_node = runtime->route.nodes[0];
                        job->estimated_distance_um = 0;
                        update_next_sensor(slot);
                }
        }
        if (runtime->leg_start_offset ==
            job->destination_route_offset) {
                job->current_node = target;
                job->current_route_offset = target_offset;
                /*
                 * A zero-motion result still owns a newly selected
                 * final-leg/braking-extension turnout plan. Queue that plan
                 * even when no direction change was needed; otherwise a
                 * later CURRENT stage could reconstruct it from this route
                 * and mistake an unsent setting for a CS3-confirmed one.
                 * Keep the full replacement footprint until the batch
                 * completes and, when applicable, the leading reverse has
                 * mechanically settled.
                 */
                int turnout_token =
                        queue_turnout_plan_for_leg(slot);
                if (turnout_token < 0 &&
                    turnout_token != CAN_SEND_BUSY) {
                        fail_job(
                                slot,
                                TC2_FAILURE_TURNOUT);
                        return -1;
                }
                runtime->zero_motion_arrival_pending = 1;
                runtime->zero_motion_arrival_ready_at_tick =
                        runtime
                                        ->prelaunch_reverse_ready_at_tick >=
                                        0 ?
                                runtime
                                        ->prelaunch_reverse_ready_at_tick :
                                now;
                runtime->turnout_queue_pending =
                        turnout_token == CAN_SEND_BUSY;
                runtime->turnout_batch_token =
                        turnout_token > 0 ?
                                (unsigned int)turnout_token : 0;
                if (runtime->prelaunch_reverse_pending &&
                    turnout_token == 0) {
                        runtime->prelaunch_turnout_settle_at_tick =
                                now;
                }
                job->ready_at_tick = -1;
                job->state = TC2_JOB_PREPARING;
                return 0;
        }

        int turnout_token = queue_turnout_plan_for_leg(slot);
        if (turnout_token < 0 &&
            turnout_token != CAN_SEND_BUSY) {
                fail_job(slot, TC2_FAILURE_TURNOUT);
                return -1;
        }
        runtime->turnout_queue_pending =
                turnout_token == CAN_SEND_BUSY;
        runtime->turnout_batch_token =
                turnout_token > 0 ?
                        (unsigned int)turnout_token : 0;
        if (runtime->prelaunch_reverse_pending &&
            turnout_token == 0) {
                runtime->prelaunch_turnout_settle_at_tick =
                        now;
        }
        runtime->ready_wave_tick = -1;
        job->ready_at_tick = -1;
        job->state = TC2_JOB_PREPARING;
        return 0;
}

/*
 * A speed command is idempotent.  The CAN server can report one transient
 * priority-command timeout while a confirmed turnout batch is retiring.
 * Treating that single timeout as a permanent trip failure strands a healthy
 * train between turnouts (D9/E11 is the reproducible case).  Retry the exact
 * same command synchronously; the caller has already proved movement
 * authority before any positive command reaches this helper.
 */
static int command_train_speed_with_retry(int train, int speed) {
        for (int attempt = 0; attempt < 3; ++attempt) {
                if (CanTrainSetSpeedPriority(
                            dispatch_can_tid, train, speed) >= 0) {
                        return 0;
                }
                if (attempt + 1 < 3) Delay(1);
        }
        return -1;
}

static void begin_stop(int slot, int now, int purpose,
                       int estimated_position, int trigger) {
        tc2_dispatch_job_snapshot *job = &dispatch_jobs[slot];
        tc2_dispatch_runtime *runtime = &dispatch_runtime[slot];
        int stop_speed =
                runtime->motion_speed_ceiling >
                                runtime->command_speed ?
                        runtime->motion_speed_ceiling :
                        runtime->command_speed;
        if (stop_speed <= 0) stop_speed = job->speed;
        int settle_ticks =
                Tc2MotionStopSettleTicks(stop_speed);
        if (settle_ticks < 0) {
                fail_job(slot, TC2_FAILURE_INTERNAL);
                return;
        }
        int stop_request_tick = Time();
        if (stop_request_tick < 0) {
                fail_job(slot, TC2_FAILURE_CAN_SERVICE);
                return;
        }
        if (command_train_speed_with_retry(job->train, 0) < 0) {
                /*
                 * A confirmed priority command can lose only its CS3
                 * acknowledgement while a turnout command is retiring.
                 * Queue the same zero as an emergency batch, retain the
                 * route/turnout state, and wait for its exact batch
                 * acknowledgement before starting the coast timer.
                 */
                int train = job->train;
                int token = CanTrainEmergencyStopBatch(
                        dispatch_can_tid, &train, 1);
                if (token <= 0) {
                        fail_job(slot, TC2_FAILURE_CAN_SERVICE);
                        return;
                }
                runtime->emergency_stop_token =
                        (unsigned int)token;
        }
        int stop_confirmed_tick = Time();
        if (stop_confirmed_tick < 0) {
                fail_job(slot, TC2_FAILURE_CAN_SERVICE);
                return;
        }
        if (purpose == TC2_STOP_TRAFFIC) {
                int64_t command_progress_um;
                int coast_um =
                        Tc2MotionMeasuredStopDistanceUm(
                                stop_speed);
                if (coast_um < 0 ||
                    runtime_private_motion_progress_um(
                            slot, stop_confirmed_tick,
                            &command_progress_um) < 0 ||
                    command_progress_um >
                            INT64_MAX - coast_um) {
                        fail_job(slot, TC2_FAILURE_INTERNAL);
                        return;
                }
                int64_t hold_um =
                        command_progress_um + coast_um;
                if (runtime->preorigin_sensor_pending && hold_um > 0) {
                        hold_um = 0;
                }
                int64_t destination_um =
                        (int64_t)job
                                ->destination_distance_mm * 1000;
                if (hold_um > destination_um) {
                        hold_um = destination_um;
                }
                runtime->preorigin_traffic_hold_valid =
                        runtime->preorigin_sensor_pending;
                runtime->preorigin_traffic_hold_distance_um =
                        runtime->preorigin_sensor_pending ?
                        hold_um : 0;
                runtime->motion_anchor_distance_um =
                        command_progress_um;
                runtime->motion_anchor_tick =
                        stop_confirmed_tick;
                runtime->motion_anchor_from_traffic = 1;
                runtime->traffic_hold_distance_um =
                        runtime->preorigin_sensor_pending ?
                        0 : hold_um;
                job->traffic_hold_distance_um =
                        runtime->traffic_hold_distance_um;
                job->estimated_distance_um =
                        runtime->preorigin_sensor_pending ?
                        0 : command_progress_um;
        }
        runtime->stop_purpose = purpose;
        runtime->settle_at_tick =
                tick_after(stop_confirmed_tick, settle_ticks);
        if (runtime->command_speed > 0) {
                if (runtime->motion_speed_ceiling <
                    runtime->command_speed) {
                        runtime->motion_speed_ceiling =
                                runtime->command_speed;
                }
        }
        runtime->speed_reduction_pending = 0;
        runtime->precision_approach_active = 0;
        runtime->command_speed = 0;
        job->command_speed = 0;
        job->stop_sent_tick = stop_confirmed_tick;
        if (!job->stop_timing_valid) {
                job->stop_request_tick = stop_request_tick;
                job->stop_confirmed_tick = stop_confirmed_tick;
                job->stop_timing_valid = 1;
        }
        job->stop_trigger = trigger;
        job->position_estimated = estimated_position;
        job->braking_at_tick = -1;
        job->state = TC2_JOB_BRAKING;
        (void)now;
}

static int traffic_effective_speed(int slot) {
        if (slot < 0 || slot >= TC2_DISPATCH_MAX_JOBS) {
                return -1;
        }
        tc2_dispatch_job_snapshot *job =
                &dispatch_jobs[slot];
        tc2_dispatch_runtime *runtime =
                &dispatch_runtime[slot];
        if (job->state == TC2_JOB_TRAFFIC_HOLD ||
            job->state == TC2_JOB_REVERSING) {
                return 0;
        }
        if (runtime->authority_resume_pending &&
            (job->state == TC2_JOB_PREPARING ||
             job->state == TC2_JOB_READY)) {
                return 0;
        }
        int speed = runtime->command_speed;
        if (runtime->motion_speed_ceiling > speed) {
                speed = runtime->motion_speed_ceiling;
        }
        if (speed <= 0 &&
            (job->state == TC2_JOB_RUNNING ||
             job->state == TC2_JOB_BRAKING)) {
                speed = job->speed;
        }
        return speed >= 0 && speed <= 120 ? speed : -1;
}

static int traffic_braking_envelope_mm(int slot) {
        int speed = traffic_effective_speed(slot);
        if (speed < 0) return -1;
        return speed == 0 ?
                0 : Tc2MotionBrakingDistanceMm(speed);
}

/*
 * The position relation already subtracts a conservative localization
 * envelope from the physical gap.  The collision threshold therefore adds
 * the measured command-to-rest distance, not a second copy of that
 * uncertainty.  Rolling-authority boundaries continue to use the larger
 * braking envelope above.
 */
static int traffic_collision_stop_distance_mm(int slot) {
        int speed = traffic_effective_speed(slot);
        if (speed < 0) return -1;
        return speed == 0 ?
                0 : Tc2MotionStopDistanceMm(speed);
}

static int traffic_resume_fits_current_authority(
        int slot, int speed) {
        if (slot < 0 || slot >= TC2_DISPATCH_MAX_JOBS ||
            speed < 1 || speed > 120) {
                return 0;
        }
        tc2_dispatch_runtime *runtime =
                &dispatch_runtime[slot];
        tc2_dispatch_job_snapshot *job =
                &dispatch_jobs[slot];
        int measured_stop =
                Tc2MotionStopDistanceMm(speed);
        if (measured_stop < 0 ||
            runtime->traffic_hold_distance_um < 0 ||
            job->destination_distance_mm < 0 ||
            runtime->traffic_hold_distance_um >
                    (int64_t)job
                            ->destination_distance_mm * 1000 ||
            runtime->traffic_hold_distance_um >
                    INT64_MAX -
                            (int64_t)measured_stop * 1000 ||
            runtime->traffic_hold_distance_um +
                            (int64_t)measured_stop * 1000 >
                    (int64_t)job
                            ->destination_distance_mm * 1000) {
                return 0;
        }
        if (!runtime->rolling_authority_active) return 1;
        if (!runtime->route_valid ||
            runtime->route.node_count < 2 ||
            runtime->authority_end_offset < 0 ||
            runtime->authority_end_offset >=
                    runtime->route.node_count) {
                return 0;
        }
        if (runtime->authority_end_offset ==
            runtime->route.node_count - 1) {
                return 1;
        }
        int braking =
                Tc2MotionBrakingDistanceMm(speed);
        if (braking < 0 ||
            runtime->authority_center_ceiling_mm < 0 ||
            runtime->traffic_hold_distance_um >
                    INT64_MAX - (int64_t)braking * 1000) {
                return 0;
        }
        return runtime->traffic_hold_distance_um +
                       (int64_t)braking * 1000 <
                (int64_t)runtime
                        ->authority_center_ceiling_mm * 1000;
}

/*
 * Resume at the requested 400 mm hysteresis edge, but lower the rear train's
 * command when necessary so its measured coast plus 200 mm still fits the
 * conservative inter-train gap.  The separate authority check prevents this
 * low-speed restart from entering an unowned turnout or block.
 */
static int following_gap_safe_resume_speed(
        int maximum_speed, int gap_mm) {
        if (maximum_speed < 1 || maximum_speed > 120 ||
            gap_mm < 0) {
                return -1;
        }
        for (int speed = maximum_speed;
             speed >= 1; --speed) {
                int stop = Tc2MotionStopDistanceMm(speed);
                if (stop < 0) return -1;
                if (stop >
                            0x7fffffff -
                                    TC2_TRAFFIC_FOLLOWING_STOP_CLEARANCE_MM ||
                     stop +
                                     TC2_TRAFFIC_FOLLOWING_STOP_CLEARANCE_MM >
                             gap_mm) {
                        continue;
                }
                return speed;
        }
        return 0;
}

static int following_safe_resume_speed(
        int slot, int maximum_speed, int gap_mm) {
        if (maximum_speed < 1 || maximum_speed > 120 ||
            gap_mm < -1) {
                return -1;
        }
        int gap_limited_speed = maximum_speed;
        if (gap_mm >= 0) {
                gap_limited_speed =
                        following_gap_safe_resume_speed(
                                maximum_speed, gap_mm);
                if (gap_limited_speed < 1) {
                        return gap_limited_speed;
                }
        }
        for (int speed = gap_limited_speed;
             speed >= 1; --speed) {
                if (traffic_resume_fits_current_authority(
                            slot, speed)) {
                        return speed;
                }
        }
        return 0;
}

static int saturated_threshold_add(int left, int right) {
        if (left < 0 || right < 0) return -1;
        return left > 0x7fffffff - right ?
                0x7fffffff : left + right;
}

static void begin_traffic_stop(
        int slot, int now, int reason, int peer_train,
        int gap_mm, int stop_threshold_mm,
        int resume_threshold_mm) {
        if (slot < 0 || slot >= TC2_DISPATCH_MAX_JOBS) return;
        tc2_dispatch_job_snapshot *job =
                &dispatch_jobs[slot];
        tc2_dispatch_runtime *runtime =
                &dispatch_runtime[slot];
        if (job->state == TC2_JOB_TRAFFIC_HOLD ||
            (job->state == TC2_JOB_BRAKING &&
             runtime->stop_purpose == TC2_STOP_TRAFFIC)) {
                job->traffic_gap_mm = gap_mm;
                return;
        }
        /*
         * A rolling-authority continuation has not received a confirmed
         * positive-speed command yet.  Keep its frozen position and newly
         * acquired reservation, but prevent READY from entering a launch
         * wave.  A LAUNCHING race is handled by process_launching_jobs(),
         * which cancels and fail-stops the complete atomic wave.
         */
        if (runtime->authority_resume_pending &&
            (job->state == TC2_JOB_PREPARING ||
             job->state == TC2_JOB_READY ||
             job->state == TC2_JOB_LAUNCHING)) {
                runtime->prelaunch_traffic_blocked = 1;
                job->traffic_gap_mm = gap_mm;
                job->traffic_stop_threshold_mm =
                        stop_threshold_mm;
                job->traffic_resume_threshold_mm =
                        resume_threshold_mm;
                return;
        }
        /*
         * A destination/reversal stop already has an acknowledged zero
         * command and keeps its reservation.  Never replace that terminal
         * purpose with a resumable traffic hold.
         */
        if (job->state != TC2_JOB_RUNNING) return;
        int resume_speed = runtime->command_speed > 0 ?
                runtime->command_speed : job->speed;
        runtime->traffic_resume_speed = resume_speed;
        runtime->traffic_peer_train = peer_train;
        runtime->traffic_reason = reason;
        runtime->traffic_hold_since_tick = -1;
        job->traffic_hold_active = 1;
        job->traffic_reason = reason;
        job->traffic_peer_train = peer_train;
        job->traffic_gap_mm = gap_mm;
        job->traffic_stop_threshold_mm =
                stop_threshold_mm;
        job->traffic_resume_threshold_mm =
                resume_threshold_mm;
        begin_stop(
                slot, now, TC2_STOP_TRAFFIC, 1,
                TC2_STOP_TRIGGER_TIMER);
}

static int resume_traffic_hold(
        int slot, int now, int expected_reason) {
        if (slot < 0 || slot >= TC2_DISPATCH_MAX_JOBS) {
                return -1;
        }
        tc2_dispatch_job_snapshot *job =
                &dispatch_jobs[slot];
        tc2_dispatch_runtime *runtime =
                &dispatch_runtime[slot];
        if (job->state != TC2_JOB_TRAFFIC_HOLD ||
            runtime->stop_purpose != TC2_STOP_TRAFFIC ||
            (expected_reason != TC2_TRAFFIC_FOLLOWING &&
             expected_reason != TC2_TRAFFIC_HEAD_ON &&
             expected_reason != TC2_TRAFFIC_AUTHORITY) ||
            runtime->traffic_reason != expected_reason ||
            runtime->traffic_resume_speed < 1 ||
            runtime->traffic_resume_speed > 120 ||
            runtime->traffic_hold_distance_um < 0) {
                return -1;
        }
        track_reservation_snapshot reservation;
        if (TrackReservationServerSnapshot(
                    dispatch_reservation_tid,
                    &reservation) < 0 ||
            reservation.generation_by_train[job->train] !=
                    job->plan_generation ||
            reservation.destination_by_train[job->train] !=
                    job->target_node ||
            !traffic_resume_fits_current_authority(
                    slot,
                    runtime->traffic_resume_speed) ||
            !conflict_zone_step_ready(
                    slot, runtime->conflict_zone_next_step) ||
            can_health_is_safe(slot, 0, now) < 0 ||
            command_train_speed_with_retry(
                    job->train,
                    runtime->traffic_resume_speed) < 0) {
                return -1;
        }
        int confirmed = Time();
        if (confirmed < 0) return -1;

        runtime->command_speed =
                runtime->traffic_resume_speed;
        runtime->motion_speed_ceiling =
                runtime->traffic_resume_speed;
        runtime->motion_anchor_distance_um =
                runtime->preorigin_sensor_pending &&
                        runtime->preorigin_traffic_hold_valid ?
                runtime->preorigin_traffic_hold_distance_um :
                runtime->traffic_hold_distance_um;
        runtime->motion_anchor_tick = confirmed;
        runtime->motion_anchor_from_traffic = 1;
        runtime->preorigin_traffic_hold_distance_um = 0;
        runtime->preorigin_traffic_hold_valid = 0;
        runtime->speed_reduction_pending = 0;
        runtime->precision_approach_active = 0;
        runtime->approach_sent = 0;
        runtime->stop_purpose = TC2_STOP_NONE;
        runtime->settle_at_tick = -1;
        runtime->traffic_hold_since_tick = -1;
        runtime->traffic_peer_train = 0;
        runtime->traffic_reason = TC2_TRAFFIC_NONE;
        runtime->traffic_resume_speed = 0;
        job->command_speed = runtime->command_speed;
        job->estimated_distance_um =
                runtime->preorigin_sensor_pending ?
                0 : runtime->motion_anchor_distance_um;
        job->traffic_hold_active = 0;
        job->traffic_reason = TC2_TRAFFIC_NONE;
        job->traffic_peer_train = 0;
        job->traffic_gap_mm = -1;
        job->traffic_stop_threshold_mm = -1;
        job->traffic_resume_threshold_mm = -1;
        job->traffic_hold_distance_um = -1;
        job->hold_active = 0;
        job->state = TC2_JOB_RUNNING;
        if (schedule_motion_deadlines(slot, confirmed) < 0) {
                fail_job(slot, TC2_FAILURE_INTERNAL);
                return -1;
        }
        return 0;
}

/*
 * A dynamic collision relation can disappear before the current physical
 * turnout has finished acquiring its exact hard ticket and settling its
 * freshly re-sent setting.  That is a normal authority wait, not a CAN
 * service failure.  Preserve the frozen position and requested resume speed,
 * discard the stale peer latch, and let process_conflict_zones() restart the
 * train only after conflict_zone_step_ready() proves the exact current hard
 * step SETTLED/OCCUPIED.  This transition is deliberately generic: it is
 * shared by HEAD_ON recovery/removal and following-hysteresis release.
 */
static int defer_traffic_resume_for_conflict_zone(int slot) {
        if (slot < 0 || slot >= TC2_DISPATCH_MAX_JOBS) return -1;
        tc2_dispatch_job_snapshot *job = &dispatch_jobs[slot];
        tc2_dispatch_runtime *runtime = &dispatch_runtime[slot];
        if (job->state != TC2_JOB_TRAFFIC_HOLD ||
            runtime->stop_purpose != TC2_STOP_TRAFFIC ||
            runtime->traffic_resume_speed < 1 ||
            runtime->traffic_resume_speed > 120 ||
            runtime->traffic_hold_distance_um < 0 ||
            conflict_zone_step_ready(
                    slot, runtime->conflict_zone_next_step)) {
                return 0;
        }
        runtime->traffic_reason = TC2_TRAFFIC_AUTHORITY;
        runtime->traffic_peer_train = 0;
        runtime->conflict_zone_hold_active = 1;
        job->traffic_reason = TC2_TRAFFIC_AUTHORITY;
        job->traffic_peer_train = 0;
        job->traffic_gap_mm = -1;
        job->traffic_stop_threshold_mm = -1;
        job->traffic_resume_threshold_mm = -1;
        return 1;
}

/*
 * Stop with the train front 20 mm before an ungranted or unsettled turnout
 * zone.  The command is issued one braking envelope earlier; after
 * ownership and settle confirmation, the ordinary resumable traffic-hold
 * path restarts it without losing destination or speed state.
 */
static int enforce_conflict_zone_gate(int slot, int now) {
        tc2_dispatch_runtime *runtime = &dispatch_runtime[slot];
        int step_index = runtime->conflict_zone_next_step;
        if (!runtime->conflict_zone_plan_valid || step_index < 0 ||
            step_index >= runtime->conflict_zone_plan.step_count ||
            conflict_zone_step_ready(slot, step_index)) {
                return 0;
        }
        const tc2_conflict_zone_route_step *step =
                &runtime->conflict_zone_plan.steps[step_index];
        int braking = traffic_braking_envelope_mm(slot);
        int64_t progress_um;
        int stop_point_mm = step->entry_distance_mm -
                TC2_TRAIN_HALF_LENGTH_MM - 20;
        if (stop_point_mm < 0) stop_point_mm = 0;
        if (braking < 0 ||
            runtime_motion_progress_um(
                    slot, now, &progress_um) < 0) {
                return -1;
        }
        if (progress_um + (int64_t)braking * 1000 <
            (int64_t)stop_point_mm * 1000) {
                return 0;
        }
        runtime->conflict_zone_hold_active = 1;
        begin_traffic_stop(
                slot, now, TC2_TRAFFIC_AUTHORITY, 0,
                stop_point_mm > (int)(progress_um / 1000) ?
                        stop_point_mm -
                                (int)(progress_um / 1000) : 0,
                braking, -1);
        return 1;
}

typedef struct {
        int valid;
        int relation;
        int rear_slot;
        int order_ambiguous;
        int gap_mm;
        int scalar_a_mm;
        int scalar_b_mm;
} tc2_live_traffic_relation;

static int traffic_slot_has_position(int slot) {
        if (slot < 0 || slot >= TC2_DISPATCH_MAX_JOBS) {
                return 0;
        }
        int state = dispatch_jobs[slot].state;
        if (state == TC2_JOB_RUNNING ||
            state == TC2_JOB_BRAKING ||
            state == TC2_JOB_TRAFFIC_HOLD) {
                return 1;
        }
        if (state == TC2_JOB_CANCEL_BRAKING ||
            state == TC2_JOB_REVERSING) {
                return dispatch_runtime[slot].route_valid;
        }
        int prelaunch =
                state == TC2_JOB_PREPARING ||
                state == TC2_JOB_READY ||
                state == TC2_JOB_LAUNCHING;
        if (!prelaunch) return 0;
        if (dispatch_runtime[slot]
                            .authority_resume_pending &&
            dispatch_runtime[slot]
                            .traffic_hold_distance_um >= 0) {
                return 1;
        }
        /*
         * A committed CURRENT replacement is still a physical stopped train
         * while its new turnouts settle (and may begin moving while its launch
         * batch is being confirmed).  Keep it in pairwise collision checks.
         */
        return dispatch_jobs[slot].start_index ==
                               TC2_DISPATCH_START_CURRENT &&
                dispatch_runtime[slot].route_valid &&
                dispatch_jobs[slot].current_node >= 0;
}

static int route_prefix_distances(
        const track_route *route, int distances[TRACK_MAX]) {
        if (!route || !distances || route->node_count < 1 ||
            route->node_count > TRACK_MAX) {
                return -1;
        }
        distances[0] = 0;
        for (int offset = 1;
             offset < route->node_count; ++offset) {
                if (TrackRouteDistanceBetweenOffsets(
                            dispatch_track, route,
                            offset - 1, offset,
                            &distances[offset]) < 0 ||
                    distances[offset - 1] >
                            0x7fffffff -
                            distances[offset]) {
                        return -1;
                }
                distances[offset] +=
                        distances[offset - 1];
        }
        return 0;
}

typedef struct {
        int point_mm;
        int minimum_mm;
        int maximum_mm;
} tc2_traffic_position_envelope;

/*
 * The latest ordered sensor is a hard lower bound: a forward-moving train
 * cannot be assumed to have advanced merely because its commanded-speed
 * model says so.  Conversely, collision braking must assume that a rear
 * train is as far forward as the point prediction plus the calibrated
 * uncertainty envelope.
 */
static int runtime_traffic_position_envelope(
        int slot, int now,
        tc2_traffic_position_envelope *envelope) {
        if (slot < 0 || slot >= TC2_DISPATCH_MAX_JOBS ||
            !envelope) {
                return -1;
        }
        tc2_dispatch_job_snapshot *job =
                &dispatch_jobs[slot];
        tc2_dispatch_runtime *runtime =
                &dispatch_runtime[slot];
        int64_t point_um;
        int authority_prelaunch =
                runtime->authority_resume_pending &&
                (job->state == TC2_JOB_PREPARING ||
                 job->state == TC2_JOB_READY ||
                 job->state == TC2_JOB_LAUNCHING);
        int current_prelaunch =
                !authority_prelaunch &&
                job->start_index ==
                        TC2_DISPATCH_START_CURRENT &&
                (job->state == TC2_JOB_PREPARING ||
                 job->state == TC2_JOB_READY ||
                 job->state == TC2_JOB_LAUNCHING);
        int reversal_wait =
                job->state == TC2_JOB_REVERSING;
        if (authority_prelaunch) {
                point_um = runtime->traffic_hold_distance_um;
                if (point_um < 0) return -1;
        } else if (current_prelaunch) {
                if (runtime->monitor.confirmed_distance_mm < 0) {
                        return -1;
                }
                point_um =
                        (int64_t)runtime->monitor
                                .confirmed_distance_mm * 1000;
        } else if (reversal_wait) {
                point_um =
                        runtime->motion_anchor_distance_um;
                if (point_um < 0) return -1;
        } else {
                if (runtime_motion_progress_um(
                            slot, now, &point_um) < 0 ||
                    point_um < 0) {
                        return -1;
                }
        }
        int speed = runtime->command_speed;
        if (runtime->motion_speed_ceiling > speed) {
                speed = runtime->motion_speed_ceiling;
        }
        if (runtime->stop_purpose == TC2_STOP_TRAFFIC &&
            runtime->traffic_resume_speed > speed) {
                speed = runtime->traffic_resume_speed;
        }
        if (speed <= 0) speed = job->speed;
        int uncertainty = Tc2MotionUncertaintyMm(speed);
        if (uncertainty < 0 ||
            point_um >
                    INT64_MAX -
                    (int64_t)uncertainty * 1000) {
                return -1;
        }
        int64_t maximum_motion_um = point_um;
        if ((authority_prelaunch || current_prelaunch) &&
            job->state == TC2_JOB_LAUNCHING) {
                int launch_speed =
                        runtime->planned_launch_speed > 0 ?
                        runtime->planned_launch_speed :
                        runtime->traffic_resume_speed;
                int velocity =
                        Tc2MotionProvisionalVelocityUmPerTick(
                                launch_speed);
                int age = motion_tick_age(
                        now, runtime->launch_wave_tick);
                if (velocity < 1 || age < 0 ||
                    maximum_motion_um >
                            INT64_MAX -
                            (int64_t)age * velocity) {
                        return -1;
                }
                maximum_motion_um +=
                        (int64_t)age * velocity;
                int64_t destination_um =
                        (int64_t)job
                                ->destination_distance_mm * 1000;
                if (destination_um >= 0 &&
                    maximum_motion_um > destination_um) {
                        maximum_motion_um = destination_um;
                }
        }
        if (maximum_motion_um >
                    INT64_MAX -
                    (int64_t)uncertainty * 1000) {
                return -1;
        }
        int64_t maximum_um =
                maximum_motion_um +
                (int64_t)uncertainty * 1000;
        int64_t maximum_mm =
                (maximum_um + 999) / 1000;
        int minimum_mm =
                runtime->monitor.confirmed_distance_mm;
        if (minimum_mm < 0 ||
            minimum_mm > point_um / 1000 ||
            point_um / 1000 > 0x7fffffff ||
            maximum_mm > 0x7fffffff) {
                return -1;
        }
        envelope->point_mm =
                (int)(point_um / 1000);
        envelope->minimum_mm = minimum_mm;
        envelope->maximum_mm = (int)maximum_mm;
        return 0;
}

static int traffic_occupancy_overlaps_interval(
        int minimum_mm, int maximum_mm,
        int first_mm, int last_mm) {
        if (minimum_mm < 0 || maximum_mm < minimum_mm ||
            first_mm < 0 || last_mm < first_mm) {
                return 0;
        }
        int half_body = (TC2_TRAIN_BODY_MM + 1) / 2;
        int64_t occupied_first =
                (int64_t)minimum_mm - half_body;
        int64_t occupied_last =
                (int64_t)maximum_mm + half_body;
        return occupied_last >= first_mm &&
                occupied_first <= last_mm;
}

static int traffic_interval_separation_mm(
        int64_t first_minimum, int64_t first_maximum,
        int64_t second_minimum, int64_t second_maximum) {
        if (first_minimum > first_maximum ||
            second_minimum > second_maximum) {
                return -1;
        }
        int64_t separation = 0;
        if (first_maximum < second_minimum) {
                separation =
                        second_minimum - first_maximum;
        } else if (second_maximum < first_minimum) {
                separation =
                        first_minimum - second_maximum;
        }
        return separation > 0x7fffffff ?
                0x7fffffff : (int)separation;
}

static int clearance_gap_mm(int64_t center_gap_mm) {
        if (center_gap_mm <= TC2_TRAIN_BODY_MM) return 0;
        center_gap_mm -= TC2_TRAIN_BODY_MM;
        return center_gap_mm > 0x7fffffff ?
                0x7fffffff : (int)center_gap_mm;
}

/*
 * Identify a physical corridor occupied by both trains.  Same-direction
 * corridors use identical directed graph nodes.  An opposite edge u->v is
 * canonically reverse(v)->reverse(u), so head-on matching walks the second
 * route in the opposite offset direction.  Merely sharing a future turnout
 * is intentionally not classified as following: the exclusive reservation
 * layer arbitrates that intersection before either turnout command is sent.
 */
static int live_traffic_relation_for_pair(
        int slot_a, int slot_b, int now,
        tc2_live_traffic_relation *relation) {
        if (!relation || slot_a < 0 ||
            slot_a >= TC2_DISPATCH_MAX_JOBS ||
            slot_b < 0 ||
            slot_b >= TC2_DISPATCH_MAX_JOBS ||
            slot_a == slot_b) {
                return -1;
        }
        relation->valid = 0;
        relation->relation =
                TC2_TRAFFIC_RELATION_HEAD_ON;
        relation->rear_slot = -1;
        relation->order_ambiguous = 0;
        relation->gap_mm = 0x7fffffff;
        relation->scalar_a_mm = -1;
        relation->scalar_b_mm = -1;

        tc2_dispatch_runtime *runtime_a =
                &dispatch_runtime[slot_a];
        tc2_dispatch_runtime *runtime_b =
                &dispatch_runtime[slot_b];
        if (!traffic_slot_has_position(slot_a) ||
            !traffic_slot_has_position(slot_b)) {
                return 0;
        }
        if (!runtime_a->route_valid ||
            !runtime_b->route_valid) {
                return -1;
        }
        tc2_traffic_position_envelope position_a;
        tc2_traffic_position_envelope position_b;
        if (runtime_traffic_position_envelope(
                    slot_a, now, &position_a) < 0 ||
            runtime_traffic_position_envelope(
                    slot_b, now, &position_b) < 0) {
                return -1;
        }
        int distance_a[TRACK_MAX];
        int distance_b[TRACK_MAX];
        if (route_prefix_distances(
                    &runtime_a->route, distance_a) < 0 ||
            route_prefix_distances(
                    &runtime_b->route, distance_b) < 0 ||
            runtime_a->leg_start_offset < 0 ||
            runtime_a->leg_end_offset <
                    runtime_a->leg_start_offset ||
            runtime_a->leg_end_offset >=
                    runtime_a->route.node_count ||
            runtime_b->leg_start_offset < 0 ||
            runtime_b->leg_end_offset <
                    runtime_b->leg_start_offset ||
            runtime_b->leg_end_offset >=
                    runtime_b->route.node_count) {
                return -1;
        }
        relation->scalar_a_mm = position_a.point_mm;
        relation->scalar_b_mm = position_b.point_mm;

        int best_same_gap = 0x7fffffff;
        int best_same_rear = -1;
        int best_same_ambiguous = 0;
        for (int first_a = runtime_a->leg_start_offset;
             first_a < runtime_a->leg_end_offset;
             ++first_a) {
                for (int first_b =
                             runtime_b->leg_start_offset;
                     first_b < runtime_b->leg_end_offset;
                     ++first_b) {
                        if (runtime_a->route
                                            .nodes[first_a] !=
                                    runtime_b->route
                                            .nodes[first_b] ||
                            runtime_a->route
                                            .nodes[first_a + 1] !=
                                    runtime_b->route
                                            .nodes[first_b + 1]) {
                                continue;
                        }
                        if (first_a >
                                    runtime_a
                                            ->leg_start_offset &&
                            first_b >
                                    runtime_b
                                            ->leg_start_offset &&
                            runtime_a->route
                                            .nodes[first_a - 1] ==
                                    runtime_b->route
                                            .nodes[first_b - 1]) {
                                continue;
                        }
                        int last_a = first_a + 1;
                        int last_b = first_b + 1;
                        while (last_a <
                                           runtime_a
                                                   ->leg_end_offset &&
                               last_b <
                                           runtime_b
                                                   ->leg_end_offset &&
                               runtime_a->route
                                               .nodes[last_a + 1] ==
                                       runtime_b->route
                                               .nodes[last_b + 1]) {
                                ++last_a;
                                ++last_b;
                        }
                        if (!traffic_occupancy_overlaps_interval(
                                    position_a.minimum_mm,
                                    position_a.maximum_mm,
                                    distance_a[first_a],
                                    distance_a[last_a]) ||
                            !traffic_occupancy_overlaps_interval(
                                    position_b.minimum_mm,
                                    position_b.maximum_mm,
                                    distance_b[first_b],
                                    distance_b[last_b])) {
                                continue;
                        }
                        int64_t mapped_b_minimum =
                                (int64_t)distance_a[first_a] +
                                position_b.minimum_mm -
                                distance_b[first_b];
                        int64_t mapped_b_maximum =
                                (int64_t)distance_a[first_a] +
                                position_b.maximum_mm -
                                distance_b[first_b];
                        int rear;
                        int ambiguous = 0;
                        int64_t center_gap;
                        if (position_a.maximum_mm <
                            mapped_b_minimum) {
                                rear = slot_a;
                                center_gap =
                                        mapped_b_minimum -
                                        position_a.maximum_mm;
                        } else if (mapped_b_maximum <
                                   position_a.minimum_mm) {
                                rear = slot_b;
                                center_gap =
                                        position_a.minimum_mm -
                                        mapped_b_maximum;
                        } else {
                                /*
                                 * The position envelopes overlap, so point
                                 * order cannot safely identify which train
                                 * is behind. Stop both at the dispatcher
                                 * layer rather than selecting the wrong rear.
                                 */
                                rear = -1;
                                center_gap = 0;
                                ambiguous = 1;
                        }
                        int gap = clearance_gap_mm(center_gap);
                        if (gap < best_same_gap) {
                                best_same_gap = gap;
                                best_same_rear = rear;
                                best_same_ambiguous =
                                        ambiguous;
                        }
                }
        }

        int best_head_gap = 0x7fffffff;
        for (int edge_a = runtime_a->leg_start_offset;
             edge_a < runtime_a->leg_end_offset;
             ++edge_a) {
                int node_a =
                        runtime_a->route.nodes[edge_a];
                int next_a =
                        runtime_a->route.nodes[edge_a + 1];
                int reverse_node_a =
                        physical_reverse_index(node_a);
                int reverse_next_a =
                        physical_reverse_index(next_a);
                if (reverse_node_a < 0 ||
                    reverse_next_a < 0) {
                        continue;
                }
                for (int edge_b =
                             runtime_b->leg_start_offset;
                     edge_b < runtime_b->leg_end_offset;
                     ++edge_b) {
                        if (runtime_b->route
                                            .nodes[edge_b] !=
                                    reverse_next_a ||
                            runtime_b->route
                                            .nodes[edge_b + 1] !=
                                    reverse_node_a) {
                                continue;
                        }
                        int first_a = edge_a;
                        int last_a = edge_a + 1;
                        int first_b = edge_b;
                        int last_b = edge_b + 1;
                        while (first_a >
                                           runtime_a
                                                   ->leg_start_offset &&
                               last_b <
                                           runtime_b
                                                   ->leg_end_offset &&
                               physical_reverse_index(
                                       runtime_a->route
                                               .nodes[first_a - 1]) ==
                                       runtime_b->route
                                               .nodes[last_b + 1]) {
                                --first_a;
                                ++last_b;
                        }
                        while (last_a <
                                           runtime_a
                                                   ->leg_end_offset &&
                               first_b >
                                           runtime_b
                                                   ->leg_start_offset &&
                               physical_reverse_index(
                                       runtime_a->route
                                               .nodes[last_a + 1]) ==
                                       runtime_b->route
                                               .nodes[first_b - 1]) {
                                ++last_a;
                                --first_b;
                        }
                        if (!traffic_occupancy_overlaps_interval(
                                    position_a.minimum_mm,
                                    position_a.maximum_mm,
                                    distance_a[first_a],
                                    distance_a[last_a]) ||
                            !traffic_occupancy_overlaps_interval(
                                    position_b.minimum_mm,
                                    position_b.maximum_mm,
                                    distance_b[first_b],
                                    distance_b[last_b])) {
                                continue;
                        }
                        int64_t mapped_b_minimum =
                                (int64_t)distance_a[last_a] -
                                (position_b.maximum_mm -
                                 distance_b[first_b]);
                        int64_t mapped_b_maximum =
                                (int64_t)distance_a[last_a] -
                                (position_b.minimum_mm -
                                 distance_b[first_b]);
                        int separation =
                                traffic_interval_separation_mm(
                                        position_a.minimum_mm,
                                        position_a.maximum_mm,
                                        mapped_b_minimum,
                                        mapped_b_maximum);
                        if (separation < 0) return -1;
                        int gap = clearance_gap_mm(separation);
                        if (gap < best_head_gap) {
                                best_head_gap = gap;
                        }
                }
        }

        /*
         * Opposite-direction occupancy always wins over a same-direction
         * alias because its safe action is the stricter stop-both result.
         */
        if (best_head_gap != 0x7fffffff) {
                relation->valid = 1;
                relation->relation =
                        TC2_TRAFFIC_RELATION_HEAD_ON;
                relation->rear_slot = -1;
                relation->gap_mm = best_head_gap;
        } else if (best_same_gap != 0x7fffffff) {
                relation->valid = 1;
                relation->order_ambiguous =
                        best_same_ambiguous;
                relation->relation =
                        best_same_ambiguous ?
                        TC2_TRAFFIC_RELATION_HEAD_ON :
                        best_same_rear == slot_a ?
                                TC2_TRAFFIC_RELATION_SAME_DIRECTION_A_REAR :
                                TC2_TRAFFIC_RELATION_SAME_DIRECTION_B_REAR;
                relation->rear_slot = best_same_rear;
                relation->gap_mm = best_same_gap;
        }
        return 0;
}

static int traffic_has_movement_authority(
        int slot,
        const track_reservation_snapshot *reservation) {
        if (slot < 0 || slot >= TC2_DISPATCH_MAX_JOBS ||
            !reservation) {
                return 0;
        }
        tc2_dispatch_job_snapshot *job =
                &dispatch_jobs[slot];
        return job->plan_generation != 0 &&
                reservation
                        ->generation_by_train[job->train] ==
                        job->plan_generation &&
                reservation
                        ->destination_by_train[job->train] ==
                        job->target_node;
}

static int traffic_resume_threshold_mm(
        int braking_envelope_mm) {
        return braking_envelope_mm < 0 ? -1 :
                TC2_TRAFFIC_FOLLOWING_RESUME_CLEARANCE_MM;
}

static void process_traffic_safety(int now) {
        int stop_reason[TC2_DISPATCH_MAX_JOBS];
        int stop_peer[TC2_DISPATCH_MAX_JOBS];
        int stop_gap[TC2_DISPATCH_MAX_JOBS];
        int stop_threshold[TC2_DISPATCH_MAX_JOBS];
        int resume_threshold[TC2_DISPATCH_MAX_JOBS];
        int resume_speed_candidate[TC2_DISPATCH_MAX_JOBS];
        int block_resume[TC2_DISPATCH_MAX_JOBS];
        int opposing_now[TC2_DISPATCH_MAX_JOBS];
        track_reservation_snapshot reservation;
        if (TrackReservationServerSnapshot(
                    dispatch_reservation_tid,
                    &reservation) < 0) {
                for (int slot = 0;
                     slot < TC2_DISPATCH_MAX_JOBS; ++slot) {
                        if (traffic_slot_has_position(slot)) {
                                fail_job(
                                        slot,
                                        TC2_FAILURE_RESERVATION_SERVICE);
                        }
                }
                return;
        }
        for (int slot = 0;
             slot < TC2_DISPATCH_MAX_JOBS; ++slot) {
                stop_reason[slot] = TC2_TRAFFIC_NONE;
                stop_peer[slot] = 0;
                stop_gap[slot] = -1;
                stop_threshold[slot] = -1;
                resume_threshold[slot] = -1;
                resume_speed_candidate[slot] = 0;
                dispatch_runtime[slot]
                        .prelaunch_traffic_blocked = 0;
                block_resume[slot] = 0;
                opposing_now[slot] = 0;
        }

        for (int slot_a = 0;
             slot_a < TC2_DISPATCH_MAX_JOBS; ++slot_a) {
                if (!traffic_slot_has_position(slot_a)) {
                        continue;
                }
                for (int slot_b = slot_a + 1;
                     slot_b < TC2_DISPATCH_MAX_JOBS; ++slot_b) {
                        if (!traffic_slot_has_position(slot_b)) {
                                continue;
                        }
                        tc2_live_traffic_relation relation;
                        int relation_status =
                                live_traffic_relation_for_pair(
                                        slot_a, slot_b, now,
                                        &relation);
                        if (relation_status < 0) {
                                /*
                                 * Position ambiguity on two potentially
                                 * active managed routes fails closed.
                                 */
                                stop_reason[slot_a] =
                                        TC2_TRAFFIC_HEAD_ON;
                                stop_reason[slot_b] =
                                        TC2_TRAFFIC_HEAD_ON;
                                stop_peer[slot_a] =
                                        dispatch_jobs[slot_b]
                                                .train;
                                stop_peer[slot_b] =
                                        dispatch_jobs[slot_a]
                                                .train;
                                block_resume[slot_a] = 1;
                                block_resume[slot_b] = 1;
                                opposing_now[slot_a] = 1;
                                opposing_now[slot_b] = 1;
                                continue;
                        }
                        if (!relation.valid) continue;
                        int speed_a =
                                traffic_effective_speed(
                                        slot_a);
                        int speed_b =
                                traffic_effective_speed(
                                        slot_b);
                        int envelope_a =
                                traffic_collision_stop_distance_mm(
                                        slot_a);
                        int envelope_b =
                                traffic_collision_stop_distance_mm(
                                        slot_b);
                        if (speed_a < 0 || speed_b < 0 ||
                            envelope_a < 0 ||
                            envelope_b < 0) {
                                stop_reason[slot_a] =
                                        TC2_TRAFFIC_HEAD_ON;
                                stop_reason[slot_b] =
                                        TC2_TRAFFIC_HEAD_ON;
                                block_resume[slot_a] = 1;
                                block_resume[slot_b] = 1;
                                opposing_now[slot_a] = 1;
                                opposing_now[slot_b] = 1;
                                continue;
                        }
                        tc2_traffic_safety_input input;
                        input.train_a_route_scalar_mm =
                                relation.scalar_a_mm;
                        input.train_b_route_scalar_mm =
                                relation.scalar_b_mm;
                        input.train_a_effective_speed =
                                speed_a;
                        input.train_b_effective_speed =
                                speed_b;
                        input.train_a_braking_envelope_mm =
                                envelope_a;
                        input.train_b_braking_envelope_mm =
                                envelope_b;
                        input.gap_mm = relation.gap_mm;
                        input.relation =
                                (tc2_traffic_relation)
                                        relation.relation;
                        input.rear_stop_latched = 0;
                        input.rear_has_movement_authority = 1;

                        if (relation.relation ==
                            TC2_TRAFFIC_RELATION_HEAD_ON) {
                                /*
                                 * Even outside the emergency stop boundary,
                                 * a latched train must not auto-resume into
                                 * an opposing occupied corridor.
                                 */
                                block_resume[slot_a] = 1;
                                block_resume[slot_b] = 1;
                                opposing_now[slot_a] = 1;
                                opposing_now[slot_b] = 1;
                                int threshold =
                                        saturated_threshold_add(
                                                envelope_a,
                                                envelope_b);
                                threshold =
                                        saturated_threshold_add(
                                                threshold,
                                                TC2_TRAFFIC_HEAD_ON_EXTRA_CLEARANCE_MM);
                                tc2_traffic_action action =
                                        Tc2TrafficSafetyEvaluate(
                                                &input);
                                if (action ==
                                    TC2_TRAFFIC_ACTION_STOP_BOTH) {
                                        stop_reason[slot_a] =
                                                TC2_TRAFFIC_HEAD_ON;
                                        stop_reason[slot_b] =
                                                TC2_TRAFFIC_HEAD_ON;
                                        stop_peer[slot_a] =
                                                dispatch_jobs[slot_b]
                                                        .train;
                                        stop_peer[slot_b] =
                                                dispatch_jobs[slot_a]
                                                        .train;
                                        stop_gap[slot_a] =
                                                relation.gap_mm;
                                        stop_gap[slot_b] =
                                                relation.gap_mm;
                                        stop_threshold[slot_a] =
                                                threshold;
                                        stop_threshold[slot_b] =
                                                threshold;
                                        block_resume[slot_a] = 1;
                                        block_resume[slot_b] = 1;
                                }
                                continue;
                        }

                        int rear = relation.rear_slot;
                        int front = rear == slot_a ?
                                slot_b : slot_a;
                        int rear_speed =
                                traffic_effective_speed(rear);
                        int rear_latched =
                                dispatch_jobs[rear].state ==
                                        TC2_JOB_TRAFFIC_HOLD ||
                                (dispatch_runtime[rear]
                                                .authority_resume_pending &&
                                 (dispatch_jobs[rear].state ==
                                          TC2_JOB_PREPARING ||
                                  dispatch_jobs[rear].state ==
                                          TC2_JOB_READY ||
                                  dispatch_jobs[rear].state ==
                                          TC2_JOB_LAUNCHING));
                        int rear_envelope =
                                rear_latched ?
                                Tc2MotionStopDistanceMm(
                                        dispatch_runtime[rear]
                                                .traffic_resume_speed) :
                                traffic_collision_stop_distance_mm(
                                        rear);
                        int rear_stop_threshold =
                                saturated_threshold_add(
                                        rear_envelope,
                                        TC2_TRAFFIC_FOLLOWING_STOP_CLEARANCE_MM);
                        int rear_resume_threshold =
                                traffic_resume_threshold_mm(
                                        rear_envelope);
                        if (rear_envelope < 0 ||
                            rear_stop_threshold < 0 ||
                            rear_resume_threshold < 0) {
                                stop_reason[rear] =
                                        TC2_TRAFFIC_HEAD_ON;
                                block_resume[rear] = 1;
                                opposing_now[rear] = 1;
                                continue;
                        }
                        if (rear == slot_a) {
                                input
                                        .train_a_braking_envelope_mm =
                                        rear_envelope;
                                input
                                        .train_a_effective_speed =
                                        rear_latched ? 0 :
                                        rear_speed;
                        } else {
                                input
                                        .train_b_braking_envelope_mm =
                                        rear_envelope;
                                input
                                        .train_b_effective_speed =
                                        rear_latched ? 0 :
                                        rear_speed;
                        }
                        input.rear_stop_latched =
                                rear_latched;
                        input.rear_has_movement_authority =
                                traffic_has_movement_authority(
                                        rear,
                                        &reservation);
                        tc2_traffic_action action =
                                Tc2TrafficSafetyEvaluate(
                                        &input);
                        dispatch_jobs[rear].traffic_gap_mm =
                                relation.gap_mm;
                        if (action ==
                            TC2_TRAFFIC_ACTION_STOP_REAR) {
                                if (stop_reason[rear] !=
                                    TC2_TRAFFIC_HEAD_ON) {
                                        stop_reason[rear] =
                                                TC2_TRAFFIC_FOLLOWING;
                                        stop_peer[rear] =
                                                dispatch_jobs[front]
                                                        .train;
                                        stop_gap[rear] =
                                                relation.gap_mm;
                                        stop_threshold[rear] =
                                                rear_stop_threshold;
                                        resume_threshold[rear] =
                                                rear_resume_threshold;
                                }
                                block_resume[rear] = 1;
                        } else if (action ==
                                   TC2_TRAFFIC_ACTION_KEEP_HOLD) {
                                block_resume[rear] = 1;
                        } else if (action ==
                                   TC2_TRAFFIC_ACTION_RESUME_REAR) {
                                int safe_speed =
                                        following_safe_resume_speed(
                                                rear,
                                                dispatch_runtime[rear]
                                                        .traffic_resume_speed,
                                                relation.gap_mm);
                                if (safe_speed < 1) {
                                        block_resume[rear] = 1;
                                } else if (
                                        resume_speed_candidate[rear] == 0 ||
                                        safe_speed <
                                                resume_speed_candidate[rear]) {
                                        resume_speed_candidate[rear] =
                                                safe_speed;
                                }
                        } else if (action !=
                                   TC2_TRAFFIC_ACTION_RESUME_REAR) {
                                block_resume[rear] = 1;
                        }
                }
        }

        /*
         * KEEP_HOLD and an occupied opposite-direction corridor deliberately
         * do not create a second stop command for a train that is already
         * stationary.  Carry that decision into the authority prelaunch
         * gate so READY cannot accidentally interpret "no new stop" as
         * permission to launch.
         */
        for (int slot = 0;
             slot < TC2_DISPATCH_MAX_JOBS; ++slot) {
                int authority_prelaunch =
                        dispatch_runtime[slot]
                                        .authority_resume_pending &&
                        (dispatch_jobs[slot].state ==
                                 TC2_JOB_PREPARING ||
                         dispatch_jobs[slot].state ==
                                 TC2_JOB_READY ||
                         dispatch_jobs[slot].state ==
                                 TC2_JOB_LAUNCHING);
                int reversal_prelaunch =
                        dispatch_jobs[slot].state ==
                        TC2_JOB_REVERSING;
                if (block_resume[slot] &&
                    (authority_prelaunch ||
                     reversal_prelaunch)) {
                        dispatch_runtime[slot]
                                .prelaunch_traffic_blocked = 1;
                }
        }

        /*
         * A historical HEAD_ON latch is not permanent.  It may clear only
         * after the opposing train has either been physically removed (its
         * job and every reservation owner are gone) or has executed a
         * recovery route and produced enough ordered motion evidence to
         * leave the old corridor.  Merely reaching STOPPED after `cancel`
         * is intentionally insufficient: the train is still on the track.
         */
        for (int slot = 0;
             slot < TC2_DISPATCH_MAX_JOBS; ++slot) {
                tc2_dispatch_job_snapshot *job =
                        &dispatch_jobs[slot];
                tc2_dispatch_runtime *runtime =
                        &dispatch_runtime[slot];
                if (job->state !=
                            TC2_JOB_TRAFFIC_HOLD ||
                    runtime->traffic_reason !=
                            TC2_TRAFFIC_HEAD_ON ||
                    runtime->stop_purpose !=
                            TC2_STOP_TRAFFIC ||
                    stop_reason[slot] !=
                            TC2_TRAFFIC_NONE ||
                    opposing_now[slot] ||
                    runtime->retarget_pending ||
                    !traffic_has_movement_authority(
                            slot, &reservation)) {
                        continue;
                }
                int peer_train =
                        runtime->traffic_peer_train;
                if (peer_train < 1 ||
                    peer_train > 255) {
                        continue;
                }
                int peer_slot =
                        find_train_job(peer_train);
                int peer_still_blocks = 0;
                if (peer_slot >= 0) {
                        /*
                         * The replacement plan carries every old-corridor
                         * owner until footprint_tail_offset() proves that the
                         * complete train plus 200 mm has cleared its new
                         * origin.  Only that generation-checked release can
                         * wake the stopped peer.  The recovery train may
                         * already be BRAKING or ARRIVED by then, so RUNNING
                         * is not a release condition.  Conversely a canceled
                         * train has no recovery witness and remains a real
                         * obstacle until `remove` deletes its job and owner
                         * set.
                         */
                        peer_still_blocks =
                                !dispatch_runtime[peer_slot]
                                         .head_on_recovery_active ||
                                !dispatch_runtime[peer_slot]
                                         .head_on_recovery_cleared ||
                                dispatch_runtime[peer_slot]
                                        .carry_ambiguity_active;
                } else {
                        for (int node = 0;
                             node < TRACK_MAX; ++node) {
                                if (reservation
                                            .owner_by_node[node] ==
                                    peer_train) {
                                        peer_still_blocks = 1;
                                        break;
                                }
                        }
                }
                if (peer_still_blocks) {
                        block_resume[slot] = 1;
                        continue;
                }

                /*
                 * Removing (or physically clearing) the opposing train does
                 * not make a stale soft/old turnout command authoritative.
                 * Give the exact current hard-zone gate first refusal before
                 * considering rolling-authority range or a positive resume
                 * speed.  In particular, following_safe_resume_speed() may
                 * quite correctly return zero at the old authority ceiling;
                 * that must not bypass the freshly re-sent current turnout
                 * setting by transferring this hold straight into the
                 * rolling-authority FSM.
                 */
                int zone_wait =
                        defer_traffic_resume_for_conflict_zone(slot);
                if (zone_wait < 0) {
                        fail_job(slot, TC2_FAILURE_INTERNAL);
                        continue;
                }
                if (zone_wait > 0) {
                        continue;
                }

                int safe_resume_speed =
                        following_safe_resume_speed(
                                slot,
                                runtime
                                        ->traffic_resume_speed,
                                -1);
                if (safe_resume_speed < 1) {
                        if (runtime
                                            ->rolling_authority_active &&
                            runtime->authority_end_offset >= 0 &&
                            runtime->authority_end_offset <
                                    runtime->route.node_count - 1) {
                                runtime->traffic_reason =
                                        TC2_TRAFFIC_AUTHORITY;
                                job->traffic_reason =
                                        TC2_TRAFFIC_AUTHORITY;
                                runtime->traffic_peer_train = 0;
                                job->traffic_peer_train = 0;
                                (void)begin_authority_request(
                                        slot, now, 0);
                        }
                        continue;
                }
                runtime->traffic_resume_speed =
                        safe_resume_speed;
                if (resume_traffic_hold(
                            slot, now,
                            TC2_TRAFFIC_HEAD_ON) < 0) {
                        fail_job(
                                slot,
                                TC2_FAILURE_CAN_SERVICE);
                }
        }

        for (int slot = 0;
             slot < TC2_DISPATCH_MAX_JOBS; ++slot) {
                if (stop_reason[slot] ==
                    TC2_TRAFFIC_NONE) {
                        continue;
                }
                int envelope =
                        traffic_collision_stop_distance_mm(
                                slot);
                int resume = resume_threshold[slot];
                if (resume < 0 &&
                    stop_reason[slot] ==
                            TC2_TRAFFIC_FOLLOWING) {
                        resume =
                                traffic_resume_threshold_mm(
                                        envelope);
                }
                int authority_latched =
                        (dispatch_jobs[slot].state ==
                                 TC2_JOB_TRAFFIC_HOLD ||
                         (dispatch_jobs[slot].state ==
                                  TC2_JOB_BRAKING &&
                          dispatch_runtime[slot]
                                  .stop_purpose ==
                                  TC2_STOP_TRAFFIC)) &&
                        dispatch_runtime[slot]
                                .traffic_reason ==
                                TC2_TRAFFIC_AUTHORITY;
                begin_traffic_stop(
                        slot, now, stop_reason[slot],
                        stop_peer[slot], stop_gap[slot],
                        stop_threshold[slot], resume);
                /*
                 * Dynamic traffic may additionally block a train already
                 * stopped at an authority limit, but it must not replace the
                 * authority latch.  Otherwise the following auto-resume path
                 * could restart the train without first extending ownership
                 * and setting the newly authorized turnout.
                 */
                if (authority_latched) {
                        block_resume[slot] = 1;
                        continue;
                }
                if (dispatch_jobs[slot].state ==
                            TC2_JOB_TRAFFIC_HOLD ||
                    (dispatch_jobs[slot].state ==
                             TC2_JOB_BRAKING &&
                     dispatch_runtime[slot]
                             .stop_purpose ==
                             TC2_STOP_TRAFFIC)) {
                        dispatch_runtime[slot]
                                .traffic_reason =
                                stop_reason[slot];
                        dispatch_jobs[slot]
                                .traffic_reason =
                                stop_reason[slot];
                }
        }

        for (int slot = 0;
             slot < TC2_DISPATCH_MAX_JOBS; ++slot) {
                if (dispatch_jobs[slot].state !=
                            TC2_JOB_TRAFFIC_HOLD ||
                    dispatch_runtime[slot]
                            .traffic_reason !=
                            TC2_TRAFFIC_FOLLOWING ||
                    stop_reason[slot] !=
                            TC2_TRAFFIC_NONE ||
                    block_resume[slot] ||
                    !traffic_has_movement_authority(
                            slot, &reservation)) {
                        continue;
                }
                int maximum_resume_speed =
                        resume_speed_candidate[slot] > 0 ?
                        resume_speed_candidate[slot] :
                        dispatch_runtime[slot]
                                .traffic_resume_speed;
                int safe_resume_speed =
                        following_safe_resume_speed(
                                slot, maximum_resume_speed,
                                -1);
                if (safe_resume_speed < 1) {
                        tc2_dispatch_runtime *runtime =
                                &dispatch_runtime[slot];
                        tc2_dispatch_job_snapshot *job =
                                &dispatch_jobs[slot];
                        if (!runtime
                                            ->rolling_authority_active ||
                            runtime->authority_end_offset < 0 ||
                            runtime->authority_end_offset >=
                                    runtime->route.node_count - 1) {
                                /*
                                 * At a complete destination authority there
                                 * may be too little route left for even speed
                                 * one.  Remaining stopped is safer than
                                 * converting a valid managed trip into a
                                 * service failure or creeping past d1-d8.
                                 */
                                continue;
                        }
                        /*
                         * Keep speed zero and hand the same frozen position
                         * to the rolling-authority FSM.  It atomically
                         * extends ownership, sets the newly authorized
                         * turnouts, waits for settle, and only then restarts.
                         */
                        runtime->traffic_reason =
                                TC2_TRAFFIC_AUTHORITY;
                        job->traffic_reason =
                                TC2_TRAFFIC_AUTHORITY;
                        (void)begin_authority_request(
                                slot, now, 0);
                        continue;
                }
                dispatch_runtime[slot].traffic_resume_speed =
                        safe_resume_speed;
                int zone_wait =
                        defer_traffic_resume_for_conflict_zone(slot);
                if (zone_wait < 0) {
                        fail_job(slot, TC2_FAILURE_INTERNAL);
                        continue;
                }
                if (zone_wait > 0) {
                        continue;
                }
                /*
                 * If the leader has left the common corridor, the exclusive
                 * generation-preserving reservation is sufficient movement
                 * authority.  Otherwise RESUME_REAR above already proved
                 * the 400mm hysteresis boundary.
                 */
                if (resume_traffic_hold(
                            slot, now,
                            TC2_TRAFFIC_FOLLOWING) < 0) {
                        fail_job(
                                slot,
                                TC2_FAILURE_CAN_SERVICE);
                }
        }
}

/*
 * A reservation extension is necessary but not sufficient to restart a
 * stopped train.  If another managed train still occupies the same directed
 * corridor, preserve the 400mm following hysteresis; if it occupies the
 * opposite direction, keep the authority waiter stopped until that corridor
 * is completely clear.  Exclusive reservation ownership continues to
 * arbitrate future-only shared intersections.
 */
static int authority_dynamic_resume_is_safe(
        int slot, int now,
        const track_reservation_snapshot *reservation) {
        if (slot < 0 || slot >= TC2_DISPATCH_MAX_JOBS ||
            !reservation ||
            dispatch_jobs[slot].state !=
                    TC2_JOB_TRAFFIC_HOLD ||
            dispatch_runtime[slot].traffic_reason !=
                    TC2_TRAFFIC_AUTHORITY ||
            !traffic_has_movement_authority(
                    slot, reservation)) {
                return 0;
        }
        int maximum_resume_speed =
                dispatch_runtime[slot]
                        .traffic_resume_speed;
        for (int peer = 0;
             peer < TC2_DISPATCH_MAX_JOBS; ++peer) {
                if (peer == slot ||
                    !traffic_slot_has_position(peer)) {
                        continue;
                }
                tc2_live_traffic_relation relation;
                if (live_traffic_relation_for_pair(
                            slot, peer, now,
                            &relation) < 0) {
                        return 0;
                }
                if (!relation.valid) continue;
                if (relation.relation ==
                            TC2_TRAFFIC_RELATION_HEAD_ON ||
                    relation.order_ambiguous) {
                        return 0;
                }
                if (relation.rear_slot != slot) {
                        continue;
                }
                if (relation.gap_mm <
                    TC2_TRAFFIC_FOLLOWING_RESUME_CLEARANCE_MM) {
                        return 0;
                }
                int safe_speed =
                        following_gap_safe_resume_speed(
                                maximum_resume_speed,
                                relation.gap_mm);
                if (safe_speed < 1) return 0;
                maximum_resume_speed = safe_speed;
        }
        dispatch_runtime[slot].traffic_resume_speed =
                maximum_resume_speed;
        return 1;
}

static int authority_request_is_older(
        int candidate, int current, int now) {
        if (current < 0) return 1;
        int candidate_age =
                tick_age(
                        now,
                        dispatch_runtime[candidate]
                                .authority_request_tick);
        int current_age =
                tick_age(
                        now,
                        dispatch_runtime[current]
                                .authority_request_tick);
        if (candidate_age != current_age) {
                return candidate_age > current_age;
        }
        unsigned int candidate_sequence =
                dispatch_runtime[candidate]
                        .authority_request_sequence;
        unsigned int current_sequence =
                dispatch_runtime[current]
                        .authority_request_sequence;
        if (candidate_sequence != current_sequence) {
                return sequence_before(
                        candidate_sequence,
                        current_sequence);
        }
        return dispatch_jobs[candidate].train <
                dispatch_jobs[current].train;
}

static int authority_resume_speed_for_remaining(
        int requested_speed, int remaining_mm) {
        if (requested_speed < 1 || requested_speed > 120 ||
            remaining_mm < 0) {
                return -1;
        }
        for (int speed = requested_speed;
             speed >= 1; --speed) {
                int stop =
                        Tc2MotionStopDistanceMm(speed);
                if (stop >= 0 && stop <= remaining_mm) {
                        return speed;
                }
        }
        return -1;
}

static int authority_route_metadata_is_valid(int slot);

static int authority_ticket_has_active_state(int slot) {
        if (slot < 0 || slot >= TC2_DISPATCH_MAX_JOBS) {
                return 0;
        }
        tc2_dispatch_runtime *runtime =
                &dispatch_runtime[slot];
        int state = dispatch_jobs[slot].state;
        return runtime->rolling_authority_active &&
                runtime->authority_request_tick >= 0 &&
                (state == TC2_JOB_PREPARING ||
                 state == TC2_JOB_READY ||
                 state == TC2_JOB_LAUNCHING ||
                 state == TC2_JOB_RUNNING ||
                 state == TC2_JOB_BRAKING ||
                 state == TC2_JOB_TRAFFIC_HOLD);
}

static int authority_route_metadata_is_valid(int slot) {
        if (slot < 0 || slot >= TC2_DISPATCH_MAX_JOBS) {
                return 0;
        }
        tc2_dispatch_runtime *runtime =
                &dispatch_runtime[slot];
        tc2_dispatch_job_snapshot *job =
                &dispatch_jobs[slot];
        int count = runtime->route.node_count;
        if (!runtime->route_valid || count < 2 ||
            count > TRACK_MAX ||
            runtime->authority_end_offset < 0 ||
            runtime->authority_end_offset >= count ||
            runtime->leg_start_offset < 0 ||
            runtime->leg_start_offset >= count ||
            runtime->leg_end_offset <
                    runtime->leg_start_offset ||
            runtime->leg_end_offset >= count ||
            job->target_route_offset < 0 ||
            job->target_route_offset >= count ||
            job->destination_route_offset <
                    job->target_route_offset ||
            job->destination_route_offset >= count ||
            job->destination_offset_mm < 0 ||
            job->destination_distance_mm < 0 ||
            job->speed < 1 || job->speed > 120) {
                return 0;
        }
        for (int offset = 0; offset < count; ++offset) {
                if (runtime->route.nodes[offset] < 0 ||
                    runtime->route.nodes[offset] >= TRACK_MAX) {
                        return 0;
                }
        }
        return 1;
}

static int authority_ticket_is_active(int slot) {
        return authority_ticket_has_active_state(slot) &&
                authority_route_metadata_is_valid(slot) &&
                dispatch_runtime[slot]
                                .authority_end_offset <
                        dispatch_runtime[slot]
                                .route.node_count - 1;
}

/*
 * FIFO arbitrates trains approaching a future shared resource.  A same-
 * direction front train is the one exception: if an older rear waiter is
 * held by the 400 mm following rule, making the front yield would freeze both
 * forever.  Letting the front proceed increases their gap and is therefore
 * the safe, live ordering.
 */
static int authority_front_may_pass_older_rear(
        int older_slot, int candidate_slot, int now) {
        tc2_live_traffic_relation relation;
        if (live_traffic_relation_for_pair(
                    older_slot, candidate_slot,
                    now, &relation) < 0 ||
            !relation.valid ||
            relation.order_ambiguous ||
            relation.relation ==
                    TC2_TRAFFIC_RELATION_HEAD_ON) {
                return 0;
        }
        return relation.rear_slot == older_slot;
}

/*
 * Test only resources newly added by this rolling extension against the
 * older waiter's exact next local request.  Neither side consults an entire
 * destination route, so a downstream intersection cannot make a train wait
 * at an unrelated current block.
 */
static int authority_extension_overlaps_older_future(
        int candidate_slot, int older_slot,
        const track_route *candidate_extension,
        const track_reservation_snapshot *reservation) {
        if (candidate_slot < 0 ||
            candidate_slot >= TC2_DISPATCH_MAX_JOBS ||
            older_slot < 0 ||
            older_slot >= TC2_DISPATCH_MAX_JOBS ||
            candidate_slot == older_slot ||
            !candidate_extension || !reservation ||
            candidate_extension->node_count < 1 ||
            candidate_extension->node_count > TRACK_MAX) {
                return -1;
        }
        tc2_dispatch_runtime *older =
                &dispatch_runtime[older_slot];
        if (!older->authority_requested_footprint_valid) {
                /*
                 * A ticket can be created before a feasible window exists
                 * (for example while too close to the current ceiling). There
                 * is no concrete shared local resource to arbitrate yet.
                 */
                return 0;
        }
        const track_route *older_request =
                &older->authority_requested_footprint;
        if (older_request->node_count < 1 ||
            older_request->node_count > TRACK_MAX) {
                return -1;
        }

        int candidate_train =
                dispatch_jobs[candidate_slot].train;
        for (int left = 0;
             left < candidate_extension->node_count; ++left) {
                int node =
                        candidate_extension->nodes[left];
                int reverse =
                        physical_reverse_index(node);
                int already_owned =
                        reservation->owner_by_node[node] ==
                                candidate_train ||
                        (reverse >= 0 &&
                         reservation->owner_by_node[reverse] ==
                                 candidate_train);
                if (already_owned) continue;
                for (int right = 0;
                     right < older_request->node_count; ++right) {
                        int older_node =
                                older_request->nodes[right];
                        int older_reverse =
                                physical_reverse_index(
                                        older_node);
                        if (node == older_node ||
                            node == older_reverse ||
                            reverse == older_node) {
                                return 1;
                        }
                }
        }
        return 0;
}

static int authority_prefetch_must_yield(
        int candidate_slot, int now,
        const track_route *candidate_extension,
        const track_reservation_snapshot *reservation) {
        tc2_dispatch_job_snapshot *candidate =
                &dispatch_jobs[candidate_slot];
        for (int older = 0;
             older < TC2_DISPATCH_MAX_JOBS; ++older) {
                if (older == candidate_slot ||
                    !authority_ticket_is_active(older) ||
                    !authority_request_is_older(
                            older, candidate_slot, now)) {
                        continue;
                }
                int overlap =
                        authority_extension_overlaps_older_future(
                                candidate_slot, older,
                                candidate_extension,
                                reservation);
                if (overlap < 0) return -1;
                int older_blocked_by_candidate =
                        dispatch_jobs[older].conflict_train ==
                                candidate->train &&
                        dispatch_jobs[older].conflict_node >= 0 &&
                        dispatch_jobs[older].conflict_node <
                                TRACK_MAX &&
                        reservation->owner_by_node[
                                dispatch_jobs[older]
                                        .conflict_node] ==
                                candidate->train;
                if (overlap &&
                    !authority_front_may_pass_older_rear(
                            older, candidate_slot, now) &&
                    !older_blocked_by_candidate) {
                        return 1;
                }
        }
        return 0;
}

/*
 * A bounded first window may require a lower launch command than requested.
 * Restore the requested speed only after a later reservation CAS has
 * committed, its turnout suffix is confirmed/settled, and the new active
 * center ceiling contains the complete conservative stop envelope.
 */
static int authority_restore_requested_speed(
        int slot, int now) {
        if (slot < 0 || slot >= TC2_DISPATCH_MAX_JOBS) {
                return -1;
        }
        tc2_dispatch_job_snapshot *job =
                &dispatch_jobs[slot];
        tc2_dispatch_runtime *runtime =
                &dispatch_runtime[slot];
        if (job->state != TC2_JOB_RUNNING ||
            !runtime->rolling_authority_active ||
            runtime->authority_prefetch_active ||
            runtime->precision_approach_active ||
            runtime->command_speed < 1 ||
            runtime->command_speed >= job->speed) {
                return 0;
        }
        int braking =
                Tc2MotionBrakingDistanceMm(job->speed);
        int64_t progress_um;
        if (braking < 0 ||
            runtime->authority_center_ceiling_mm < 0 ||
            runtime_motion_progress_um(
                    slot, now, &progress_um) < 0 ||
            progress_um < 0 ||
            progress_um >
                    INT64_MAX -
                            (int64_t)braking * 1000 ||
            progress_um + (int64_t)braking * 1000 >
                    (int64_t)runtime
                            ->authority_center_ceiling_mm * 1000) {
                return 0;
        }
        if (command_train_speed_with_retry(
                    job->train, job->speed) < 0) {
                return -1;
        }
        int confirmed = Time();
        if (confirmed < 0) return -1;
        /*
         * Rebase with the old acknowledged-speed model at the confirmation
         * instant.  Updating command_speed before this calculation would
         * fabricate an acceleration jump in both collision spacing and UI.
         */
        if (runtime_motion_progress_um(
                    slot, confirmed,
                    &progress_um) < 0 ||
            progress_um < 0) {
                return -1;
        }
        runtime->motion_anchor_distance_um =
                progress_um;
        runtime->motion_anchor_tick = confirmed;
        runtime->motion_anchor_from_traffic = 0;
        runtime->command_speed = job->speed;
        runtime->motion_speed_ceiling = job->speed;
        runtime->speed_reduction_pending = 0;
        runtime->precision_approach_active = 0;
        runtime->observed_velocity_um_per_tick = -1;
        job->command_speed = job->speed;
        return schedule_motion_deadlines(
                slot, confirmed) < 0 ? -1 : 1;
}

/*
 * Stopped reversal handoff.
 *
 * The moving leg never reserves across a reversal.  Once zero speed has
 * settled at the reversal sensor, atomically replace the old window with the
 * union of (a) that incoming window and (b) one bounded outbound window.
 * No turnout or reverse command is sent until this CAS succeeds.  The
 * incoming union becomes reversal_carry and is released only after ordered
 * outbound motion places the train tail plus 200 mm beyond the sensor.
 *
 * Returns 0 after acquiring the outbound window, 1 while safely waiting for
 * a conflict/FIFO predecessor, and -1 on a deterministic/service failure.
 */
static int acquire_reversal_authority(
        int slot, int now) {
        if (slot < 0 || slot >= TC2_DISPATCH_MAX_JOBS) {
                return -1;
        }
        tc2_dispatch_job_snapshot *job =
                &dispatch_jobs[slot];
        tc2_dispatch_runtime *runtime =
                &dispatch_runtime[slot];
        if (!runtime->rolling_authority_active ||
            !runtime->route_valid ||
            runtime->stop_purpose != TC2_STOP_REVERSAL ||
            runtime->leg_end_offset <
                    runtime->leg_start_offset ||
            runtime->leg_end_offset >=
                    job->destination_route_offset) {
                return -1;
        }
        int next_leg = runtime->leg_end_offset + 1;
        if (next_leg + 1 >= runtime->route.node_count ||
            dispatch_track[
                    runtime->route.nodes[
                            runtime->leg_end_offset]]
                            .reverse !=
                    &dispatch_track[
                            runtime->route.nodes[next_leg]]) {
                return -1;
        }
        track_reservation_snapshot reservation;
        train_sensor_snapshot_t sensors;
        if (TrackReservationServerSnapshot(
                    dispatch_reservation_tid,
                    &reservation) < 0 ||
            TrainSensorGetLatest(
                    dispatch_sensor_tid, &sensors) < 0 ||
            reservation
                            .generation_by_train[job->train] !=
                    job->plan_generation ||
            reservation
                            .destination_by_train[job->train] !=
                    job->target_node ||
            sensor_health_is_safe(
                    slot, &sensors, 0, now) < 0 ||
            can_health_is_safe(slot, 0, now) < 0) {
                return -1;
        }
        int progress_mm;
        if (route_distance_to_offset(
                    &runtime->route, next_leg,
                    &progress_mm) < 0) {
                return -1;
        }
        int selected_speed = job->speed;
        track_route outbound;
        outbound.node_count = 0;
        outbound.distance_mm = 0;
        outbound.optimization_cost_mm = 0;
        outbound.reversal_count = 0;
        track_reservation_conflict conflict;
        int outbound_end = -1;
        int outbound_ceiling = -1;
        int selected =
                select_bounded_rolling_authority_window(
                        &runtime->route, next_leg,
                        next_leg + 1,
                        job->target_route_offset,
                        job->destination_route_offset,
                        job->destination_offset_mm,
                        job->destination_distance_mm,
                        selected_speed, 1, progress_mm,
                        job->train, &reservation, &sensors,
                        &outbound, &outbound_end,
                        &outbound_ceiling, &selected_speed,
                        &conflict);
        if (selected > 0) {
                job->conflict_train =
                        conflict.owner_train;
                job->conflict_node =
                        conflict.node_index;
                if (begin_authority_request(
                            slot, now,
                            outbound.node_count > 0 ?
                                    &outbound : 0) < 0) {
                        return -1;
                }
                return 1;
        }
        if (selected < 0 ||
            outbound_end < next_leg + 1 ||
            outbound_ceiling < progress_mm) {
                return -1;
        }

        track_route incoming =
                runtime->safety_footprint;
        if (incoming.node_count < 1 ||
            append_route_nodes(
                    &outbound, &incoming) < 0) {
                return -1;
        }
        if (begin_authority_request(
                    slot, now, &outbound) < 0) {
                return -1;
        }
        int yield =
                authority_prefetch_must_yield(
                        slot, now, &outbound,
                        &reservation);
        if (yield < 0) return -1;
        if (yield > 0) return 1;

        unsigned int generation = 0;
        int update =
                TrackReservationServerUpdateWindow(
                        dispatch_reservation_tid,
                        &outbound, job->train,
                        job->target_node,
                        job->plan_generation,
                        &conflict, &generation);
        if (update == 1) {
                job->conflict_train =
                        conflict.owner_train;
                job->conflict_node =
                        conflict.node_index;
                return 1;
        }
        if (update < 0 ||
            generation != job->plan_generation) {
                return -1;
        }

        unsigned int baseline =
                runtime->last_journal_sequence;
        runtime->reversal_carry_footprint =
                incoming;
        runtime->reversal_carry_active = 1;
        runtime->reversal_carry_origin_offset =
                next_leg;
        runtime->safety_footprint = outbound;
        runtime->authority_end_offset =
                outbound_end;
        runtime->authority_center_ceiling_mm =
                outbound_ceiling;
        runtime->authority_prefetch_active = 0;
        runtime->authority_prefetch_end_offset = -1;
        runtime->authority_prefetch_center_ceiling_mm = -1;
        runtime->authority_prefetch_settle_at_tick = -1;
        runtime->authority_prefetch_rearm_progress_mm = -1;
        clear_authority_request(runtime);
        runtime->planned_launch_speed =
                selected_speed;
        runtime->leg_start_offset = next_leg;
        ++job->reversals_completed;
        ++job->current_leg;
        if (init_leg_monitor(slot, baseline) < 0) {
                return -1;
        }
        job->conflict_train = 0;
        job->conflict_node = -1;
        return 0;
}

/*
 * Two stationary rolling-authority jobs can form a real wait-for cycle when
 * each next local window contains track still owned by the other.  This is
 * distinct from an ordinary FIFO intersection wait: neither grant can make
 * progress without first changing one physical plan.
 *
 * Promote only a mutual two-owner cycle to HEAD_ON recovery.  The existing
 * traffic-hold reservation remains untouched, so the operator can safely:
 *   1. cancel one train, wait for STOPPED, physically remove it, then remove;
 *   2. stage a CURRENT destination for either train and let the atomic
 *      replacement choose a reverse/reroute path.
 */
static void expose_mutual_authority_deadlocks(void) {
        for (int slot = 0;
             slot < TC2_DISPATCH_MAX_JOBS; ++slot) {
                tc2_dispatch_job_snapshot *job =
                        &dispatch_jobs[slot];
                tc2_dispatch_runtime *runtime =
                        &dispatch_runtime[slot];
                if (job->state != TC2_JOB_TRAFFIC_HOLD ||
                    runtime->traffic_reason !=
                            TC2_TRAFFIC_AUTHORITY ||
                    runtime->stop_purpose !=
                            TC2_STOP_TRAFFIC ||
                    job->conflict_train < 1 ||
                    job->conflict_train > 255) {
                        continue;
                }
                int peer =
                        find_train_job(job->conflict_train);
                if (peer <= slot) continue;
                tc2_dispatch_job_snapshot *peer_job =
                        &dispatch_jobs[peer];
                tc2_dispatch_runtime *peer_runtime =
                        &dispatch_runtime[peer];
                if (peer_job->state !=
                            TC2_JOB_TRAFFIC_HOLD ||
                    peer_runtime->traffic_reason !=
                            TC2_TRAFFIC_AUTHORITY ||
                    peer_runtime->stop_purpose !=
                            TC2_STOP_TRAFFIC ||
                    peer_job->conflict_train != job->train) {
                        continue;
                }

                runtime->traffic_reason =
                        TC2_TRAFFIC_HEAD_ON;
                runtime->traffic_peer_train =
                        peer_job->train;
                job->traffic_reason =
                        TC2_TRAFFIC_HEAD_ON;
                job->traffic_peer_train =
                        peer_job->train;
                peer_runtime->traffic_reason =
                        TC2_TRAFFIC_HEAD_ON;
                peer_runtime->traffic_peer_train =
                        job->train;
                peer_job->traffic_reason =
                        TC2_TRAFFIC_HEAD_ON;
                peer_job->traffic_peer_train =
                        job->train;
        }
}

/*
 * First-arrival ordered rolling-authority extension.
 *
 * The reservation CAS happens before any turnout command.  A successful CAS
 * preserves the plan generation, replaces only the rolling footprint, and
 * then moves the stationary train through PREPARING/READY/LAUNCHING.  The
 * launch confirmation path rebases motion at the frozen coast endpoint, so
 * neither the controller nor the UI can jump back to an old sensor.
 */
static void process_authority_holds(int now) {
        unsigned char processed[TC2_DISPATCH_MAX_JOBS];
        for (int slot = 0;
             slot < TC2_DISPATCH_MAX_JOBS; ++slot) {
                processed[slot] = 0;
                if (dispatch_jobs[slot].state ==
                            TC2_JOB_TRAFFIC_HOLD &&
                    dispatch_runtime[slot]
                            .traffic_reason ==
                            TC2_TRAFFIC_AUTHORITY &&
                    !dispatch_runtime[slot]
                            .conflict_zone_hold_active &&
                    dispatch_runtime[slot]
                            .authority_request_tick < 0) {
                        (void)begin_authority_request(
                                slot, now, 0);
                }
        }

        for (int pass = 0;
             pass < TC2_DISPATCH_MAX_JOBS; ++pass) {
                int oldest = -1;
                for (int slot = 0;
                     slot < TC2_DISPATCH_MAX_JOBS; ++slot) {
                        if (processed[slot] ||
                            dispatch_jobs[slot].state !=
                                    TC2_JOB_TRAFFIC_HOLD ||
                            dispatch_runtime[slot]
                                    .traffic_reason !=
                                    TC2_TRAFFIC_AUTHORITY ||
                            dispatch_runtime[slot]
                                    .conflict_zone_hold_active) {
                                continue;
                        }
                        if (authority_request_is_older(
                                    slot, oldest, now)) {
                                oldest = slot;
                        }
                }
                if (oldest < 0) break;
                processed[oldest] = 1;

                tc2_dispatch_job_snapshot *job =
                        &dispatch_jobs[oldest];
                tc2_dispatch_runtime *runtime =
                        &dispatch_runtime[oldest];
                if (!authority_route_metadata_is_valid(
                            oldest)) {
                        fail_job(
                                oldest,
                                TC2_FAILURE_ROUTE);
                        continue;
                }
                /*
                 * A turnout/intersection gate can stop a train while a
                 * rolling prefetch suffix is already reserved but not yet
                 * promoted.  Stationary recovery replaces that suffix with
                 * a fresh bounded window; carrying the old prefetch marker
                 * across the replacement makes active_end == prefetch_end
                 * and the next RUNNING pass (correctly) rejects the stale
                 * invariant.  First shrink ownership back to the confirmed
                 * active window, then take the snapshot used for this FIFO
                 * request.  This is route- and train-independent.
                 */
                if (runtime->authority_prefetch_active &&
                    abandon_authority_prefetch(oldest) < 0) {
                        fail_job(
                                oldest,
                                TC2_FAILURE_RESERVATION);
                        continue;
                }
                track_reservation_snapshot reservation;
                train_sensor_snapshot_t sensors;
                if (TrackReservationServerSnapshot(
                            dispatch_reservation_tid,
                            &reservation) < 0 ||
                    TrainSensorGetLatest(
                            dispatch_sensor_tid,
                            &sensors) < 0) {
                        fail_job(
                                oldest,
                                TC2_FAILURE_RESERVATION_SERVICE);
                        continue;
                }
                if (reservation
                                    .generation_by_train[job->train] !=
                            job->plan_generation ||
                    reservation
                                    .destination_by_train[job->train] !=
                            job->target_node) {
                        fail_job(
                                oldest,
                                TC2_FAILURE_RESERVATION);
                        continue;
                }
                if (sensor_health_is_safe(
                            oldest, &sensors, 0, now) < 0) {
                        fail_job(
                                oldest,
                                TC2_FAILURE_SENSOR_SERVICE);
                        continue;
                }
                if (can_health_is_safe(
                            oldest, 0, now) < 0) {
                        fail_job(
                                oldest,
                                TC2_FAILURE_CAN_HEALTH);
                        continue;
                }
                if (!authority_dynamic_resume_is_safe(
                            oldest, now,
                            &reservation)) {
                        continue;
                }
                if (!runtime->rolling_authority_active ||
                    runtime->authority_end_offset < 0 ||
                    runtime->authority_end_offset >=
                            runtime->route.node_count - 1 ||
                    runtime->traffic_hold_distance_um < 0 ||
                    runtime->traffic_resume_speed < 1 ||
                    runtime->traffic_resume_speed > 120) {
                        fail_job(
                                oldest,
                                TC2_FAILURE_INTERNAL);
                        continue;
                }

                int tail =
                        footprint_tail_offset(
                                runtime, job);
                int current_progress_mm =
                        (int)((runtime
                                              ->traffic_hold_distance_um +
                                      999) /
                              1000);
                int remaining_mm =
                        job->destination_distance_mm -
                        current_progress_mm;
                int authority_speed =
                        authority_resume_speed_for_remaining(
                                job->speed,
                                remaining_mm);
                if (tail < 0 ||
                    tail > runtime->authority_end_offset ||
                    current_progress_mm < 0 ||
                    remaining_mm < 0 ||
                    authority_speed < 1) {
                        fail_job(
                                oldest,
                                TC2_FAILURE_INTERNAL);
                        continue;
                }

                track_route extension;
                extension.node_count = 0;
                extension.distance_mm = 0;
                extension.optimization_cost_mm = 0;
                extension.reversal_count = 0;
                track_reservation_conflict conflict;
                int new_end = -1;
                int new_ceiling = -1;
                int selected_window_speed = -1;
                int selected =
                        select_bounded_rolling_authority_window_for_leg(
                                &runtime->route, tail,
                                runtime->leg_start_offset,
                                runtime
                                                ->authority_end_offset +
                                        1,
                                job->target_route_offset,
                                job->destination_route_offset,
                                job->destination_offset_mm,
                                job->destination_distance_mm,
                                authority_speed,
                                1,
                                current_progress_mm,
                                job->train,
                                &reservation, &sensors,
                                &extension, &new_end,
                                &new_ceiling,
                                &selected_window_speed,
                                &conflict);
                if (selected > 0) {
                        job->conflict_train =
                                conflict.owner_train;
                        job->conflict_node =
                                conflict.node_index;
                        if (begin_authority_request(
                                    oldest, now,
                                    extension.node_count > 0 ?
                                            &extension : 0) < 0) {
                                fail_job(
                                        oldest,
                                        TC2_FAILURE_INTERNAL);
                        }
                        continue;
                }
                if (selected < 0 ||
                    new_end <=
                            runtime->authority_end_offset ||
                    new_ceiling <
                            current_progress_mm ||
                    selected_window_speed < 1 ||
                    selected_window_speed >
                            authority_speed) {
                        fail_job(
                                oldest,
                                TC2_FAILURE_ROUTE);
                        continue;
                }
                if (begin_authority_request(
                            oldest, now,
                            &extension) < 0) {
                        fail_job(
                                oldest,
                                TC2_FAILURE_INTERNAL);
                        continue;
                }
                /*
                 * FIFO is local to the resources this extension would add.
                 * Comparing complete remaining routes here would make a
                 * distant merge block a train at its present authority limit
                 * even though every intervening block is free.
                 */
                int yield_to_older =
                        authority_prefetch_must_yield(
                                oldest, now, &extension,
                                &reservation);
                if (yield_to_older < 0) {
                        fail_job(
                                oldest,
                                TC2_FAILURE_ROUTE);
                        continue;
                }
                if (yield_to_older > 0) {
                        continue;
                }

                unsigned int generation = 0;
                int update =
                        TrackReservationServerUpdateWindow(
                                dispatch_reservation_tid,
                                &extension, job->train,
                                job->target_node,
                                job->plan_generation,
                                &conflict, &generation);
                if (update == 1) {
                        job->conflict_train =
                                conflict.owner_train;
                        job->conflict_node =
                                conflict.node_index;
                        continue;
                }
                if (update < 0 ||
                    generation !=
                            job->plan_generation) {
                        fail_job(
                                oldest,
                                TC2_FAILURE_RESERVATION);
                        continue;
                }

                int old_end =
                        runtime->authority_end_offset;
                runtime->safety_footprint =
                        extension;
                runtime->authority_previous_end_offset =
                        old_end;
                runtime->authority_end_offset =
                        new_end;
                runtime->authority_center_ceiling_mm =
                        new_ceiling;
                /*
                 * This request has acquired its exact local window.  Its FIFO
                 * turn is therefore consumed now; a later extension (and a
                 * later shared intersection) must take a fresh ticket.
                 */
                clear_authority_request(runtime);
                runtime->authority_resume_pending = 1;
                /*
                 * Admission is closed until the next complete dynamic
                 * traffic pass evaluates the frozen train against every
                 * other managed position.
                 */
                runtime->prelaunch_traffic_blocked = 1;
                runtime->traffic_resume_speed =
                        authority_speed;
                runtime->planned_launch_speed =
                        selected_window_speed;
                runtime->ready_wave_tick = -1;
                runtime->prepare_attributed_sequence =
                        sensors.attributed_count;
                runtime
                        ->prepare_attribution_unavailable_count =
                        sensors
                                .attribution_unavailable_count;
                runtime->prepare_unattributed_count =
                        sensors.unattributed_count;
                for (int sensor_index = 0;
                     sensor_index < TRAIN_SENSOR_COUNT;
                     ++sensor_index) {
                        runtime
                                ->prepare_sensor_state[sensor_index] =
                                sensors
                                        .sensor_state[sensor_index];
                }
                job->conflict_train = 0;
                job->conflict_node = -1;
                job->ready_at_tick = -1;

                int turnout_token =
                        queue_turnout_plan_for_leg(
                                oldest);
                if (turnout_token < 0 &&
                    turnout_token != CAN_SEND_BUSY) {
                        fail_job(
                                oldest,
                                TC2_FAILURE_TURNOUT);
                        continue;
                }
                runtime->turnout_queue_pending =
                        turnout_token == CAN_SEND_BUSY;
                runtime->turnout_batch_token =
                        turnout_token > 0 ?
                        (unsigned int)turnout_token : 0;
                job->state = TC2_JOB_PREPARING;
        }
        expose_mutual_authority_deadlocks();
}

/*
 * Keep at most one not-yet-usable authority window in front of a moving
 * train.  The reservation CAS may succeed while the train continues under
 * its old ceiling, but that ceiling is never promoted until every turnout in
 * the new suffix is confirmed and has completed its mechanical settle time.
 *
 * `authority_prefetch_rearm_progress_mm` prevents a stationary or very slow
 * train from chaining successful prefetches until it owns its whole route:
 * after promotion it must physically pass the old movement ceiling before it
 * may ask for another window.
 */
static int process_running_authority_prefetch(
        int slot, int now,
        const track_reservation_snapshot *reservation,
        const train_sensor_snapshot_t *sensors) {
        if (slot < 0 || slot >= TC2_DISPATCH_MAX_JOBS ||
            !reservation || !sensors) {
                return TC2_FAILURE_INTERNAL;
        }
        tc2_dispatch_job_snapshot *job =
                &dispatch_jobs[slot];
        tc2_dispatch_runtime *runtime =
                &dispatch_runtime[slot];
        if (job->state != TC2_JOB_RUNNING ||
            !runtime->rolling_authority_active ||
            !runtime->route_valid ||
            runtime->authority_end_offset < 0 ||
            runtime->authority_end_offset >=
                    runtime->route.node_count ||
            runtime->authority_center_ceiling_mm < 0) {
                return 0;
        }

        if (runtime->authority_prefetch_active) {
                if (runtime->authority_prefetch_end_offset <=
                            runtime->authority_end_offset ||
                    runtime->authority_prefetch_end_offset >=
                            runtime->route.node_count ||
                    runtime
                                    ->authority_prefetch_center_ceiling_mm <=
                            runtime->authority_center_ceiling_mm) {
                        return TC2_FAILURE_INTERNAL;
                }
                if (runtime->turnout_queue_pending) {
                        int queued =
                                queue_turnout_plan_range(
                                        slot,
                                        runtime
                                                ->authority_end_offset,
                                        runtime
                                                ->authority_prefetch_end_offset);
                        if (queued == CAN_SEND_BUSY) {
                                return 0;
                        }
                        if (queued < 0) {
                                return TC2_FAILURE_TURNOUT;
                        }
                        runtime->turnout_queue_pending = 0;
                        runtime->turnout_batch_token =
                                queued > 0 ?
                                (unsigned int)queued : 0;
                        runtime->authority_prefetch_settle_at_tick =
                                queued == 0 ? now : -1;
                }
                if (runtime->turnout_batch_token != 0) {
                        can_batch_status_t status;
                        unsigned int token =
                                runtime->turnout_batch_token;
                        if (CanGetBatchStatus(
                                    dispatch_can_tid, token,
                                    &status) < 0 ||
                            status.token != token ||
                            status.total < 1) {
                                return TC2_FAILURE_TURNOUT;
                        }
                        if (status.state == CAN_BATCH_FAILED) {
                                int retry =
                                        queue_turnout_plan_range(
                                                slot,
                                                runtime
                                                        ->authority_end_offset,
                                                runtime
                                                        ->authority_prefetch_end_offset);
                                if (retry == CAN_SEND_BUSY) {
                                        runtime
                                                ->turnout_batch_token = 0;
                                        runtime
                                                ->turnout_queue_pending = 1;
                                        return 0;
                                }
                                if (retry < 0) {
                                        return TC2_FAILURE_TURNOUT;
                                }
                                runtime->turnout_batch_token =
                                        retry > 0 ?
                                        (unsigned int)retry : 0;
                                runtime
                                        ->authority_prefetch_settle_at_tick =
                                        retry == 0 ? now : -1;
                                return 0;
                        }
                        if (status.state != CAN_BATCH_COMPLETE ||
                            status.confirmed != status.total) {
                                return 0;
                        }
                        runtime->turnout_batch_token = 0;
                        runtime->authority_prefetch_settle_at_tick =
                                tick_after(
                                        now,
                                        TC2_TURNOUT_SETTLE_TICKS);
                        return 0;
                }
                if (runtime->authority_prefetch_settle_at_tick < 0 ||
                    !tick_reached(
                            now,
                            runtime
                                    ->authority_prefetch_settle_at_tick)) {
                        return 0;
                }

                int old_ceiling =
                        runtime->authority_center_ceiling_mm;
                runtime->authority_end_offset =
                        runtime->authority_prefetch_end_offset;
                runtime->authority_center_ceiling_mm =
                        runtime
                                ->authority_prefetch_center_ceiling_mm;
                runtime->authority_prefetch_active = 0;
                runtime->authority_prefetch_end_offset = -1;
                runtime
                        ->authority_prefetch_center_ceiling_mm = -1;
                runtime->authority_prefetch_settle_at_tick = -1;
                runtime->authority_prefetch_rearm_progress_mm =
                        old_ceiling;
                job->conflict_train = 0;
                job->conflict_node = -1;
                clear_authority_request(runtime);
                if (authority_restore_requested_speed(
                            slot, now) < 0) {
                        return TC2_FAILURE_CAN_SERVICE;
                }
                return 0;
        }

        /*
         * A reversal sensor is the exact terminal of the current motion leg.
         * Never prefetch across it while moving; the stopped reversal
         * handoff atomically acquires the next outbound window.
         */
        if (runtime->leg_end_offset <
                    job->destination_route_offset &&
            runtime->authority_end_offset >=
                    runtime->leg_end_offset) {
                return 0;
        }

        if (runtime->authority_end_offset >=
            runtime->route.node_count - 1) {
                return 0;
        }
        int64_t progress_um;
        int braking_envelope =
                traffic_braking_envelope_mm(slot);
        if (runtime_motion_progress_um(
                    slot, now, &progress_um) < 0 ||
            progress_um < 0 || braking_envelope < 0) {
                return TC2_FAILURE_INTERNAL;
        }
        int current_progress_mm =
                (int)((progress_um + 999) / 1000);
        int speed = traffic_effective_speed(slot);
        if (speed < 1 || speed > 120) {
                return TC2_FAILURE_INTERNAL;
        }
        int selection_speed =
                runtime->precision_approach_active ?
                runtime->command_speed :
                runtime->command_speed < job->speed ?
                job->speed : speed;
        int prefetch_lead =
                authority_prefetch_lead_mm(speed);
        int shared_sparse_rebase_route =
                uses_measured_direct_stop(job) &&
                job->start_index >= 2;
        int final_approach_suffix_required = 0;
        if (uses_measured_direct_stop(job) &&
            runtime->leg_end_offset ==
                    job->destination_route_offset &&
            runtime->authority_end_offset <
                    runtime->route.node_count - 1) {
                int prior_sensor =
                        sensor_before_offset(
                                &runtime->route,
                                job->target_route_offset);
                final_approach_suffix_required =
                        prior_sensor >= 0 &&
                        runtime->authority_end_offset >=
                                prior_sensor;
        }
        /*
         * A detector report can legitimately rebase progress by more than
         * one scheduler tick.  On the final approach, begin the conflict-
         * checked suffix request one additional CAN/settle lead early so an
         * ordered forward report cannot jump from just before the ordinary
         * trigger to beyond the old hard braking line.
         */
        int trigger_lead = prefetch_lead;
        /*
         * Track-D can omit adjacent reports on the long C/D/E/F routes. A
         * later ordered detector is still valid evidence, but can rebase
         * progress across two intervals at once. Begin every such direct-trip
         * extension three CAN/settle leads early, not only the final suffix,
         * so that a legitimate rebase remains inside confirmed movement
         * authority. Selection stays local and conflict checked.
         */
        if (shared_sparse_rebase_route) {
                trigger_lead =
                        saturated_threshold_add(
                                prefetch_lead,
                                prefetch_lead);
                trigger_lead =
                        saturated_threshold_add(
                                trigger_lead,
                                prefetch_lead);
        }
        if (final_approach_suffix_required) {
                trigger_lead =
                        saturated_threshold_add(
                                prefetch_lead,
                                prefetch_lead);
                trigger_lead =
                        saturated_threshold_add(
                                trigger_lead,
                                prefetch_lead);
        }
        if (runtime->authority_prefetch_rearm_progress_mm >= 0) {
                int rearm_progress =
                        runtime
                                ->authority_prefetch_rearm_progress_mm;
                /*
                 * Do not chain ordinary windows while stationary.  A final
                 * approach is different: the newly promoted local window is
                 * already beyond the penultimate detector and the next
                 * request is only its conflict-checked stop guard.  Rearm one
                 * lead before the old ceiling so the request can begin
                 * before a missing-detector rebase jumps across the narrow
                 * interval between old ceiling and new hard braking line.
                 */
                if (final_approach_suffix_required ||
                    shared_sparse_rebase_route) {
                        rearm_progress =
                                rearm_progress > prefetch_lead ?
                                rearm_progress -
                                        prefetch_lead : 0;
                }
                if (current_progress_mm <= rearm_progress) {
                        return 0;
                }
                runtime->authority_prefetch_rearm_progress_mm = -1;
        }
        int prefetch_headroom =
                saturated_threshold_add(
                        braking_envelope,
                        trigger_lead);
        if (prefetch_headroom < 0 ||
            runtime->authority_center_ceiling_mm -
                            current_progress_mm >
                    prefetch_headroom) {
                return 0;
        }

        int tail = footprint_tail_offset(runtime, job);
        if (tail < 0 || tail > runtime->authority_end_offset) {
                return TC2_FAILURE_INTERNAL;
        }
        track_route extension;
        extension.node_count = 0;
        extension.distance_mm = 0;
        extension.optimization_cost_mm = 0;
        extension.reversal_count = 0;
        track_reservation_conflict conflict;
        int new_end = -1;
        int new_ceiling = -1;
        int selected_window_speed = -1;
        int next_window_origin =
                saturated_threshold_add(
                        runtime
                                ->authority_center_ceiling_mm,
                        prefetch_lead);
        int required_confirmation_leads =
                speed == 120 ? 3 : 2;
        for (int lead_index = 1;
             lead_index < required_confirmation_leads &&
             next_window_origin >= 0;
             ++lead_index) {
                next_window_origin =
                        saturated_threshold_add(
                                next_window_origin,
                                prefetch_lead);
        }
        if (next_window_origin < 0) {
                return TC2_FAILURE_INTERNAL;
        }
        int selected =
                select_bounded_rolling_authority_window_for_leg(
                        &runtime->route, tail,
                        runtime->leg_start_offset,
                        runtime->authority_end_offset + 1,
                        job->target_route_offset,
                        job->destination_route_offset,
                        job->destination_offset_mm,
                        job->destination_distance_mm,
                        /*
                         * Include at least a second CAN/settle lead in the
                         * requested origin before requiring a complete stop.
                         * Command 120 keeps a third lead because one scheduler
                         * sensor rebase can consume most of the first lead
                         * before the next turnout batch is queued (the
                         * observed A->d3 pause at C9/C10).
                         *
                         * A single
                         * lead left only three millimetres between the A->d3
                         * re-arm point and its next prefetch trigger; the hard
                         * E7 observation could jump across that boundary and
                         * force speed zero before another scheduler cycle.
                         * The extra confirmation lead moves the local boundary
                         * far enough ahead to absorb a detector rebase while
                         * retaining one bounded braking window and releasing
                         * the tail normally.
                         */
                        selection_speed,
                        0,
                        next_window_origin,
                        job->train, reservation, sensors,
                        &extension, &new_end, &new_ceiling,
                        &selected_window_speed,
                        &conflict);
        /*
         * Near the destination, the requested-speed braking envelope can
         * extend beyond every remaining intermediate authority boundary.
         * The bounded selector then either offers a shorter low-speed prefix
         * or has no local prefix at all, even though the clear complete
         * destination footprint is the only window that can preserve the
         * already acknowledged command speed.  A moving train cannot safely
         * apply a lower-speed envelope retroactively: try the final suffix
         * instead.  This remains rolling authority because the released tail
         * is not reacquired, and the ordinary occupancy/reservation check can
         * still reject a shared or blocked suffix.
         */
        if (final_approach_suffix_required ||
            (selected == 0 &&
             selected_window_speed < selection_speed) ||
            (selected > 0 &&
             conflict.node_index < 0)) {
                track_route final_extension;
                final_extension.node_count = 0;
                final_extension.distance_mm = 0;
                final_extension.optimization_cost_mm = 0;
                final_extension.reversal_count = 0;
                track_reservation_conflict final_conflict;
                final_conflict.node_index = -1;
                final_conflict.owner_train = 0;
                int final_end = -1;
                int final_ceiling = -1;
                int final_window_origin =
                        saturated_threshold_add(
                                current_progress_mm,
                                prefetch_lead);
                if (final_window_origin < 0) {
                        return TC2_FAILURE_INTERNAL;
                }
                int final_status =
                        select_rolling_authority_window_for_leg(
                                &runtime->route, tail,
                                runtime->leg_start_offset,
                                runtime->route.node_count - 1,
                                job->target_route_offset,
                                job->destination_route_offset,
                                job->destination_offset_mm,
                                job->destination_distance_mm,
                                selection_speed,
                                final_window_origin,
                                job->train, reservation, sensors,
                                &final_extension, &final_end,
                                &final_ceiling,
                                &final_conflict);
                if (final_status < 0) {
                        return TC2_FAILURE_ROUTE;
                }
                if (final_status == 0 &&
                    final_end ==
                            runtime->route.node_count - 1) {
                        extension = final_extension;
                        new_end = final_end;
                        new_ceiling = final_ceiling;
                        selected_window_speed =
                                selection_speed;
                        selected = 0;
                        conflict.node_index = -1;
                        conflict.owner_train = 0;
                }
        }
        if (selected > 0) {
                job->conflict_train =
                        conflict.owner_train;
                job->conflict_node =
                        conflict.node_index;
                if (begin_authority_request(
                            slot, now,
                            extension.node_count > 0 ?
                                    &extension : 0) < 0) {
                        return TC2_FAILURE_INTERNAL;
                }
                return 0;
        }
        if (selected < 0 ||
            new_end <= runtime->authority_end_offset ||
            new_ceiling <=
                    runtime->authority_center_ceiling_mm) {
                return TC2_FAILURE_ROUTE;
        }
        /*
         * The bounded suffix is safe only at a lower command.  Do not enlarge
         * authority using that smaller envelope while the train is still
         * moving at the old acknowledged speed.  It will stop under the
         * already-owned ceiling; the stationary authority path can then grant
         * this same bounded window and relaunch at selected_window_speed.
         */
        if (selected_window_speed < selection_speed) {
                return 0;
        }

        if (begin_authority_request(
                    slot, now, &extension) < 0) {
                return TC2_FAILURE_INTERNAL;
        }
        int yield =
                authority_prefetch_must_yield(
                        slot, now, &extension, reservation);
        if (yield < 0) {
                return TC2_FAILURE_ROUTE;
        }
        if (yield > 0) {
                return 0;
        }

        unsigned int generation = 0;
        int update =
                TrackReservationServerUpdateWindow(
                        dispatch_reservation_tid,
                        &extension, job->train,
                        job->target_node,
                        job->plan_generation,
                        &conflict, &generation);
        if (update == 1) {
                job->conflict_train =
                        conflict.owner_train;
                job->conflict_node =
                        conflict.node_index;
                return 0;
        }
        if (update < 0 ||
            generation != job->plan_generation) {
                return TC2_FAILURE_RESERVATION;
        }

        runtime->safety_footprint = extension;
        runtime->authority_prefetch_active = 1;
        runtime->authority_prefetch_end_offset = new_end;
        runtime->authority_prefetch_center_ceiling_mm =
                new_ceiling;
        runtime->authority_prefetch_settle_at_tick = -1;
        /*
         * Ownership of this exact extension has committed.  Do not carry its
         * FIFO age into a different downstream turnout.
         */
        clear_authority_request(runtime);
        int turnout_token =
                queue_turnout_plan_range(
                        slot,
                        runtime->authority_end_offset,
                        new_end);
        if (turnout_token < 0 &&
            turnout_token != CAN_SEND_BUSY) {
                return TC2_FAILURE_TURNOUT;
        }
        runtime->turnout_queue_pending =
                turnout_token == CAN_SEND_BUSY;
        runtime->turnout_batch_token =
                turnout_token > 0 ?
                (unsigned int)turnout_token : 0;
        runtime->authority_prefetch_settle_at_tick =
                turnout_token == 0 ? now : -1;
        job->conflict_train = 0;
        job->conflict_node = -1;
        return 0;
}

/*
 * If CAN/turnout confirmation cannot finish before the active braking
 * boundary, release the unused pending suffix and stop under the old active
 * authority.  The normal stationary FIFO extension path may acquire it
 * again later; no unconfirmed turnout ever enlarges movement authority.
 */
static int abandon_authority_prefetch(int slot) {
        if (slot < 0 || slot >= TC2_DISPATCH_MAX_JOBS) {
                return -1;
        }
        tc2_dispatch_job_snapshot *job =
                &dispatch_jobs[slot];
        tc2_dispatch_runtime *runtime =
                &dispatch_runtime[slot];
        if (!runtime->authority_prefetch_active) return 0;
        if (runtime->turnout_batch_token != 0) {
                (void)CanCancelBatch(
                        dispatch_can_tid,
                        runtime->turnout_batch_token);
        }
        track_route active;
        int tail = footprint_tail_offset(runtime, job);
        int footprint_speed =
                traffic_effective_speed(slot);
        if (tail < 0 ||
            footprint_speed < 1 ||
            build_local_authority_footprint(
                    &runtime->route, tail,
                    runtime->authority_end_offset,
                    runtime->leg_start_offset,
                    job->destination_route_offset,
                    footprint_speed, &active) < 0) {
                return -1;
        }
        if ((runtime->carry_ambiguity_active &&
             append_route_nodes(
                     &active,
                     &runtime
                              ->carry_ambiguity_footprint) < 0) ||
            (runtime->reversal_carry_active &&
             append_route_nodes(
                     &active,
                     &runtime
                              ->reversal_carry_footprint) < 0)) {
                return -1;
        }
        track_reservation_conflict conflict;
        unsigned int generation = 0;
        int update =
                TrackReservationServerUpdateWindow(
                        dispatch_reservation_tid,
                        &active, job->train,
                        job->target_node,
                        job->plan_generation,
                        &conflict, &generation);
        if (update != 0 ||
            generation != job->plan_generation) {
                return -1;
        }
        runtime->safety_footprint = active;
        runtime->authority_prefetch_active = 0;
        runtime->authority_prefetch_end_offset = -1;
        runtime->authority_prefetch_center_ceiling_mm = -1;
        runtime->authority_prefetch_settle_at_tick = -1;
        runtime->turnout_batch_token = 0;
        runtime->turnout_queue_pending = 0;
        return 0;
}

/*
 * Fixed-siding direct dispatch treats detectors as ordered localization
 * evidence, not as mandatory motion gates.  A physical Track-D capture of
 * C->d1 contained A10, C7, D8, E8, C14: B7 was absent and both E11/D10 were
 * absent between C7 and D8.  The strict monitor correctly classifies D8 as
 * farther than its one-missing lookahead, but emergency-stopping there makes
 * the train coast to C14 instead of reaching d1.
 *
 * Recover only an attributed, generation-matching sensor occurrence that is
 * monotonically ahead on the already selected motion leg and inside the
 * currently usable authority.  The caller still applies the independent
 * fastest-possible travel-time check before committing this trial state.
 * Reservation release also remains conservative: the recovered observation
 * is marked MISSING_ONE, so ownership is retained until a subsequent ordered
 * detector corroborates progress.
 */
static int recover_forward_direct_sensor_observation(
        int slot, int sensor_node,
        tc2_route_monitor *trial,
        tc2_route_observation *observation) {
        if (slot < 0 || slot >= TC2_DISPATCH_MAX_JOBS ||
            !trial || !observation ||
            sensor_node < 0 || sensor_node >= TRACK_MAX) {
                return -1;
        }
        tc2_dispatch_job_snapshot *job =
                &dispatch_jobs[slot];
        tc2_dispatch_runtime *runtime =
                &dispatch_runtime[slot];
        if (!uses_measured_direct_stop(job) ||
            observation->classification !=
                    TC2_ROUTE_SENSOR_SPURIOUS ||
            dispatch_track[sensor_node].type != NODE_SENSOR ||
            runtime->monitor.confirmed_offset < 0 ||
            runtime->leg_end_offset <
                    runtime->monitor.confirmed_offset) {
                return 1;
        }

        int usable_end = runtime->leg_end_offset;
        if (runtime->rolling_authority_active &&
            runtime->authority_end_offset < usable_end) {
                usable_end = runtime->authority_end_offset;
        }
        int matched_offset = -1;
        int skipped_sensors = 0;
        for (int offset =
                     runtime->monitor.confirmed_offset + 1;
             offset <= usable_end; ++offset) {
                int node = runtime->route.nodes[offset];
                if (dispatch_track[node].type != NODE_SENSOR) {
                        continue;
                }
                if (node == sensor_node) {
                        matched_offset = offset;
                        break;
                }
                ++skipped_sensors;
        }
        /*
         * The ordinary monitor already owns the next and next-next cases.
         * Reach this recovery path only for a larger confirmed forward gap.
         */
        if (matched_offset < 0 || skipped_sensors < 2) {
                return 1;
        }

        int matched_distance = -1;
        if (Tc2RouteDistanceAtOffset(
                    dispatch_track, &runtime->route,
                    matched_offset, &matched_distance) < 0 ||
            matched_distance <
                    runtime->monitor.confirmed_distance_mm) {
                return -1;
        }
        observation->classification =
                TC2_ROUTE_SENSOR_MISSING_ONE;
        observation->missing = observation->expected;
        observation->matched.route_offset =
                matched_offset;
        observation->matched.node_index =
                sensor_node;
        observation->matched.distance_mm =
                matched_distance;
        observation->advance_mm =
                matched_distance -
                runtime->monitor.confirmed_distance_mm;
        trial->confirmed_offset = matched_offset;
        trial->confirmed_distance_mm = matched_distance;
        trial->missing_sensor_count = 1;
        return 0;
}

/*
 * A traffic stop endpoint is produced by the measured command-to-rest model;
 * it is not a second source of track position truth.  In particular, a train
 * may cross the next physical detector while the zero-speed command is still
 * coasting.  An ordered detector that is still inside the train's owned
 * authority must therefore rebase the resumable hold instead of being
 * mislabeled as a sensor-sequence failure.
 *
 * Keep this bounded by the independent motion uncertainty and by both forms
 * of authority.  A detector outside either bound remains fail-closed, so this
 * cannot turn a real reservation overrun into accepted progress.
 */
static int traffic_hold_sensor_rebase_is_safe(
        int slot, int matched_offset, int matched_node,
        int64_t sensor_distance_um) {
        if (slot < 0 || slot >= TC2_DISPATCH_MAX_JOBS ||
            matched_offset < 0 || matched_node < 0 ||
            matched_node >= TRACK_MAX || sensor_distance_um < 0) {
                return 0;
        }
        tc2_dispatch_job_snapshot *job = &dispatch_jobs[slot];
        tc2_dispatch_runtime *runtime = &dispatch_runtime[slot];
        if (runtime->stop_purpose != TC2_STOP_TRAFFIC ||
            runtime->traffic_hold_distance_um < 0 ||
            sensor_distance_um <=
                    runtime->traffic_hold_distance_um ||
            sensor_distance_um >
                    (int64_t)job->destination_distance_mm * 1000) {
                return 0;
        }
        int stop_speed =
                runtime->traffic_resume_speed > 0 ?
                        runtime->traffic_resume_speed :
                runtime->motion_speed_ceiling > 0 ?
                        runtime->motion_speed_ceiling :
                        job->speed;
        int uncertainty_mm = Tc2MotionUncertaintyMm(stop_speed);
        if (uncertainty_mm < 0 ||
            sensor_distance_um -
                            runtime->traffic_hold_distance_um >
                    (int64_t)uncertainty_mm * 1000) {
                return 0;
        }
        if (runtime->rolling_authority_active) {
                int owned_end = runtime->authority_prefetch_active ?
                        runtime->authority_prefetch_end_offset :
                        runtime->authority_end_offset;
                if (owned_end < 0 || matched_offset > owned_end) {
                        return 0;
                }
        }
        int reverse = physical_reverse_index(matched_node);
        return route_contains_node(
                           &runtime->safety_footprint,
                           matched_node) ||
                (reverse >= 0 &&
                 route_contains_node(
                         &runtime->safety_footprint, reverse));
}

static int process_sensor_observation(
        int slot, const train_sensor_attributed_event_t *event,
        int event_tick, int action_tick) {
        tc2_dispatch_job_snapshot *job = &dispatch_jobs[slot];
        tc2_dispatch_runtime *runtime = &dispatch_runtime[slot];
        tc2_route_observation observation;
        tc2_route_monitor trial = runtime->monitor;
        int prior_confirmed_offset =
                runtime->monitor.confirmed_offset;
        int preorigin_event = 0;
        int deadline_anchor_tick = event_tick;
        if (event_tick < 0 || action_tick < 0) return -1;
        runtime->last_journal_sequence = event->sequence;

        if (event->generation != job->plan_generation) {
                ++job->duplicate_sensor_count;
                return 0;
        }
        if (Tc2RouteMonitorObserve(
                    dispatch_track, &runtime->route, &trial,
                    event->sensor_index, event->sequence,
                    &observation) < 0) {
                return -1;
        }
        if (observation.classification != TC2_ROUTE_SENSOR_DUPLICATE &&
            runtime->preorigin_sensor_pending) {
                int origin_offset = runtime->leg_start_offset;
                int origin_node =
                        origin_offset >= 0 &&
                                origin_offset < runtime->route.node_count ?
                        runtime->route.nodes[origin_offset] : -1;
                int reverse_origin =
                        physical_reverse_index(origin_node);
                int origin_distance_mm;
                /*
                 * Do this before the ordinary NORMAL/MISSING/SPURIOUS
                 * returns.  A loop can contain the same physical detector
                 * later, and the generic monitor is initialized as if its
                 * origin were already occupied.  While the prefix is pending,
                 * only a fresh event from the exact physical origin may
                 * consume it; stale sequence tokens remain DUPLICATE below.
                 */
                if (origin_node >= 0 &&
                    dispatch_track[origin_node].type == NODE_SENSOR &&
                    (event->sensor_index == origin_node ||
                     event->sensor_index == reverse_origin) &&
                    Tc2RouteDistanceAtOffset(
                            dispatch_track, &runtime->route,
                            origin_offset,
                            &origin_distance_mm) == 0) {
                        observation.classification =
                                TC2_ROUTE_SENSOR_NORMAL;
                        observation.matched.route_offset =
                                origin_offset;
                        observation.matched.node_index =
                                origin_node;
                        observation.matched.distance_mm =
                                origin_distance_mm;
                        observation.advance_mm =
                                runtime->preorigin_distance_mm;
                        trial.confirmed_offset = origin_offset;
                        trial.confirmed_distance_mm =
                                origin_distance_mm;
                        trial.missing_sensor_count =
                                runtime->monitor.missing_sensor_count;
                        trial.last_observed_node = origin_node;
                        preorigin_event = 1;
                }
        }
        if (observation.classification == TC2_ROUTE_SENSOR_DUPLICATE) {
                ++job->duplicate_sensor_count;
                runtime->monitor.last_sequence = event->sequence;
                return 0;
        }
        if (observation.classification == TC2_ROUTE_SENSOR_SPURIOUS) {
                int recovered =
                        recover_forward_direct_sensor_observation(
                                slot, event->sensor_index,
                                &trial, &observation);
                if (recovered < 0) return -1;
                if (recovered == 0) {
                        /*
                         * Continue through the common elapsed-time,
                         * footprint, and deadline checks below.
                         */
                } else {
                runtime->monitor.last_sequence = event->sequence;
                if (runtime->carry_ambiguity_active &&
                    route_contains_node(
                            &runtime->carry_ambiguity_footprint,
                            event->sensor_index)) {
                        /*
                         * The physical train may still be catching up to
                         * the estimated CURRENT origin. Such an event never
                         * advances the new monitor or releases legacy
                         * ownership, so retaining it as non-progress
                         * evidence is safe and avoids charging a real
                         * pre-origin pulse as the one tolerated fault.
                         */
                        ++job->duplicate_sensor_count;
                        return 0;
                }
                ++job->spurious_sensor_count;
                return job->spurious_sensor_count >
                        TC2_MAX_SENSOR_SPURIOUS ? -1 : 0;
                }
        }

        /*
         * A batch can contain multiple events received at different times.
         * Plausibility must use each event's server arrival tick, not the
         * later ticker time at which this page happens to be drained.
         */
        if (runtime->last_confirmed_tick != -1 &&
            !tick_reached(
                    event_tick,
                    runtime->last_confirmed_tick)) {
                /*
                 * An attributed event received before this motion epoch (or
                 * out of order across the half-range) must never advance the
                 * route and release track behind a train that has not moved.
                 */
                return -1;
        }
        int elapsed =
                tick_age(event_tick, runtime->last_confirmed_tick);
        int motion_speed =
                runtime->speed_reduction_pending &&
                        runtime->motion_speed_ceiling > 0 ?
                        runtime->motion_speed_ceiling :
                runtime->command_speed > 0 ?
                        runtime->command_speed :
                        runtime->motion_speed_ceiling;
        if (motion_speed <= 0) motion_speed = job->speed;
        /*
         * The supervised provisional model is a point prediction for the
         * stop-command deadline, not a certified maximum train velocity.
         * Using its narrow +20% band here rejects physically valid reports
         * whenever pickup placement or the first measured run is faster
         * than that point estimate.  Sensor plausibility must retain the
         * independent conservative fast envelope used by every managed
         * trip; only schedule_motion_deadlines() may use the provisional
         * point model.
         */
        int earliest_ticks =
                Tc2MotionFastTravelTicks(
                        motion_speed,
                        observation.advance_mm);
        if (earliest_ticks < 0) return -1;
        if (runtime->last_confirmed_tick != -1 &&
            elapsed < earliest_ticks) {
                runtime->monitor.last_sequence = event->sequence;
                ++job->spurious_sensor_count;
                return job->spurious_sensor_count >
                        TC2_MAX_SENSOR_SPURIOUS ? -1 : 0;
        }

        if (!uses_measured_direct_stop(job) &&
            (job->provisional_prediction ||
             runtime->prediction_plan_valid) &&
            runtime->last_confirmed_tick != -1 &&
            elapsed > 0 && observation.advance_mm > 0) {
                int observed_velocity =
                        Tc2MotionObservedVelocityUmPerTick(
                                observation.advance_mm, elapsed);
                if (observed_velocity < 1) return -1;
                /*
                 * This is retained only for the legacy point-timing path.
                 * Measured direct dispatch deliberately never records this
                 * value: its sensors rebase position only, while the stable
                 * command-speed curve owns spatial progress and braking.
                 */
                runtime->observed_velocity_um_per_tick =
                        observed_velocity;
        }

        if (observation.classification ==
            TC2_ROUTE_SENSOR_MISSING_ONE) {
                ++job->missing_sensor_count;
                if (job->missing_sensor_count >
                            TC2_MAX_SENSOR_MISSING &&
                    !uses_measured_direct_stop(job)) {
                        return -1;
                }
                runtime->pending_missing_offset =
                        observation.matched.route_offset;
        } else {
                /*
                 * A single false report at the exact expected sensor is
                 * indistinguishable from a real one.  Release only through
                 * the previously confirmed sensor, after a second ordered
                 * observation corroborates progress.
                 */
                runtime->reservation_anchor_offset =
                        prior_confirmed_offset;
                runtime->pending_missing_offset = -1;
        }

        runtime->monitor = trial;
        if (preorigin_event) {
                runtime->preorigin_sensor_pending = 0;
                runtime->preorigin_distance_mm = 0;
                runtime->carry_preorigin_distance_mm = 0;
                runtime->preorigin_traffic_hold_distance_um = 0;
                runtime->preorigin_traffic_hold_valid = 0;
        }
        runtime->last_confirmed_tick = event_tick;
        job->current_route_offset =
                observation.matched.route_offset;
        job->current_node = observation.matched.node_index;
        job->confirmed_distance_mm =
                observation.matched.distance_mm;
        /*
         * The motion anchor is independent of the reservation anchor.  The
         * latter deliberately trails by one corroborated observation; the
         * former is the best monotonic scalar position used by distance
         * control, traffic spacing, and UI prediction.
         */
        runtime->motion_anchor_distance_um =
                (int64_t)observation.matched.distance_mm * 1000;
        runtime->motion_anchor_tick = event_tick;
        runtime->motion_anchor_from_traffic = 0;
        if (runtime->stop_purpose == TC2_STOP_TRAFFIC &&
            runtime->traffic_hold_distance_um <
                    runtime->motion_anchor_distance_um) {
                if (!traffic_hold_sensor_rebase_is_safe(
                            slot,
                            observation.matched.route_offset,
                            observation.matched.node_index,
                            runtime->motion_anchor_distance_um)) {
                        job->position_estimated = 1;
                        return -1;
                }
                runtime->traffic_hold_distance_um =
                        runtime->motion_anchor_distance_um;
                job->traffic_hold_distance_um =
                        runtime->traffic_hold_distance_um;
                if (job->estimated_distance_um <
                    runtime->motion_anchor_distance_um) {
                        job->estimated_distance_um =
                                runtime->motion_anchor_distance_um;
                }
        }
        int carried_before_update =
                runtime->carry_ambiguity_active;
        if (update_remaining_footprint(slot) < 0) return -1;
        int localized_now =
                carried_before_update &&
                !runtime->carry_ambiguity_active;
        if (runtime->speed_reduction_pending &&
            runtime->command_speed > 0) {
                /*
                 * A lower speed command is not an instantaneous physical
                 * speed bound. Keep the prior upper bound through the first
                 * independently ordered sensor, then use the new command
                 * for later-event plausibility.
                 */
                runtime->motion_speed_ceiling =
                        runtime->command_speed;
                runtime->speed_reduction_pending = 0;
        }

        if (!runtime->localization_only &&
            job->destination_offset_mm > 0 &&
            observation.matched.route_offset ==
                    job->target_route_offset) {
                /*
                 * This is the physical anchor, not the destination. Rebase
                 * the exact scalar motion model here and keep RUNNING for
                 * the configured downstream offset.
                 */
                runtime->target_seen = 1;
                runtime->target_corroborated = 1;
                job->position_estimated = 1;
                if (uses_precision_approach(job) &&
                    job->destination_offset_mm > 0) {
                        /*
                         * The final timer is rebased on the physical anchor.
                         * The train was already reduced to the measured
                         * precision speed before reaching this sensor.
                         */
                        runtime->approach_sent = 0;
                        deadline_anchor_tick = action_tick;
                }
                /*
                 * No sensor represents the interior-edge destination.
                 * Once its physical anchor is confirmed, the remaining
                 * progress is timer/model based rather than another
                 * expected sensor observation.
                 */
                job->next_sensor_node = -1;
        }

        if (!runtime->localization_only &&
            job->destination_offset_mm > 0 &&
            observation.matched.route_offset >=
                    job->destination_route_offset) {
                /*
                 * A sensor at or beyond the route-node ceiling proves the
                 * train passed the interior-edge virtual stop point. Keep
                 * the complete owned guard and fail closed.
                 */
                fail_job(slot, TC2_FAILURE_SENSOR_SEQUENCE);
                return -1;
        }

        if (!runtime->localization_only &&
            job->destination_offset_mm > 0 &&
            observation.matched.route_offset ==
                    job->target_route_offset) {
                /*
                 * The directed sensor is only the localization anchor for
                 * an interior lowercase destination.  Even when the graph
                 * represents that anchor as the current motion-leg end, it
                 * must not enter the reversal/destination stop branch below.
                 * Continue at precision speed and stop only after travelling
                 * the measured downstream offset.
                 */
                if (job->state == TC2_JOB_BRAKING) {
                        return 0;
                }
                if (schedule_motion_deadlines(
                            slot, deadline_anchor_tick) < 0) {
                        return -1;
                }
                return 0;
        }

        if (job->state == TC2_JOB_BRAKING &&
            runtime->stop_purpose == TC2_STOP_DESTINATION &&
            job->destination_offset_mm == 0 &&
            runtime->target_seen &&
            observation.matched.route_offset >
                    job->target_route_offset) {
                runtime->target_corroborated = 1;
                job->position_estimated = 0;
                return 0;
        }

        if (observation.matched.route_offset >=
            runtime->leg_end_offset) {
                if (runtime->leg_end_offset <
                    job->destination_route_offset) {
                        if (observation.classification ==
                            TC2_ROUTE_SENSOR_MISSING_ONE) {
                                fail_job(
                                        slot,
                                        TC2_FAILURE_SENSOR_SEQUENCE);
                                return -1;
                        }
                        if (observation.matched.route_offset !=
                            runtime->leg_end_offset) {
                                return -1;
                        }
                        runtime->target_seen = 1;
                        begin_stop(
                                slot, action_tick,
                                TC2_STOP_REVERSAL, 1,
                                TC2_STOP_TRIGGER_SENSOR);
                } else {
                        int estimated =
                                observation.matched.route_offset >
                                        job->destination_route_offset ||
                                observation.classification ==
                                        TC2_ROUTE_SENSOR_MISSING_ONE;
                        runtime->target_seen = !estimated;
                        if (uses_measured_direct_stop(job)) {
                                /*
                                 * Even when d5/d6 is co-located with its
                                 * detector, the detector is position
                                 * evidence only. The running distance-domain
                                 * check issues speed zero when the remaining
                                 * route distance equals the calibrated
                                 * command-to-rest distance.
                                 */
                                if (job->state ==
                                            TC2_JOB_BRAKING &&
                                    runtime->stop_purpose ==
                                            TC2_STOP_DESTINATION) {
                                        job->position_estimated =
                                                estimated;
                                        return 0;
                                }
                                if (schedule_motion_deadlines(
                                            slot,
                                            deadline_anchor_tick) < 0) {
                                        return -1;
                                }
                                return 0;
                        }
                        if (job->state == TC2_JOB_BRAKING &&
                            runtime->stop_purpose ==
                                    TC2_STOP_DESTINATION) {
                                job->position_estimated = estimated;
                                return 0;
                        }
                        begin_stop(slot, action_tick,
                                   TC2_STOP_DESTINATION,
                                   estimated,
                                   TC2_STOP_TRIGGER_SENSOR);
                }
                return job->state == TC2_JOB_FAILED ? -1 : 1;
        }
        if (job->state == TC2_JOB_BRAKING) {
                /*
                 * A timer-triggered destination stop can coast across
                 * ordinary sensors.  Progress still tightens the footprint,
                 * but command_speed is already zero and must never be fed
                 * back into the positive-speed deadline model.
                 */
                return 0;
        }
        if (localized_now &&
            runtime->command_speed != job->speed &&
            !runtime->precision_approach_active) {
                if (runtime->rolling_authority_active) {
                        int restored =
                                authority_restore_requested_speed(
                                        slot,
                                        deadline_anchor_tick);
                        if (restored < 0) return -1;
                        if (restored > 0) return 0;
                } else {
                        if (command_train_speed_with_retry(
                                    job->train, job->speed) < 0) {
                                return -1;
                        }
                        runtime->command_speed = job->speed;
                        job->command_speed = job->speed;
                        runtime->motion_speed_ceiling = job->speed;
                        runtime->speed_reduction_pending = 0;
                        runtime->precision_approach_active = 0;
                }
        }
        if (schedule_motion_deadlines(
                    slot, deadline_anchor_tick) < 0) {
                return -1;
        }
        return 0;
}

static int drain_sensor_journal(int slot, int now) {
        tc2_dispatch_job_snapshot *job = &dispatch_jobs[slot];
        tc2_dispatch_runtime *runtime = &dispatch_runtime[slot];
        for (int page = 0; page < 8; ++page) {
                train_sensor_event_batch_t batch;
                if (TrainSensorGetAttributedEvents(
                            dispatch_sensor_tid, job->train,
                            runtime->last_journal_sequence,
                            &batch) < 0) {
                        return -2;
                }
                if (batch.lost) {
                        job->journal_lost = 1;
                        return -1;
                }
                if (batch.count < 0 ||
                    batch.count > TRAIN_SENSOR_EVENT_BATCH_CAPACITY) {
                        return -2;
                }
                for (int event = 0; event < batch.count; ++event) {
                        int status = process_sensor_observation(
                                slot, &batch.events[event],
                                batch.events[event].arrival_tick, now);
                        if (status != 0) return status;
                }
                if (!batch.has_more || batch.count == 0) return 0;
        }
        return -2;
}

static int prelaunch_failure_reason(
        int slot, const track_reservation_snapshot *reservation,
        const train_sensor_snapshot_t *sensors, int now) {
        tc2_dispatch_job_snapshot *job = &dispatch_jobs[slot];
        tc2_dispatch_runtime *runtime = &dispatch_runtime[slot];

        int owns_footprint = 1;
        for (int offset = 0;
             offset < runtime->safety_footprint.node_count;
             ++offset) {
                int node =
                        runtime->safety_footprint.nodes[offset];
                int physical[2] = {
                        node, physical_reverse_index(node)
                };
                for (int side = 0; side < 2; ++side) {
                        int candidate = physical[side];
                        if (candidate >= 0 &&
                            candidate < TRACK_MAX &&
                            reservation
                                            ->owner_by_node[candidate] !=
                                    job->train) {
                                owns_footprint = 0;
                        }
                }
        }
        if (reservation->generation_by_train[job->train] !=
                    job->plan_generation ||
            reservation->destination_by_train[job->train] !=
                    job->target_node ||
            !owns_footprint) {
                return TC2_FAILURE_RESERVATION;
        }
        if (sensor_health_is_safe(
                    slot, sensors, 0, now) < 0) {
                return TC2_FAILURE_SENSOR_SERVICE;
        }
        if (sensors->attribution_unavailable_count !=
                    runtime->prepare_attribution_unavailable_count ||
            sensors->unattributed_count !=
                    runtime->prepare_unattributed_count ||
            !prelaunch_journal_is_clean(job, runtime) ||
            prelaunch_sensor_changed(runtime, sensors) >= 0) {
                return TC2_FAILURE_SENSOR_SEQUENCE;
        }
        return TC2_FAILURE_NONE;
}

static void launch_ready_jobs(int now) {
        int slots[TC2_DISPATCH_MAX_JOBS];
        int trains[TC2_DISPATCH_MAX_JOBS];
        int speeds[TC2_DISPATCH_MAX_JOBS];
        int count = 0;
        track_reservation_snapshot reservation;
        train_sensor_snapshot_t sensors;

        if (TrackReservationServerSnapshot(
                    dispatch_reservation_tid, &reservation) < 0 ||
            TrainSensorGetLatest(
                    dispatch_sensor_tid, &sensors) < 0) {
                for (int slot = 0;
                     slot < TC2_DISPATCH_MAX_JOBS; ++slot) {
                        if (dispatch_jobs[slot].state == TC2_JOB_READY &&
                            tick_reached(
                                    now,
                                    dispatch_jobs[slot]
                                            .ready_at_tick)) {
                                fail_job(
                                        slot,
                                        TC2_FAILURE_SENSOR_SERVICE);
                        }
                }
                return;
        }
        for (int slot = 0; slot < TC2_DISPATCH_MAX_JOBS; ++slot) {
                tc2_dispatch_job_snapshot *job = &dispatch_jobs[slot];
                tc2_dispatch_runtime *runtime = &dispatch_runtime[slot];
                if (job->state != TC2_JOB_READY ||
                    !tick_reached(now, job->ready_at_tick)) {
                        continue;
                }
                if (runtime->authority_resume_pending &&
                    runtime->prelaunch_traffic_blocked) {
                        continue;
                }
                if (!conflict_zone_step_ready(
                            slot,
                            runtime->conflict_zone_next_step)) {
                        continue;
                }
                int failure_reason =
                        prelaunch_failure_reason(
                                slot, &reservation, &sensors, now);
                if (failure_reason != TC2_FAILURE_NONE) {
                        fail_job(slot, failure_reason);
                        continue;
                }
                if (can_health_is_safe(
                            slot, 0, now) < 0) {
                        fail_job(slot, TC2_FAILURE_CAN_HEALTH);
                        continue;
                }
                slots[count] = slot;
                trains[count] = job->train;
                speeds[count] =
                        runtime->planned_launch_speed > 0 ?
                        runtime->planned_launch_speed :
                        job->speed;
                ++count;
        }
        if (count == 0) return;

        track_reservation_snapshot final_reservation;
        train_sensor_snapshot_t final_sensors;
        if (TrackReservationServerSnapshot(
                    dispatch_reservation_tid,
                    &final_reservation) < 0 ||
            TrainSensorGetLatest(
                    dispatch_sensor_tid,
                    &final_sensors) < 0) {
                for (int index = 0; index < count; ++index) {
                        fail_job(
                                slots[index],
                                TC2_FAILURE_SENSOR_SERVICE);
                }
                return;
        }
        for (int index = 0; index < count; ++index) {
                int slot = slots[index];
                int failure_reason =
                        prelaunch_failure_reason(
                                slot, &final_reservation,
                                &final_sensors, now);
                if (failure_reason != TC2_FAILURE_NONE) {
                        for (int failed = 0;
                             failed < count; ++failed) {
                                fail_job(
                                        slots[failed], failure_reason);
                        }
                        return;
                }
        }
        sensors = final_sensors;

        /*
         * One atomic queue request defines the launch wave.  The CAN server
         * then serializes the frames and waits for each CS3 response, as the
         * Märklin protocol requires, without making slot order change the
         * scheduler's admission/launch decision.
         */
        int batch_token = CanTrainSetSpeedBatch(
                dispatch_can_tid, trains, speeds, count);
        if (batch_token == CAN_SEND_BUSY) {
                return;
        }
        if (batch_token <= 0) {
                for (int index = 0; index < count; ++index) {
                        fail_job(
                                slots[index],
                                TC2_FAILURE_CAN_HEALTH);
                }
                return;
        }
        int launch_tick = now;

        for (int index = 0; index < count; ++index) {
                int slot = slots[index];
                tc2_dispatch_job_snapshot *job =
                        &dispatch_jobs[slot];
                tc2_dispatch_runtime *runtime =
                        &dispatch_runtime[slot];
                runtime->command_speed =
                        runtime->planned_launch_speed > 0 ?
                        runtime->planned_launch_speed :
                        job->speed;
                job->command_speed = runtime->command_speed;
                runtime->motion_speed_ceiling =
                        runtime->command_speed;
                runtime->speed_reduction_pending = 0;
                runtime->approach_sent = 0;
                runtime->launch_batch_token =
                        (unsigned int)batch_token;
                runtime->launch_batch_index = index;
                runtime->launch_wave_tick = launch_tick;
                job->launch_tick = -1;
                job->launch_skew_ticks = -1;
                job->launch_attributed_sequence =
                        sensors.attributed_count;
                runtime->last_journal_sequence =
                        job->launch_attributed_sequence;
                runtime->monitor.last_sequence =
                        job->launch_attributed_sequence;
                if (!runtime->authority_resume_pending) {
                        runtime->last_confirmed_tick = -1;
                }
                job->launch_attribution_unavailable_count =
                        sensors.attribution_unavailable_count;
                runtime->launch_train_attribution_failure =
                        sensors.attribution_failure_by_train[job->train];
                runtime->launch_unattributed_count =
                        sensors.unattributed_count;
                runtime->unattributed_spurious_tolerated = 0;
                job->ready_at_tick = -1;
                job->state = TC2_JOB_LAUNCHING;
        }
}

static void fail_launch_wave(unsigned int token, int reason) {
        if (token == 0) return;
        (void)CanCancelBatch(dispatch_can_tid, token);
        for (int slot = 0; slot < TC2_DISPATCH_MAX_JOBS; ++slot) {
                tc2_dispatch_runtime *runtime =
                        &dispatch_runtime[slot];
                int state = dispatch_jobs[slot].state;
                if (runtime->launch_batch_token != token ||
                    (state != TC2_JOB_LAUNCHING &&
                     state != TC2_JOB_RUNNING)) {
                        continue;
                }
                runtime->launch_batch_token = 0;
                fail_job(slot, reason);
        }
}

static void process_launching_jobs(int now) {
        train_sensor_snapshot_t sensors;
        int have_launching = 0;
        for (int slot = 0; slot < TC2_DISPATCH_MAX_JOBS; ++slot) {
                if (dispatch_jobs[slot].state ==
                    TC2_JOB_LAUNCHING) {
                        have_launching = 1;
                        break;
                }
        }
        if (!have_launching) return;
        if (TrainSensorGetLatest(
                    dispatch_sensor_tid, &sensors) < 0) {
                for (int slot = 0;
                     slot < TC2_DISPATCH_MAX_JOBS; ++slot) {
                        if (dispatch_jobs[slot].state ==
                            TC2_JOB_LAUNCHING) {
                                fail_launch_wave(
                                        dispatch_runtime[slot]
                                                .launch_batch_token,
                                        TC2_FAILURE_SENSOR_SERVICE);
                        }
                }
                return;
        }
        for (int slot = 0; slot < TC2_DISPATCH_MAX_JOBS; ++slot) {
                tc2_dispatch_job_snapshot *job =
                        &dispatch_jobs[slot];
                tc2_dispatch_runtime *runtime =
                        &dispatch_runtime[slot];
                if (job->state != TC2_JOB_LAUNCHING) continue;

                can_batch_status_t status;
                unsigned int token = runtime->launch_batch_token;
                if (runtime->authority_resume_pending &&
                    runtime->prelaunch_traffic_blocked) {
                        fail_launch_wave(
                                token,
                                TC2_FAILURE_RESERVATION);
                        continue;
                }
                if (token == 0 ||
                    CanGetBatchStatus(
                            dispatch_can_tid, token, &status) < 0 ||
                    status.total < 1 ||
                    runtime->launch_batch_index < 0 ||
                    runtime->launch_batch_index >= status.total) {
                        fail_launch_wave(
                                token, TC2_FAILURE_CAN_HEALTH);
                        continue;
                }
                if (sensor_health_is_safe(
                            slot, &sensors, 0, now) < 0) {
                        fail_launch_wave(
                                token, TC2_FAILURE_SENSOR_SERVICE);
                        continue;
                }
                if (can_health_is_safe(slot, 0, now) < 0) {
                        fail_launch_wave(
                                token, TC2_FAILURE_CAN_HEALTH);
                        continue;
                }
                if (status.state == CAN_BATCH_FAILED) {
                        fail_launch_wave(
                                token, TC2_FAILURE_CAN_HEALTH);
                        continue;
                }
                if (status.confirmed <=
                    runtime->launch_batch_index) {
                        continue;
                }

                job->state = TC2_JOB_RUNNING;
                job->launch_tick = now;
                job->launch_skew_ticks =
                        tick_age(
                                now,
                                runtime->launch_wave_tick);
                if (runtime->authority_resume_pending) {
                        if (runtime->traffic_hold_distance_um < 0) {
                                fail_launch_wave(
                                        token,
                                        TC2_FAILURE_INTERNAL);
                                continue;
                        }
                        runtime->motion_anchor_distance_um =
                                runtime->preorigin_sensor_pending &&
                                        runtime
                                                ->preorigin_traffic_hold_valid ?
                                runtime
                                        ->preorigin_traffic_hold_distance_um :
                                runtime
                                        ->traffic_hold_distance_um;
                        runtime->motion_anchor_tick = now;
                        runtime->motion_anchor_from_traffic = 1;
                        runtime->preorigin_traffic_hold_distance_um = 0;
                        runtime->preorigin_traffic_hold_valid = 0;
                        runtime->stop_purpose =
                                TC2_STOP_NONE;
                        runtime->traffic_reason =
                                TC2_TRAFFIC_NONE;
                        runtime->traffic_peer_train = 0;
                        runtime->traffic_hold_since_tick = -1;
                        /*
                         * A successful local extension consumed its ticket at
                         * reservation commit.  Keep the cleared state here so
                         * the next downstream window gets a new arrival order.
                         */
                        clear_authority_request(runtime);
                        runtime->authority_resume_pending = 0;
                        runtime->prelaunch_traffic_blocked = 0;
                        runtime->authority_previous_end_offset =
                                -1;
                        runtime->traffic_resume_speed = 0;
                        runtime->traffic_hold_distance_um = -1;
                        job->traffic_hold_active = 0;
                        job->traffic_reason =
                                TC2_TRAFFIC_NONE;
                        job->traffic_peer_train = 0;
                        job->traffic_gap_mm = -1;
                        job->traffic_stop_threshold_mm = -1;
                        job->traffic_resume_threshold_mm = -1;
                        job->traffic_hold_distance_um = -1;
                        job->hold_active = 0;
                } else {
                        int confirmed_distance_mm;
                        if (effective_confirmed_distance_mm(
                                    runtime,
                                    &confirmed_distance_mm) < 0) {
                                fail_launch_wave(
                                        token,
                                        TC2_FAILURE_INTERNAL);
                                continue;
                        }
                        runtime->last_confirmed_tick = now;
                        runtime->motion_anchor_distance_um =
                                (int64_t)confirmed_distance_mm * 1000;
                        runtime->motion_anchor_tick = now;
                        runtime->motion_anchor_from_traffic = 0;
                }
                if (schedule_motion_deadlines(
                            slot, now) < 0) {
                        fail_launch_wave(
                                token,
                                TC2_FAILURE_INTERNAL);
                        continue;
                }
                /*
                 * If confirmation was delayed, running processing below
                 * receives the real current tick and immediately catches an
                 * already-due braking/watchdog deadline.
                 */
                (void)now;
        }

        /*
         * Keep every confirmed member attached to its launch wave until all
         * members have passed their post-confirmation initialization.  If a
         * later member fails, fail_launch_wave() can then still stop the
         * earlier RUNNING members instead of leaving a partial batch moving.
         */
        for (int slot = 0;
             slot < TC2_DISPATCH_MAX_JOBS; ++slot) {
                unsigned int token =
                        dispatch_runtime[slot]
                                .launch_batch_token;
                if (token == 0 ||
                    dispatch_jobs[slot].state !=
                            TC2_JOB_RUNNING) {
                        continue;
                }
                int first_for_token = 1;
                int still_launching = 0;
                for (int peer = 0;
                     peer < TC2_DISPATCH_MAX_JOBS; ++peer) {
                        if (peer < slot &&
                            dispatch_runtime[peer]
                                            .launch_batch_token ==
                                    token) {
                                first_for_token = 0;
                                break;
                        }
                        if (dispatch_runtime[peer]
                                            .launch_batch_token ==
                                    token &&
                            dispatch_jobs[peer].state ==
                                    TC2_JOB_LAUNCHING) {
                                still_launching = 1;
                        }
                }
                if (!first_for_token || still_launching) {
                        continue;
                }
                can_batch_status_t status;
                if (CanGetBatchStatus(
                            dispatch_can_tid, token,
                            &status) < 0 ||
                    status.state != CAN_BATCH_COMPLETE) {
                        continue;
                }
                for (int member = 0;
                     member < TC2_DISPATCH_MAX_JOBS;
                     ++member) {
                        if (dispatch_runtime[member]
                                            .launch_batch_token ==
                                    token &&
                            dispatch_jobs[member].state ==
                                    TC2_JOB_RUNNING) {
                                dispatch_runtime[member]
                                        .launch_batch_token = 0;
                        }
                }
        }
}

static void process_preparing_jobs(int now) {
        track_reservation_snapshot reservation;
        train_sensor_snapshot_t sensors;
        int have_preparing = 0;
        for (int slot = 0; slot < TC2_DISPATCH_MAX_JOBS; ++slot) {
                if (dispatch_jobs[slot].state ==
                    TC2_JOB_PREPARING) {
                        have_preparing = 1;
                        break;
                }
        }
        if (!have_preparing) return;

        if (TrackReservationServerSnapshot(
                    dispatch_reservation_tid, &reservation) < 0 ||
            TrainSensorGetLatest(
                    dispatch_sensor_tid, &sensors) < 0) {
                for (int slot = 0;
                     slot < TC2_DISPATCH_MAX_JOBS; ++slot) {
                        if (dispatch_jobs[slot].state ==
                            TC2_JOB_PREPARING) {
                                fail_job(
                                        slot,
                                        TC2_FAILURE_SENSOR_SERVICE);
                        }
                }
                return;
        }

        /*
         * Every route owns its complete physical turnout resources before
         * its batch is queued. CAN can therefore serialize preparation for
         * several stationary jobs while unrelated trains keep moving.
         * Polling here keeps the dispatch server responsive to sensor and
         * watchdog ticks; no synchronous switch command blocks this task.
         */
        for (int slot = 0; slot < TC2_DISPATCH_MAX_JOBS; ++slot) {
                tc2_dispatch_job_snapshot *job =
                        &dispatch_jobs[slot];
                tc2_dispatch_runtime *runtime =
                        &dispatch_runtime[slot];
                if (job->state != TC2_JOB_PREPARING) continue;

                if (reservation.generation_by_train[job->train] !=
                            job->plan_generation) {
                        fail_job(slot, TC2_FAILURE_RESERVATION);
                        continue;
                }
                if (sensor_health_is_safe(
                            slot, &sensors, 0, now) < 0) {
                        fail_job(slot, TC2_FAILURE_SENSOR_SERVICE);
                        continue;
                }
                if (sensors.attribution_unavailable_count !=
                            runtime
                                    ->prepare_attribution_unavailable_count ||
                    sensors.unattributed_count !=
                            runtime->prepare_unattributed_count ||
                    !prelaunch_journal_is_clean(job, runtime) ||
                    prelaunch_sensor_changed(runtime, &sensors) >= 0) {
                        fail_job(slot, TC2_FAILURE_SENSOR_SEQUENCE);
                        continue;
                }
                if (can_health_is_safe(slot, 0, now) < 0) {
                        fail_job(slot, TC2_FAILURE_CAN_HEALTH);
                        continue;
                }

                if (runtime->turnout_queue_pending) {
                        int queued =
                                queue_turnout_plan_for_leg(slot);
                        if (queued == CAN_SEND_BUSY) {
                                continue;
                        }
                        if (queued < 0) {
                                fail_job(
                                        slot,
                                        TC2_FAILURE_TURNOUT);
                                continue;
                        }
                        runtime->turnout_queue_pending = 0;
                        runtime->turnout_batch_token =
                                queued > 0 ?
                                        (unsigned int)queued : 0;
                        if (queued > 0) continue;
                        if (runtime->prelaunch_reverse_pending) {
                                runtime
                                        ->prelaunch_turnout_settle_at_tick =
                                        now;
                        }
                }

                if (runtime->turnout_batch_token != 0) {
                        can_batch_status_t status;
                        if (CanGetBatchStatus(
                                    dispatch_can_tid,
                                    runtime->turnout_batch_token,
                                    &status) < 0 ||
                            status.token !=
                                    runtime->turnout_batch_token ||
                            status.total < 1) {
                                fail_job(
                                        slot,
                                        TC2_FAILURE_TURNOUT);
                                continue;
                        }
                        if (status.state == CAN_BATCH_FAILED) {
                                /*
                                 * Priority braking/reverse commands
                                 * deliberately preempt speculative turnout
                                 * traffic without incrementing CAN failure
                                 * health. Requeue the idempotent plan; a real
                                 * transport fault was already caught by the
                                 * health gate above.
                                 */
                                int retry =
                                        queue_turnout_plan_for_leg(
                                                slot);
                                if (retry == CAN_SEND_BUSY) {
                                        runtime
                                                ->turnout_batch_token = 0;
                                        runtime
                                                ->turnout_queue_pending = 1;
                                        continue;
                                }
                                if (retry < 0) {
                                        fail_job(
                                                slot,
                                                TC2_FAILURE_TURNOUT);
                                        continue;
                                }
                                runtime->turnout_batch_token =
                                        (unsigned int)retry;
                                if (retry != 0) continue;
                                if (runtime
                                            ->prelaunch_reverse_pending) {
                                        runtime
                                                ->prelaunch_turnout_settle_at_tick =
                                                now;
                                }
                        }
                        if (status.state != CAN_BATCH_COMPLETE ||
                            (runtime->turnout_batch_token != 0 &&
                             status.confirmed != status.total)) {
                                continue;
                        }
                        runtime->turnout_batch_token = 0;
                        if (runtime->prelaunch_reverse_pending) {
                                runtime
                                        ->prelaunch_turnout_settle_at_tick =
                                        tick_after(
                                                now,
                                                TC2_TURNOUT_SETTLE_TICKS);
                        }
                }
        }

        /*
         * A zero-physical-distance plan has no speed batch, but it remains
         * non-arrived until its turnout batch is confirmed and any leading
         * direction change has settled. Keep it out of READY promotion.
         */
        for (int slot = 0;
             slot < TC2_DISPATCH_MAX_JOBS; ++slot) {
                tc2_dispatch_job_snapshot *job =
                        &dispatch_jobs[slot];
                tc2_dispatch_runtime *runtime =
                        &dispatch_runtime[slot];
                if (job->state != TC2_JOB_PREPARING ||
                    !runtime->zero_motion_arrival_pending ||
                    runtime->turnout_queue_pending ||
                    runtime->turnout_batch_token != 0 ||
                    runtime->prelaunch_reverse_pending ||
                    (runtime
                                     ->prelaunch_reverse_ready_at_tick >=
                             0 &&
                     !tick_reached(
                             now,
                             runtime
                                     ->prelaunch_reverse_ready_at_tick)) ||
                    runtime
                            ->zero_motion_arrival_ready_at_tick < 0 ||
                    !tick_reached(
                            now,
                            runtime
                                    ->zero_motion_arrival_ready_at_tick)) {
                        continue;
                }
                if (runtime->carry_ambiguity_active) {
                        /*
                         * No motion evidence was created by the direction
                         * change. Retain every carried owner until a later
                         * trip supplies ordered localization evidence.
                         */
                        job->hold_active = 1;
                } else if (update_arrival_hold(slot) < 0) {
                        fail_job(
                                slot,
                                TC2_FAILURE_RESERVATION);
                        continue;
                }
                runtime->zero_motion_arrival_pending = 0;
                runtime->zero_motion_arrival_ready_at_tick = -1;
                job->ready_at_tick = -1;
                job->state = TC2_JOB_ARRIVED;
        }

        /*
         * A leading zero-distance reversal follows the same safe ordering as
         * an in-route reversal: reserve the outbound window, confirm and
         * mechanically settle every turnout, then reverse, wait for the
         * direction change to settle, and only afterwards enter READY.
         */
        for (int slot = 0;
             slot < TC2_DISPATCH_MAX_JOBS; ++slot) {
                tc2_dispatch_job_snapshot *job =
                        &dispatch_jobs[slot];
                tc2_dispatch_runtime *runtime =
                        &dispatch_runtime[slot];
                if (job->state != TC2_JOB_PREPARING ||
                    !runtime->prelaunch_reverse_pending ||
                    runtime->turnout_queue_pending ||
                    runtime->turnout_batch_token != 0 ||
                    runtime
                                    ->prelaunch_turnout_settle_at_tick <
                            0 ||
                    !tick_reached(
                            now,
                            runtime
                                    ->prelaunch_turnout_settle_at_tick)) {
                        continue;
                }
                if (CanTrainReversePriority(
                            dispatch_can_tid,
                            job->train) < 0) {
                        fail_job(
                                slot,
                                TC2_FAILURE_CAN_SERVICE);
                        continue;
                }
                int confirmed = Time();
                if (confirmed < 0) {
                        fail_job(
                                slot,
                                TC2_FAILURE_CAN_SERVICE);
                        continue;
                }
                runtime->prelaunch_reverse_pending = 0;
                runtime->prelaunch_turnout_settle_at_tick = -1;
                runtime->prelaunch_reverse_ready_at_tick =
                        tick_after(
                                confirmed,
                                TC2_REVERSE_SETTLE_TICKS);
                if (runtime
                            ->zero_motion_arrival_pending &&
                    !tick_reached(
                            runtime
                                    ->zero_motion_arrival_ready_at_tick,
                            runtime
                                    ->prelaunch_reverse_ready_at_tick)) {
                        runtime
                                ->zero_motion_arrival_ready_at_tick =
                                runtime
                                        ->prelaunch_reverse_ready_at_tick;
                }
        }

        /*
         * Jobs from one `go` epoch cross a common settle barrier after every
         * admitted turnout batch in that epoch has been confirmed. Waiting
         * jobs whose routes still conflict do not hold back the disjoint
         * group; they join a later admission wave when track is released.
         */
        for (int seed = 0;
             seed < TC2_DISPATCH_MAX_JOBS; ++seed) {
                if (dispatch_jobs[seed].state !=
                            TC2_JOB_PREPARING ||
                    !conflict_zone_step_ready(
                            seed,
                            dispatch_runtime[seed]
                                    .conflict_zone_next_step) ||
                    dispatch_runtime[seed]
                            .zero_motion_arrival_pending ||
                    dispatch_runtime[seed]
                            .prelaunch_reverse_pending ||
                    (dispatch_runtime[seed]
                                     .prelaunch_reverse_ready_at_tick >=
                             0 &&
                     !tick_reached(
                             now,
                             dispatch_runtime[seed]
                                     .prelaunch_reverse_ready_at_tick)) ||
                    dispatch_runtime[seed]
                            .turnout_queue_pending ||
                    dispatch_runtime[seed]
                            .turnout_batch_token != 0) {
                        continue;
                }
                unsigned int epoch =
                        dispatch_runtime[seed].launch_epoch;
                int pending = 0;
                for (int slot = 0;
                     slot < TC2_DISPATCH_MAX_JOBS; ++slot) {
                        if (dispatch_jobs[slot].state ==
                                    TC2_JOB_PREPARING &&
                            dispatch_runtime[slot].launch_epoch ==
                                    epoch &&
                            (dispatch_runtime[slot]
                                     .prelaunch_reverse_pending ||
                             (dispatch_runtime[slot]
                                              .prelaunch_reverse_ready_at_tick >=
                                      0 &&
                              !tick_reached(
                                      now,
                                      dispatch_runtime[slot]
                                              .prelaunch_reverse_ready_at_tick)) ||
                             dispatch_runtime[slot]
                                     .turnout_queue_pending ||
                             dispatch_runtime[slot]
                                     .turnout_batch_token != 0)) {
                                pending = 1;
                                break;
                        }
                }
                if (pending) continue;

                int ready_tick =
                        tick_after(
                                now,
                                TC2_TURNOUT_SETTLE_TICKS);
                for (int slot = 0;
                     slot < TC2_DISPATCH_MAX_JOBS; ++slot) {
                        tc2_dispatch_runtime *runtime =
                                &dispatch_runtime[slot];
                        if (dispatch_jobs[slot].state ==
                                    TC2_JOB_PREPARING &&
                            runtime->launch_epoch == epoch &&
                            runtime
                                    ->prelaunch_reverse_ready_at_tick >=
                                    0 &&
                            !tick_reached(
                                    ready_tick,
                                    runtime
                                            ->prelaunch_reverse_ready_at_tick)) {
                                ready_tick =
                                        runtime
                                                ->prelaunch_reverse_ready_at_tick;
                        }
                }
                int promoted = 0;
                for (int slot = 0;
                     slot < TC2_DISPATCH_MAX_JOBS; ++slot) {
                        tc2_dispatch_job_snapshot *job =
                                &dispatch_jobs[slot];
                        tc2_dispatch_runtime *runtime =
                                &dispatch_runtime[slot];
                        if (job->state != TC2_JOB_PREPARING ||
                            !conflict_zone_step_ready(
                                    slot,
                                    runtime
                                            ->conflict_zone_next_step) ||
                            runtime->zero_motion_arrival_pending ||
                            runtime->prelaunch_reverse_pending ||
                            (runtime
                                             ->prelaunch_reverse_ready_at_tick >=
                                     0 &&
                             !tick_reached(
                                     now,
                                     runtime
                                             ->prelaunch_reverse_ready_at_tick)) ||
                            runtime->launch_epoch != epoch ||
                            runtime->turnout_queue_pending ||
                            runtime->turnout_batch_token != 0) {
                                continue;
                        }
                        runtime->ready_wave_tick = ready_tick;
                        job->ready_at_tick = ready_tick;
                        job->state = TC2_JOB_READY;
                        promoted = 1;
                }
                if (promoted) {
                        dispatch_batch_ready_at_tick = ready_tick;
                }
        }
}

static void process_running_jobs(int now) {
        track_reservation_snapshot reservation;
        train_sensor_snapshot_t sensors;
        int have_running = 0;
        for (int slot = 0; slot < TC2_DISPATCH_MAX_JOBS; ++slot) {
                if (dispatch_jobs[slot].state == TC2_JOB_RUNNING ||
                    dispatch_jobs[slot].state == TC2_JOB_BRAKING) {
                        have_running = 1;
                        break;
                }
        }
        if (!have_running) return;
        if (TrackReservationServerSnapshot(
                    dispatch_reservation_tid, &reservation) < 0 ||
            TrainSensorGetLatest(dispatch_sensor_tid, &sensors) < 0) {
                for (int slot = 0; slot < TC2_DISPATCH_MAX_JOBS; ++slot) {
                        if (dispatch_jobs[slot].state ==
                                    TC2_JOB_RUNNING ||
                            dispatch_jobs[slot].state ==
                                    TC2_JOB_BRAKING) {
                                fail_job(slot, TC2_FAILURE_SENSOR_SERVICE);
                        }
                }
                return;
        }

        int unexpected_unattributed = 0;
        unsigned int unattributed_delta = 0;
        for (int slot = 0; slot < TC2_DISPATCH_MAX_JOBS; ++slot) {
                if ((dispatch_jobs[slot].state ==
                            TC2_JOB_RUNNING ||
                     dispatch_jobs[slot].state ==
                            TC2_JOB_BRAKING) &&
                    sensors.unattributed_count !=
                            dispatch_runtime[slot]
                                    .launch_unattributed_count) {
                        unsigned int delta =
                                sensors.unattributed_count -
                                dispatch_runtime[slot]
                                        .launch_unattributed_count;
                        if (delta != 1 ||
                            dispatch_runtime[slot]
                                    .unattributed_spurious_tolerated ||
                            dispatch_jobs[slot]
                                    .spurious_sensor_count >=
                                    TC2_MAX_SENSOR_SPURIOUS) {
                                unexpected_unattributed = 1;
                                break;
                        }
                        unattributed_delta = delta;
                }
        }
        if (!unexpected_unattributed && unattributed_delta == 1) {
                /*
                 * A single rising report outside every reservation is the
                 * one globally unattributed spurious event that TC2 must
                 * tolerate. Consume it for every train that was moving in
                 * this observation window; a second such event is no longer
                 * ambiguous and fails the whole active set closed.
                 */
                for (int slot = 0;
                     slot < TC2_DISPATCH_MAX_JOBS; ++slot) {
                        if (dispatch_jobs[slot].state !=
                                    TC2_JOB_RUNNING &&
                            dispatch_jobs[slot].state !=
                                    TC2_JOB_BRAKING) {
                                continue;
                        }
                        if (sensors.unattributed_count !=
                            dispatch_runtime[slot]
                                    .launch_unattributed_count) {
                                dispatch_runtime[slot]
                                        .launch_unattributed_count =
                                        sensors.unattributed_count;
                                dispatch_runtime[slot]
                                        .unattributed_spurious_tolerated =
                                        1;
                                ++dispatch_jobs[slot]
                                          .spurious_sensor_count;
                        }
                }
        }
        for (int slot = 0; slot < TC2_DISPATCH_MAX_JOBS; ++slot) {
                if ((dispatch_jobs[slot].state ==
                            TC2_JOB_RUNNING ||
                     dispatch_jobs[slot].state ==
                            TC2_JOB_BRAKING) &&
                    sensors.unattributed_count !=
                            dispatch_runtime[slot]
                                    .launch_unattributed_count) {
                        unexpected_unattributed = 1;
                        break;
                }
        }
        if (unexpected_unattributed) {
                for (int slot = 0;
                     slot < TC2_DISPATCH_MAX_JOBS; ++slot) {
                        if (dispatch_jobs[slot].state ==
                                    TC2_JOB_RUNNING ||
                            dispatch_jobs[slot].state ==
                                    TC2_JOB_BRAKING) {
                                fail_job(
                                        slot,
                                        TC2_FAILURE_SENSOR_SEQUENCE);
                        }
                }
                return;
        }

        for (int slot = 0; slot < TC2_DISPATCH_MAX_JOBS; ++slot) {
                tc2_dispatch_job_snapshot *job = &dispatch_jobs[slot];
                tc2_dispatch_runtime *runtime = &dispatch_runtime[slot];
                if (job->state != TC2_JOB_RUNNING &&
                    job->state != TC2_JOB_BRAKING) {
                        continue;
                }
                if (reservation.generation_by_train[job->train] !=
                            job->plan_generation) {
                        fail_job(slot, TC2_FAILURE_RESERVATION);
                        continue;
                }
                if (sensor_health_is_safe(
                            slot, &sensors, 0, now) < 0) {
                        fail_job(slot, TC2_FAILURE_SENSOR_SERVICE);
                        continue;
                }
                if (sensors.attribution_unavailable_count !=
                            job->launch_attribution_unavailable_count ||
                    sensors.attribution_failure_by_train[job->train] !=
                            runtime
                                    ->launch_train_attribution_failure) {
                        fail_job(slot, TC2_FAILURE_SENSOR_SEQUENCE);
                        continue;
                }
                if (can_health_is_safe(
                            slot, 0, now) < 0) {
                        fail_job(slot, TC2_FAILURE_CAN_HEALTH);
                        continue;
                }
                int journal = drain_sensor_journal(slot, now);
                if (journal < 0) {
                        fail_job(
                                slot,
                                journal == -1 ||
                                        job->journal_lost ?
                                TC2_FAILURE_SENSOR_SEQUENCE :
                                TC2_FAILURE_SENSOR_SERVICE);
                        continue;
                }
                if (job->state != TC2_JOB_RUNNING) continue;

                int prefetch_failure =
                        process_running_authority_prefetch(
                                slot, now,
                                &reservation, &sensors);
                if (prefetch_failure !=
                    TC2_FAILURE_NONE) {
                        fail_job(slot, prefetch_failure);
                        continue;
                }

                int zone_gate =
                        enforce_conflict_zone_gate(slot, now);
                if (zone_gate < 0) {
                        fail_job(slot, TC2_FAILURE_INTERNAL);
                        continue;
                }
                if (zone_gate > 0) continue;

                /*
                 * A partial rolling authority is a hard movement limit, not
                 * merely reservation metadata.  Issue speed zero while the
                 * complete conservative braking envelope still fits before
                 * the train-center ceiling.  begin_stop() records the
                 * measured coast endpoint for a monotonic UI hold; the
                 * larger envelope here is what keeps the physical body out
                 * of the unowned block.
                 */
                if (runtime->rolling_authority_active &&
                    runtime->authority_end_offset >= 0 &&
                    runtime->authority_end_offset <
                            runtime->route.node_count - 1 &&
                    !(runtime->leg_end_offset <
                                      job
                                              ->destination_route_offset &&
                      runtime->authority_end_offset >=
                              runtime->leg_end_offset)) {
                        int64_t progress_um;
                        int braking_envelope =
                                traffic_braking_envelope_mm(
                                        slot);
                        int64_t ceiling_um =
                                (int64_t)runtime
                                        ->authority_center_ceiling_mm *
                                1000;
                        if (runtime
                                            ->authority_center_ceiling_mm <
                                    0 ||
                            braking_envelope < 0 ||
                            runtime_motion_progress_um(
                                    slot, now,
                                    &progress_um) < 0 ||
                            progress_um < 0 ||
                            progress_um >
                                    INT64_MAX -
                                    (int64_t)braking_envelope *
                                            1000) {
                                fail_job(
                                        slot,
                                        TC2_FAILURE_INTERNAL);
                                continue;
                        }
                        if (progress_um +
                                    (int64_t)braking_envelope *
                                            1000 >=
                            ceiling_um) {
                                if (abandon_authority_prefetch(
                                            slot) < 0) {
                                        fail_job(
                                                slot,
                                                TC2_FAILURE_RESERVATION);
                                        continue;
                                }
                                int64_t gap_um =
                                        ceiling_um -
                                        progress_um;
                                int gap_mm =
                                        gap_um > 0 ?
                                        (int)((gap_um + 999) /
                                              1000) : 0;
                                begin_traffic_stop(
                                        slot, now,
                                        TC2_TRAFFIC_AUTHORITY,
                                        job->conflict_train,
                                        gap_mm,
                                        braking_envelope, -1);
                                if (job->state ==
                                            TC2_JOB_BRAKING &&
                                    runtime->stop_purpose ==
                                            TC2_STOP_TRAFFIC &&
                                    runtime->traffic_reason ==
                                            TC2_TRAFFIC_AUTHORITY &&
                                    runtime
                                                    ->authority_request_tick <
                                            0) {
                                        (void)begin_authority_request(
                                                slot, now, 0);
                                }
                                continue;
                        }
                }

                int direct_final_leg =
                        uses_measured_direct_stop(job) &&
                        runtime->leg_end_offset ==
                                job->destination_route_offset;
                if (direct_final_leg &&
                    !runtime->approach_sent) {
                        int64_t progress_um;
                        int64_t command_um;
                        if (measured_direct_progress_um(
                                    slot, now, &progress_um,
                                    &command_um) < 0) {
                                fail_job(
                                        slot,
                                        TC2_FAILURE_INTERNAL);
                                continue;
                        }
                        job->estimated_distance_um =
                                progress_um;
                        int64_t remaining_um =
                                (int64_t)job
                                        ->destination_distance_mm *
                                        1000 -
                                progress_um;
                        job->remaining_distance_mm =
                                remaining_um > 0 ?
                                (int)((remaining_um + 999) /
                                      1000) : 0;
                        if (progress_um >= command_um) {
                                /*
                                 * A virtual lowercase destination is
                                 * localized from its final directed sensor.
                                 * At high speed the calibrated braking point
                                 * can occur before that sensor (notably
                                 * A->d8 before D11).  Never turn that model
                                 * crossing into a blind stop.  Reduce to the
                                 * measured crawl speed, keep the job RUNNING,
                                 * and let the physical anchor rebase the
                                 * remaining destination offset.  If the
                                 * sensor never arrives, the existing
                                 * sensor/model watchdog only marks the
                                 * position estimated; turnout authority and
                                 * live traffic safety remain the mechanisms
                                 * that can stop motion.
                                 */
                                if (uses_final_sensor_crawl(job) &&
                                    !runtime->target_seen) {
                                        if (runtime->command_speed >
                                                    TC2_PRECISION_APPROACH_SPEED &&
                                            (reduce_for_anchor_stage(
                                                     slot,
                                                     TC2_PRECISION_APPROACH_SPEED) <
                                                     0 ||
                                             schedule_motion_deadlines(
                                                     slot, now) < 0)) {
                                                fail_job(
                                                        slot,
                                                        TC2_FAILURE_CAN_SERVICE);
                                        }
                                        continue;
                                }
                                runtime->approach_sent = 1;
                                begin_stop(
                                        slot, now,
                                        TC2_STOP_DESTINATION, 1,
                                        TC2_STOP_TRIGGER_TIMER);
                                continue;
                        }

                        /*
                         * Publish the next distance crossing for inspection
                         * and watchdog diagnostics. Correctness comes from
                         * the progress comparison above on every scheduler
                         * tick, not from this timestamp.
                         */
                        int velocity =
                                measured_direct_velocity_um_per_tick(
                                        job,
                                        runtime->command_speed);
                        int64_t until_command_um =
                                command_um - progress_um;
                        if (velocity < 1 ||
                            until_command_um < 0 ||
                            until_command_um > 0x7fffffff) {
                                fail_job(
                                        slot,
                                        TC2_FAILURE_INTERNAL);
                                continue;
                        }
                        int ticks =
                                Tc2MotionTravelTicksUmAtVelocity(
                                        velocity,
                                        (int)until_command_um);
                        if (ticks < 0) {
                                fail_job(
                                        slot,
                                        TC2_FAILURE_INTERNAL);
                                continue;
                        }
                        job->braking_at_tick =
                                tick_after(now, ticks);
                        job->prediction_velocity_um_per_tick =
                                velocity;
                        job->prediction_anchor_tick = now;
                        job->prediction_anchor_distance_mm =
                                (int)((progress_um + 999) / 1000);
                        job->prediction_command_at_tick =
                                job->braking_at_tick;
                        job->prediction_timing_valid = 1;
                }

                if (!direct_final_leg &&
                    !runtime->approach_sent &&
                    tick_reached(now, job->braking_at_tick)) {
                        int purpose =
                                runtime->leg_end_offset <
                                                job
                                                        ->destination_route_offset ?
                                        TC2_STOP_REVERSAL :
                                        TC2_STOP_DESTINATION;
                        if (purpose == TC2_STOP_DESTINATION &&
                            uses_precision_approach(job) &&
                            !runtime->target_seen &&
                            runtime->command_speed >
                                    TC2_PRECISION_APPROACH_SPEED) {
                                /*
                                 * Do not turn an unobserved point estimate
                                 * into a final stop. Slow enough to acquire
                                 * the physical anchor, then let the sensor
                                 * handler rebase the crawl timer.
                                 */
                                if (reduce_for_anchor_stage(
                                            slot,
                                            TC2_PRECISION_APPROACH_SPEED) < 0 ||
                                    schedule_motion_deadlines(
                                            slot, now) < 0) {
                                        fail_job(
                                                slot,
                                                TC2_FAILURE_CAN_SERVICE);
                                        continue;
                                }
                                continue;
                        }
                        runtime->approach_sent = 1;
                        if (purpose == TC2_STOP_REVERSAL) {
                                /*
                                 * At this fast-bound deadline the train is
                                 * at most its calibrated stopping distance
                                 * before the reversal sensor. Command zero
                                 * now. The larger reserved braking guard
                                 * includes model uncertainty beyond that
                                 * sensor, while observations remain live
                                 * during coast and can still prove the
                                 * reversal point before the speed-derived
                                 * settle deadline. Starting a second timer
                                 * from the old sensor would double-count
                                 * distance and is forbidden.
                                 */
                                begin_stop(
                                        slot, now,
                                        TC2_STOP_REVERSAL, 1,
                                        TC2_STOP_TRIGGER_TIMER);
                        } else {
                                begin_stop(
                                        slot, now, purpose, 1,
                                        TC2_STOP_TRIGGER_TIMER);
                        }
                        continue;
                }
                if (job->watchdog_at_tick >= 0 &&
                    tick_reached(now, job->watchdog_at_tick)) {
                        /*
                         * A detector deadline is localization evidence, not
                         * movement authority.  In particular, next-next is
                         * only an opportunistic turnout pre-switch and a
                         * late detector must not manufacture a zero-speed
                         * command when the current hard turnout is owned and
                         * traffic safety reports no conflict.
                         *
                         * Keep the route, destination, command speed, hard
                         * turnout ticket, and reservation intact.  Mark the
                         * position estimated and disarm this one prediction;
                         * the next ordered physical detector rebases motion
                         * and schedule_motion_deadlines() arms a fresh
                         * diagnostic deadline.  Real authority, next-turnout
                         * FIFO, head-on/following safety, CAN health, and the
                         * destination controller continue to fail/stop by
                         * their own independent checks above.
                         */
                        job->position_estimated = 1;
                        job->watchdog_at_tick = -1;
                }
        }
}

static int accept_unattributed_for_job(
        int slot,
        const train_sensor_snapshot_t *sensors) {
        tc2_dispatch_job_snapshot *job =
                &dispatch_jobs[slot];
        tc2_dispatch_runtime *runtime =
                &dispatch_runtime[slot];
        unsigned int delta =
                sensors->unattributed_count -
                runtime->launch_unattributed_count;
        if (delta == 0) return 0;
        if (delta != 1 ||
            runtime->unattributed_spurious_tolerated ||
            job->spurious_sensor_count >=
                    TC2_MAX_SENSOR_SPURIOUS) {
                return -1;
        }
        runtime->launch_unattributed_count =
                sensors->unattributed_count;
        runtime->unattributed_spurious_tolerated = 1;
        ++job->spurious_sensor_count;
        return 0;
}

static int reversing_preflight_failure(int slot, int now) {
        tc2_dispatch_job_snapshot *job =
                &dispatch_jobs[slot];
        tc2_dispatch_runtime *runtime =
                &dispatch_runtime[slot];
        track_reservation_snapshot reservation;
        train_sensor_snapshot_t sensors;
        if (TrackReservationServerSnapshot(
                    dispatch_reservation_tid,
                    &reservation) < 0 ||
            TrainSensorGetLatest(
                    dispatch_sensor_tid, &sensors) < 0) {
                return TC2_FAILURE_SENSOR_SERVICE;
        }
        if (reservation.generation_by_train[job->train] !=
                    job->plan_generation) {
                return TC2_FAILURE_RESERVATION;
        }
        if (sensor_health_is_safe(
                    slot, &sensors, 0, now) < 0) {
                return TC2_FAILURE_SENSOR_SERVICE;
        }
        if (sensors.attribution_unavailable_count !=
                    job->launch_attribution_unavailable_count ||
            sensors.attribution_failure_by_train[job->train] !=
                    runtime->launch_train_attribution_failure ||
            accept_unattributed_for_job(slot, &sensors) < 0 ||
            !journal_is_clean_after(
                    job->train,
                    runtime->last_journal_sequence)) {
                return TC2_FAILURE_SENSOR_SEQUENCE;
        }
        if (can_health_is_safe(slot, 0, now) < 0) {
                return TC2_FAILURE_CAN_HEALTH;
        }
        return TC2_FAILURE_NONE;
}

static int restage_after_localization(int slot, int now) {
        tc2_dispatch_job_snapshot *job =
                &dispatch_jobs[slot];
        if (job->provisional_prediction) return -1;
        tc2_dispatch_request request;
        int train = job->train;
        int speed = job->speed;
        int destination = job->destination_index;
        int force_reverse_first =
                dispatch_runtime[slot].force_reverse_first;
        int localization_hops =
                dispatch_runtime[slot].localization_hops + 1;
        unsigned int launch_epoch =
                job->job_launch_epoch;
        clear_request(&request, TC2_DISPATCH_MSG_STAGE);
        request.train = train;
        request.start_index =
                TC2_DISPATCH_START_CURRENT;
        request.speed = speed;
        request.destination_index = destination;
        request.reserved =
                force_reverse_first ?
                TC2_DISPATCH_REQUEST_FORCE_REVERSE_FIRST : 0;
        if (localization_hops > TC2_MAX_LOCALIZATION_HOPS ||
            stage_job(&request) < 0) return -1;
        int restaged = find_train_job(train);
        if (restaged != slot) return -1;
        dispatch_jobs[slot].state = TC2_JOB_WAITING;
        dispatch_jobs[slot].queue_sequence =
                ++dispatch_queue_sequence;
        dispatch_jobs[slot].job_launch_epoch =
                launch_epoch;
        dispatch_runtime[slot].queue_sequence =
                dispatch_jobs[slot].queue_sequence;
        dispatch_runtime[slot].wait_since_tick = now;
        dispatch_runtime[slot].launch_epoch =
                launch_epoch;
        dispatch_runtime[slot].localization_hops =
                localization_hops;
        return 0;
}

static void process_braking_and_reversing(int now) {
        for (int slot = 0; slot < TC2_DISPATCH_MAX_JOBS; ++slot) {
                tc2_dispatch_job_snapshot *job = &dispatch_jobs[slot];
                tc2_dispatch_runtime *runtime = &dispatch_runtime[slot];
                if (job->state == TC2_JOB_BRAKING &&
                    runtime->emergency_stop_token != 0) {
                        can_batch_status_t status;
                        int result = CanGetBatchStatus(
                                dispatch_can_tid,
                                runtime->emergency_stop_token,
                                &status);
                        /*
                         * A status-query timeout says nothing about whether
                         * CS3 already accepted the queued zero.  Preserve the
                         * token and poll it again instead of issuing duplicate
                         * emergency stops on every dispatcher tick.
                         */
                        if (result < 0) {
                                continue;
                        }
                        if (result == 0 &&
                            status.token ==
                                    runtime->emergency_stop_token &&
                            status.state == CAN_BATCH_PENDING) {
                                continue;
                        }
                        if (result == 0 &&
                            status.token ==
                                    runtime->emergency_stop_token &&
                            status.state == CAN_BATCH_COMPLETE) {
                                int stop_speed =
                                        runtime->motion_speed_ceiling > 0 ?
                                        runtime->motion_speed_ceiling :
                                        job->speed;
                                int settle_ticks =
                                        Tc2MotionStopSettleTicks(
                                                stop_speed);
                                if (settle_ticks < 0) {
                                        fail_job(
                                                slot,
                                                TC2_FAILURE_INTERNAL);
                                        continue;
                                }
                                runtime->emergency_stop_token = 0;
                                runtime->settle_at_tick =
                                        tick_after(now, settle_ticks);
                                job->stop_confirmed_tick = now;
                                continue;
                        }

                        /* Keep the trip stopped and retry only the zero. */
                        int train = job->train;
                        int token = CanTrainEmergencyStopBatch(
                                dispatch_can_tid, &train, 1);
                        if (token > 0) {
                                runtime->emergency_stop_token =
                                        (unsigned int)token;
                        }
                        continue;
                }
                if (job->state == TC2_JOB_CANCEL_BRAKING &&
                    tick_reached(now, runtime->settle_at_tick)) {
                        job->state = TC2_JOB_STOPPED;
                        job->stop_sent_tick = now;
                } else if (job->state == TC2_JOB_BRAKING &&
                    tick_reached(now, runtime->settle_at_tick)) {
                        if (runtime->stop_purpose ==
                            TC2_STOP_DESTINATION) {
                                if (!runtime->localization_only &&
                                    job->destination_offset_mm > 0) {
                                        /*
                                         * No sensor exists at an
                                         * interior-edge virtual point.
                                         * Arrival is therefore the
                                         * calibrated timer result after a
                                         * real anchor observation, and is
                                         * explicitly estimated.  The
                                         * rolling route/window is no longer
                                         * movement authority once the train
                                         * is stationary: atomically replace
                                         * it with the conservative endpoint
                                         * hold.  Keeping the old corridor
                                         * here leaves already-cleared
                                         * turnouts (notably SW12) owned
                                         * forever and starves a FIFO waiter.
                                         */
                                        if (job->stop_trigger !=
                                                    TC2_STOP_TRIGGER_TIMER ||
                                            !runtime->target_seen) {
                                                job->position_estimated = 1;
                                                job->hold_active = 1;
                                                job->state =
                                                        TC2_JOB_STOP_UNCONFIRMED;
                                                continue;
                                        }
                                        job->position_estimated = 1;
                                        if (update_arrival_hold(slot) < 0) {
                                                fail_job(
                                                        slot,
                                                        TC2_FAILURE_RESERVATION);
                                                continue;
                                        }
                                        job->state = TC2_JOB_ARRIVED;
                                        job->remaining_distance_mm = 0;
                                        job->next_sensor_node = -1;
                                        continue;
                                }
                                /*
                                 * A braking timer is only a conservative
                                 * command deadline; it is not evidence that
                                 * the train reached the destination.  If
                                 * the last confirmed route point is still
                                 * before the selected destination, retain
                                 * the complete remaining reservation and
                                 * the last confirmed progress/next sensor.
                                 * The operator must use cancel/remove
                                 * before the train or reservation can leave
                                 * this fail-closed hold.
                                 *
                                 * Progress at or beyond the target keeps
                                 * the existing estimated-arrival recovery:
                                 * it covers an exact target pulse awaiting
                                 * corroboration and the one-missing-sensor
                                 * case where an ordered guard observation
                                 * already proved passage beyond the target.
                                 */
                                if (job->current_route_offset <
                                    job->target_route_offset) {
                                        job->position_estimated = 1;
                                        job->hold_active = 1;
                                        job->state =
                                                TC2_JOB_STOP_UNCONFIRMED;
                                        continue;
                                }
                                int hold_status;
                                int continue_after_localization =
                                        runtime
                                                ->localization_only;
                                if (!runtime->target_corroborated) {
                                        /*
                                         * An expected target report can be
                                         * the single false sensor report.
                                         * Until a later ordered sensor has
                                         * corroborated it, retain the
                                         * anchor-based remaining route. A
                                         * CURRENT continuation must carry
                                         * this complete ambiguity into its
                                         * next atomic plan replacement.
                                         */
                                        job->position_estimated = 1;
                                        hold_status =
                                                update_remaining_footprint(
                                                        slot);
                                        if (hold_status == 0) {
                                                job->hold_active = 1;
                                        }
                                } else {
                                        hold_status =
                                                update_arrival_hold(slot);
                                }
                                if (hold_status < 0) {
                                        fail_job(
                                                slot,
                                                TC2_FAILURE_RESERVATION);
                                } else {
                                        job->state = TC2_JOB_ARRIVED;
                                        job->remaining_distance_mm = 0;
                                        job->next_sensor_node = -1;
                                        if (continue_after_localization &&
                                            restage_after_localization(
                                                    slot, now) < 0) {
                                                fail_job(
                                                        slot,
                                                        TC2_FAILURE_ROUTE);
                                        }
                                }
                        } else if (runtime->stop_purpose ==
                                   TC2_STOP_TRAFFIC) {
                                /*
                                 * Keep the exact route reservation and the
                                 * destination controller alive.  Unlike an
                                 * ARRIVED hold, this stop is resumable and
                                 * must freeze at the modeled coast endpoint
                                 * rather than jump back to the last sensor.
                                 */
                                if (runtime
                                            ->traffic_hold_distance_um <
                                    0) {
                                        fail_job(
                                                slot,
                                                TC2_FAILURE_INTERNAL);
                                        continue;
                                }
                                runtime
                                        ->motion_anchor_distance_um =
                                        runtime->preorigin_sensor_pending &&
                                                runtime
                                                        ->preorigin_traffic_hold_valid ?
                                        runtime
                                                ->preorigin_traffic_hold_distance_um :
                                        runtime
                                                ->traffic_hold_distance_um;
                                runtime->motion_anchor_tick = now;
                                runtime->motion_anchor_from_traffic = 1;
                                runtime->traffic_hold_since_tick =
                                        now;
                                job->estimated_distance_um =
                                        runtime->preorigin_sensor_pending ?
                                        0 : runtime
                                                ->motion_anchor_distance_um;
                                int64_t remaining_um =
                                        (int64_t)job
                                                ->destination_distance_mm *
                                                1000 -
                                        job->estimated_distance_um;
                                job->remaining_distance_mm =
                                        remaining_um > 0 ?
                                        (int)((remaining_um + 999) /
                                              1000) : 0;
                                job->hold_active = 1;
                                job->traffic_hold_active = 1;
                                job->state =
                                        TC2_JOB_TRAFFIC_HOLD;
                        } else if (runtime->stop_purpose ==
                                   TC2_STOP_REVERSAL) {
                                if (!runtime->target_seen ||
                                    !next_leg_turnouts_clear_of_uncertain_train(
                                            slot)) {
                                        fail_job(
                                                slot,
                                                TC2_FAILURE_SENSOR_SEQUENCE);
                                        continue;
                                }
                                if (runtime
                                            ->rolling_authority_active) {
                                        int acquired =
                                                acquire_reversal_authority(
                                                        slot,
                                                        now);
                                        if (acquired > 0) {
                                                /*
                                                 * Speed is already zero and
                                                 * the complete incoming
                                                 * authority remains owned.
                                                 * Retry in FIFO order without
                                                 * touching a turnout or the
                                                 * direction bit.
                                                 */
                                                continue;
                                        }
                                        if (acquired < 0) {
                                                fail_job(
                                                        slot,
                                                        TC2_FAILURE_RESERVATION);
                                                continue;
                                        }
                                } else {
                                        int next_leg =
                                                runtime
                                                        ->leg_end_offset +
                                                1;
                                        if (next_leg >=
                                                    runtime
                                                            ->route
                                                            .node_count ||
                                            dispatch_track[
                                                runtime
                                                        ->route
                                                        .nodes[
                                                            runtime
                                                                    ->leg_end_offset]]
                                                            .reverse !=
                                                    &dispatch_track[
                                                        runtime
                                                                ->route
                                                                .nodes[
                                                                    next_leg]]) {
                                                fail_job(
                                                        slot,
                                                        TC2_FAILURE_ROUTE);
                                                continue;
                                        }
                                        runtime->leg_start_offset =
                                                next_leg;
                                        ++job->reversals_completed;
                                        ++job->current_leg;
                                        if (init_leg_monitor(
                                                    slot,
                                                    runtime
                                                            ->last_journal_sequence) <
                                            0 ||
                                            update_remaining_footprint(
                                                    slot) < 0) {
                                                fail_job(
                                                        slot,
                                                        TC2_FAILURE_TURNOUT);
                                                continue;
                                        }
                                }
                                int turnout_token =
                                        queue_turnout_plan_for_leg(slot);
                                if (turnout_token < 0 &&
                                    turnout_token !=
                                            CAN_SEND_BUSY) {
                                        fail_job(
                                                slot,
                                                TC2_FAILURE_TURNOUT);
                                        continue;
                                }
                                runtime->turnout_queue_pending =
                                        turnout_token ==
                                                CAN_SEND_BUSY;
                                runtime->turnout_batch_token =
                                        turnout_token > 0 ?
                                                (unsigned int)
                                                turnout_token :
                                                0;
                                runtime->reverse_ready_at_tick =
                                        turnout_token == 0 ?
                                        now :
                                        -1;
                                runtime->reversal_phase =
                                        TC2_REVERSAL_PHASE_TURNOUT;
                                runtime->stop_purpose = TC2_STOP_NONE;
                                job->state = TC2_JOB_REVERSING;
                        } else {
                                fail_job(slot, TC2_FAILURE_INTERNAL);
                        }
                } else if (job->state == TC2_JOB_REVERSING) {
                        int preflight_failure =
                                reversing_preflight_failure(
                                        slot, now);
                        if (preflight_failure != TC2_FAILURE_NONE) {
                                fail_job(slot, preflight_failure);
                                continue;
                        }
                        if (runtime->turnout_queue_pending) {
                                int queued =
                                        queue_turnout_plan_for_leg(
                                                slot);
                                if (queued == CAN_SEND_BUSY) {
                                        continue;
                                }
                                if (queued < 0) {
                                        fail_job(
                                                slot,
                                                TC2_FAILURE_TURNOUT);
                                        continue;
                                }
                                runtime->turnout_queue_pending = 0;
                                runtime->turnout_batch_token =
                                        queued > 0 ?
                                                (unsigned int)queued :
                                                0;
                                if (queued == 0) {
                                        runtime
                                                ->reverse_ready_at_tick =
                                                now;
                                } else {
                                        continue;
                                }
                        }
                        if (runtime->turnout_batch_token != 0) {
                                can_batch_status_t status;
                                if (CanGetBatchStatus(
                                            dispatch_can_tid,
                                            runtime
                                                    ->turnout_batch_token,
                                            &status) < 0 ||
                                    status.token !=
                                            runtime
                                                    ->turnout_batch_token) {
                                        fail_job(
                                                slot,
                                                TC2_FAILURE_TURNOUT);
                                        continue;
                                }
                                if (status.state == CAN_BATCH_FAILED) {
                                        int retry =
                                                queue_turnout_plan_for_leg(
                                                        slot);
                                        if (retry ==
                                            CAN_SEND_BUSY) {
                                                runtime
                                                    ->turnout_batch_token =
                                                    0;
                                                runtime
                                                    ->turnout_queue_pending =
                                                    1;
                                                continue;
                                        }
                                        if (retry < 0) {
                                                fail_job(
                                                        slot,
                                                        TC2_FAILURE_TURNOUT);
                                                continue;
                                        }
                                        runtime->turnout_batch_token =
                                                (unsigned int)retry;
                                        if (retry == 0) {
                                                runtime
                                                        ->reverse_ready_at_tick =
                                                        now;
                                        }
                                        continue;
                                }
                                if (status.state !=
                                    CAN_BATCH_COMPLETE) {
                                        continue;
                                }
                                runtime->turnout_batch_token = 0;
                                runtime->reverse_ready_at_tick =
                                        tick_after(
                                                now,
                                                TC2_TURNOUT_SETTLE_TICKS);
                                continue;
                        }
                        if (runtime->reverse_ready_at_tick < 0 ||
                            !tick_reached(
                                    now,
                                    runtime
                                            ->reverse_ready_at_tick)) {
                                continue;
                        }
                        if (runtime->reversal_phase ==
                            TC2_REVERSAL_PHASE_TURNOUT) {
                                if (CanTrainReversePriority(
                                            dispatch_can_tid,
                                            job->train) < 0) {
                                        fail_job(
                                                slot,
                                                TC2_FAILURE_CAN_SERVICE);
                                        continue;
                                }
                                int reverse_confirmed =
                                        Time();
                                if (reverse_confirmed < 0) {
                                        fail_job(
                                                slot,
                                                TC2_FAILURE_CAN_SERVICE);
                                        continue;
                                }
                                runtime->reversal_phase =
                                        TC2_REVERSAL_PHASE_SETTLE;
                                runtime->reverse_ready_at_tick =
                                        tick_after(
                                                reverse_confirmed,
                                                TC2_REVERSE_SETTLE_TICKS);
                                continue;
                        }
                        if (runtime->reversal_phase !=
                            TC2_REVERSAL_PHASE_SETTLE) {
                                fail_job(
                                        slot,
                                        TC2_FAILURE_INTERNAL);
                                continue;
                        }
                        /*
                         * Traffic safety runs earlier in every dispatcher tick.
                         * A stopped reversal handoff is therefore admitted to
                         * positive speed only when that complete pairwise pass
                         * found neither an opposing corridor nor a rear-train
                         * 200/400 mm hold. Keep direction changed and speed zero
                         * while blocked; the next tick reevaluates all peers.
                         */
                        if (runtime->prelaunch_traffic_blocked) {
                                continue;
                        }
                        int launch_speed =
                                runtime
                                                ->planned_launch_speed >
                                        0 ?
                                runtime
                                        ->planned_launch_speed :
                                job->speed;
                        if (command_train_speed_with_retry(
                                    job->train, launch_speed) < 0) {
                                fail_job(slot,
                                         TC2_FAILURE_CAN_SERVICE);
                                continue;
                        }
                        int motion_confirmed_tick = Time();
                        if (motion_confirmed_tick < 0) {
                                fail_job(
                                        slot,
                                        TC2_FAILURE_CAN_SERVICE);
                                continue;
                        }
                        runtime->command_speed = launch_speed;
                        job->command_speed = launch_speed;
                        runtime->motion_speed_ceiling =
                                launch_speed;
                        runtime->speed_reduction_pending = 0;
                        runtime->precision_approach_active = 0;
                        runtime->approach_sent = 0;
                        runtime->last_confirmed_tick =
                                motion_confirmed_tick;
                        runtime->motion_anchor_distance_um =
                                (int64_t)runtime->monitor
                                        .confirmed_distance_mm * 1000;
                        runtime->motion_anchor_tick =
                                motion_confirmed_tick;
                        runtime->motion_anchor_from_traffic = 0;
                        runtime->reversal_phase =
                                TC2_REVERSAL_PHASE_NONE;
                        runtime->reverse_ready_at_tick = -1;
                        job->state = TC2_JOB_RUNNING;
                        if (schedule_motion_deadlines(
                                    slot,
                                    motion_confirmed_tick) < 0) {
                                fail_job(slot, TC2_FAILURE_INTERNAL);
                        }
                }
        }
}

static void process_stop_retries(void) {
        int trains[TC2_DISPATCH_MAX_JOBS];
        int slots[TC2_DISPATCH_MAX_JOBS];
        int retry_count = 0;
        for (int slot = 0; slot < TC2_DISPATCH_MAX_JOBS; ++slot) {
                tc2_dispatch_job_snapshot *job = &dispatch_jobs[slot];
                tc2_dispatch_runtime *runtime =
                        &dispatch_runtime[slot];
                if (job->state != TC2_JOB_FAILED ||
                    !job->needs_stop_retry) {
                        continue;
                }
                if (runtime->emergency_stop_token != 0) {
                        can_batch_status_t status;
                        int batch_result = CanGetBatchStatus(
                                dispatch_can_tid,
                                runtime->emergency_stop_token,
                                &status);
                        if (batch_result == 0 &&
                            status.state == CAN_BATCH_PENDING) {
                                continue;
                        }
                        if (batch_result == 0 &&
                            status.token ==
                                    runtime
                                            ->emergency_stop_token &&
                            status.state == CAN_BATCH_COMPLETE) {
                                runtime->emergency_stop_token = 0;
                                job->needs_stop_retry = 0;
                                continue;
                        }
                        runtime->emergency_stop_token = 0;
                }
                trains[retry_count] = job->train;
                slots[retry_count] = slot;
                ++retry_count;
        }
        if (retry_count == 0) return;
        int token = CanTrainEmergencyStopBatch(
                dispatch_can_tid, trains, retry_count);
        if (token <= 0) return;
        for (int index = 0; index < retry_count; ++index) {
                dispatch_runtime[slots[index]]
                        .emergency_stop_token =
                        (unsigned int)token;
        }
}

static int has_active_motion_job(void) {
        for (int slot = 0; slot < TC2_DISPATCH_MAX_JOBS; ++slot) {
                int state = dispatch_jobs[slot].state;
                if (state == TC2_JOB_LAUNCHING ||
                    state == TC2_JOB_RUNNING ||
                    state == TC2_JOB_BRAKING ||
                    state == TC2_JOB_REVERSING ||
                    state == TC2_JOB_CANCEL_BRAKING ||
                    (state == TC2_JOB_FAILED &&
                     dispatch_jobs[slot].needs_stop_retry)) {
                        return 1;
                }
        }
        return 0;
}

static int collect_active_trains(int *trains, int capacity) {
        int count = 0;
        if (!trains || capacity < 1) return 0;
        for (int slot = 0;
             slot < TC2_DISPATCH_MAX_JOBS &&
             count < capacity; ++slot) {
                int state = dispatch_jobs[slot].state;
                if (state == TC2_JOB_LAUNCHING ||
                    state == TC2_JOB_RUNNING ||
                    state == TC2_JOB_BRAKING ||
                    state == TC2_JOB_REVERSING ||
                    state == TC2_JOB_CANCEL_BRAKING ||
                    (state == TC2_JOB_FAILED &&
                     dispatch_jobs[slot].needs_stop_retry)) {
                        trains[count++] =
                                dispatch_jobs[slot].train;
                }
        }
        return count;
}

static int supervisor_observe(
        unsigned int heartbeat, int logical_time,
        unsigned int *last_heartbeat, int *last_logical_time,
        int *stall_events, int active) {
        if (!last_heartbeat || !last_logical_time ||
            !stall_events) {
                return 1;
        }
        if (!active) {
                *last_heartbeat = heartbeat;
                *last_logical_time = logical_time;
                *stall_events = 0;
                return 0;
        }
        /*
         * The dispatch heartbeat is advanced by the request loop itself and
         * is the authoritative liveness signal.  Time() is allowed to repeat
         * across several fast observations, especially while a compound
         * turnout zone is sending and settling independent motor commands;
         * treating either unchanged value as a stall caused healthy routes
         * through SW5/SW7 and SW153-SW156 to be emergency-stopped.
         */
        if (heartbeat == *last_heartbeat) {
                ++*stall_events;
        } else {
                *stall_events = 0;
        }
        *last_heartbeat = heartbeat;
        *last_logical_time = logical_time;
        if (*stall_events >= TC2_SUPERVISOR_STALL_EVENTS) {
                *stall_events = 0;
                return 1;
        }
        return 0;
}

static int supervisor_wall_stalled(
        unsigned long long now_usec,
        unsigned long long last_dispatch_progress_usec,
        int active) {
        if (!active) return 0;
        /*
         * A late safety-task wakeup says only that this observer did not run
         * on time; it is not evidence that dispatch stopped making progress.
         * Treating the observer's own scheduling latency as a dispatcher
         * failure caused healthy single-train trips to be emergency-stopped
         * and permanently reported as watchdog failures.  Only dispatcher
         * progress is authoritative here; supervisor_observe() independently
         * checks the logical heartbeat below.
         */
        return now_usec - last_dispatch_progress_usec >
                TC2_SUPERVISOR_STALL_USEC;
}

void Tc2DispatchSafetyTask(void) {
        unsigned int last_heartbeat = dispatch_heartbeat;
        int last_logical_time = dispatch_last_logical_time;
        int stall_events = 0;
        int stop_latched = 0;
        unsigned long long retry_at_usec = 0;
        for (;;) {
                if (AwaitEvent(EVENT_TIMER) < 0) continue;
                unsigned long long now_usec = timer_get_usec();
                int active = has_active_motion_job();
                if (!active) {
                        /*
                         * The dispatcher has consumed the previous trip (or
                         * all motion completed), so a later independent stall
                         * may be reported.  Reset through supervisor_observe
                         * as well, keeping its heartbeat baseline coherent.
                         */
                        stop_latched = 0;
                        retry_at_usec = 0;
                        (void)supervisor_observe(
                                dispatch_heartbeat,
                                dispatch_last_logical_time,
                                &last_heartbeat,
                                &last_logical_time,
                                &stall_events, 0);
                        continue;
                }
                if (stop_latched) {
                        /* One physical emergency-stop batch per stall. */
                        continue;
                }
                int stalled = supervisor_wall_stalled(
                        now_usec, dispatch_last_progress_usec, active);
                if (!stalled && !supervisor_observe(
                            dispatch_heartbeat,
                            dispatch_last_logical_time,
                            &last_heartbeat,
                            &last_logical_time,
                            &stall_events, active)) {
                        continue;
                }
                int trains[TC2_DISPATCH_MAX_JOBS];
                int count = collect_active_trains(
                        trains, TC2_DISPATCH_MAX_JOBS);
                if (count > 0 && now_usec >= retry_at_usec) {
                        int token = CanTrainEmergencyStopBatch(
                                dispatch_can_tid, trains, count);
                        if (token < 0) {
                                /*
                                 * A busy CAN queue is not permission to emit
                                 * one zero every timer interrupt.  Retry at a
                                 * bounded cadence while the dispatcher is
                                 * still genuinely stalled.
                                 */
                                retry_at_usec = now_usec +
                                        TC2_SUPERVISOR_RETRY_USEC;
                                continue;
                        }
                        dispatch_supervisor_trip = 1;
                        stop_latched = 1;
                        retry_at_usec = 0;
                }
        }
}

static void process_waiting_jobs(int now) {
        unsigned char processed[TC2_DISPATCH_MAX_JOBS];
        for (int slot = 0; slot < TC2_DISPATCH_MAX_JOBS; ++slot) {
                processed[slot] = 0;
                if (dispatch_jobs[slot].state == TC2_JOB_WAITING) {
                        dispatch_jobs[slot].wait_age_ticks =
                                tick_age(
                                        now,
                                        dispatch_runtime[slot]
                                                .wait_since_tick);
                }
        }
        for (int pass = 0; pass < TC2_DISPATCH_MAX_JOBS; ++pass) {
                int oldest = -1;
                for (int slot = 0; slot < TC2_DISPATCH_MAX_JOBS; ++slot) {
                        if (processed[slot] ||
                            dispatch_jobs[slot].state !=
                                    TC2_JOB_WAITING) {
                                continue;
                        }
                        if (oldest < 0 ||
                            sequence_before(
                                    dispatch_jobs[slot]
                                            .queue_sequence,
                                    dispatch_jobs[oldest]
                                            .queue_sequence)) {
                                oldest = slot;
                        }
                }
                if (oldest < 0) break;
                processed[oldest] = 1;
                (void)prepare_waiting_job(oldest, now);
        }
        process_preparing_jobs(now);
        launch_ready_jobs(now);
}

static int retarget_request_is_older(
        int candidate, int current) {
        if (current < 0) return 1;
        unsigned int candidate_sequence =
                dispatch_runtime[candidate]
                        .retarget_queue_sequence;
        unsigned int current_sequence =
                dispatch_runtime[current]
                        .retarget_queue_sequence;
        if (candidate_sequence != current_sequence) {
                return sequence_before(
                        candidate_sequence,
                        current_sequence);
        }
        return dispatch_jobs[candidate].train <
                dispatch_jobs[current].train;
}

/*
 * Commit an armed CURRENT recovery as one dispatcher transaction.
 *
 * stage_job()/prepare_waiting_job() are reused under this single-writer
 * server, but the live hold, projection, and runtime are copied aside first.
 * Until ReplacePlanExpected succeeds the reservation table is unchanged; a
 * route conflict restores the old slot byte-for-byte.  Once the generation
 * advances, the new reservation and its fail-closed state are authoritative
 * and must never be rolled back to stale metadata.
 */
static void process_pending_retargets(int now) {
        unsigned char processed[TC2_DISPATCH_MAX_JOBS];
        for (int slot = 0;
             slot < TC2_DISPATCH_MAX_JOBS; ++slot) {
                processed[slot] = 0;
        }
        for (int pass = 0;
             pass < TC2_DISPATCH_MAX_JOBS; ++pass) {
                int oldest = -1;
                for (int slot = 0;
                     slot < TC2_DISPATCH_MAX_JOBS; ++slot) {
                        if (processed[slot] ||
                            !dispatch_runtime[slot]
                                    .retarget_pending ||
                            !dispatch_runtime[slot]
                                    .retarget_armed) {
                                continue;
                        }
                        if (retarget_request_is_older(
                                    slot, oldest)) {
                                oldest = slot;
                        }
                }
                if (oldest < 0) break;
                processed[oldest] = 1;

                tc2_dispatch_job_snapshot *held_job =
                        &dispatch_jobs[oldest];
                tc2_dispatch_runtime *held_runtime =
                        &dispatch_runtime[oldest];
                int retarget_speed =
                        held_runtime->retarget_speed;
                int retarget_destination =
                        held_runtime
                                ->retarget_destination_index;
                int retarget_force_reverse_first =
                        held_runtime
                                ->retarget_force_reverse_first;
                unsigned int expected_generation =
                        held_runtime
                                ->retarget_expected_generation;
                int expected_destination =
                        held_runtime
                                ->retarget_expected_destination;
                unsigned int queue_sequence =
                        held_runtime
                                ->retarget_queue_sequence;
                unsigned int launch_epoch =
                        held_runtime
                                ->retarget_launch_epoch;
                if (held_job->state !=
                            TC2_JOB_TRAFFIC_HOLD ||
                    held_runtime->stop_purpose !=
                            TC2_STOP_TRAFFIC ||
                    held_runtime->traffic_hold_distance_um < 0 ||
                    held_job->plan_generation !=
                            expected_generation ||
                    held_job->target_node !=
                            expected_destination ||
                    retarget_speed < 1 ||
                    retarget_speed > 120 ||
                    retarget_destination < 0 ||
                    retarget_destination >=
                            TC2_DISPATCH_DESTINATION_COUNT) {
                        held_runtime->retarget_pending = 0;
                        held_runtime->retarget_armed = 0;
                        continue;
                }

                /*
                 * The candidate preparation below may call fail_job() before
                 * its reservation CAS.  A memory rollback can restore the
                 * old job, zone owner, and FIFO tickets, but it cannot
                 * resurrect a CAN batch that fail_job() canceled in the CAN
                 * server.  Let the normal conflict-zone poll finish any old
                 * hard/soft batch first.  The request remains armed and is
                 * retried on the next scheduler tick; the train is already
                 * stopped under the live traffic hold throughout this wait.
                 */
                if (held_runtime->conflict_zone_batch_token != 0 ||
                    held_runtime->conflict_zone_soft_batch_token != 0 ||
                    held_runtime->conflict_zone_settle_at_tick >= 0) {
                        continue;
                }

                /*
                 * Capture old physical-zone geometry while localization is
                 * still trustworthy.  The temporary estimated ARRIVED view
                 * below must not downgrade an already proved entry/exit
                 * relationship to an ambiguous synthetic carry.
                 */
                if (snapshot_owned_conflict_zone_carries(oldest) < 0) {
                        dispatch_scheduler_healthy = 0;
                        held_runtime->retarget_pending = 0;
                        held_runtime->retarget_armed = 0;
                        continue;
                }
                dispatch_retarget_job_backup =
                        dispatch_jobs[oldest];
                dispatch_retarget_runtime_backup =
                        dispatch_runtime[oldest];
                dispatch_retarget_projection_backup =
                        dispatch_projection_publications[oldest];
                dispatch_retarget_conflict_zone_backup =
                        dispatch_conflict_zones;

                tc2_dispatch_request request;
                request.type = TC2_DISPATCH_MSG_STAGE;
                request.train = held_job->train;
                request.start_index =
                        TC2_DISPATCH_START_CURRENT;
                request.speed = retarget_speed;
                request.destination_index =
                        retarget_destination;
                request.node_index = -1;
                request.projection_serial = 0;
                request.projection_generation = 0;
                request.projection_launch_epoch = 0;
                request.projection_first_waypoint = -1;
                request.reserved =
                        retarget_force_reverse_first ?
                        TC2_DISPATCH_REQUEST_FORCE_REVERSE_FIRST : 0;

                /*
                 * This temporary ARRIVED view is not externally observable:
                 * the server cannot receive another request until the
                 * transaction returns.  Mark the mid-edge traffic position
                 * estimated so the replacement carries every old owner.
                 */
                dispatch_jobs[oldest].state =
                        TC2_JOB_ARRIVED;
                dispatch_jobs[oldest].hold_active = 1;
                dispatch_jobs[oldest].position_estimated = 1;
                dispatch_retarget_candidate_pre_cas = 1;
                int staged = stage_job(&request);
                int prepared = -1;
                if (staged == 0) {
                        dispatch_jobs[oldest].state =
                                TC2_JOB_WAITING;
                        dispatch_jobs[oldest]
                                .job_launch_epoch =
                                launch_epoch;
                        dispatch_jobs[oldest]
                                .queue_sequence =
                                queue_sequence;
                        dispatch_runtime[oldest]
                                .queue_sequence =
                                queue_sequence;
                        dispatch_runtime[oldest]
                                .launch_epoch =
                                launch_epoch;
                        dispatch_runtime[oldest]
                                .wait_since_tick = now;
                        prepared =
                                prepare_waiting_job(
                                        oldest, now);
                }
                dispatch_retarget_candidate_pre_cas = 0;
                if (staged == 0 && prepared == 0) {
                        continue;
                }

                track_reservation_snapshot after;
                int have_after =
                        TrackReservationServerSnapshot(
                                dispatch_reservation_tid,
                                &after) == 0;
                int old_plan_still_authoritative =
                        have_after &&
                        after.generation_by_train[
                                request.train] ==
                                expected_generation &&
                        after.destination_by_train[
                                request.train] ==
                                expected_destination;
                if (old_plan_still_authoritative) {
                        int candidate_conflict_train =
                                dispatch_jobs[oldest]
                                        .conflict_train;
                        int candidate_conflict_node =
                                dispatch_jobs[oldest]
                                        .conflict_node;
                        dispatch_jobs[oldest] =
                                dispatch_retarget_job_backup;
                        dispatch_runtime[oldest] =
                                dispatch_retarget_runtime_backup;
                        dispatch_projection_publications[oldest] =
                                dispatch_retarget_projection_backup;
                        dispatch_conflict_zones =
                                dispatch_retarget_conflict_zone_backup;
                        dispatch_jobs[oldest].conflict_train =
                                candidate_conflict_train;
                        dispatch_jobs[oldest].conflict_node =
                                candidate_conflict_node;
                        if (staged < 0 || prepared < 0) {
                                /*
                                 * A deterministic candidate failure leaves
                                 * the original hold safe and lets the user
                                 * choose another destination or cancel.
                                 */
                                dispatch_runtime[oldest]
                                        .retarget_pending = 0;
                                dispatch_runtime[oldest]
                                        .retarget_armed = 0;
                        }
                        continue;
                }

                /*
                 * The CAS may have committed before a later projection/CAN
                 * failure.  Keep that new fail-closed job.  If the snapshot
                 * itself is unavailable, rolling back would risk pairing an
                 * old generation with new ownership, so retain the new state
                 * and emergency-stop it.
                 */
                dispatch_runtime[oldest].retarget_pending = 0;
                dispatch_runtime[oldest].retarget_armed = 0;
                if (dispatch_jobs[oldest].state !=
                            TC2_JOB_FAILED) {
                        fail_job(
                                oldest,
                                have_after ?
                                TC2_FAILURE_INTERNAL :
                                TC2_FAILURE_RESERVATION_SERVICE);
                }
        }
}

static int cancel_job(int train) {
        int slot = find_train_job(train);
        if (slot < 0) return -1;
        dispatch_runtime[slot].retarget_pending = 0;
        dispatch_runtime[slot].retarget_armed = 0;
        /*
         * Hide the motion projection immediately, while keeping the
         * reservation and stop recovery active until operator removal.
         */
        invalidate_projection(slot);
        int conflict_cleanup_status =
                discard_future_conflict_zone_work(slot);
        if (conflict_cleanup_status < 0) {
                dispatch_scheduler_healthy = 0;
        }
        if (dispatch_jobs[slot].state ==
                    TC2_JOB_CANCEL_BRAKING ||
            dispatch_jobs[slot].state == TC2_JOB_STOPPED) {
                return conflict_cleanup_status;
        }
        int now = Time();
        int stop_speed =
                dispatch_runtime[slot].motion_speed_ceiling >
                                dispatch_runtime[slot].command_speed ?
                        dispatch_runtime[slot].motion_speed_ceiling :
                        dispatch_runtime[slot].command_speed;
        if (stop_speed <= 0) {
                stop_speed = dispatch_jobs[slot].speed;
        }
        int settle_ticks =
                Tc2MotionStopSettleTicks(stop_speed);
        int64_t cancel_progress_um = 0;
        if (now >= 0 &&
            runtime_motion_progress_um(
                    slot, now, &cancel_progress_um) < 0) {
                cancel_progress_um =
                        dispatch_runtime[slot].route_valid &&
                                dispatch_runtime[slot]
                                                .monitor
                                                .confirmed_distance_mm >=
                                        0 ?
                        (int64_t)dispatch_runtime[slot]
                                        .monitor
                                        .confirmed_distance_mm *
                                1000 :
                        0;
        }
        int cancel_coast_um =
                Tc2MotionMeasuredStopDistanceUm(stop_speed);
        if (dispatch_jobs[slot].state == TC2_JOB_LAUNCHING &&
            dispatch_runtime[slot].launch_batch_token != 0) {
                (void)CanCancelBatch(
                        dispatch_can_tid,
                        dispatch_runtime[slot]
                                .launch_batch_token);
        }
        if (dispatch_runtime[slot].turnout_batch_token != 0) {
                (void)CanCancelBatch(
                        dispatch_can_tid,
                        dispatch_runtime[slot]
                                .turnout_batch_token);
                dispatch_runtime[slot].turnout_batch_token = 0;
        }
        dispatch_runtime[slot].turnout_queue_pending = 0;
        if (now < 0 || settle_ticks < 0 ||
            cancel_coast_um < 0 ||
            cancel_progress_um < 0 ||
            cancel_progress_um >
                    INT64_MAX - cancel_coast_um) {
                if (slot >= 0) {
                        fail_job(slot, TC2_FAILURE_CAN_SERVICE);
                }
                return -1;
        }
        int stop_request_tick = Time();
        if (stop_request_tick < 0 ||
            CanTrainSetSpeedPriority(
                    dispatch_can_tid, train, 0) < 0) {
                if (slot >= 0) {
                        fail_job(slot, TC2_FAILURE_CAN_SERVICE);
                }
                return -1;
        }
        int stop_confirmed_tick = Time();
        if (stop_confirmed_tick < 0) {
                fail_job(slot, TC2_FAILURE_CAN_SERVICE);
                return -1;
        }
        int64_t cancel_endpoint_um =
                cancel_progress_um + cancel_coast_um;
        int64_t destination_um =
                (int64_t)dispatch_jobs[slot]
                        .destination_distance_mm * 1000;
        if (destination_um >= 0 &&
            cancel_endpoint_um > destination_um) {
                cancel_endpoint_um = destination_um;
        }
        dispatch_runtime[slot].motion_anchor_distance_um =
                cancel_progress_um;
        dispatch_runtime[slot].motion_anchor_tick =
                stop_confirmed_tick;
        dispatch_runtime[slot].motion_anchor_from_traffic = 1;
        dispatch_runtime[slot].traffic_hold_distance_um =
                cancel_endpoint_um;
        dispatch_runtime[slot].command_speed = 0;
        if (dispatch_runtime[slot].motion_speed_ceiling <
            stop_speed) {
                dispatch_runtime[slot].motion_speed_ceiling =
                        stop_speed;
        }
        dispatch_jobs[slot].command_speed = 0;
        dispatch_jobs[slot].estimated_distance_um =
                cancel_progress_um;
        dispatch_jobs[slot].state = TC2_JOB_CANCEL_BRAKING;
        dispatch_jobs[slot].ready_at_tick = -1;
        dispatch_jobs[slot].braking_at_tick = -1;
        dispatch_jobs[slot].needs_stop_retry = 0;
        dispatch_jobs[slot].stop_sent_tick =
                stop_confirmed_tick;
        if (!dispatch_jobs[slot].stop_timing_valid) {
                dispatch_jobs[slot].stop_request_tick =
                        stop_request_tick;
                dispatch_jobs[slot].stop_confirmed_tick =
                        stop_confirmed_tick;
                dispatch_jobs[slot].stop_timing_valid = 1;
        }
        dispatch_runtime[slot].settle_at_tick =
                tick_after(stop_confirmed_tick, settle_ticks);
        return conflict_cleanup_status;
}

/*
 * A physically removed train can be named by several independent survivor
 * latches.  Reservation/conflict ownership is released first; then this
 * reconciliation pass removes stale reservation-conflict diagnostics and
 * lets the ordinary traffic FSM decide whether each survivor may resume,
 * must acquire fresh turnout authority, or must remain stopped for another
 * live train.  In particular, do not special-case a train number or route.
 *
 * A survivor still coasting to a TC2_STOP_TRAFFIC hold cannot resume yet, but
 * it must not retain a peer-specific latch for a train which no longer
 * exists.  Transfer only that survivor to the ordinary AUTHORITY hold.  The
 * frozen stop point, route reservation, turnout gate, destination, and resume
 * speed remain intact; after braking settles, the normal conflict-zone and
 * rolling-authority FSMs decide when it may move.  References belonging to a
 * different live peer are never touched.
 */
static void reconcile_survivors_after_remove(
        int removed_train, int now) {
        for (int slot = 0;
             slot < TC2_DISPATCH_MAX_JOBS; ++slot) {
                tc2_dispatch_job_snapshot *job =
                        &dispatch_jobs[slot];
                if (job->state == TC2_JOB_EMPTY) continue;
                if (job->conflict_train == removed_train) {
                        job->conflict_train = 0;
                        job->conflict_node = -1;
                }
        }

        if (now >= 0) {
                process_traffic_safety(now);
        }

        for (int slot = 0;
             slot < TC2_DISPATCH_MAX_JOBS; ++slot) {
                tc2_dispatch_job_snapshot *job =
                        &dispatch_jobs[slot];
                tc2_dispatch_runtime *runtime =
                        &dispatch_runtime[slot];
                if (job->state == TC2_JOB_EMPTY) continue;

                int runtime_named_removed =
                        runtime->traffic_peer_train == removed_train;
                int job_named_removed =
                        job->traffic_peer_train == removed_train;
                int named_removed =
                        runtime_named_removed || job_named_removed;
                int removed_dynamic_latch =
                        runtime->traffic_reason ==
                                TC2_TRAFFIC_HEAD_ON ||
                        runtime->traffic_reason ==
                                TC2_TRAFFIC_FOLLOWING ||
                        job->traffic_reason ==
                                TC2_TRAFFIC_HEAD_ON ||
                        job->traffic_reason ==
                                TC2_TRAFFIC_FOLLOWING;

                if (runtime_named_removed) {
                        runtime->traffic_peer_train = 0;
                }
                if (job_named_removed) {
                        job->traffic_peer_train = 0;
                }

                if (!named_removed || !removed_dynamic_latch) {
                        continue;
                }

                int resumable_traffic_stop =
                        runtime->stop_purpose == TC2_STOP_TRAFFIC &&
                        (job->state == TC2_JOB_BRAKING ||
                         job->state == TC2_JOB_TRAFFIC_HOLD);
                if (resumable_traffic_stop) {
                        /*
                         * NONE has no recovery owner.  In particular, a
                         * survivor still in BRAKING would otherwise settle
                         * into a permanent TRAFFIC_HOLD after this remove.
                         * AUTHORITY is the generic, peer-free recovery owner.
                         */
                        runtime->traffic_reason =
                                TC2_TRAFFIC_AUTHORITY;
                        job->traffic_reason =
                                TC2_TRAFFIC_AUTHORITY;
                        job->traffic_gap_mm = -1;
                        job->traffic_stop_threshold_mm = -1;
                        job->traffic_resume_threshold_mm = -1;
                        runtime->conflict_zone_hold_active =
                                !conflict_zone_step_ready(
                                        slot,
                                        runtime
                                                ->conflict_zone_next_step);
                } else {
                        /* Terminal/non-resumable bookkeeping is stale. */
                        if (runtime->traffic_reason ==
                                    TC2_TRAFFIC_HEAD_ON ||
                            runtime->traffic_reason ==
                                    TC2_TRAFFIC_FOLLOWING) {
                                runtime->traffic_reason =
                                        TC2_TRAFFIC_NONE;
                        }
                        if (job->traffic_reason ==
                                    TC2_TRAFFIC_HEAD_ON ||
                            job->traffic_reason ==
                                    TC2_TRAFFIC_FOLLOWING) {
                                job->traffic_reason =
                                        TC2_TRAFFIC_NONE;
                        }
                }
        }
}

static int remove_job(int train) {
        int slot = find_train_job(train);
        if (slot < 0 ||
            dispatch_jobs[slot].state != TC2_JOB_STOPPED ||
            CanTrainSetSpeed(dispatch_can_tid, train, 0) < 0) {
                return -1;
        }
        /*
         * Clear the local zone owner/waiters before releasing the route
         * reservation.  If either half fails, restore the exact local zone
         * table and retain the stopped job plus its route reservation so the
         * operator can safely retry.  This prevents a partial remove from
         * losing FIFO/turnout ownership while the physical train remains.
         */
        dispatch_remove_conflict_zone_backup =
                dispatch_conflict_zones;
        if (TC2_DISPATCH_CONFLICT_ZONE_REMOVE_TRAIN(
                    &dispatch_conflict_zones, train, 1) < 0) {
                dispatch_conflict_zones =
                        dispatch_remove_conflict_zone_backup;
                return -1;
        }
        if (TrackReservationServerRelease(
                    dispatch_reservation_tid, train) < 0) {
                dispatch_conflict_zones =
                        dispatch_remove_conflict_zone_backup;
                return -1;
        }
        invalidate_projection(slot);
        clear_job(&dispatch_jobs[slot]);
        clear_runtime(&dispatch_runtime[slot]);
        int now = Time();
        if (now < 0) {
                now = dispatch_last_logical_time;
        }
        reconcile_survivors_after_remove(train, now);
        return 0;
}

static void fill_turnout_plan_snapshot(
        int slot, tc2_dispatch_job_snapshot *job) {
        if (!job || slot < 0 || slot >= TC2_DISPATCH_MAX_JOBS) return;

        job->turnout_plan_step_count = 0;
        job->turnout_plan_action_count = 0;
        job->next_turnout_action_count = 0;
        job->next_next_turnout_action_count = 0;
        for (int action = 0;
             action < TC2_DISPATCH_TURNOUT_ACTIONS_PER_STEP; ++action) {
                job->next_turnout_switch[action] = -1;
                job->next_turnout_direction[action] = 0;
                job->next_next_turnout_switch[action] = -1;
                job->next_next_turnout_direction[action] = 0;
        }

        const tc2_dispatch_runtime *runtime = &dispatch_runtime[slot];
        if (!runtime->conflict_zone_plan_valid ||
            job->state == TC2_JOB_EMPTY ||
            job->state == TC2_JOB_FAILED ||
            job->state == TC2_JOB_CANCEL_BRAKING ||
            job->state == TC2_JOB_STOPPED ||
            job->state == TC2_JOB_STOP_UNCONFIRMED ||
            job->state == TC2_JOB_ARRIVED) {
                /*
                 * A stationary terminal train is a physical block only.
                 * Its exact OCCUPIED carries remain inside the conflict-zone
                 * table, but future next/next-next actions are not movement
                 * authority and must never make another train wait.
                 */
                return;
        }

        job->turnout_plan_step_count =
                runtime->conflict_zone_plan.step_count;
        for (int step_index = 0;
             step_index < runtime->conflict_zone_plan.step_count;
             ++step_index) {
                job->turnout_plan_action_count +=
                        runtime->conflict_zone_plan.steps[step_index]
                                .setting.action_count;
        }

        int step_index = runtime->conflict_zone_next_step;
        int action_index = 0;
        if (runtime->conflict_zone_action_step == step_index &&
            runtime->conflict_zone_action_ticket != 0 &&
            runtime->conflict_zone_action_ticket ==
                    runtime->conflict_zone_hard_ticket &&
            runtime->conflict_zone_action_index >= 0) {
                action_index = runtime->conflict_zone_action_index;
        }

        /*
         * A conflict-zone step can contain several very close physical
         * motors (SW5/SW7 or SW153..SW156).  They share one stop/FIFO ticket,
         * but they are not one turnout: publish the flattened route program
         * one actuator at a time, preserving each motor's own S/C setting.
         */
        for (int lookahead = 0; lookahead < 2; ++lookahead) {
                while (step_index >= 0 &&
                       step_index <
                               runtime->conflict_zone_plan.step_count) {
                        const tc2_conflict_zone_setting *setting =
                                &runtime->conflict_zone_plan
                                         .steps[step_index].setting;
                        if (action_index < setting->action_count) {
                                break;
                        }
                        ++step_index;
                        action_index = 0;
                }
                if (step_index < 0 ||
                    step_index >=
                            runtime->conflict_zone_plan.step_count) {
                        break;
                }
                const tc2_conflict_turnout_action *action =
                        &runtime->conflict_zone_plan.steps[step_index]
                                 .setting.actions[action_index];
                char direction =
                        action->direction ==
                                        TC2_CONFLICT_TURNOUT_CURVED ?
                                'C' : 'S';
                if (lookahead == 0) {
                        job->next_turnout_action_count = 1;
                        job->next_turnout_switch[0] =
                                action->switch_number;
                        job->next_turnout_direction[0] = direction;
                } else {
                        job->next_next_turnout_action_count = 1;
                        job->next_next_turnout_switch[0] =
                                action->switch_number;
                        job->next_next_turnout_direction[0] = direction;
                }
                ++action_index;
        }
}

static void fill_snapshot(tc2_dispatch_snapshot *snapshot) {
        snapshot->job_count = 0;
        snapshot->launch_epoch = dispatch_launch_epoch;
        snapshot->blocked_physical_count =
                count_blocked_physical();
        snapshot->batch_ready_at_tick =
                dispatch_batch_ready_at_tick;
        snapshot->scheduler_healthy =
                dispatch_scheduler_healthy;
        for (int node = 0; node < TRACK_MAX; ++node) {
                snapshot->blocked_by_node[node] =
                        dispatch_blocked_by_node[node];
        }
        for (int slot = 0; slot < TC2_DISPATCH_MAX_JOBS; ++slot) {
                snapshot->jobs[slot] = dispatch_jobs[slot];
                fill_turnout_plan_snapshot(slot, &snapshot->jobs[slot]);
                if (dispatch_jobs[slot].state != TC2_JOB_EMPTY) {
                        ++snapshot->job_count;
                }
        }
}

/*
 * The scheduling core is defined below in small, independently testable
 * helpers. Keeping the IPC shell complete here also keeps MODE_TC2 builds
 * linkable while the state-machine helpers remain single-writer server state.
 */
static void dispatch_server_core(void);

void Tc2DispatchTickerTask(void) {
        int parent = MyParentTid();
        tc2_dispatch_request request;
        tc2_dispatch_reply reply;
        clear_request(&request, TC2_DISPATCH_MSG_TICK);
        int elapsed = 0;
        for (;;) {
                if (AwaitEvent(EVENT_TIMER) < 0) continue;
                ++elapsed;
                if (elapsed < TC2_DISPATCH_TICK_TICKS) continue;
                elapsed = 0;
                (void)Send(
                        parent, (const char *)&request,
                        sizeof(request), (char *)&reply,
                        sizeof(reply));
        }
}

void Tc2DispatchServerTask(void) {
        dispatch_server_core();
}

static void dispatch_server_core(void) {
        tc2_dispatch_request request;
        tc2_dispatch_reply reply;
        tc2_dispatch_snapshot snapshot;
        tc2_dispatch_blocked_snapshot blocked_snapshot;
        tc2_dispatch_projection_header projection_header;
        tc2_dispatch_projection_page projection_page;
        int sender;

        init_trackb(dispatch_track);
        initialize_conflict_zone_table();
        int catalog_ok =
                Tc2TrackCatalogValidate(dispatch_track) == 0;
        /*
         * Keep this bounded clear in the same separate translation-unit
         * helper used for prepare_sensor_state.  GCC 12's AArch64 -O3 bounds
         * analysis otherwise reports a false one-past-end
         * -Wstringop-overflow for the valid TRACK_MAX-byte initialization.
         */
        util_zero_bytes(dispatch_blocked_by_node,
                        (unsigned int)sizeof(
                                dispatch_blocked_by_node));
        for (int slot = 0; slot < TC2_DISPATCH_MAX_JOBS; ++slot) {
                clear_job(&dispatch_jobs[slot]);
                clear_runtime(&dispatch_runtime[slot]);
                initialize_projection_publication(slot);
        }
        Tc2RouteProjectionInitialize(
                &dispatch_projection_scratch);
        dispatch_projection_serial = 0;
        dispatch_launch_epoch = 0;
        dispatch_queue_sequence = 0;
        dispatch_batch_ready_at_tick = -1;
        dispatch_scheduler_healthy = 0;
        dispatch_heartbeat = 0;
        dispatch_last_logical_time = -1;
        dispatch_supervisor_trip = 0;
        dispatch_last_progress_usec = timer_get_usec();
        int register_status =
                RegisterAs(TC2_DISPATCH_SERVER_NAME);
        dispatch_can_tid = WhoIs(CAN_SERVER_NAME);
        dispatch_sensor_tid = WhoIs(TRAIN_SENSOR_SERVER_NAME);
        dispatch_reservation_tid =
                WhoIs(TRACK_RESERVATION_SERVER_NAME);
        int ticker_tid = Create(
                TC2_DISPATCH_TICKER_PRIORITY,
                Tc2DispatchTickerTask);
        int safety_tid = Create(3, Tc2DispatchSafetyTask);
        int turnout_authority_status =
                CanRegisterTurnoutAuthority(dispatch_can_tid);
        train_sensor_snapshot_t startup_sensor;
        track_reservation_snapshot startup_reservation;
        can_health_t startup_can;
        int clock_tick = Time();
        int startup_sensor_ok =
                wait_for_sensor_startup(
                        dispatch_sensor_tid,
                        &startup_sensor,
                        TC2_SENSOR_STARTUP_MAX_ATTEMPTS);
        int startup_reservation_ok =
                TrackReservationServerSnapshot(
                        dispatch_reservation_tid,
                        &startup_reservation) == 0;
        int startup_can_ok =
                CanGetHealth(dispatch_can_tid, &startup_can) == 0 &&
                startup_can.hw_ready;
        dispatch_scheduler_healthy =
                scheduler_dependencies_healthy(
                        register_status, dispatch_can_tid,
                        dispatch_sensor_tid,
                        dispatch_reservation_tid, ticker_tid,
                        safety_tid) &&
                clock_tick >= 0 && startup_sensor_ok &&
                startup_reservation_ok && startup_can_ok &&
                turnout_authority_status == 0 &&
                catalog_ok;
        if (dispatch_scheduler_healthy) {
                /*
                 * Establish one atomic, known-stopped startup state for every
                 * TC2 locomotive before accepting any dispatch request.  Do
                 * not initialize turnouts here: their first physical setting
                 * is selected by the conflict-zone owner at the approach.
                 */
                int startup_trains[] = {14, 15, 17, 18};
                if (CanTrainEmergencyStopBatch(
                            dispatch_can_tid, startup_trains,
                            (int)(sizeof(startup_trains) /
                                  sizeof(startup_trains[0]))) < 0) {
                        dispatch_scheduler_healthy = 0;
                }
        }
        if (clock_tick >= 0) {
                dispatch_last_logical_time = clock_tick;
        }

        for (;;) {
                int length = Receive(&sender, (char *)&request,
                                     sizeof(request));
                reply.status = -1;
                reply.affected = 0;
                if (length != (int)sizeof(request)) {
                        Reply(sender, (const char *)&reply,
                              sizeof(reply));
                        continue;
                }

                int allowed_while_unhealthy =
                        request.type == TC2_DISPATCH_MSG_SNAPSHOT ||
                        request.type == TC2_DISPATCH_MSG_CANCEL ||
                        request.type == TC2_DISPATCH_MSG_REMOVE ||
                        request.type ==
                                TC2_DISPATCH_MSG_BLOCKED_SNAPSHOT ||
                        request.type ==
                                TC2_DISPATCH_MSG_IS_MANAGED ||
                        request.type ==
                                TC2_DISPATCH_MSG_PROJECTION_HEADER ||
                        request.type ==
                                TC2_DISPATCH_MSG_PROJECTION_PAGE ||
                        request.type == TC2_DISPATCH_MSG_TICK;
                if (!dispatch_scheduler_healthy &&
                    !allowed_while_unhealthy) {
                        Reply(sender, (const char *)&reply,
                              sizeof(reply));
                        continue;
                }

                if (request.type == TC2_DISPATCH_MSG_STAGE ||
                    request.type ==
                            TC2_DISPATCH_MSG_STAGE_CALIBRATION) {
                        /*
                         * Reject before stage_job can reserve track or any
                         * later state can enqueue speed/turnout CAN traffic.
                         * CURRENT reroutes use this same message path, so the
                         * fleet contract applies uniformly to first dispatch
                         * and recovery dispatch.
                         */
                        reply.status = production_stage_job(&request);
                        reply.affected = reply.status == 0;
                        Reply(sender, (const char *)&reply,
                              sizeof(reply));
                } else if (request.type ==
                           TC2_DISPATCH_MSG_START_ALL) {
                        int now = Time();
                        if (now >= 0) {
                                ++dispatch_launch_epoch;
                                if (dispatch_launch_epoch == 0) {
                                        ++dispatch_launch_epoch;
                                }
                                for (int slot = 0;
                                     slot < TC2_DISPATCH_MAX_JOBS;
                                     ++slot) {
                                        if (dispatch_runtime[slot]
                                                    .retarget_pending &&
                                            !dispatch_runtime[slot]
                                                    .retarget_armed) {
                                                ++dispatch_queue_sequence;
                                                if (dispatch_queue_sequence ==
                                                    0) {
                                                        ++dispatch_queue_sequence;
                                                }
                                                dispatch_runtime[slot]
                                                        .retarget_armed = 1;
                                                dispatch_runtime[slot]
                                                        .retarget_queue_sequence =
                                                        dispatch_queue_sequence;
                                                dispatch_runtime[slot]
                                                        .retarget_launch_epoch =
                                                        dispatch_launch_epoch;
                                                ++reply.affected;
                                                continue;
                                        }
                                        if (dispatch_jobs[slot].state !=
                                            TC2_JOB_STAGED) {
                                                continue;
                                        }
                                        dispatch_jobs[slot].state =
                                                TC2_JOB_WAITING;
                                        dispatch_jobs[slot]
                                                .job_launch_epoch =
                                                dispatch_launch_epoch;
                                        ++dispatch_queue_sequence;
                                        if (dispatch_queue_sequence == 0) {
                                                ++dispatch_queue_sequence;
                                        }
                                        dispatch_jobs[slot]
                                                .queue_sequence =
                                                dispatch_queue_sequence;
                                        dispatch_runtime[slot]
                                                .queue_sequence =
                                                dispatch_queue_sequence;
                                        dispatch_runtime[slot]
                                                .launch_epoch =
                                                dispatch_launch_epoch;
                                        dispatch_runtime[slot]
                                                .wait_since_tick = now;
                                        ++reply.affected;
                                }
                        }
                        reply.status = reply.affected > 0 ? 0 : -1;
                        Reply(sender, (const char *)&reply,
                              sizeof(reply));
                } else if (request.type ==
                           TC2_DISPATCH_MSG_CANCEL) {
                        reply.status = cancel_job(request.train);
                        reply.affected = reply.status == 0;
                        Reply(sender, (const char *)&reply,
                              sizeof(reply));
                } else if (request.type ==
                           TC2_DISPATCH_MSG_REMOVE) {
                        reply.status = remove_job(request.train);
                        reply.affected = reply.status == 0;
                        Reply(sender, (const char *)&reply,
                              sizeof(reply));
                } else if (request.type ==
                           TC2_DISPATCH_MSG_BLOCK) {
                        reply.status =
                                block_node(request.node_index, 1);
                        reply.affected = reply.status == 0;
                        Reply(sender, (const char *)&reply,
                              sizeof(reply));
                } else if (request.type ==
                           TC2_DISPATCH_MSG_UNBLOCK) {
                        reply.status =
                                block_node(request.node_index, 0);
                        reply.affected = reply.status == 0;
                        Reply(sender, (const char *)&reply,
                              sizeof(reply));
                } else if (request.type ==
                           TC2_DISPATCH_MSG_IS_MANAGED) {
                        reply.status =
                                find_train_job(request.train) >= 0 ?
                                1 : 0;
                        Reply(sender, (const char *)&reply,
                              sizeof(reply));
                } else if (request.type ==
                           TC2_DISPATCH_MSG_SNAPSHOT) {
                        fill_snapshot(&snapshot);
                        Reply(sender, (const char *)&snapshot,
                              sizeof(snapshot));
                } else if (request.type ==
                           TC2_DISPATCH_MSG_BLOCKED_SNAPSHOT) {
                        blocked_snapshot.blocked_physical_count =
                                count_blocked_physical();
                        for (int node = 0; node < TRACK_MAX; ++node) {
                                blocked_snapshot.blocked_by_node[node] =
                                        dispatch_blocked_by_node[node];
                        }
                        Reply(sender,
                              (const char *)&blocked_snapshot,
                              sizeof(blocked_snapshot));
                } else if (request.type ==
                           TC2_DISPATCH_MSG_PROJECTION_HEADER) {
                        (void)projection_header_for_train(
                                request.train,
                                &projection_header);
                        Reply(sender,
                              (const char *)&projection_header,
                              sizeof(projection_header));
                } else if (request.type ==
                           TC2_DISPATCH_MSG_PROJECTION_PAGE) {
                        (void)projection_page_for_train(
                                request.train,
                                request.projection_serial,
                                request.projection_generation,
                                request.projection_launch_epoch,
                                request.projection_first_waypoint,
                                &projection_page);
                        Reply(sender,
                              (const char *)&projection_page,
                              sizeof(projection_page));
                } else if (request.type ==
                           TC2_DISPATCH_MSG_TICK) {
                        ++dispatch_heartbeat;
                        dispatch_last_progress_usec =
                                timer_get_usec();
                        int now = Time();
                        if (now >= 0) {
                                dispatch_last_logical_time = now;
                        }
                        if (now < 0 ||
                            !dispatch_scheduler_healthy ||
                            dispatch_supervisor_trip) {
                                int failure_reason =
                                        dispatch_supervisor_trip ?
                                        TC2_FAILURE_WATCHDOG :
                                        TC2_FAILURE_INTERNAL;
                                dispatch_supervisor_trip = 0;
                                if (now < 0) {
                                        dispatch_scheduler_healthy = 0;
                                }
                                for (int slot = 0;
                                     slot < TC2_DISPATCH_MAX_JOBS;
                                     ++slot) {
                                        int state =
                                                dispatch_jobs[slot].state;
                                        if (state ==
                                                    TC2_JOB_PREPARING ||
                                            state == TC2_JOB_READY ||
                                            state ==
                                                    TC2_JOB_LAUNCHING ||
                                            state == TC2_JOB_RUNNING ||
                                            state == TC2_JOB_BRAKING ||
                                            state ==
                                                    TC2_JOB_REVERSING ||
                                            state ==
                                                    TC2_JOB_CANCEL_BRAKING) {
                                                fail_job(
                                                        slot,
                                                        failure_reason);
                                        }
                                }
                                process_stop_retries();
                        } else {
                                process_stop_retries();
                                process_conflict_zones(now);
                                process_running_jobs(now);
                                process_traffic_safety(now);
                                process_pending_retargets(now);
                                process_launching_jobs(now);
                                process_braking_and_reversing(now);
                                process_authority_holds(now);
                                process_waiting_jobs(now);
                                process_conflict_zones(now);
                                reply.status = 0;
                        }
                        Reply(sender, (const char *)&reply,
                              sizeof(reply));
                } else {
                        Reply(sender, (const char *)&reply,
                              sizeof(reply));
                }
        }
}

#endif
