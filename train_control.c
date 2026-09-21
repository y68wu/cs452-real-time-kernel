#include "train_control.h"
#include "terminal.h"
#include "can.h"
#include "clock.h"
#include "nameserver.h"
#include "syscall.h"
#include "train_sensor.h"
#include "track_route.h"
#include "track_reservation_server.h"
#include "tc2_dispatch.h"
#ifdef MODE_TC2
#include "tc2_motion_model.h"
#include "tc2_live_ui.h"
#include "tc2_offline_controller.h"
#include "tc2_route_projection.h"
#include "tc2_track_d_layout.h"
#endif


#define TC_MAX_USER_SPEED 120

#ifdef MODE_TC2
#define TC2_REVERSAL_PENALTY_MM 400
static track_node tc2_track[TRACK_MAX];
static int tc_reservation_server_tid = -1;
static int tc_dispatch_server_tid = -1;
static int tc_can_server_tid = -1;
static tc2_offline_controller tc2_offline_controller_state;
static tc2_live_ui tc2_live_ui_state;
static tc2_dispatch_projection_waypoint
        tc2_live_projection_scratch[TC2_LIVE_UI_MAX_WAYPOINTS];
static int tc2_offline_controller_ready;
static int tc2_ui_active;
static int tc2_ui_live;
static int tc2_ui_shutdown_requested;
static int tc2_ui_shutdown_frame_queued;
static uint32_t tc2_ui_next_render_tick;
static char tc2_ui_status[192];
static char tc2_ui_restore[TC2_UI_TX_RESTORE_CAPACITY];
static int tc2_ui_status_dirty = 1;

#define TC2_DEMO_TRAIN_COUNT 3
#define TC2_DEMO_MINIMUM_RUNTIME_TICKS 18000u
#define TC2_DEMO_STOP_DWELL_TICKS 200u
#define TC2_DEMO_REVERSE_SETTLE_TICKS 200u
#define TC2_DEMO_WAIT_TURNOUT_TICKS 200u
#define TC2_DEMO_SENSOR_TIMEOUT_TICKS 6000u
#define TC2_DEMO_SENSOR_RECOVERY_DWELL_TICKS 100u
#define TC2_DEMO_MAX_SENSOR_RECOVERY_RESTARTS 2u
#define TC2_DEMO_SENSOR_FAILURE_LIMIT 20u
#define TC2_DEMO_POLL_TICKS 5u
#define TC2_DEMO_MAX_ROUTE_SENSORS 10
#define TC2_DEMO_MAX_LEG_SWITCHES 8
#define TC2_DEMO_TURNOUT_BATCH_LIMIT 4
#define TC2_DEMO_ENDPOINT_TRIM_SPEED 40
#define TC2_DEMO_ENDPOINT_BRAKE_SETTLE_TICKS 200u
#define TC2_DEMO_T17_RELEASE_CLEARANCE_MM 1800
#define TC2_DEMO_T15_RETURN_DWELL_TICKS 1500u
#define TC2_DEMO_UI_SEGMENT_DISTANCE_MM 500
#define TC2_DEMO_UI_PROGRESS_SCALE 1000u
#define TC2_DEMO_UI_UNCONFIRMED_LIMIT 900u

typedef enum {
        TC2_DEMO_IDLE = 0,
        TC2_DEMO_INITIAL_STOPPING,
        TC2_DEMO_WAIT_TURNOUT,
        TC2_DEMO_RUNNING,
        TC2_DEMO_STOPPED,
        TC2_DEMO_REVERSING,
        TC2_DEMO_ENDPOINT_TRIM_STOPPED,
        TC2_DEMO_ENDPOINT_TRIM_RUNNING,
        TC2_DEMO_SENSOR_RECOVERY,
        TC2_DEMO_PAUSED
} tc2_demo_phase;

typedef struct {
        int active;
        int initializing;
        int profile_id;
        uint32_t started_tick;
        uint32_t next_poll_tick;
        unsigned int initial_stop_token;
        unsigned int action_count;
        unsigned int route_armed_mask[TC2_DEMO_TRAIN_COUNT];
        unsigned int sensor_recovery_restarts[TC2_DEMO_TRAIN_COUNT];
        unsigned int sensor_failure_count;
        unsigned int completed_cycles[TC2_DEMO_TRAIN_COUNT];
        int phase[TC2_DEMO_TRAIN_COUNT];
        uint32_t phase_started_tick[TC2_DEMO_TRAIN_COUNT];
        uint32_t wait_until_tick[TC2_DEMO_TRAIN_COUNT];
        unsigned int pending_batch_token[TC2_DEMO_TRAIN_COUNT];
        unsigned char outbound_leg[TC2_DEMO_TRAIN_COUNT];
        unsigned char first_departure[TC2_DEMO_TRAIN_COUNT];
        int route_sensor_cursor[TC2_DEMO_TRAIN_COUNT];
        unsigned int ui_plan_generation[TC2_DEMO_TRAIN_COUNT];
	unsigned int ui_position_revision[TC2_DEMO_TRAIN_COUNT];
	unsigned int ui_sensor_sequence[TC2_DEMO_TRAIN_COUNT];
	uint32_t ui_segment_started_tick[TC2_DEMO_TRAIN_COUNT];
	unsigned int ui_segment_progress[TC2_DEMO_TRAIN_COUNT];
	int ui_segment_anchor_sensor[TC2_DEMO_TRAIN_COUNT];
	int ui_segment_anchor_row[TC2_DEMO_TRAIN_COUNT];
	int ui_segment_anchor_column[TC2_DEMO_TRAIN_COUNT];
	unsigned char resume_after_wait[TC2_DEMO_TRAIN_COUNT];
	unsigned char reverse_after_wait[TC2_DEMO_TRAIN_COUNT];
	unsigned char deferred_initial_prepare[TC2_DEMO_TRAIN_COUNT];
	unsigned char station_stop[TC2_DEMO_TRAIN_COUNT];
	unsigned char return_leg_started[TC2_DEMO_TRAIN_COUNT];
	uint32_t motion_account_tick[TC2_DEMO_TRAIN_COUNT];
	uint32_t return_running_ticks[TC2_DEMO_TRAIN_COUNT];
	uint32_t return_prepare_release_tick[TC2_DEMO_TRAIN_COUNT];
        int last_running;
        int last_waiting;
        int last_stopped;
        int last_reversing;
        int last_elapsed_seconds;
        int last_phase[TC2_DEMO_TRAIN_COUNT];
} tc2_physical_demo;

static tc2_physical_demo tc2_demo_state;
#endif

#ifndef MODE_TC2
static int tc_speed_model_index(int speed) {
        int idx;

        if (speed <= 0) {
                return 0;
        }

        idx = (speed * 14 + 60) / 120;

        if (idx < 1) {
                idx = 1;
        }

        if (idx > 14) {
                idx = 14;
        }

        return idx;
}
#endif

static int tc_streq(const char *a, const char *b) {
        while (*a && *b) {
                char ca = *a;
                char cb = *b;

                if (ca >= 'a' && ca <= 'z') {
                        ca = (char)(ca - 'a' + 'A');
                }

                if (cb >= 'a' && cb <= 'z') {
                        cb = (char)(cb - 'a' + 'A');
                }

                if (ca != cb) {
                        return 0;
                }

                a++;
                b++;
        }

        return *a == 0 && *b == 0;
}

static int tc_target120_ticks(const char *target_name, int unused_base_ticks) {
        (void)unused_base_ticks;

        /*
         * Speed-120 target calibration from the same fixed start.
         * A5 is known-good at 20 ticks. Other targets are absolute
         * calibrated timing guesses relative to the marked Track D photo.
         */
        if (tc_streq(target_name, "D10")) {
                return 28;
        }

        if (tc_streq(target_name, "B7")) {
                return 28;
        }

        if (tc_streq(target_name, "A5")) {
                return 19;
        }

        if (tc_streq(target_name, "C8")) {
                return 37;
        }

        if (tc_streq(target_name, "LOOP")) {
                return 32;
        }

        return 20;
}


#define TC_DEFAULT_TRAIN 14
#define TC_DEFAULT_SPEED 30
#define TC_MAX_LINE 80

typedef struct {
        const char *name;
        int distance_mm;
        int switch_a;
        char dir_a;
        int switch_b;
        char dir_b;
        int switch_c;
        char dir_c;
} tc_target_t;

typedef struct {
        int train;
        int speed;
        const char *last_target;
        int last_run_ticks;
        int last_stop_distance_mm;
        int last_prediction_error_ticks;
} tc_state_t;

/*
 * This is a simple initial route table for TC1. The goal is to make the
 * train-control application structure testable first. The exact switch
 * choices can be tuned during the lab demo after checking the physical track.
 */
static tc_target_t targets[] = {
        {"A5",   1200, 8, 'S', 0, 0, 0, 0},
        {"B7",   1700, 8, 'S', 17, 'S', 155, 'S'},
        {"C8",   2300, 8, 'S', 17, 'S', 156, 'S'},
        {"D10",  2900, 8, 'S', 17, 'S', 156, 'S'},
        {"LOOP", 3600, 8, 'C', 7, 'S', 18, 'C'}
};

static const int num_targets = sizeof(targets) / sizeof(targets[0]);

/*
 * Model values are intentionally conservative seed values. They are static
 * calibration data and can be adjusted after observing the real train.
 *
 * velocity_mm_per_tick uses a 10 ms clock tick.
 * stop_distance_mm is the approximate distance travelled after speed zero.
 */
#ifndef MODE_TC2
static int velocity_mm_per_tick[15] = {
        0, 12, 18, 24, 31,
        38, 45, 52, 59, 66,
        73, 80, 87, 94, 101
};

static int stop_distance_mm[15] = {
        0, 80, 120, 160, 210,
        260, 320, 390, 470, 560,
        660, 770, 900, 1040, 1200
};
#endif

static int tc_velocity_for_speed(int speed) {
#ifdef MODE_TC2
        return Tc2MotionVelocityMmPerTick(speed);
#else
        return velocity_mm_per_tick[tc_speed_model_index(speed)];
#endif
}

static int tc_stop_distance_for_speed(int speed) {
#ifdef MODE_TC2
        return Tc2MotionStopDistanceMm(speed);
#else
        return stop_distance_mm[tc_speed_model_index(speed)];
#endif
}

static tc_state_t state = {
        TC_DEFAULT_TRAIN,
        0,
        "unknown",
        0,
        0,
        0
};

#ifdef MODE_TC2
static int tc_sensor_server_tid = -1;
#endif

static int is_space(char c) {
        return c == ' ' || c == '\t';
}

static void skip_spaces(char **p) {
        while (**p && is_space(**p)) {
                (*p)++;
        }
}

static int streq(const char *a, const char *b) {
        while (*a && *b) {
                if (*a != *b) {
                        return 0;
                }
                a++;
                b++;
        }

        return *a == 0 && *b == 0;
}

#ifndef MODE_TC2
static int starts_with(const char *s, const char *prefix) {
        while (*prefix) {
                if (*s != *prefix) {
                        return 0;
                }
                s++;
                prefix++;
        }

        return 1;
}
#endif

#ifdef MODE_TC2
static int command_only(const char *s, const char *command) {
        while (*command) {
                if (*s != *command) return 0;
                ++s;
                ++command;
        }
        while (*s && is_space(*s)) ++s;
        return *s == 0;
}

static int command_token(const char *s, const char *command) {
        while (*command) {
                if (*s != *command) return 0;
                ++s;
                ++command;
        }
        return *s == 0 || is_space(*s);
}

static int tc2_boot_diagnostic_line(const char *s) {
        return command_token(s, "xHC") ||
               command_token(s, "XHC");
}

static int no_more_args(char *p) {
        skip_spaces(&p);
        return *p == 0;
}

static void uppercase_word(char *word) {
        while (*word) {
                if (*word >= 'a' && *word <= 'z') {
                        *word = (char)(*word - 'a' + 'A');
                }
                ++word;
        }
}

static size_t tc2_status_append(char *buffer, size_t capacity,
                                size_t length, const char *text) {
        if (!buffer || capacity == 0 || !text) return length;
        while (*text && length + 1 < capacity) {
                buffer[length++] = *text++;
        }
        buffer[length] = 0;
        return length;
}

static size_t tc2_status_append_uint(char *buffer, size_t capacity,
                                     size_t length, unsigned int value) {
        char digits[12];
        int count = 0;
        if (value == 0) {
                return tc2_status_append(buffer, capacity, length, "0");
        }
        while (value && count < (int)sizeof(digits)) {
                digits[count++] = (char)('0' + value % 10);
                value /= 10;
        }
        while (count > 0 && length + 1 < capacity) {
                buffer[length++] = digits[--count];
        }
        buffer[length] = 0;
        return length;
}

static void tc2_status_set(const char *text) {
        tc2_ui_status[0] = 0;
        (void)tc2_status_append(
                tc2_ui_status, sizeof(tc2_ui_status), 0, text);
        tc2_ui_status_dirty = 1;
}
#endif

static int command_no_args(const char *s, const char *command) {
#ifdef MODE_TC2
        return command_only(s, command);
#else
        return starts_with(s, command);
#endif
}

static int command_with_args(const char *s, const char *command) {
#ifdef MODE_TC2
        return command_token(s, command);
#else
        return starts_with(s, command);
#endif
}

static int command_args_complete(char *p) {
#ifdef MODE_TC2
        return no_more_args(p);
#else
        (void)p;
        return 1;
#endif
}

static int parse_uint(char **p, int *out) {
        int value = 0;
        int seen = 0;

        skip_spaces(p);

        while (**p >= '0' && **p <= '9') {
                int digit = **p - '0';
                if (value > 214748364 ||
                    (value == 214748364 && digit > 7)) {
                        return -1;
                }
                value = value * 10 + digit;
                (*p)++;
                seen = 1;
        }

        if (!seen) {
                return -1;
        }
        if (**p && !is_space(**p)) {
                return -1;
        }

        *out = value;
        return 0;
}

static int parse_word(char **p, char *out, int max_len) {
        int i = 0;

        skip_spaces(p);

        while (**p && !is_space(**p)) {
                if (i >= max_len - 1) {
                        return -1;
                }
                out[i++] = **p;
                (*p)++;
        }

        out[i] = 0;

        return i > 0 ? 0 : -1;
}


static void tc_busy_wait_ticks(int ticks) {
        if (ticks < 1) {
                ticks = 1;
        }

        /*
         * TC1 fallback timing: use a bounded busy wait so the stop command
         * always completes and returns to the command prompt.
         */
        for (int t = 0; t < ticks; t++) {
                for (volatile int d = 0; d < 20000000; d++) {
                }
        }
}

static void tc_put_char(int terminal_tid, unsigned char ch) {
        if (terminal_tid < 1) {
                return;
        }

        while (Putc(terminal_tid, ch) < 0) {
                Yield();
        }
}

static void tc_puts(int terminal_tid, const char *s) {
        while (*s) {
                tc_put_char(terminal_tid, (unsigned char)*s);
                ++s;
        }
}

static void tc_put_uint(int terminal_tid, unsigned int value) {
        char buf[12];
        int i = 0;

        if (value == 0) {
                tc_put_char(terminal_tid, '0');
                return;
        }

        while (value > 0 && i < 11) {
                buf[i++] = (char)('0' + (value % 10));
                value /= 10;
        }

        while (i > 0) {
                tc_put_char(terminal_tid, (unsigned char)buf[--i]);
        }
}

static void tc_put_int(int terminal_tid, int value) {
        if (value < 0) {
                // Unsigned subtraction安全处理INT_MIN，避免signed negation overflow。
                unsigned int magnitude = 0u - (unsigned int)value;
                tc_put_char(terminal_tid, '-');
                tc_put_uint(terminal_tid, magnitude);
        } else {
                tc_put_uint(terminal_tid, (unsigned int)value);
        }
}

#ifdef MODE_TC2
static void tc2_emit_offline_status(int terminal_tid) {
        tc_puts(terminal_tid, "\r\n");
        tc_puts(terminal_tid, Tc2OfflineControllerEvidenceLabel());
        tc_puts(terminal_tid, "\r\n");
        tc_puts(terminal_tid, tc2_ui_status);
        tc_puts(terminal_tid, "\r\n");
}

static int tc2_tick_due(uint32_t now_tick, uint32_t deadline_tick) {
        return (int32_t)(now_tick - deadline_tick) >= 0;
}

static int tc2_terminal_sink(void *context, const char *bytes,
                             size_t length) {
        int terminal_tid;
        int status;

        if (!context || !bytes || length == 0 ||
            length > TERMINAL_TRY_WRITE_CAPACITY) {
                return TC2_UI_TX_SINK_FAILED;
        }

        terminal_tid = *(const int *)context;
        status = TerminalTryWrite(terminal_tid, bytes, (int)length);
        if (status == TERMINAL_TRY_WRITE_ACCEPTED) {
                return TC2_UI_TX_SINK_ACCEPTED;
        }
        if (status == TERMINAL_TRY_WRITE_WOULD_BLOCK) {
                return TC2_UI_TX_SINK_WOULD_BLOCK;
        }
        return TC2_UI_TX_SINK_FAILED;
}

static size_t tc2_build_prompt_restore(const char *line, int line_length) {
        static const char status_row[] =
                "\033[0m\033[37;1H\033[2K";
        static const char prompt_row[] =
                "\033[38;1H\033[2K> ";
        size_t length = 0;
        int full_redraw =
                tc2_ui_live ?
                !tc2_live_ui_state.renderer.initialized :
                !tc2_offline_controller_state.renderer.initialized;

        if (!line || line_length < 0) line_length = 0;
        if (tc2_ui_status_dirty || full_redraw) {
                length = tc2_status_append(
                        tc2_ui_restore, sizeof(tc2_ui_restore), length,
                        status_row);
                length = tc2_status_append(
                        tc2_ui_restore, sizeof(tc2_ui_restore), length,
                        tc2_ui_status);
                tc2_ui_status_dirty = 0;
        }
        length = tc2_status_append(
                tc2_ui_restore, sizeof(tc2_ui_restore), length,
                prompt_row);
        for (int index = 0; index < line_length &&
             length + 1 < sizeof(tc2_ui_restore); ++index) {
                tc2_ui_restore[length++] = line[index];
        }
        tc2_ui_restore[length] = 0;
        length = tc2_status_append(
                tc2_ui_restore, sizeof(tc2_ui_restore), length,
                "\033[?25h");
        return length;
}

static int tc2_demo_trains[TC2_DEMO_TRAIN_COUNT] = {
        14, 15, 17
};

typedef struct {
        int speed;
        int endpoint_trim_outbound_mm[TC2_DEMO_TRAIN_COUNT];
        int endpoint_trim_return_mm[TC2_DEMO_TRAIN_COUNT];
        int first_outbound_sensors[TC2_DEMO_TRAIN_COUNT]
                                  [TC2_DEMO_MAX_ROUTE_SENSORS];
        int first_outbound_count[TC2_DEMO_TRAIN_COUNT];
        int outbound_sensors[TC2_DEMO_TRAIN_COUNT]
                            [TC2_DEMO_MAX_ROUTE_SENSORS];
        int outbound_count[TC2_DEMO_TRAIN_COUNT];
        int return_sensors[TC2_DEMO_TRAIN_COUNT]
                          [TC2_DEMO_MAX_ROUTE_SENSORS];
        int return_count[TC2_DEMO_TRAIN_COUNT];
        int first_outbound_station_cursor[TC2_DEMO_TRAIN_COUNT];
        int outbound_station_cursor[TC2_DEMO_TRAIN_COUNT];
        int return_station_cursor[TC2_DEMO_TRAIN_COUNT];
        uint32_t endpoint_dwell_ticks[TC2_DEMO_TRAIN_COUNT];
        uint32_t initial_launch_stagger_ticks[TC2_DEMO_TRAIN_COUNT];
        int outbound_switch_count[TC2_DEMO_TRAIN_COUNT];
        int outbound_switches[TC2_DEMO_TRAIN_COUNT]
                             [TC2_DEMO_MAX_LEG_SWITCHES];
        char outbound_directions[TC2_DEMO_TRAIN_COUNT]
                                [TC2_DEMO_MAX_LEG_SWITCHES];
        int return_switch_count[TC2_DEMO_TRAIN_COUNT];
        int return_switches[TC2_DEMO_TRAIN_COUNT]
                           [TC2_DEMO_MAX_LEG_SWITCHES];
        char return_directions[TC2_DEMO_TRAIN_COUNT]
                              [TC2_DEMO_MAX_LEG_SWITCHES];
        int ui_destinations[TC2_DEMO_TRAIN_COUNT];
        int initial_completion_dependency[TC2_DEMO_TRAIN_COUNT];
        int initial_dependency_release_cursor[TC2_DEMO_TRAIN_COUNT];
        int return_prepare_dependency[TC2_DEMO_TRAIN_COUNT];
        int return_prepare_release_cursor[TC2_DEMO_TRAIN_COUNT];
        int return_prepare_wait_for_return[TC2_DEMO_TRAIN_COUNT];
        int return_prepare_wait_for_endpoint[TC2_DEMO_TRAIN_COUNT];
        unsigned int return_prepare_completion_mask[TC2_DEMO_TRAIN_COUNT];
        int outbound_final_approach_speed[TC2_DEMO_TRAIN_COUNT];
        int return_final_approach_speed[TC2_DEMO_TRAIN_COUNT];
	int return_correction_stop_cursor[TC2_DEMO_TRAIN_COUNT];
	int return_correction_speed[TC2_DEMO_TRAIN_COUNT];
        int defer_third_train;
        int third_train_release_clearance_mm;
} tc2_demo_profile;

enum {
        TC2_DEMO_PROFILE_INTERSECTIONS = 0,
        TC2_DEMO_PROFILE_DESTINATIONS = 1,
        TC2_DEMO_PROFILE_COUNT = 2
};

/*
 * Both profiles are fixed physical scripts.  Detector values are directed
 * Track-D indices; the runtime accepts either member of each physical pair.
 * Every return list is the exact reverse physical corridor of its outbound
 * list, and every return turnout batch preserves the same physical route.
 */
static const tc2_demo_profile tc2_demo_profiles[TC2_DEMO_PROFILE_COUNT] = {
        {
                .speed = 60,
                .endpoint_trim_outbound_mm = {90, 40, 90},
                .endpoint_trim_return_mm = {90, 40, 90},
                .first_outbound_sensors = {
                        {0, 44, 70, 54, 56, 75, 58, 47, -1, -1},
                        {22, 9, 38, -1, -1, -1, -1, -1, -1, -1},
                        {35, 37, 30, -1, -1, -1, -1, -1, -1, -1}
                },
                .first_outbound_count = {8, 3, 3},
                .outbound_sensors = {
                        {44, 70, 54, 56, 75, 58, 47, -1, -1, -1},
                        {38, -1, -1, -1, -1, -1, -1, -1, -1, -1},
                        {37, 30, -1, -1, -1, -1, -1, -1, -1, -1}
                },
                .outbound_count = {7, 1, 2},
                .return_sensors = {
                        {46, 59, 74, 57, 55, 71, 45, 1, -1, -1},
                        {8, -1, -1, -1, -1, -1, -1, -1, -1, -1},
                        {36, 34, -1, -1, -1, -1, -1, -1, -1, -1}
                },
                .return_count = {8, 1, 2},
                .first_outbound_station_cursor = {2, -1, 1},
                .outbound_station_cursor = {1, -1, 0},
                .return_station_cursor = {2, -1, 0},
                .endpoint_dwell_ticks = {300u, 500u, 400u},
                .initial_launch_stagger_ticks = {0u, 600u, 0u},
                .outbound_switch_count = {2, 0, 1},
                .outbound_switches = {
                        {8, 7, 0, 0}, {0, 0, 0, 0}, {18, 0, 0, 0}
                },
                .outbound_directions = {
                        {'S', 'S', 0, 0}, {0, 0, 0, 0}, {'C', 0, 0, 0}
                },
                .return_switch_count = {3, 3, 3},
                .return_switches = {
                        {9, 11, 12, 0}, {3, 2, 1, 0}, {15, 6, 5, 0}
                },
                .return_directions = {
                        {'S', 'S', 'S', 0}, {'C', 'S', 'C', 0},
                        {'S', 'C', 'S', 0}
                },
                .ui_destinations = {5, 3, 1},
                .initial_completion_dependency = {-1, -1, -1},
                .initial_dependency_release_cursor = {-1, -1, -1},
                .return_prepare_dependency = {-1, -1, -1},
                .return_prepare_release_cursor = {-1, -1, -1},
                .return_prepare_wait_for_return = {0, 0, 0},
                .return_prepare_wait_for_endpoint = {0, 0, 0},
                .return_prepare_completion_mask = {0u, 0u, 0u},
                .outbound_final_approach_speed = {0, 0, 0},
                .return_final_approach_speed = {0, 0, 0},
		.return_correction_stop_cursor = {-1, -1, -1},
		.return_correction_speed = {0, 0, 0},
                .defer_third_train = 1,
                .third_train_release_clearance_mm =
                        TC2_DEMO_T17_RELEASE_CLEARANCE_MM
        },
        {
                .speed = 80,
                /* T14 stops at its final detector; no extra d4 trim pulse. */
                .endpoint_trim_outbound_mm = {0, 254, 381},
                /*
                 * T14 returns to A at the final physical detector.  Do not
                 * add a timed trim pulse after that detector: at speed 40 it
                 * was carrying the train beyond the scripted endpoint.
                 */
                .endpoint_trim_return_mm = {0, 0, 140},
                .first_outbound_sensors = {
                        /* A -> d4: A1,C13,E7,D7,E10,E13,D13,B2. */
                        {0, 44, 70, 54, 73, 76, 60, 17, -1, -1},
                        /* C -> d3: B7,A10,C7,E11,D10,D5,E6,D4,B6. */
                        {22, 9, 38, 74, 57, 52, 69, 51, 21, -1},
                        /* F -> d1: C4,C8,A12. */
                        {35, 39, 11, -1, -1, -1, -1, -1, -1, -1}
                },
                .first_outbound_count = {8, 9, 3},
                .outbound_sensors = {
                        {44, 70, 54, 73, 76, 60, 17, -1, -1, -1},
                        {9, 38, 74, 57, 52, 69, 51, 21, -1, -1},
                        {39, 11, -1, -1, -1, -1, -1, -1, -1, -1}
                },
                .outbound_count = {7, 8, 2},
                .return_sensors = {
                        /* d4 -> A. */
                        {16, 61, 77, 72, 55, 71, 45, 1, -1, -1},
                        /* d3 -> C. */
			{20, 50, 68, 53, 56, 75, 39, 8, -1, -1},
                        /* d1 -> F. */
                        {10, 38, 34, -1, -1, -1, -1, -1, -1, -1}
                },
		.return_count = {8, 8, 3},
                .first_outbound_station_cursor = {2, 2, 1},
                .outbound_station_cursor = {1, 1, 0},
                .return_station_cursor = {2, 3, 1},
                /*
                 * T15 remains stopped at d3 for fifteen seconds, then the
                 * fixed script prepares its complete return turnout batch.
                 * This profile deliberately uses elapsed choreography rather
                 * than waiting for peer completion bookkeeping.
                 */
                .endpoint_dwell_ticks = {
                        300u, TC2_DEMO_T15_RETURN_DWELL_TICKS, 500u
                },
                .initial_launch_stagger_ticks = {0u, 0u, 0u},
                .outbound_switch_count = {2, 3, 4},
                .outbound_switches = {
                        {8, 17, 0, 0}, {5, 9, 10, 0}, {18, 3, 2, 1}
                },
                .outbound_directions = {
                        {'C', 'S', 0, 0}, {'C', 'C', 'S', 0},
                        {'S', 'C', 'S', 'S'}
                },
                /*
                 * T15 takes the complete d3 -> C return corridor through
                 * SW13/8/7/18/3/2/1.  Prepare the entire fixed batch before
                 * reverse so the scripted train cannot inherit a turnout
                 * left behind by T14 or T17.  The normal two-second settle
                 * window remains mandatory before motion.
                 *
                 * T17 returns through SW5 after T15 has cleared it.  SW5 is
                 * the branch that selects the route back to F, so it must be
                 * commanded straight and settled before reversing.
                 */
                .return_switch_count = {3, 7, 1},
                .return_switches = {
                        {9, 11, 12},
                        {13, 8, 7, 18, 3, 2, 1},
                        {5}
                },
                .return_directions = {
                        {'S', 'S', 'S'},
                        {'S', 'S', 'C', 'S', 'C', 'S', 'C'},
                        {'S'}
                },
                .ui_destinations = {3, 2, 0},
                /*
                 * T15's d3 corridor shares the SW8 throat with T14.  E10
                 * only confirms that T14's head has passed the throat; its
                 * tail can still be over the turnout when T15 starts its
                 * switch batch.  Hold T15 until T14 reaches E13, the next
                 * real detector, so the complete train has cleared before
                 * another route is allowed to move the shared turnouts.
                 * This still avoids waiting for T14's complete d4 -> A
                 * cycle.
                 */
                .initial_completion_dependency = {-1, 0, -1},
                .initial_dependency_release_cursor = {-1, 6, -1},
                /*
		 * T15's return cursor reaches 8 at A9/A10, the scripted final
		 * detector.  That is the release boundary for the other return
		 * legs; this fake profile deliberately does not chase B7/B8.
                 * T15 prepares its own complete return switch batch after its
                 * fixed fifteen-second endpoint dwell.
                 */
                .return_prepare_dependency = {1, -1, 1},
		.return_prepare_release_cursor = {8, -1, 8},
                .return_prepare_wait_for_return = {0, 0, 0},
                .return_prepare_wait_for_endpoint = {1, 0, 1},
                .return_prepare_completion_mask = {0u, 0u, 0u},
                /*
                 * Slow T14 before B2 and T15 before B6.  The detector still
                 * remains the authoritative trim anchor, but reaching it at
                 * 40 avoids adding an uncontrolled speed-80 coast to the
                 * measured endpoint offset.
                 */
                .outbound_final_approach_speed = {40, 50, 0},
                /*
                 * Return endpoints are approached at low speed so the stop
                 * issued by their final real detector does not coast T14 or
                 * T15 beyond A/C/F.  T17 then receives only its measured,
                 * bounded 140 mm terminal trim.
                 */
                .return_final_approach_speed = {40, 40, 40},
		/*
		 * T15's d3 -> C return enters its final correction corridor at
		 * C7/C8 (cursor 6).  Stop there, then pass A9/A10 at speed 40;
		 * A9/A10 is the terminal detector and immediately issues zero.
		 */
		.return_correction_stop_cursor = {-1, 6, -1},
		.return_correction_speed = {0, 40, 0},
                .defer_third_train = 0,
                .third_train_release_clearance_mm = 0
        }
};

static const int tc2_demo_starts[TC2_DEMO_TRAIN_COUNT] = {
        0, 2, 5 /* A, C, F */
};

static const tc2_demo_profile *tc2_demo_profile_config(void) {
        int profile = tc2_demo_state.profile_id;
        if (profile < 0 || profile >= TC2_DEMO_PROFILE_COUNT) {
                profile = TC2_DEMO_PROFILE_INTERSECTIONS;
        }
        return &tc2_demo_profiles[profile];
}

static int tc2_demo_leg_speed(int slot) {
	const tc2_demo_profile *profile = tc2_demo_profile_config();
	int stop_cursor;

	if (slot < 0 || slot >= TC2_DEMO_TRAIN_COUNT) {
		return profile->speed;
	}
	stop_cursor = profile->return_correction_stop_cursor[slot];
	if (!tc2_demo_state.outbound_leg[slot] && stop_cursor >= 0 &&
	    tc2_demo_state.route_sensor_cursor[slot] > stop_cursor &&
	    profile->return_correction_speed[slot] > 0) {
		return profile->return_correction_speed[slot];
	}
	return profile->speed;
}

static int tc2_demo_is_return_correction_stop(
	int slot, int matched_cursor) {
	const tc2_demo_profile *profile = tc2_demo_profile_config();

	return slot >= 0 && slot < TC2_DEMO_TRAIN_COUNT &&
	       !tc2_demo_state.outbound_leg[slot] &&
	       profile->return_correction_stop_cursor[slot] >= 0 &&
	       matched_cursor == profile->return_correction_stop_cursor[slot];
}


static void tc2_demo_reset_state(void) {
        tc2_demo_state.active = 0;
        tc2_demo_state.initializing = 0;
        tc2_demo_state.profile_id = TC2_DEMO_PROFILE_INTERSECTIONS;
        tc2_demo_state.started_tick = 0;
        tc2_demo_state.next_poll_tick = 0;
        tc2_demo_state.initial_stop_token = 0;
        tc2_demo_state.action_count = 0;
        tc2_demo_state.sensor_failure_count = 0;
        tc2_demo_state.last_running = -1;
        tc2_demo_state.last_waiting = -1;
        tc2_demo_state.last_stopped = -1;
        tc2_demo_state.last_reversing = -1;
        tc2_demo_state.last_elapsed_seconds = -1;
        for (int slot = 0; slot < TC2_DEMO_TRAIN_COUNT; ++slot) {
                tc2_demo_state.completed_cycles[slot] = 0;
                tc2_demo_state.phase[slot] = TC2_DEMO_IDLE;
                tc2_demo_state.phase_started_tick[slot] = 0;
                tc2_demo_state.wait_until_tick[slot] = 0;
                tc2_demo_state.pending_batch_token[slot] = 0;
                tc2_demo_state.outbound_leg[slot] = 1;
                tc2_demo_state.first_departure[slot] = 1;
                tc2_demo_state.route_sensor_cursor[slot] = 0;
                tc2_demo_state.route_armed_mask[slot] = 0;
                tc2_demo_state.sensor_recovery_restarts[slot] = 0;
                tc2_demo_state.ui_plan_generation[slot] =
                        (unsigned int)(slot + 1);
                tc2_demo_state.ui_position_revision[slot] = 0;
		tc2_demo_state.ui_sensor_sequence[slot] = 0;
		tc2_demo_state.ui_segment_started_tick[slot] = 0;
		tc2_demo_state.ui_segment_progress[slot] = 0;
		tc2_demo_state.ui_segment_anchor_sensor[slot] = -1;
		tc2_demo_state.ui_segment_anchor_row[slot] = -1;
		tc2_demo_state.ui_segment_anchor_column[slot] = -1;
		tc2_demo_state.resume_after_wait[slot] = 0;
		tc2_demo_state.reverse_after_wait[slot] = 0;
		tc2_demo_state.deferred_initial_prepare[slot] = 0;
		tc2_demo_state.station_stop[slot] = 0;
		tc2_demo_state.return_leg_started[slot] = 0;
		tc2_demo_state.motion_account_tick[slot] = 0;
		tc2_demo_state.return_running_ticks[slot] = 0;
		tc2_demo_state.return_prepare_release_tick[slot] = 0;
                tc2_demo_state.last_phase[slot] = -1;
        }
}

static const char *tc2_demo_phase_name(int phase) {
        switch (phase) {
        case TC2_DEMO_INITIAL_STOPPING:
        case TC2_DEMO_STOPPED:
        case TC2_DEMO_ENDPOINT_TRIM_STOPPED:
        case TC2_DEMO_SENSOR_RECOVERY:
        case TC2_DEMO_PAUSED:
                return "STOPPED";
        case TC2_DEMO_WAIT_TURNOUT:
                return "WAIT_TURNOUT";
        case TC2_DEMO_RUNNING:
        case TC2_DEMO_ENDPOINT_TRIM_RUNNING:
                return "RUNNING";
        case TC2_DEMO_REVERSING:
                return "REVERSING";
        default:
                return "IDLE";
        }
}

static const int *tc2_demo_leg_sensors(int slot) {
        const tc2_demo_profile *profile = tc2_demo_profile_config();

        if (slot < 0 || slot >= TC2_DEMO_TRAIN_COUNT) return 0;
        if (!tc2_demo_state.outbound_leg[slot]) {
                return profile->return_sensors[slot];
        }
        if (tc2_demo_state.first_departure[slot]) {
                return profile->first_outbound_sensors[slot];
        }
        return profile->outbound_sensors[slot];
}

static int tc2_demo_leg_sensor_count(int slot) {
        const tc2_demo_profile *profile = tc2_demo_profile_config();

        if (slot < 0 || slot >= TC2_DEMO_TRAIN_COUNT) return 0;
        if (!tc2_demo_state.outbound_leg[slot]) {
                return profile->return_count[slot];
        }
        if (tc2_demo_state.first_departure[slot]) {
                return profile->first_outbound_count[slot];
        }
        return profile->outbound_count[slot];
}

static int tc2_demo_leg_station_cursor(int slot) {
        const tc2_demo_profile *profile = tc2_demo_profile_config();

        if (slot < 0 || slot >= TC2_DEMO_TRAIN_COUNT) return -1;
        if (!tc2_demo_state.outbound_leg[slot]) {
                return profile->return_station_cursor[slot];
        }
        if (tc2_demo_state.first_departure[slot]) {
                return profile->first_outbound_station_cursor[slot];
        }
        return profile->outbound_station_cursor[slot];
}

static int tc2_demo_sensor_pair_active(
        const train_sensor_snapshot_t *sensors, int sensor) {
        int first = sensor & ~1;
        if (!sensors || first < 0 ||
            first + 1 >= TRAIN_SENSOR_COUNT) {
                return 0;
        }
        return sensors->sensor_state[first] == 1 ||
                sensors->sensor_state[first + 1] == 1;
}

static int tc2_demo_active_sensor(
        const train_sensor_snapshot_t *sensors, int sensor) {
        int first = sensor & ~1;
        if (!sensors || first < 0 ||
            first + 1 >= TRAIN_SENSOR_COUNT) {
                return -1;
        }
        if (sensors->sensor_state[first] == 1) return first;
        if (sensors->sensor_state[first + 1] == 1) return first + 1;
        return -1;
}

/*
 * Accept the nearest newly-active detector at or ahead of the current cursor.
 * A detector must first have been observed low during this leg before its
 * high level is consumable.  This prevents the endpoint detector from the
 * previous leg (or any other stale high level) from being consumed again and
 * making a train appear to reverse immediately.  Looking ahead still permits
 * recovery from one genuinely missed intermediate pulse without moving the
 * UI backwards.
 */
static int tc2_demo_find_route_sensor(
        const train_sensor_snapshot_t *sensors, int slot,
        int *matched_cursor) {
        const int *route = tc2_demo_leg_sensors(slot);
        int count = tc2_demo_leg_sensor_count(slot);
        int first_cursor;

        if (!route || !matched_cursor) return -1;
        first_cursor = tc2_demo_state.route_sensor_cursor[slot];
        if (first_cursor < 0) first_cursor = 0;
        for (int cursor = first_cursor; cursor < count; ++cursor) {
                unsigned int bit = 1u << (unsigned int)cursor;
                if (!tc2_demo_sensor_pair_active(
                            sensors, route[cursor])) {
                        tc2_demo_state.route_armed_mask[slot] |= bit;
                }
        }
        for (int cursor = first_cursor; cursor < count; ++cursor) {
                unsigned int bit = 1u << (unsigned int)cursor;
                if ((tc2_demo_state.route_armed_mask[slot] & bit) != 0u &&
                    tc2_demo_sensor_pair_active(sensors, route[cursor])) {
                        *matched_cursor = cursor;
                        return tc2_demo_active_sensor(
                                sensors, route[cursor]);
                }
        }
        return -1;
}

static int tc2_demo_initial_stop_complete(void) {
        can_batch_status_t status;

        if (tc2_demo_state.initial_stop_token == 0) return 1;
        if (tc_can_server_tid < 0 ||
            CanGetBatchStatus(
                    tc_can_server_tid,
                    tc2_demo_state.initial_stop_token,
                    &status) < 0 ||
            status.token != tc2_demo_state.initial_stop_token ||
            status.state == CAN_BATCH_FAILED) {
                return -1;
        }
        if (status.state == CAN_BATCH_PENDING) return 0;
        tc2_demo_state.initial_stop_token = 0;
        return 1;
}

static void tc2_demo_set_ui_speed(
        int slot, int speed, uint32_t now_tick) {
        (void)Tc2UiOverlaySetTrainSpeed(
                &tc2_live_ui_state.overlay,
                tc2_demo_trains[slot], speed,
                tc2_demo_state.ui_plan_generation[slot],
                now_tick);
}

/*
 * Keep the fake demonstration visually continuous without inventing sensor
 * evidence.  Each running segment starts at the last displayed/confirmed
 * cell and may advance only to a point strictly before its next unconfirmed
 * detector.  The real detector event below remains authoritative and rebases
 * both the marker and the next visual segment.
 */
static void tc2_demo_begin_ui_segment(
        int slot, uint32_t now_tick) {
        const tc2_ui_train_overlay *shown;

        if (slot < 0 || slot >= TC2_DEMO_TRAIN_COUNT) return;
        shown = Tc2UiOverlayFindTrain(
                &tc2_live_ui_state.overlay,
                tc2_demo_trains[slot]);
        if (!shown) return;
        tc2_demo_state.ui_segment_started_tick[slot] = now_tick;
        tc2_demo_state.ui_segment_progress[slot] = 0;
        tc2_demo_state.ui_segment_anchor_row[slot] = shown->row;
        tc2_demo_state.ui_segment_anchor_column[slot] = shown->column;
        if (shown->sensor_index >= 0) {
                tc2_demo_state.ui_segment_anchor_sensor[slot] =
                        shown->sensor_index;
        }
}

static uint32_t tc2_demo_ui_segment_ticks(int slot) {
	int velocity = tc_velocity_for_speed(tc2_demo_leg_speed(slot));
        uint32_t ticks;

        if (velocity < 1) velocity = 1;
        ticks = (uint32_t)((TC2_DEMO_UI_SEGMENT_DISTANCE_MM +
                            velocity - 1) / velocity);
        if (ticks < TC2_DEMO_POLL_TICKS * 4u) {
                ticks = TC2_DEMO_POLL_TICKS * 4u;
        }
        return ticks;
}

static void tc2_demo_update_ui_prediction(
        int slot, uint32_t now_tick) {
        const int *route;
        const tc2_ui_train_overlay *shown;
        tc2_track_d_directed_sensor_cell next_cell;
        tc2_dispatch_projection_waypoint first = {0};
        tc2_dispatch_projection_waypoint second = {0};
        uint32_t duration;
        uint32_t elapsed;
        unsigned int progress;
        int cursor;
        int count;
        int row;
        int column;

        if (slot < 0 || slot >= TC2_DEMO_TRAIN_COUNT ||
            tc2_demo_state.phase[slot] != TC2_DEMO_RUNNING) {
                return;
        }
        route = tc2_demo_leg_sensors(slot);
        cursor = tc2_demo_state.route_sensor_cursor[slot];
        count = tc2_demo_leg_sensor_count(slot);
        if (!route || cursor < 0 || cursor >= count ||
            tc2_demo_state.ui_segment_anchor_row[slot] < 0 ||
            tc2_demo_state.ui_segment_anchor_column[slot] < 0 ||
            Tc2TrackDDirectedSensorCell(
                    route[cursor], &next_cell) < 0) {
                return;
        }

	duration = tc2_demo_ui_segment_ticks(slot);
        elapsed = now_tick -
                tc2_demo_state.ui_segment_started_tick[slot];
        progress = elapsed >= duration ?
                TC2_DEMO_UI_UNCONFIRMED_LIMIT :
                (unsigned int)(((uint64_t)elapsed *
                                TC2_DEMO_UI_PROGRESS_SCALE) /
                               duration);
        if (progress > TC2_DEMO_UI_UNCONFIRMED_LIMIT) {
                progress = TC2_DEMO_UI_UNCONFIRMED_LIMIT;
        }
        if (progress <= tc2_demo_state.ui_segment_progress[slot]) {
                return;
        }

        first.kind =
                tc2_demo_state.ui_segment_anchor_sensor[slot] >= 0 ?
                TC2_ROUTE_WAYPOINT_SENSOR : TC2_ROUTE_WAYPOINT_START;
        first.sensor_index = (int16_t)
                tc2_demo_state.ui_segment_anchor_sensor[slot];
        first.destination_index = -1;
        first.ui_row = (uint8_t)
                tc2_demo_state.ui_segment_anchor_row[slot];
        first.ui_column = (uint8_t)
                tc2_demo_state.ui_segment_anchor_column[slot];
        first.ui_width = 1;
        second.kind = TC2_ROUTE_WAYPOINT_SENSOR;
        second.sensor_index = (int16_t)route[cursor];
        second.destination_index = -1;
        second.ui_row = next_cell.row;
        second.ui_column = next_cell.column;
        second.ui_width = next_cell.width;
        if (!Tc2LiveUiInterpolateTrackCell(
                    &first, &second,
                    (int64_t)progress,
                    (int64_t)TC2_DEMO_UI_PROGRESS_SCALE,
                    &row, &column)) {
                return;
        }
        tc2_demo_state.ui_segment_progress[slot] = progress;
        shown = Tc2UiOverlayFindTrain(
                &tc2_live_ui_state.overlay,
                tc2_demo_trains[slot]);
        if (shown && shown->row == row && shown->column == column) {
                return;
        }
        ++tc2_demo_state.ui_position_revision[slot];
        if (tc2_demo_state.ui_position_revision[slot] == 0) {
                ++tc2_demo_state.ui_position_revision[slot];
        }
        (void)Tc2UiOverlayUpdatePredictedCell(
                &tc2_live_ui_state.overlay,
                tc2_demo_trains[slot], row, column,
                tc2_demo_state.ui_plan_generation[slot],
                tc2_demo_state.ui_position_revision[slot],
                now_tick);
}

static void tc2_demo_set_ui_at_sensor(
        int slot, int sensor_index, uint32_t now_tick) {
	tc2_track_d_directed_sensor_cell cell;

        ++tc2_demo_state.ui_position_revision[slot];
        ++tc2_demo_state.ui_sensor_sequence[slot];
        (void)Tc2UiOverlayUpdateAtSensor(
                &tc2_live_ui_state.overlay,
                tc2_demo_trains[slot], sensor_index,
                tc2_demo_state.ui_plan_generation[slot],
                tc2_demo_state.ui_position_revision[slot],
                tc2_demo_state.ui_sensor_sequence[slot],
                TC2_UI_QUALITY_SENSOR_CONFIRMED, now_tick);
	if (Tc2TrackDDirectedSensorCell(sensor_index, &cell) >= 0) {
		tc2_demo_state.ui_segment_started_tick[slot] = now_tick;
		tc2_demo_state.ui_segment_progress[slot] = 0;
		tc2_demo_state.ui_segment_anchor_sensor[slot] = sensor_index;
		tc2_demo_state.ui_segment_anchor_row[slot] = cell.row;
		tc2_demo_state.ui_segment_anchor_column[slot] = cell.column;
	}
}

static void tc2_demo_publish_status(
        uint32_t now_tick, int running, int waiting,
        int stopped, int reversing, int force) {
        const tc2_demo_profile *profile = tc2_demo_profile_config();
        size_t length;
        int phase_changed = 0;
        unsigned int elapsed_seconds =
                (unsigned int)((now_tick - tc2_demo_state.started_tick) /
                               100u);

        for (int slot = 0; slot < TC2_DEMO_TRAIN_COUNT; ++slot) {
                if (tc2_demo_state.last_phase[slot] !=
                    tc2_demo_state.phase[slot]) {
                        phase_changed = 1;
                        break;
                }
        }
        if (!force &&
            !phase_changed &&
            running == tc2_demo_state.last_running &&
            waiting == tc2_demo_state.last_waiting &&
            stopped == tc2_demo_state.last_stopped &&
            reversing == tc2_demo_state.last_reversing &&
            (int)elapsed_seconds ==
                    tc2_demo_state.last_elapsed_seconds) {
                return;
        }
        tc2_demo_state.last_running = running;
        tc2_demo_state.last_waiting = waiting;
        tc2_demo_state.last_stopped = stopped;
        tc2_demo_state.last_reversing = reversing;
        tc2_demo_state.last_elapsed_seconds =
                (int)elapsed_seconds;
        for (int slot = 0; slot < TC2_DEMO_TRAIN_COUNT; ++slot) {
                tc2_demo_state.last_phase[slot] =
                        tc2_demo_state.phase[slot];
        }

        tc2_ui_status[0] = 0;
        length = tc2_status_append(
                tc2_ui_status, sizeof(tc2_ui_status), 0,
                "RETURNBACK speed=");
        length = tc2_status_append_uint(
                tc2_ui_status, sizeof(tc2_ui_status), length,
                (unsigned int)profile->speed);
        length = tc2_status_append(
                tc2_ui_status, sizeof(tc2_ui_status), length,
                " elapsed=");
        length = tc2_status_append_uint(
                tc2_ui_status, sizeof(tc2_ui_status), length,
                elapsed_seconds);
        length = tc2_status_append(
                tc2_ui_status, sizeof(tc2_ui_status), length,
                "s RUNNING=");
        length = tc2_status_append_uint(
                tc2_ui_status, sizeof(tc2_ui_status), length,
                (unsigned int)running);
        length = tc2_status_append(
                tc2_ui_status, sizeof(tc2_ui_status), length,
                " WAIT_TURNOUT=");
        length = tc2_status_append_uint(
                tc2_ui_status, sizeof(tc2_ui_status), length,
                (unsigned int)waiting);
        length = tc2_status_append(
                tc2_ui_status, sizeof(tc2_ui_status), length,
                " STOPPED=");
        length = tc2_status_append_uint(
                tc2_ui_status, sizeof(tc2_ui_status), length,
                (unsigned int)stopped);
        length = tc2_status_append(
                tc2_ui_status, sizeof(tc2_ui_status), length,
                " REVERSING=");
        length = tc2_status_append_uint(
                tc2_ui_status, sizeof(tc2_ui_status), length,
                (unsigned int)reversing);
        length = tc2_status_append(
                tc2_ui_status, sizeof(tc2_ui_status), length,
                " actions=");
        length = tc2_status_append_uint(
                tc2_ui_status, sizeof(tc2_ui_status), length,
                tc2_demo_state.action_count);
        length = tc2_status_append(
                tc2_ui_status, sizeof(tc2_ui_status), length,
                " T14=");
        length = tc2_status_append(
                tc2_ui_status, sizeof(tc2_ui_status), length,
                tc2_demo_phase_name(tc2_demo_state.phase[0]));
        length = tc2_status_append(
                tc2_ui_status, sizeof(tc2_ui_status), length,
                " T15=");
        length = tc2_status_append(
                tc2_ui_status, sizeof(tc2_ui_status), length,
                tc2_demo_phase_name(tc2_demo_state.phase[1]));
        length = tc2_status_append(
                tc2_ui_status, sizeof(tc2_ui_status), length,
                " T17=");
        (void)tc2_status_append(
                tc2_ui_status, sizeof(tc2_ui_status), length,
                tc2_demo_phase_name(tc2_demo_state.phase[2]));
        tc2_ui_status_dirty = 1;
}

static void tc2_demo_publish_phase_status(
        uint32_t now_tick, int force) {
        int running = 0;
        int waiting = 0;
        int stopped = 0;
        int reversing = 0;

        for (int slot = 0; slot < TC2_DEMO_TRAIN_COUNT; ++slot) {
                switch (tc2_demo_state.phase[slot]) {
                case TC2_DEMO_WAIT_TURNOUT:
                        ++waiting;
                        break;
                case TC2_DEMO_RUNNING:
                case TC2_DEMO_ENDPOINT_TRIM_RUNNING:
                        ++running;
                        break;
                case TC2_DEMO_REVERSING:
                        ++reversing;
                        break;
                case TC2_DEMO_INITIAL_STOPPING:
                case TC2_DEMO_STOPPED:
                case TC2_DEMO_ENDPOINT_TRIM_STOPPED:
                case TC2_DEMO_SENSOR_RECOVERY:
                case TC2_DEMO_PAUSED:
                        ++stopped;
                        break;
                default:
                        break;
                }
        }
        tc2_demo_publish_status(
                now_tick, running, waiting, stopped, reversing, force);
}

static void tc2_demo_pause_train(int slot, uint32_t now_tick) {
        if (slot < 0 || slot >= TC2_DEMO_TRAIN_COUNT) return;
        if (tc_can_server_tid >= 0) {
                (void)CanTrainSetSpeedPriority(
                        tc_can_server_tid,
                        tc2_demo_trains[slot], 0);
        }
        tc2_demo_state.phase[slot] = TC2_DEMO_PAUSED;
        tc2_demo_state.phase_started_tick[slot] = now_tick;
	tc2_demo_state.pending_batch_token[slot] = 0;
	tc2_demo_state.resume_after_wait[slot] = 0;
	tc2_demo_state.reverse_after_wait[slot] = 0;
	tc2_demo_state.deferred_initial_prepare[slot] = 0;
	tc2_demo_state.station_stop[slot] = 0;
        tc2_demo_state.route_armed_mask[slot] = 0;
        tc2_demo_set_ui_speed(slot, 0, now_tick);
        ++tc2_demo_state.action_count;
}

/*
 * A missed detector never authorizes a reverse.  Stop only the affected
 * corridor, retain its forward route cursor, and perform at most a small
 * number of same-direction restart attempts.  The old implementation stayed
 * at speed zero while waiting for a detector in front of the train, which was
 * an unrecoverable state by construction.
 */
static void tc2_demo_enter_sensor_recovery(
        int slot, uint32_t now_tick) {
        if (slot < 0 || slot >= TC2_DEMO_TRAIN_COUNT) return;
        if (tc_can_server_tid >= 0) {
                (void)CanTrainSetSpeedPriority(
                        tc_can_server_tid,
                        tc2_demo_trains[slot], 0);
        }
        tc2_demo_state.phase[slot] = TC2_DEMO_SENSOR_RECOVERY;
	tc2_demo_state.phase_started_tick[slot] = now_tick;
	tc2_demo_state.resume_after_wait[slot] = 0;
	tc2_demo_state.reverse_after_wait[slot] = 0;
	tc2_demo_state.station_stop[slot] = 0;
        tc2_demo_set_ui_speed(slot, 0, now_tick);
        ++tc2_demo_state.sensor_recovery_restarts[slot];
        ++tc2_demo_state.action_count;
}

static void tc2_demo_pause_all(
        uint32_t now_tick, const char *reason) {
        if (tc_can_server_tid >= 0) {
                (void)CanTrainEmergencyStopBatch(
                        tc_can_server_tid, tc2_demo_trains,
                        TC2_DEMO_TRAIN_COUNT);
        }
        tc2_demo_state.initializing = 0;
        tc2_demo_state.initial_stop_token = 0;
        for (int slot = 0; slot < TC2_DEMO_TRAIN_COUNT; ++slot) {
                tc2_demo_state.phase[slot] = TC2_DEMO_PAUSED;
                tc2_demo_state.phase_started_tick[slot] = now_tick;
		tc2_demo_state.pending_batch_token[slot] = 0;
		tc2_demo_state.resume_after_wait[slot] = 0;
		tc2_demo_state.reverse_after_wait[slot] = 0;
		tc2_demo_state.deferred_initial_prepare[slot] = 0;
		tc2_demo_state.station_stop[slot] = 0;
                tc2_demo_state.route_armed_mask[slot] = 0;
                tc2_demo_set_ui_speed(slot, 0, now_tick);
        }
        tc2_demo_publish_phase_status(now_tick, 1);
        tc2_status_set(reason);
}

/*
 * The dispatcher relay transports at most four turnout commands per IPC
 * request.  A fixed demo leg may describe a longer logical route (T15's
 * d3 -> C return currently has seven turnouts), so submit that route as
 * consecutive, synchronously confirmed relay batches.  The train remains
 * stopped throughout this helper and tc2_demo_prepare_leg starts the normal
 * two-second mechanical settle interval only after every chunk succeeds.
 */
static int tc2_demo_set_leg_turnouts(
        const int *switches, const char *directions, int count) {
        int offset = 0;

        if (count < 0 || count > TC2_DEMO_MAX_LEG_SWITCHES) return -1;
        if (count == 0) return 0;
        if (!switches || !directions) return -1;

        while (offset < count) {
                int chunk = count - offset;
                if (chunk > TC2_DEMO_TURNOUT_BATCH_LIMIT) {
                        chunk = TC2_DEMO_TURNOUT_BATCH_LIMIT;
                }
                if (Tc2DispatchDemoSwitchBatch(
                            tc_dispatch_server_tid,
                            switches + offset,
                            directions + offset,
                            chunk) < 0) {
                        return -1;
                }
                offset += chunk;
        }
        return 0;
}

/*
 * Turnouts are issued by the dispatcher, the sole CAN turnout authority.
 * Fixed-profile dependencies serialize any shared throat before a train
 * reaches this function.  Each accepted call therefore prepares one complete
 * leg while its train remains stopped, without relying on ordinary dispatcher
 * job state.
 */
static int tc2_demo_prepare_leg(
        int slot, uint32_t now_tick,
        const train_sensor_snapshot_t *sensors) {
        const tc2_demo_profile *profile = tc2_demo_profile_config();
        const int *switches;
        const char *directions;
        int count;
        int raw_time;
        uint32_t confirmed_tick;

        if (slot < 0 || slot >= TC2_DEMO_TRAIN_COUNT) return -1;
        if (tc2_demo_state.outbound_leg[slot]) {
                switches = profile->outbound_switches[slot];
                directions = profile->outbound_directions[slot];
                count = profile->outbound_switch_count[slot];
        } else {
                switches = profile->return_switches[slot];
                directions = profile->return_directions[slot];
                count = profile->return_switch_count[slot];
        }
        if (tc2_demo_set_leg_turnouts(
                    switches, directions, count) < 0) {
                return -1;
        }
        raw_time = Time();
        confirmed_tick = raw_time < 0 ? now_tick : (uint32_t)raw_time;
        tc2_demo_state.phase[slot] = TC2_DEMO_WAIT_TURNOUT;
        tc2_demo_state.phase_started_tick[slot] = confirmed_tick;
        tc2_demo_state.wait_until_tick[slot] =
                confirmed_tick + TC2_DEMO_WAIT_TURNOUT_TICKS +
                ((tc2_demo_state.outbound_leg[slot] &&
                  tc2_demo_state.first_departure[slot]) ?
                 profile->initial_launch_stagger_ticks[slot] : 0u);
        tc2_demo_state.pending_batch_token[slot] = 0;
        tc2_demo_state.route_sensor_cursor[slot] = 0;
        tc2_demo_state.route_armed_mask[slot] = 0;
        tc2_demo_state.sensor_recovery_restarts[slot] = 0;
        {
                const int *route = tc2_demo_leg_sensors(slot);
                int sensor_count = tc2_demo_leg_sensor_count(slot);
                for (int cursor = 0; cursor < sensor_count; ++cursor) {
                        if (!tc2_demo_sensor_pair_active(
                                    sensors, route[cursor])) {
                                tc2_demo_state.route_armed_mask[slot] |=
                                        1u << (unsigned int)cursor;
                        }
                }
        }
        tc2_demo_state.resume_after_wait[slot] = 0;
        tc2_demo_state.station_stop[slot] = 0;
        tc2_demo_state.action_count += (unsigned int)count;
        return 0;
}

static uint32_t tc2_demo_endpoint_trim_ticks(int slot) {
	const tc2_demo_profile *profile = tc2_demo_profile_config();
	int velocity = tc_velocity_for_speed(TC2_DEMO_ENDPOINT_TRIM_SPEED);
	int distance = 40;
	if (slot >= 0 && slot < TC2_DEMO_TRAIN_COUNT) {
		distance = tc2_demo_state.outbound_leg[slot] ?
			profile->endpoint_trim_outbound_mm[slot] :
			profile->endpoint_trim_return_mm[slot];
	}
	if (velocity < 1) velocity = 1;
	return (uint32_t)((distance + velocity - 1) /
	                  velocity);
}

static uint32_t tc2_demo_t17_release_clearance_ticks(void) {
	const tc2_demo_profile *profile = tc2_demo_profile_config();
	int velocity = tc_velocity_for_speed(profile->speed);
	if (velocity < 1) velocity = 1;
	return (uint32_t)((profile->third_train_release_clearance_mm +
	                  velocity - 1) /
	                  velocity);
}

/*
 * Each scripted endpoint has a fixed, hardware-measured trim beyond its final
 * detector.  First stop and settle the train, then perform one bounded
 * low-speed trim.  This phase is terminal-only; intermediate detector and
 * station handling never enters it.
 */
static int tc2_demo_begin_endpoint_trim(int slot, uint32_t now_tick) {
	if (slot < 0 || slot >= TC2_DEMO_TRAIN_COUNT ||
	    tc_can_server_tid < 0 ||
	    CanTrainSetSpeedPriority(
		    tc_can_server_tid, tc2_demo_trains[slot], 0) < 0) {
		return -1;
	}
	tc2_demo_state.phase[slot] = TC2_DEMO_ENDPOINT_TRIM_STOPPED;
	tc2_demo_state.phase_started_tick[slot] = now_tick;
	tc2_demo_state.wait_until_tick[slot] =
		now_tick + TC2_DEMO_ENDPOINT_BRAKE_SETTLE_TICKS;
	tc2_demo_state.station_stop[slot] = 0;
	tc2_demo_set_ui_speed(slot, 0, now_tick);
	++tc2_demo_state.action_count;
	return 0;
}

static int tc2_start_physical_demo(
        uint32_t now_tick, int train_a, int train_c, int train_f,
        int profile_id) {
        tc2_dispatch_snapshot dispatch;
        train_sensor_snapshot_t sensors;
        can_health_t health;
        int token;

        if (tc_dispatch_server_tid < 0 ||
            tc_sensor_server_tid < 0 ||
            tc_can_server_tid < 0 ||
            Tc2DispatchGetSnapshot(
                    tc_dispatch_server_tid, &dispatch) < 0 ||
            !dispatch.scheduler_healthy || dispatch.job_count != 0 ||
            TrainSensorGetLatest(
                    tc_sensor_server_tid, &sensors) < 0 ||
            !sensors.sensor_server_registered || !sensors.courier_ready ||
            CanGetHealth(tc_can_server_tid, &health) < 0 ||
            !health.hw_ready) {
                return -1;
        }

        if (profile_id < 0 || profile_id >= TC2_DEMO_PROFILE_COUNT ||
            train_a < 1 || train_a > 255 ||
            train_c < 1 || train_c > 255 ||
            train_f < 1 || train_f > 255 ||
            train_a == train_c || train_a == train_f ||
            train_c == train_f) {
                return -1;
        }

        tc2_demo_reset_state();
        tc2_demo_state.profile_id = profile_id;
        tc2_demo_trains[0] = train_a;
        tc2_demo_trains[1] = train_c;
        tc2_demo_trains[2] = train_f;
        token = CanTrainEmergencyStopBatch(
                tc_can_server_tid, tc2_demo_trains,
                TC2_DEMO_TRAIN_COUNT);
        if (token <= 0) return -1;

        tc2_demo_state.active = 1;
        tc2_demo_state.initializing = 1;
        tc2_demo_state.started_tick = now_tick;
        tc2_demo_state.next_poll_tick = now_tick;
        tc2_demo_state.initial_stop_token = (unsigned int)token;
        Tc2LiveUiInitialize(&tc2_live_ui_state, now_tick);
        for (int slot = 0; slot < TC2_DEMO_TRAIN_COUNT; ++slot) {
                tc2_demo_state.phase[slot] =
                        TC2_DEMO_INITIAL_STOPPING;
                tc2_demo_state.phase_started_tick[slot] = now_tick;
                if (Tc2UiOverlayUpsertAtStart(
                            &tc2_live_ui_state.overlay,
                            tc2_demo_trains[slot],
                            tc2_demo_starts[slot],
                            tc2_demo_profile_config()
                                    ->ui_destinations[slot],
                            tc2_demo_state.ui_plan_generation[slot],
                            TC2_UI_QUALITY_ESTIMATED,
                            now_tick) < 0) {
                        tc2_demo_pause_all(
                                now_tick,
                                "RETURNBACK PAUSED: UI initialization failed");
                        return -1;
                }
		{
			const tc2_track_d_start_layout *start =
				Tc2TrackDStartLayout((size_t)tc2_demo_starts[slot]);
			if (start) {
				tc2_demo_state.ui_segment_started_tick[slot] =
					now_tick;
				tc2_demo_state.ui_segment_progress[slot] = 0;
				tc2_demo_state.ui_segment_anchor_sensor[slot] = -1;
				tc2_demo_state.ui_segment_anchor_row[slot] =
					start->row;
				tc2_demo_state.ui_segment_anchor_column[slot] =
					start->column;
			}
		}
        }
        tc2_ui_active = 1;
        tc2_ui_live = 1;
        tc2_ui_shutdown_requested = 0;
        tc2_ui_shutdown_frame_queued = 0;
        tc2_ui_next_render_tick = 0;
        tc2_demo_publish_phase_status(now_tick, 1);
        return 0;
}

static void tc2_step_physical_demo(uint32_t now_tick) {
        const tc2_demo_profile *profile = tc2_demo_profile_config();
        train_sensor_snapshot_t sensors;

        if (!tc2_demo_state.active ||
            !tc2_tick_due(
                    now_tick, tc2_demo_state.next_poll_tick)) {
                return;
        }
        tc2_demo_state.next_poll_tick =
                now_tick + TC2_DEMO_POLL_TICKS;
        if (tc_sensor_server_tid < 0 ||
            TrainSensorGetLatest(
                    tc_sensor_server_tid, &sensors) < 0 ||
            !sensors.sensor_server_registered || !sensors.courier_ready) {
                ++tc2_demo_state.sensor_failure_count;
                if (tc2_demo_state.sensor_failure_count >=
                    TC2_DEMO_SENSOR_FAILURE_LIMIT) {
                        tc2_demo_pause_all(
                                now_tick,
                                "RETURNBACK PAUSED: sustained sensor service failure");
                }
                return;
        }
        tc2_demo_state.sensor_failure_count = 0;
        (void)Tc2UiOverlayExpireSensorFlashes(
                &tc2_live_ui_state.overlay, now_tick);

        if (tc2_demo_state.initializing) {
                int complete = tc2_demo_initial_stop_complete();
                if (complete < 0) {
                        tc2_demo_pause_all(
                                now_tick,
                                "RETURNBACK PAUSED: initial stop failed");
                        return;
                }
                if (complete == 0 ||
                    !tc2_tick_due(
                            now_tick,
                            tc2_demo_state.started_tick +
                                    TC2_DEMO_STOP_DWELL_TICKS)) {
                        tc2_demo_publish_phase_status(now_tick, 0);
                        return;
                }
                tc2_demo_state.initializing = 0;
		for (int slot = 0;
		     slot < TC2_DEMO_TRAIN_COUNT; ++slot) {
			/*
			 * A profile may defer one fixed corridor until its declared
			 * predecessor has completed, or use the original T17 return-leg
			 * clearance gate.  Trains without either gate prepare immediately.
			 */
			if ((slot == 2 && profile->defer_third_train) ||
			    profile->initial_completion_dependency[slot] >= 0) {
				tc2_demo_state.phase[slot] =
					TC2_DEMO_WAIT_TURNOUT;
				tc2_demo_state.phase_started_tick[slot] =
					now_tick;
				tc2_demo_state.deferred_initial_prepare[slot] = 1;
				continue;
			}
			if (tc2_demo_prepare_leg(
				    slot, now_tick, &sensors) < 0) {
                                tc2_demo_pause_train(slot, now_tick);
                        }
                }
                tc2_demo_publish_phase_status(now_tick, 1);
                return;
        }

        for (int slot = 0; slot < TC2_DEMO_TRAIN_COUNT; ++slot) {
		uint32_t previous_account_tick =
			tc2_demo_state.motion_account_tick[slot];
		if (previous_account_tick != 0u &&
		    !tc2_demo_state.outbound_leg[slot] &&
		    tc2_demo_state.phase[slot] == TC2_DEMO_RUNNING) {
			uint32_t elapsed = now_tick - previous_account_tick;
			uint32_t required =
				tc2_demo_t17_release_clearance_ticks();
			uint32_t accumulated =
				tc2_demo_state.return_running_ticks[slot];
			if (accumulated < required) {
				uint32_t remaining = required - accumulated;
				tc2_demo_state.return_running_ticks[slot] +=
					elapsed < remaining ? elapsed : remaining;
			}
		}
		tc2_demo_state.motion_account_tick[slot] = now_tick;
                switch (tc2_demo_state.phase[slot]) {
	case TC2_DEMO_WAIT_TURNOUT:
			if (tc2_demo_state.deferred_initial_prepare[slot]) {
				int dependency =
					profile->initial_completion_dependency[slot];
				uint32_t clearance_ticks =
					tc2_demo_t17_release_clearance_ticks();
				/*
				 * Count only commanded RUNNING time on the return legs.  Stops,
				 * turnout waits and reversing time do not consume clearance.
				 * Endpoint confirmation is also sufficient for a short route.
				 */
				if (dependency >= 0 &&
				    dependency < TC2_DEMO_TRAIN_COUNT) {
					int release_cursor =
						profile->initial_dependency_release_cursor[slot];
					int dependency_clear =
						tc2_demo_state.completed_cycles[dependency] ||
						(release_cursor >= 0 &&
						 tc2_demo_state.route_sensor_cursor[dependency] >=
							release_cursor);
					if (!dependency_clear) {
						break;
					}
				} else if (!tc2_demo_state.return_leg_started[0] ||
				    !tc2_demo_state.return_leg_started[1] ||
				    (tc2_demo_state.return_running_ticks[0] <
					     clearance_ticks &&
				     tc2_demo_state.route_sensor_cursor[0] <
					     profile->return_count[0]) ||
				    (tc2_demo_state.return_running_ticks[1] <
					     clearance_ticks &&
				     tc2_demo_state.route_sensor_cursor[1] <
					     profile->return_count[1])) {
					break;
				}
				tc2_demo_state.deferred_initial_prepare[slot] = 0;
				if (tc2_demo_prepare_leg(
					    slot, now_tick, &sensors) < 0) {
					tc2_demo_pause_train(slot, now_tick);
				}
				break;
			}
			if (!tc2_tick_due(
                                    now_tick,
                                    tc2_demo_state
						.wait_until_tick[slot])) {
				break;
			}
			/*
			 * At a turnaround, prepare and settle the complete return
			 * turnout set while the train is still stopped.  Only after
			 * that two-second settle window may the physical direction be
			 * reversed.  This prevents a train from reversing on a turnout
			 * that has just moved underneath it.
			 */
			if (tc2_demo_state.reverse_after_wait[slot]) {
				if (CanTrainReversePriority(
						    tc_can_server_tid,
						    tc2_demo_trains[slot]) < 0) {
					tc2_demo_pause_train(slot, now_tick);
					break;
				}
				tc2_demo_state.reverse_after_wait[slot] = 0;
				tc2_demo_state.phase[slot] =
					TC2_DEMO_REVERSING;
				tc2_demo_state.phase_started_tick[slot] = now_tick;
				++tc2_demo_state.action_count;
				break;
			}
			{
				int leg_speed = tc2_demo_leg_speed(slot);
				if (CanTrainSetSpeedPriority(
					    tc_can_server_tid,
					    tc2_demo_trains[slot],
					    leg_speed) < 0) {
                                tc2_demo_pause_train(slot, now_tick);
                                break;
				}
                        tc2_demo_state.phase[slot] = TC2_DEMO_RUNNING;
                        tc2_demo_state.phase_started_tick[slot] = now_tick;
                        if (!tc2_demo_state.resume_after_wait[slot]) {
                                tc2_demo_state.route_sensor_cursor[slot] = 0;
                        }
			if (!tc2_demo_state.outbound_leg[slot]) {
				tc2_demo_state.return_leg_started[slot] = 1;
			}
                        tc2_demo_state.resume_after_wait[slot] = 0;
                        tc2_demo_set_ui_speed(
                                slot, leg_speed, now_tick);
			tc2_demo_begin_ui_segment(slot, now_tick);
                        ++tc2_demo_state.action_count;
			}
                        break;

                case TC2_DEMO_RUNNING: {
                        int matched_cursor = -1;
                        int sensor = tc2_demo_find_route_sensor(
                                &sensors, slot, &matched_cursor);
                        int count = tc2_demo_leg_sensor_count(slot);
                        if (sensor >= 0 && matched_cursor >= 0) {
                                tc2_demo_set_ui_at_sensor(
                                        slot, sensor, now_tick);
                                tc2_demo_state.route_sensor_cursor[slot] =
                                        matched_cursor + 1;
                                tc2_demo_state.phase_started_tick[slot] =
                                        now_tick;
                                tc2_demo_state
                                        .sensor_recovery_restarts[slot] = 0;
                                if (matched_cursor + 2 == count) {
                                        int approach_speed =
                                                tc2_demo_state.outbound_leg[slot] ?
                                                profile->outbound_final_approach_speed
                                                        [slot] :
                                                profile->return_final_approach_speed
                                                        [slot];
                                        if (approach_speed > 0) {
                                                if (CanTrainSetSpeedPriority(
                                                            tc_can_server_tid,
                                                            tc2_demo_trains[slot],
                                                            approach_speed) < 0) {
                                                        tc2_demo_pause_train(
                                                                slot, now_tick);
                                                        break;
                                                }
                                                tc2_demo_set_ui_speed(
                                                        slot, approach_speed,
                                                        now_tick);
                                                ++tc2_demo_state.action_count;
                                        }
                                }
                                if (matched_cursor + 1 >= count) {
                                        if (tc2_demo_begin_endpoint_trim(
                                                    slot, now_tick) < 0) {
                                                tc2_demo_pause_train(
                                                        slot, now_tick);
                                        }
				} else if (tc2_demo_is_return_correction_stop(
						   slot, matched_cursor)) {
					if (CanTrainSetSpeedPriority(
						    tc_can_server_tid,
						    tc2_demo_trains[slot], 0) < 0) {
						tc2_demo_pause_train(slot, now_tick);
						break;
					}
					tc2_demo_state.phase[slot] =
						TC2_DEMO_STOPPED;
					tc2_demo_state.phase_started_tick[slot] =
						now_tick;
					tc2_demo_state.station_stop[slot] = 1;
					tc2_demo_set_ui_speed(slot, 0, now_tick);
					++tc2_demo_state.action_count;
                                } else if (matched_cursor ==
                                           tc2_demo_leg_station_cursor(
                                                   slot)) {
                                        if (CanTrainSetSpeedPriority(
                                                    tc_can_server_tid,
                                                    tc2_demo_trains[slot],
                                                    0) < 0) {
                                                tc2_demo_pause_train(
                                                        slot, now_tick);
                                                break;
                                        }
                                        tc2_demo_state.phase[slot] =
                                                TC2_DEMO_STOPPED;
                                        tc2_demo_state
                                                .phase_started_tick[slot] =
                                                        now_tick;
                                        tc2_demo_state.station_stop[slot] = 1;
                                        tc2_demo_set_ui_speed(
                                                slot, 0, now_tick);
                                        ++tc2_demo_state.action_count;
                                }
			} else {
				tc2_demo_update_ui_prediction(slot, now_tick);
				if (tc2_tick_due(
                                           now_tick,
                                           tc2_demo_state
                                                   .phase_started_tick[slot] +
                                                   TC2_DEMO_SENSOR_TIMEOUT_TICKS)) {
                                if (tc2_demo_state
                                            .sensor_recovery_restarts[slot] >=
                                    TC2_DEMO_MAX_SENSOR_RECOVERY_RESTARTS) {
                                        tc2_demo_pause_train(slot, now_tick);
                                } else {
                                        /* Stop only the affected corridor. */
                                        tc2_demo_enter_sensor_recovery(
                                                slot, now_tick);
                                }
				}
                        }
                        break;
                }

                case TC2_DEMO_SENSOR_RECOVERY: {
                        int matched_cursor = -1;
                        int sensor = tc2_demo_find_route_sensor(
                                &sensors, slot, &matched_cursor);
                        int count = tc2_demo_leg_sensor_count(slot);

                        if (sensor < 0 || matched_cursor < 0) {
                                if (!tc2_tick_due(
                                            now_tick,
                                            tc2_demo_state
                                                    .phase_started_tick[slot] +
                                                    TC2_DEMO_SENSOR_RECOVERY_DWELL_TICKS)) {
                                        break;
                                }
				{
					int leg_speed = tc2_demo_leg_speed(slot);
					if (CanTrainSetSpeedPriority(
						    tc_can_server_tid,
						    tc2_demo_trains[slot],
						    leg_speed) < 0) {
                                        tc2_demo_pause_train(slot, now_tick);
                                        break;
					}
                                tc2_demo_state.phase[slot] =
                                        TC2_DEMO_RUNNING;
                                tc2_demo_state.phase_started_tick[slot] =
                                        now_tick;
                                tc2_demo_set_ui_speed(
						slot, leg_speed, now_tick);
                                ++tc2_demo_state.action_count;
				}
                                break;
                        }
                        tc2_demo_set_ui_at_sensor(
                                slot, sensor, now_tick);
                        tc2_demo_state.route_sensor_cursor[slot] =
                                matched_cursor + 1;
                        tc2_demo_state.phase_started_tick[slot] = now_tick;
                        tc2_demo_state.sensor_recovery_restarts[slot] = 0;
                        if (matched_cursor + 1 >= count) {
                                if (tc2_demo_begin_endpoint_trim(
                                            slot, now_tick) < 0) {
                                        tc2_demo_pause_train(slot, now_tick);
                                }
			} else if (tc2_demo_is_return_correction_stop(
					   slot, matched_cursor)) {
				tc2_demo_state.phase[slot] = TC2_DEMO_STOPPED;
				tc2_demo_state.phase_started_tick[slot] = now_tick;
				tc2_demo_state.station_stop[slot] = 1;
				tc2_demo_set_ui_speed(slot, 0, now_tick);
                        } else if (matched_cursor ==
                                  tc2_demo_leg_station_cursor(slot)) {
                                tc2_demo_state.phase[slot] =
                                        TC2_DEMO_STOPPED;
                                tc2_demo_state.station_stop[slot] = 1;
                        } else {
                                tc2_demo_state.phase[slot] =
                                        TC2_DEMO_WAIT_TURNOUT;
                                tc2_demo_state.wait_until_tick[slot] =
                                        now_tick +
                                        TC2_DEMO_WAIT_TURNOUT_TICKS;
                                tc2_demo_state.resume_after_wait[slot] = 1;
                        }
                        break;
                }

		case TC2_DEMO_ENDPOINT_TRIM_STOPPED:
			if (!tc2_tick_due(
				    now_tick,
				    tc2_demo_state.wait_until_tick[slot])) {
				break;
			}
			/*
			 * A zero endpoint trim means the final detector is the scripted
			 * stopping boundary.  Do not emit even a one-tick correction-speed
			 * pulse: on the physical layout that pulse is enough to carry T14
			 * beyond d4 and toward the turnout shared with T15.
			 */
			if (tc2_demo_endpoint_trim_ticks(slot) == 0u) {
				tc2_demo_state.phase[slot] = TC2_DEMO_STOPPED;
				tc2_demo_state.phase_started_tick[slot] = now_tick;
				tc2_demo_state.station_stop[slot] = 0;
				if (tc2_demo_state.outbound_leg[slot]) {
					tc2_demo_state.first_departure[slot] = 0;
				}
				tc2_demo_set_ui_speed(slot, 0, now_tick);
				break;
			}
			if (CanTrainSetSpeedPriority(
				    tc_can_server_tid,
				    tc2_demo_trains[slot],
				    TC2_DEMO_ENDPOINT_TRIM_SPEED) < 0) {
				tc2_demo_pause_train(slot, now_tick);
				break;
			}
			tc2_demo_state.phase[slot] =
				TC2_DEMO_ENDPOINT_TRIM_RUNNING;
				tc2_demo_state.phase_started_tick[slot] = now_tick;
				tc2_demo_state.wait_until_tick[slot] =
					now_tick + tc2_demo_endpoint_trim_ticks(slot);
			tc2_demo_set_ui_speed(
				slot, TC2_DEMO_ENDPOINT_TRIM_SPEED, now_tick);
			++tc2_demo_state.action_count;
			break;

		case TC2_DEMO_ENDPOINT_TRIM_RUNNING:
			if (!tc2_tick_due(
				    now_tick,
				    tc2_demo_state.wait_until_tick[slot])) {
				break;
			}
			if (CanTrainSetSpeedPriority(
				    tc_can_server_tid,
				    tc2_demo_trains[slot], 0) < 0) {
				tc2_demo_pause_train(slot, now_tick);
				break;
			}
			tc2_demo_state.phase[slot] = TC2_DEMO_STOPPED;
			tc2_demo_state.phase_started_tick[slot] = now_tick;
			tc2_demo_state.station_stop[slot] = 0;
			if (tc2_demo_state.outbound_leg[slot]) {
				tc2_demo_state.first_departure[slot] = 0;
			}
			tc2_demo_set_ui_speed(slot, 0, now_tick);
			++tc2_demo_state.action_count;
			break;

                case TC2_DEMO_STOPPED:
                        if (!tc2_tick_due(
                                    now_tick,
                                    tc2_demo_state
                                            .phase_started_tick[slot] +
                                            (tc2_demo_state
                                                     .station_stop[slot] ?
                                             TC2_DEMO_STOP_DWELL_TICKS :
                                             profile->endpoint_dwell_ticks
                                                     [slot]))) {
                                break;
                        }
                        if (tc2_demo_state.station_stop[slot]) {
                                /*
                                 * This is a station stop, not a turnaround:
                                 * retain the forward sensor cursor and wait
                                 * for the already prepared next turnout set
                                 * to remain stable before continuing.
                                 */
                                tc2_demo_state.station_stop[slot] = 0;
                                tc2_demo_state.phase[slot] =
                                        TC2_DEMO_WAIT_TURNOUT;
                                tc2_demo_state.phase_started_tick[slot] =
                                        now_tick;
                                tc2_demo_state.wait_until_tick[slot] =
                                        now_tick +
                                        TC2_DEMO_WAIT_TURNOUT_TICKS;
                                tc2_demo_state.resume_after_wait[slot] = 1;
                                break;
                        }
                        /*
                         * The fixed demonstration is exactly one outbound
                         * leg and one return leg.  Reaching the end of the
                         * return leg is terminal: leave the train stopped
                         * and keep STOPPED visible instead of starting a
                         * third leg.
                         */
                        if (!tc2_demo_state.outbound_leg[slot]) {
				/*
				 * Make the terminal stop explicit even when the endpoint
				 * trim was zero or its earlier stop acknowledgement was
				 * delayed.  This is especially important for T15 in the
				 * d4/d3/d1 profile: a completed script must never leave a
				 * stale non-zero locomotive command behind.
				 */
				if (!tc2_demo_state.completed_cycles[slot]) {
					if (CanTrainSetSpeedPriority(
						    tc_can_server_tid,
						    tc2_demo_trains[slot], 0) < 0) {
						tc2_demo_pause_train(slot, now_tick);
						break;
					}
					tc2_demo_set_ui_speed(slot, 0, now_tick);
					++tc2_demo_state.action_count;
				}
                                tc2_demo_state.completed_cycles[slot] = 1;
                                break;
                        }
			{
				unsigned int completion_mask =
					profile->return_prepare_completion_mask[slot];
				int dependency =
					profile->return_prepare_dependency[slot];
				int completions_ready = 1;

				for (int peer = 0;
				     peer < TC2_DEMO_TRAIN_COUNT; ++peer) {
					if ((completion_mask &
					     (1u << (unsigned int)peer)) != 0u) {
						int peer_stopped_at_return_endpoint =
							!tc2_demo_state.outbound_leg[peer] &&
							tc2_demo_state.phase[peer] ==
								TC2_DEMO_STOPPED &&
							!tc2_demo_state.station_stop[peer] &&
							tc2_demo_state.route_sensor_cursor[peer] >=
								profile->return_count[peer];

						/*
						 * The final return detector plus speed zero is
						 * already the physical clearance proof needed by
						 * the fixed choreography.  Do not make T15 wait
						 * forever for a later bookkeeping tick to set
						 * completed_cycles.
						 */
						if (!tc2_demo_state.completed_cycles[peer] &&
						    !peer_stopped_at_return_endpoint) {
							completions_ready = 0;
							break;
						}
					}
				}
				if (!completions_ready) {
					tc2_demo_state
						.return_prepare_release_tick[slot] = 0;
					break;
				}
				if (dependency >= 0 &&
				    dependency < TC2_DEMO_TRAIN_COUNT) {
					int release_cursor =
						profile->return_prepare_release_cursor[slot];
					int wait_for_return =
						profile->return_prepare_wait_for_return[slot];
					int wait_for_endpoint =
						profile->return_prepare_wait_for_endpoint[slot];
					int dependency_clear;

					if (wait_for_endpoint) {
						dependency_clear =
							tc2_demo_state.outbound_leg[dependency] &&
							tc2_demo_state.phase[dependency] ==
								TC2_DEMO_STOPPED &&
							!tc2_demo_state.station_stop[dependency] &&
							release_cursor >= 0 &&
							tc2_demo_state
								.route_sensor_cursor[dependency] >=
								release_cursor;
					} else if (wait_for_return) {
						dependency_clear =
							tc2_demo_state.completed_cycles[dependency] ||
							(!tc2_demo_state.outbound_leg[dependency] &&
							 release_cursor >= 0 &&
							 tc2_demo_state
								 .route_sensor_cursor[dependency] >=
								release_cursor);
					} else {
						dependency_clear =
							tc2_demo_state.completed_cycles[dependency] ||
							!tc2_demo_state.outbound_leg[dependency] ||
							(release_cursor >= 0 &&
							 tc2_demo_state
								 .route_sensor_cursor[dependency] >=
								release_cursor);
					}

					if (!dependency_clear) {
						tc2_demo_state
							.return_prepare_release_tick[slot] = 0;
						break;
					}
					/*
					 * A detector confirms only the head of the dependency
					 * train.  Keep this turnout unchanged for another two
					 * seconds so its tail clears before preparing the return
					 * route that may command the same physical switch.
					 */
					if (tc2_demo_state
						    .return_prepare_release_tick[slot] == 0u) {
						tc2_demo_state
							.return_prepare_release_tick[slot] =
								now_tick;
						break;
					}
					if (!tc2_tick_due(
						    now_tick,
						    tc2_demo_state
							    .return_prepare_release_tick[slot] +
							    TC2_DEMO_WAIT_TURNOUT_TICKS)) {
						break;
					}
					tc2_demo_state
						.return_prepare_release_tick[slot] = 0;
				}
			}
			/*
			 * Select the return leg before issuing reverse so its complete
			 * turnout batch is confirmed and mechanically settled first.
			 */
			tc2_demo_state.outbound_leg[slot] = 0;
			if (tc2_demo_prepare_leg(
					    slot, now_tick, &sensors) < 0) {
				tc2_demo_state.outbound_leg[slot] = 1;
				tc2_demo_pause_train(slot, now_tick);
				break;
			}
			tc2_demo_state.reverse_after_wait[slot] = 1;
			break;

                case TC2_DEMO_REVERSING:
                        if (!tc2_tick_due(
                                    now_tick,
                                    tc2_demo_state
                                            .phase_started_tick[slot] +
                                            TC2_DEMO_REVERSE_SETTLE_TICKS)) {
                                break;
                        }
			/*
			 * Return turnouts were prepared before reverse.  Do not issue
			 * another switch batch here; after reverse itself has settled,
			 * the normal WAIT_TURNOUT path may start the train.
			 */
			tc2_demo_state.phase[slot] = TC2_DEMO_WAIT_TURNOUT;
			tc2_demo_state.phase_started_tick[slot] = now_tick;
			tc2_demo_state.wait_until_tick[slot] = now_tick;
			break;

                case TC2_DEMO_INITIAL_STOPPING:
                case TC2_DEMO_PAUSED:
                case TC2_DEMO_IDLE:
                default:
                        break;
                }
        }
        tc2_demo_publish_phase_status(now_tick, 0);
}

static int tc2_stop_physical_demo(void) {
        int token;
        int stopped_trains[TC2_DEMO_TRAIN_COUNT];
        int raw_time = Time();
        uint32_t now_tick = raw_time < 0 ? 0u : (uint32_t)raw_time;

        for (int slot = 0; slot < TC2_DEMO_TRAIN_COUNT; ++slot) {
                stopped_trains[slot] = tc2_demo_trains[slot];
        }
        if (tc_can_server_tid < 0) {
                tc2_status_set(
                        "RETURNBACK STOP failed: CAN unavailable; script and UI preserved");
                return 0;
        }
        token = CanTrainEmergencyStopBatch(
                tc_can_server_tid, stopped_trains,
                TC2_DEMO_TRAIN_COUNT);
        if (token <= 0) {
                tc2_status_set(
                        "RETURNBACK STOP failed: stop batch rejected; script and UI preserved");
                return 0;
        }

        /*
         * A successful stop submission terminates the scripted session.  Do
         * not leave its synthetic train records behind: uion must reopen a
         * clean live overlay rather than resurrecting the three returnback
         * trains from their last displayed positions.
         */
        tc2_demo_reset_state();
        for (int slot = 0; slot < TC2_DEMO_TRAIN_COUNT; ++slot) {
                tc2_demo_trains[slot] = 0;
        }
        Tc2LiveUiInitialize(&tc2_live_ui_state, now_tick);
        tc2_ui_active = 0;
        tc2_ui_live = 1;
        tc2_ui_shutdown_requested = 0;
        tc2_ui_shutdown_frame_queued = 0;
        tc2_ui_next_render_tick = 0;
        tc2_status_set(
                "RETURNBACK STOP accepted: fixed routes and live UI cleared");
        return TC2_DEMO_TRAIN_COUNT;
}

/*
 * Prediction-only commands have a distinct namespace.  This function never
 * calls CAN, the live dispatcher, or the physical reservation service.
 * Returning one means the command token belonged to the offline namespace,
 * including malformed commands (which are rejected with a status message).
 */
static int tc2_handle_offline_command(char *line, int emit,
                                      int terminal_tid) {
        char *p = line;
        char start[12];
        char destination[12];
        int train;
        int speed;
        int status;

        skip_spaces(&p);
        if (!command_token(p, "simdispatch") &&
            !command_token(p, "simgo") &&
            !command_token(p, "simtrips") &&
            !command_token(p, "simremove") &&
            !command_token(p, "simreset") &&
            !command_token(p, "uion") &&
            !command_token(p, "uioff")) {
                return 0;
        }

        if (!tc2_offline_controller_ready) {
                tc2_status_set("offline controller is not ready");
                if (emit) tc2_emit_offline_status(terminal_tid);
                return 1;
        }

        if (command_token(p, "simdispatch")) {
                p += 11;
                if (parse_uint(&p, &train) < 0 ||
                    parse_word(&p, start, sizeof(start)) < 0 ||
                    parse_uint(&p, &speed) < 0 ||
                    parse_word(&p, destination,
                               sizeof(destination)) < 0 ||
                    !no_more_args(p)) {
                        tc2_status_set(
                                "usage: simdispatch <train 1..255> <A-F> "
                                "<speed 1..120> <d1-d8>");
                } else {
                        int start_index;
                        int destination_index;
                        uppercase_word(start);
                        start_index = Tc2DispatchParseStart(start);
                        destination_index =
                                Tc2DispatchParseDestination(destination);
                        status = Tc2OfflineControllerStageTrip(
                                &tc2_offline_controller_state,
                                tc2_track, train, start_index, speed,
                                destination_index);
                        if (status == TC2_OFFLINE_CONTROLLER_OK) {
                                size_t length = 0;
                                tc2_ui_status[0] = 0;
                                length = tc2_status_append(
                                        tc2_ui_status,
                                        sizeof(tc2_ui_status), length,
                                        "staged predicted trip T");
                                length = tc2_status_append_uint(
                                        tc2_ui_status,
                                        sizeof(tc2_ui_status), length,
                                        (unsigned int)train);
                                length = tc2_status_append(
                                        tc2_ui_status,
                                        sizeof(tc2_ui_status), length,
                                        " ");
                                length = tc2_status_append(
                                        tc2_ui_status,
                                        sizeof(tc2_ui_status), length,
                                        start);
                                length = tc2_status_append(
                                        tc2_ui_status,
                                        sizeof(tc2_ui_status), length,
                                        " -> ");
                                length = tc2_status_append(
                                        tc2_ui_status,
                                        sizeof(tc2_ui_status), length,
                                        destination);
                                length = tc2_status_append(
                                        tc2_ui_status,
                                        sizeof(tc2_ui_status), length,
                                        " speed=");
                                (void)tc2_status_append_uint(
                                        tc2_ui_status,
                                        sizeof(tc2_ui_status), length,
                                        (unsigned int)speed);
                        } else {
                                tc2_status_set(
                                        status ==
                                        TC2_OFFLINE_CONTROLLER_CAPACITY ?
                                        "offline capacity reached (max 16)" :
                                        "simdispatch rejected: invalid, "
                                        "duplicate, or unroutable trip");
                        }
                }
        } else if (command_only(p, "simgo")) {
                status = Tc2OfflineControllerStartAll(
                        &tc2_offline_controller_state);
                if (status == TC2_OFFLINE_CONTROLLER_OK) {
                        tc2_status_set(
                                "all staged predicted trips started together");
                        tc2_ui_active = 1;
                        tc2_ui_shutdown_requested = 0;
                        tc2_ui_shutdown_frame_queued = 0;
                        tc2_ui_next_render_tick = 0;
                } else {
                        tc2_status_set(
                                "simgo rejected: no startable staged trip");
                }
        } else if (command_only(p, "simtrips")) {
                tc2_offline_controller_snapshot snapshot;
                status = Tc2OfflineControllerGetSnapshot(
                        &tc2_offline_controller_state, &snapshot);
                if (status == TC2_OFFLINE_CONTROLLER_OK) {
                        size_t length = 0;
                        tc2_ui_status[0] = 0;
                        length = tc2_status_append(
                                tc2_ui_status,
                                sizeof(tc2_ui_status), length,
                                "predicted trips total=");
                        length = tc2_status_append_uint(
                                tc2_ui_status,
                                sizeof(tc2_ui_status), length,
                                (unsigned int)snapshot.runtime.train_count);
                        length = tc2_status_append(
                                tc2_ui_status,
                                sizeof(tc2_ui_status), length,
                                " running=");
                        length = tc2_status_append_uint(
                                tc2_ui_status,
                                sizeof(tc2_ui_status), length,
                                (unsigned int)snapshot.runtime.running_count);
                        length = tc2_status_append(
                                tc2_ui_status,
                                sizeof(tc2_ui_status), length,
                                " waiting=");
                        length = tc2_status_append_uint(
                                tc2_ui_status,
                                sizeof(tc2_ui_status), length,
                                (unsigned int)snapshot.runtime.waiting_count);
                        length = tc2_status_append(
                                tc2_ui_status,
                                sizeof(tc2_ui_status), length,
                                " arrived=");
                        (void)tc2_status_append_uint(
                                tc2_ui_status,
                                sizeof(tc2_ui_status), length,
                                (unsigned int)snapshot.runtime.arrived_count);
                } else {
                        tc2_status_set("offline snapshot unavailable");
                }
        } else if (command_token(p, "simremove")) {
                p += 9;
                if (parse_uint(&p, &train) < 0 ||
                    !no_more_args(p)) {
                        tc2_status_set(
                                "usage: simremove <train 1..255>");
                } else {
                        status = Tc2OfflineControllerRemoveTrip(
                                &tc2_offline_controller_state, train);
                        tc2_status_set(
                                status == TC2_OFFLINE_CONTROLLER_OK ?
                                "predicted trip removed" :
                                "simremove rejected: train not found");
                }
        } else if (command_only(p, "simreset")) {
                int now = Time();
                Tc2OfflineControllerReset(
                        &tc2_offline_controller_state,
                        now < 0 ? 0u : (uint32_t)now);
                tc2_status_set(
                        "offline simulation reset; no physical state changed");
        } else if (command_only(p, "uion")) {
                tc2_ui_active = 1;
                tc2_ui_shutdown_requested = 0;
                tc2_ui_shutdown_frame_queued = 0;
                tc2_ui_next_render_tick = 0;
                tc2_status_set("dynamic predicted Track D dashboard enabled");
        } else if (command_only(p, "uioff")) {
                tc2_status_set(
                        "dynamic dashboard stopping after final complete frame");
                if (tc2_ui_active) {
                        tc2_ui_shutdown_requested = 1;
                        tc2_ui_shutdown_frame_queued = 0;
                }
        } else {
                tc2_status_set("invalid offline command arguments");
        }

        if (emit) tc2_emit_offline_status(terminal_tid);
        return 1;
}

static int tc2_load_live_projection(
        const tc2_dispatch_job_snapshot *job, uint32_t now_tick) {
        tc2_dispatch_projection_header header;
        int count = 0;
        if (!job || job->train < 1 || job->plan_generation == 0 ||
            Tc2LiveUiHasPlan(
                    &tc2_live_ui_state, job->train,
                    job->plan_generation)) {
                return 0;
        }
        if (job->projection_valid) {
                int header_status =
                        Tc2DispatchGetProjectionHeader(
                                tc_dispatch_server_tid,
                                job->train, &header);
                if (header_status ==
                            TC2_DISPATCH_PROJECTION_NOT_FOUND ||
                    header_status ==
                            TC2_DISPATCH_PROJECTION_NOT_READY ||
                    header_status ==
                            TC2_DISPATCH_PROJECTION_STALE) {
                        return 1;
                }
                if (header_status !=
                            TC2_DISPATCH_PROJECTION_OK ||
                    !header.valid ||
                    header.train != job->train ||
                    header.publication_serial !=
                            job->projection_publication_serial ||
                    header.plan_generation !=
                            job->plan_generation ||
                    header.plan_generation !=
                            job->projection_plan_generation ||
                    header.launch_epoch !=
                            job->job_launch_epoch ||
                    header.launch_epoch !=
                            job->projection_launch_epoch ||
                    header.waypoint_count < 1 ||
                    header.waypoint_count >
                            TC2_LIVE_UI_MAX_WAYPOINTS) {
                        /*
                         * A valid but different publication means the
                         * snapshot raced a newer dispatcher generation.
                         * Skip this refresh; never combine its old job
                         * record with the new header.
                         */
                        if (header_status ==
                                    TC2_DISPATCH_PROJECTION_OK &&
                            header.valid &&
                            header.train == job->train &&
                            (header.publication_serial !=
                                     job->projection_publication_serial ||
                             header.plan_generation !=
                                     job->plan_generation ||
                             header.launch_epoch !=
                                     job->job_launch_epoch)) {
                                return 1;
                        }
                        return -1;
                }
                while (count < header.waypoint_count) {
                        tc2_dispatch_projection_page page;
                        int status = Tc2DispatchGetProjectionPage(
                                tc_dispatch_server_tid, job->train,
                                header.publication_serial,
                                header.plan_generation,
                                header.launch_epoch, count, &page);
                        if (status ==
                                    TC2_DISPATCH_PROJECTION_NOT_FOUND ||
                            status ==
                                    TC2_DISPATCH_PROJECTION_NOT_READY ||
                            status ==
                                    TC2_DISPATCH_PROJECTION_STALE) {
                                return 1;
                        }
                        if (status != TC2_DISPATCH_PROJECTION_OK ||
                            page.status !=
                                    TC2_DISPATCH_PROJECTION_OK ||
                            page.train != header.train ||
                            page.publication_serial !=
                                    header.publication_serial ||
                            page.plan_generation !=
                                    header.plan_generation ||
                            page.launch_epoch !=
                                    header.launch_epoch ||
                            page.first_waypoint != count ||
                            page.total_waypoints !=
                                    header.waypoint_count ||
                            page.count < 1 ||
                            page.count >
                                    TC2_DISPATCH_PROJECTION_PAGE_CAPACITY ||
                            count + page.count >
                                    header.waypoint_count) {
                                return -1;
                        }
                        for (int index = 0; index < page.count; ++index) {
                                tc2_live_projection_scratch[count++] =
                                        page.waypoints[index];
                        }
                }
                return Tc2LiveUiAcceptProjection(
                        &tc2_live_ui_state, &header,
                        tc2_live_projection_scratch,
                        count, now_tick);
        }

        /*
         * An estimated CURRENT handoff may first run one short localization
         * leg before its exact shortest route can be selected. Keep the same
         * physical marker and color bound to the new generation instead of
         * deleting it or drawing a guessed branch. Attributed CAN detectors
         * still flash; the exact projection replaces this one-cell hold as
         * soon as localization commits.
         */
        if (job->start_index ==
                    TC2_DISPATCH_START_CURRENT) {
                const tc2_ui_train_overlay *shown =
                        Tc2UiOverlayFindTrain(
                                &tc2_live_ui_state.overlay,
                                job->train);
                if (!shown ||
                    !Tc2TrackDLayoutCellIsOccupied(
                            shown->row, shown->column)) {
                        return 0;
                }
                for (unsigned int index = 0;
                     index < sizeof(header); ++index) {
                        ((unsigned char *)&header)[index] = 0;
                }
                header.valid = 1;
                header.train = job->train;
                header.plan_generation =
                        job->plan_generation;
                header.launch_epoch =
                        job->job_launch_epoch;
                header.start_index =
                        TC2_DISPATCH_START_CURRENT;
                header.destination_index =
                        job->destination_index;
                header.speed = job->speed;
                header.waypoint_count = 1;
                tc2_live_projection_scratch[0] =
                        (tc2_dispatch_projection_waypoint){0};
                tc2_live_projection_scratch[0].kind =
                        TC2_ROUTE_WAYPOINT_START;
                tc2_live_projection_scratch[0].route_offset = 0;
                tc2_live_projection_scratch[0].graph_node = -1;
                tc2_live_projection_scratch[0].sensor_index = -1;
                tc2_live_projection_scratch[0].switch_number = -1;
                tc2_live_projection_scratch[0].turnout_direction = -1;
                tc2_live_projection_scratch[0].destination_index = -1;
                tc2_live_projection_scratch[0].ui_row =
                        (uint8_t)shown->row;
                tc2_live_projection_scratch[0].ui_column =
                        (uint8_t)shown->column;
                tc2_live_projection_scratch[0].ui_width = 1;
                return Tc2LiveUiAcceptProjection(
                        &tc2_live_ui_state, &header,
                        tc2_live_projection_scratch,
                        1, now_tick);
        }

        /*
         * caldispatch deliberately has no ordinary immutable projection.
         * Give the live dashboard a bounded start-to-OP prediction anyway;
         * real attributed sensor events still replace it and flash at their
         * exact directed cells.
         */
        if (!job->provisional_prediction ||
            job->start_index < 0 ||
            job->start_index >= TC2_TRACK_D_START_COUNT ||
            job->destination_index < 0 ||
            job->destination_index >=
                    TC2_TRACK_D_DESTINATION_COUNT) {
                return 0;
        }
        const tc2_track_d_start_layout *start =
                Tc2TrackDStartLayout(
                        (size_t)job->start_index);
        const tc2_track_d_destination_layout *destination =
                Tc2TrackDDestinationLayout(
                        (size_t)job->destination_index);
        if (!start || !destination) return -1;
        for (unsigned int index = 0;
             index < sizeof(header); ++index) {
                ((unsigned char *)&header)[index] = 0;
        }
        header.valid = 1;
        header.train = job->train;
        header.plan_generation = job->plan_generation;
        header.launch_epoch = job->job_launch_epoch;
        header.start_index = job->start_index;
        header.destination_index = job->destination_index;
        header.speed = job->speed;
        header.waypoint_count = 2;
        tc2_live_projection_scratch[0] =
                (tc2_dispatch_projection_waypoint){0};
        tc2_live_projection_scratch[0].kind =
                TC2_ROUTE_WAYPOINT_START;
        tc2_live_projection_scratch[0].sensor_index = -1;
        tc2_live_projection_scratch[0].ui_row = start->row;
        tc2_live_projection_scratch[0].ui_column =
                start->column;
        tc2_live_projection_scratch[0].ui_width = 1;
        tc2_live_projection_scratch[1] =
                (tc2_dispatch_projection_waypoint){0};
        tc2_live_projection_scratch[1].kind =
                TC2_ROUTE_WAYPOINT_DESTINATION;
        tc2_live_projection_scratch[1].sensor_index = -1;
        tc2_live_projection_scratch[1].destination_index =
                (int8_t)job->destination_index;
        tc2_live_projection_scratch[1].distance_um =
                (int64_t)job->destination_distance_mm * 1000;
        tc2_live_projection_scratch[1].ui_row =
                destination->row;
        tc2_live_projection_scratch[1].ui_column =
                destination->column;
        tc2_live_projection_scratch[1].ui_width = 1;
        return Tc2LiveUiAcceptProjection(
                &tc2_live_ui_state, &header,
                tc2_live_projection_scratch, 2, now_tick);
}

static int tc2_sync_live_ui(uint32_t now_tick) {
        tc2_dispatch_snapshot dispatch;
        train_sensor_snapshot_t sensors;
        if (tc_dispatch_server_tid < 0 ||
            tc_sensor_server_tid < 0 ||
            Tc2DispatchGetSnapshot(
                    tc_dispatch_server_tid, &dispatch) < 0 ||
            TrainSensorGetLatest(
                    tc_sensor_server_tid, &sensors) < 0) {
                return -1;
        }
        for (int slot = 0;
             slot < TC2_DISPATCH_MAX_JOBS; ++slot) {
                const tc2_dispatch_job_snapshot *job =
                        &dispatch.jobs[slot];
                if (job->state == TC2_JOB_EMPTY ||
                    job->plan_generation == 0) {
                        continue;
                }
                int load_status =
                        tc2_load_live_projection(
                                job, now_tick);
                if (load_status < 0) {
                        return -1;
                }
                if (load_status > 0) {
                        /* One transient publication must not freeze peers. */
                        continue;
                }
        }
        return Tc2LiveUiSync(
                &tc2_live_ui_state, &dispatch,
                &sensors, now_tick);
}
#endif

static tc_target_t *find_target(const char *name) {
        for (int i = 0; i < num_targets; i++) {
                if (streq(targets[i].name, name)) {
                        return &targets[i];
                }
        }

        return 0;
}

static void print_help(int terminal_tid) {
#ifdef MODE_TC2
        tc_puts(terminal_tid, "\r\nTC2 commands (arguments must match exactly):\r\n");
        tc_puts(terminal_tid, "\r\n");
        tc_puts(terminal_tid, Tc2OfflineControllerEvidenceLabel());
        tc_puts(terminal_tid, "\r\n");
        tc_puts(terminal_tid,
                "  simdispatch <train 1..255> <A-F> <speed 1..120> <d1-d8>\r\n");
        tc_puts(terminal_tid,
                "                               stage one prediction-only trip\r\n");
        tc_puts(terminal_tid,
                "  simgo                       start every staged predicted trip together\r\n");
        tc_puts(terminal_tid,
                "  simtrips                    summarize predicted running/wait/arrived state\r\n");
        tc_puts(terminal_tid,
                "  simremove <train>           remove one prediction-only trip\r\n");
        tc_puts(terminal_tid,
                "  simreset                    reset prediction state; no physical state changes\r\n");
        tc_puts(terminal_tid,
                "  uion / uioff                enable / cleanly stop the dynamic Track D UI\r\n");
        tc_puts(terminal_tid,
                "  returnback <intersection/destination> <intersection/destination> <intersection/destination>\r\n");
        tc_puts(terminal_tid,
                "                               FAKE fixed-script demo for T14@A, T15@C, T17@F\r\n");
        tc_puts(terminal_tid,
                "                               UI shows RUNNING/WAIT_TURNOUT/STOPPED/REVERSING; this is not live route planning\r\n");
        tc_puts(terminal_tid,
                "                               FAKE means pre-scripted, not simulated: the three physical trains still move by CAN\r\n");
        tc_puts(terminal_tid,
                "  returnbackstop               emergency-stop all three scripted trains\r\n");
        tc_puts(terminal_tid,
                "  sim* offline commands never send CAN, move a train, or change a physical switch.\r\n\r\n");
        tc_puts(terminal_tid, "  dispatch <train 14|15|17|18> <A-F|CURRENT> <speed 1..120> <d1-d8>\r\n");
        tc_puts(terminal_tid, "                               stage one managed trip\r\n");
        tc_puts(terminal_tid, "  reroute <train 14|15|17|18> <speed> <d1-d8>\r\n");
        tc_puts(terminal_tid, "                               stage a safe stop/replan from the managed train's CURRENT position\r\n");
        tc_puts(terminal_tid, "  caldispatch 14 A <speed 1..120> d7\r\n");
        tc_puts(terminal_tid, "                               supervised provisional T14 prediction run\r\n");
        tc_puts(terminal_tid, "  go                           submit every staged trip together\r\n");
        tc_puts(terminal_tid, "  trips                        print route, position, wait, safety, and CAN state\r\n");
        tc_puts(terminal_tid, "  cancel <train>               brake; protection stays through CANCEL_BRAKING\r\n");
        tc_puts(terminal_tid, "  remove <train>               after STOP_HOLD + physical removal, release protection\r\n");
        tc_puts(terminal_tid, "  block <track-node>           mark a physical track location unavailable\r\n");
        tc_puts(terminal_tid, "  unblock <track-node>         clear one physical track block\r\n");
        tc_puts(terminal_tid, "  blocks                       list blocked physical locations\r\n");
        tc_puts(terminal_tid, "  trackview                    print complete labelled Track D map and state\r\n");
        tc_puts(terminal_tid, "  sensors                      print structured sensor/owner sample\r\n");
        tc_puts(terminal_tid, "  positions                    print aligned candidate/plan sample\r\n");
        tc_puts(terminal_tid, "  sensorroute <node>           preview route from latest occupied sensor\r\n");
        tc_puts(terminal_tid, "  reservations                print owners, destinations, and sensor landmarks\r\n");
        tc_puts(terminal_tid, "  cal                          print the shared TC2 motion model\r\n");
        tc_puts(terminal_tid, "  help                         show this help\r\n");
        tc_puts(terminal_tid,
                "  tr/sw/route/stop/reserve/release are disabled in TC2; all physical motion uses dispatch\r\n");
        tc_puts(terminal_tid, "  q                            quit command client\r\n");
        return;
#else
        tc_puts(terminal_tid, "\r\nTC1 commands:\r\n");
        tc_puts(terminal_tid, "  help                         show this help\r\n");
        tc_puts(terminal_tid, "  cal                          print seed calibration model\r\n");
        tc_puts(terminal_tid, "  targets                      print known stopping targets\r\n");
        tc_puts(terminal_tid, "  plan <target> <speed>        preview stop plan without moving train\r\n");
        tc_puts(terminal_tid, "  obs <target> <speed> <ticks> compare predicted vs observed timing\r\n");
        tc_puts(terminal_tid, "  route <target>               set switches for a target, e.g. route C8\r\n");
        tc_puts(terminal_tid, "  stop <train> <target> <spd>  run then stop at target, e.g. stop 14 C8 30\r\n");
        tc_puts(terminal_tid, "  tr <train> <speed>           direct speed command, 0..120\r\n");
        tc_puts(terminal_tid, "  sw <switch> <S|C>            direct switch command\r\n");
        tc_puts(terminal_tid, "  status                       print latest train-control state\r\n");
        tc_puts(terminal_tid, "  q                            quit command client\r\n");
#endif
}

static void print_route_switches(int terminal_tid, const char *target_name) {
        if (streq(target_name, "A5")) {
                tc_puts(terminal_tid, "8:S");
        } else if (streq(target_name, "B7")) {
                tc_puts(terminal_tid, "8:S, 17:S, 155:S");
        } else if (streq(target_name, "C8")) {
                tc_puts(terminal_tid, "8:S, 17:S, 156:S, 15:S");
        } else if (streq(target_name, "D10")) {
                tc_puts(terminal_tid, "8:S, 17:S, 156:S");
        } else if (streq(target_name, "LOOP")) {
                tc_puts(terminal_tid, "8:C, 7:S, 18:C, 3:S, 2:C, 1:C");
        } else {
                tc_puts(terminal_tid, "unknown");
        }
}

static void print_targets(int terminal_tid) {
#ifdef MODE_TC2
        tc_puts(terminal_tid, "\r\nKnown legacy stopping targets:\r\n");
#else
        tc_puts(terminal_tid, "\r\nKnown TC1 targets:\r\n");
#endif

        for (int i = 0; i < num_targets; i++) {
                tc_puts(terminal_tid, "  ");
                tc_puts(terminal_tid, targets[i].name);
                tc_puts(terminal_tid, " distance=");
                tc_put_uint(terminal_tid, (unsigned int)targets[i].distance_mm);
                tc_puts(terminal_tid, "mm switches=[");
                print_route_switches(terminal_tid, targets[i].name);
                tc_puts(terminal_tid, "]\r\n");
        }
}

static void print_calibration(int terminal_tid) {
#ifdef MODE_TC2
        static const int reference_speeds[15] = {
                0, 9, 17, 26, 34,
                43, 51, 60, 69, 77,
                86, 94, 103, 111, 120
        };
        tc_puts(terminal_tid, "\r\nShared TC2 conservative motion model (10 ms tick):\r\n");
        tc_puts(terminal_tid,
                "speed | velocity(0.1mm/tick) | stop-mm | uncertainty | braking-mm | watchdog-ticks\r\n");
        for (int index = 0; index < 15; ++index) {
                int speed = reference_speeds[index];
                tc_put_uint(terminal_tid, (unsigned int)speed);
                tc_puts(terminal_tid, " | ");
                if (speed == 0) {
                        tc_puts(terminal_tid, "0 | 0 | 0 | 0 | n/a\r\n");
                        continue;
                }
                tc_put_uint(terminal_tid,
                            (unsigned int)
                                    Tc2MotionVelocityTenthsMmPerTick(
                                            speed));
                tc_puts(terminal_tid, " | ");
                tc_put_uint(terminal_tid,
                            (unsigned int)Tc2MotionStopDistanceMm(speed));
                tc_puts(terminal_tid, " | ");
                tc_put_uint(terminal_tid,
                            (unsigned int)Tc2MotionUncertaintyMm(speed));
                tc_puts(terminal_tid, " | ");
                tc_put_uint(terminal_tid,
                            (unsigned int)Tc2MotionBrakingDistanceMm(speed));
                tc_puts(terminal_tid, " | ");
                tc_put_uint(
                        terminal_tid,
                        (unsigned int)Tc2MotionWatchdogMarginTicks(speed));
                tc_puts(terminal_tid, "\r\n");
        }
        static const int provisional_speeds[16] = {
                1, 9, 17, 20, 26, 34, 43, 51,
                60, 69, 77, 86, 94, 103, 111, 120
        };
        tc_puts(terminal_tid,
                "\r\nProvisional T14 A->D7 point prediction (one physical run; NOT accepted calibration):\r\n");
        tc_puts(terminal_tid,
                "all command speeds 1..120 are continuously interpolated\r\n");
        tc_puts(terminal_tid,
                "speed | velocity(um/10ms tick) | predicted command-to-stop(mm)\r\n");
        for (int index = 0; index < 16; ++index) {
                int speed = provisional_speeds[index];
                tc_put_uint(terminal_tid, (unsigned int)speed);
                tc_puts(terminal_tid, " | ");
                tc_put_uint(
                        terminal_tid,
                        (unsigned int)
                                Tc2MotionProvisionalVelocityUmPerTick(
                                        speed));
                tc_puts(terminal_tid, " | ");
                tc_put_uint(
                        terminal_tid,
                        (unsigned int)
                                Tc2MotionProvisionalStopDistanceMm(
                                        speed));
                tc_puts(terminal_tid, "\r\n");
        }
#else
        tc_puts(terminal_tid, "\r\nSeed calibration table:\r\n");
        tc_puts(terminal_tid, "speed | velocity(mm/tick) | stop distance(mm)\r\n");

        for (int speed = 0; speed <= 14; speed++) {
                tc_put_uint(terminal_tid, (unsigned int)speed);
                tc_puts(terminal_tid, "     | ");
                tc_put_uint(terminal_tid, (unsigned int)velocity_mm_per_tick[tc_speed_model_index(speed)]);
                tc_puts(terminal_tid, "                 | ");
                tc_put_uint(terminal_tid, (unsigned int)stop_distance_mm[tc_speed_model_index(speed)]);
                tc_puts(terminal_tid, "\r\n");
        }
#endif
}


static void print_status(int terminal_tid) {
#ifdef MODE_TC2
        tc_puts(terminal_tid, "\r\nLegacy single-train control state:\r\n");
#else
        tc_puts(terminal_tid, "\r\nTC1 state:\r\n");
#endif
        tc_puts(terminal_tid, "  train=");
        tc_put_uint(terminal_tid, (unsigned int)state.train);
        tc_puts(terminal_tid, "\r\n  speed=");
        tc_put_uint(terminal_tid, (unsigned int)state.speed);
        tc_puts(terminal_tid, "\r\n  last_target=");
        tc_puts(terminal_tid, state.last_target);
        tc_puts(terminal_tid, "\r\n  last_run_ticks=");
        tc_put_uint(terminal_tid, (unsigned int)state.last_run_ticks);
        tc_puts(terminal_tid, "\r\n  last_stop_distance_mm=");
        tc_put_uint(terminal_tid, (unsigned int)state.last_stop_distance_mm);
        tc_puts(terminal_tid, "\r\n  last_prediction_error_ticks=");
        tc_put_int(terminal_tid, state.last_prediction_error_ticks);
        tc_puts(terminal_tid, "\r\n");
}

#ifdef MODE_TC2
static void print_sensor_owner_token(int terminal_tid, int has_reservations,
                                     int owner) {
        // Owner token统一区分unknown、unreserved、invalid和T1..T255。
        if (!has_reservations) {
                tc_puts(terminal_tid, "unknown");
        } else if (owner == 0) {
                tc_puts(terminal_tid, "unreserved");
        } else if (owner < 1 || owner > 255) {
                tc_puts(terminal_tid, "invalid");
        } else {
                tc_puts(terminal_tid, "T");
                tc_put_uint(terminal_tid, (unsigned int)owner);
        }
}

static void print_sensor_status(int terminal_tid) {
        train_sensor_snapshot_t snapshot;
        track_reservation_snapshot reservation_snapshot;
        char bank;
        int number;
        int active_count = 0;
        int has_reservations = 0;

        // 这个UI query reads the server snapshot，never invents a train position。
        if (tc_sensor_server_tid < 0 ||
            TrainSensorGetLatest(tc_sensor_server_tid, &snapshot) < 0) {
                tc_puts(terminal_tid,
                        "\r\n================= TC2 SENSOR SAMPLE ==================\r\n"
                        " sensor_snapshot=UNAVAILABLE | "
                        "reservation_snapshot=NOT_READ\r\n"
                        "------------------------------------------------------\r\n"
                        "sensor status unavailable\r\n");
                return;
        }
        if (tc_reservation_server_tid >= 0 &&
            TrackReservationServerSnapshot(tc_reservation_server_tid,
                                           &reservation_snapshot) == 0) {
                has_reservations = 1;
        }
        for (int sensor = 0; sensor < TRAIN_SENSOR_COUNT; ++sensor) {
                if (snapshot.sensor_state[sensor]) ++active_count;
        }

        // Sensor和reservation来自sequential IPC reads，所以banner明确标成sample。
        tc_puts(terminal_tid,
                "\r\n================= TC2 SENSOR SAMPLE ==================\r\n"
                " sensor_snapshot=OK | reservation_snapshot=");
        tc_puts(terminal_tid, has_reservations ? "OK" : "UNAVAILABLE");
        tc_puts(terminal_tid,
                "\r\n------------------------------------------------------\r\n");
        tc_puts(terminal_tid, "service: registered=");
        tc_puts(terminal_tid,
                snapshot.sensor_server_registered ? "yes" : "no");
        tc_puts(terminal_tid, " courier_created=");
        tc_puts(terminal_tid, snapshot.courier_created ? "yes" : "no");
        tc_puts(terminal_tid, " ready=");
        tc_puts(terminal_tid, snapshot.courier_ready ? "yes" : "no");
        tc_puts(terminal_tid, "\r\nheartbeat: last_tick=");
        tc_put_int(terminal_tid, snapshot.courier_last_heartbeat_tick);
        tc_puts(terminal_tid, " count=");
        tc_put_uint(terminal_tid, snapshot.courier_heartbeat_count);
        tc_puts(terminal_tid, "\r\ncourier: received=");
        tc_put_uint(terminal_tid, snapshot.courier_receive_count);
        tc_puts(terminal_tid, " receive_failures=");
        tc_put_uint(
                terminal_tid,
                snapshot.courier_receive_failure_count);
        tc_puts(terminal_tid, " time_failures=");
        tc_put_uint(terminal_tid, snapshot.time_failure_count);
        tc_puts(terminal_tid, "\r\nattribution: attributed=");
        tc_put_uint(terminal_tid, snapshot.attributed_count);
        tc_puts(terminal_tid, " unavailable=");
        tc_put_uint(
                terminal_tid,
                snapshot.attribution_unavailable_count);
        tc_puts(terminal_tid, " unattributed=");
        tc_put_uint(terminal_tid, snapshot.unattributed_count);
        tc_puts(terminal_tid, "\r\n");

        if (!snapshot.has_occupied) {
                tc_puts(terminal_tid, "latest_occupied: waiting\r\n");
        } else if (TrainSensorLabel(snapshot.latest_occupied.sensor_index,
                                    &bank,
                                    &number) < 0) {
                // UI和monitor share one checked label mapping，避免A1..E16 conversion drift。
                tc_puts(terminal_tid,
                        "latest_occupied: invalid sensor index\r\n");
        } else {
                int latest_sensor = snapshot.latest_occupied.sensor_index;
                tc_puts(terminal_tid, "latest_occupied: ");
                tc_put_char(terminal_tid, (unsigned char)bank);
                tc_put_uint(terminal_tid, (unsigned int)number);
                tc_puts(terminal_tid, " | node=");
                tc_puts(terminal_tid,
                        tc2_track[latest_sensor].name ?
                        tc2_track[latest_sensor].name : "sensor?");
                // latest_occupied跨release保留，currently_active单独呈现current state。
                tc_puts(terminal_tid, "\r\nevent: protocol_raw_6_7=");
                tc_put_int(terminal_tid, snapshot.latest_occupied.timestamp);
                tc_puts(terminal_tid, " | currently_active=");
                tc_puts(terminal_tid,
                        snapshot.sensor_state[latest_sensor] ? "yes" : "no");
                // latest event可retained，owner/destination必须标成current sample。
                tc_puts(terminal_tid, "\r\nreservation_now: owner=");
                int owner = has_reservations ?
                        reservation_snapshot.owner_by_node[latest_sensor] : 0;
                print_sensor_owner_token(terminal_tid, has_reservations, owner);
                if (has_reservations && owner >= 1 && owner <= 255) {
                        tc_puts(terminal_tid, " | plan_destination=");
                        int destination =
                                reservation_snapshot.destination_by_train[owner];
                        if (destination >= 0 && destination < TRACK_MAX) {
                                tc_puts(terminal_tid,
                                        tc2_track[destination].name ?
                                        tc2_track[destination].name : "node?");
                        } else {
                                tc_puts(terminal_tid,
                                        destination == -1 ? "none" : "invalid");
                        }
                }
                tc_puts(terminal_tid, "\r\n");
        }

        tc_puts(terminal_tid, "counts: events=");
        tc_put_uint(terminal_tid, snapshot.event_count);
        tc_puts(terminal_tid, " | occupied=");
        tc_put_uint(terminal_tid, snapshot.occupied_count);
        tc_puts(terminal_tid, " | inconsistent=");
        tc_put_uint(terminal_tid, snapshot.inconsistent_count);
        tc_puts(terminal_tid,
                "\r\n------------------------------------------------------\r\n"
                "active sensors now (");
        tc_put_uint(terminal_tid, (unsigned int)active_count);
        tc_puts(terminal_tid, "):\r\n");

        // Active entries每行最多four，80 sensors也保持bounded terminal width。
        int columns = 0;
        for (int sensor = 0; sensor < TRAIN_SENSOR_COUNT; ++sensor) {
                if (!snapshot.sensor_state[sensor]) continue;
                tc_puts(terminal_tid, "  ");
                tc_puts(terminal_tid,
                        tc2_track[sensor].name ?
                        tc2_track[sensor].name : "sensor?");
                tc_puts(terminal_tid, "=");
                int owner = has_reservations ?
                        reservation_snapshot.owner_by_node[sensor] : 0;
                print_sensor_owner_token(terminal_tid,
                                         has_reservations, owner);
                ++columns;
                if (columns == 4) {
                        tc_puts(terminal_tid, "\r\n");
                        columns = 0;
                }
        }
        if (active_count == 0) {
                tc_puts(terminal_tid, "  none\r\n");
        } else if (columns != 0) {
                tc_puts(terminal_tid, "\r\n");
        }
}

static int position_matches_current_plan(
        const train_sensor_snapshot_t *sensor_snapshot,
        const track_reservation_snapshot *reservation_snapshot,
        int train) {
        int destination = reservation_snapshot->destination_by_train[train];
        // UI共用one generation rule，避免positions和trackview对stale状态解释不同。
        return destination >= 0 && destination < TRACK_MAX &&
                sensor_snapshot->position_generation_by_train[train] ==
                reservation_snapshot->generation_by_train[train];
}

// 这些small field helpers只输出bounded padding，不依赖heap或formatting library。
static void tc_put_spaces(int terminal_tid, int count) {
        for (int i = 0; i < count; ++i) {
                tc_puts(terminal_tid, " ");
        }
}

static int tc_text_length(const char *text) {
        int length = 0;
        while (text[length]) ++length;
        return length;
}

static int tc_uint_digits(unsigned int value) {
        int digits = 1;
        while (value >= 10) {
                value /= 10;
                ++digits;
        }
        return digits;
}

static void tc_put_text_field(int terminal_tid, const char *text, int width) {
        tc_puts(terminal_tid, text);
        tc_put_spaces(terminal_tid, width - tc_text_length(text));
}

static void tc_put_uint_field(int terminal_tid, unsigned int value, int width) {
        tc_put_uint(terminal_tid, value);
        tc_put_spaces(terminal_tid, width - tc_uint_digits(value));
}

static const char *tc_track_name_or(int node, const char *fallback) {
        // Snapshot node先做bounds/name check，坏数据不会让diagnostic UI解引用NULL。
        if (node >= 0 && node < TRACK_MAX && tc2_track[node].name) {
                return tc2_track[node].name;
        }
        return fallback;
}

static void print_position_candidates(int terminal_tid) {
        train_sensor_snapshot_t sensor_snapshot;
        track_reservation_snapshot reservation_snapshot;
        int has_reservations = 0;
        int printed = 0;

        if (tc_sensor_server_tid < 0 ||
            TrainSensorGetLatest(tc_sensor_server_tid, &sensor_snapshot) < 0) {
                tc_puts(terminal_tid,
                        "\r\n================= TC2 POSITION SAMPLE ================\r\n"
                        " sensor_state=UNAVAILABLE | reservation_state=NOT_READ\r\n"
                        "------------------------------------------------------\r\n"
                        "position candidates unavailable\r\n");
                return;
        }
        if (tc_reservation_server_tid >= 0 &&
            TrackReservationServerSnapshot(tc_reservation_server_tid,
                                           &reservation_snapshot) == 0) {
                has_reservations = 1;
        }

        // Position table只由真实new rising events更新，并明确标成sequential sample。
        tc_puts(terminal_tid,
                "\r\n================= TC2 POSITION SAMPLE ================\r\n"
                " sensor_state=OK | reservation_state=");
        tc_puts(terminal_tid, has_reservations ? "OK" : "UNAVAILABLE");
        tc_puts(terminal_tid,
                "\r\n------------------------------------------------------\r\n"
                "TRAIN  CANDIDATE  DEST   PLAN-MATCH  SEQUENCE    RAW6/7\r\n");
        for (int train = 1; train <= 255; ++train) {
                int sensor = sensor_snapshot.position_by_train[train];
                int has_candidate = sensor >= 0 && sensor < TRAIN_SENSOR_COUNT;
                int destination = has_reservations ?
                        reservation_snapshot.destination_by_train[train] : -1;
                int has_destination =
                        destination >= 0 && destination < TRACK_MAX;
                // 已有destination但尚无sensor的train也必须visible，不能从TC2 UI消失。
                if (!has_candidate && !has_destination) continue;

                tc_puts(terminal_tid, "T");
                tc_put_uint(terminal_tid, (unsigned int)train);
                tc_put_spaces(terminal_tid,
                              4 - tc_uint_digits((unsigned int)train));
                tc_puts(terminal_tid, "  ");
                tc_put_text_field(terminal_tid,
                                  has_candidate ?
                                  tc_track_name_or(sensor, "sensor?") :
                                  "waiting", 9);
                tc_puts(terminal_tid, "  ");
                tc_put_text_field(terminal_tid,
                                  has_destination ?
                                  tc_track_name_or(destination, "node?") :
                                  "none", 5);
                tc_puts(terminal_tid, "  ");
                if (!has_reservations) {
                        tc_put_text_field(terminal_tid, "unknown", 10);
                } else if (!has_candidate) {
                        tc_put_text_field(terminal_tid, "waiting", 10);
                } else if (position_matches_current_plan(
                                   &sensor_snapshot,
                                   &reservation_snapshot,
                                   train)) {
                        tc_put_text_field(terminal_tid, "current", 10);
                } else {
                        tc_put_text_field(terminal_tid, "stale", 10);
                }
                tc_puts(terminal_tid, "  ");
                if (has_candidate) {
                        tc_put_uint_field(
                                terminal_tid,
                                sensor_snapshot.position_sequence_by_train[train],
                                10);
                } else {
                        tc_put_text_field(terminal_tid, "-", 10);
                }
                tc_puts(terminal_tid, "  ");
                if (has_candidate) {
                        // Bytes 6/7是opaque protocol diagnostic，不是clock ticks。
                        tc_put_int(
                                terminal_tid,
                                sensor_snapshot.position_timestamp_by_train[train]);
                } else {
                        tc_puts(terminal_tid, "-");
                }
                tc_puts(terminal_tid, "\r\n");
                printed = 1;
        }
        if (!printed) tc_puts(terminal_tid, "none\r\n");
        tc_puts(terminal_tid,
                "------------------------------------------------------\r\n"
                "events: attributed=");
        tc_put_uint(terminal_tid, sensor_snapshot.attributed_count);
        tc_puts(terminal_tid, " | unattributed=");
        tc_put_uint(terminal_tid, sensor_snapshot.unattributed_count);
        tc_puts(terminal_tid, "\r\n        unavailable=");
        tc_put_uint(terminal_tid,
                    sensor_snapshot.attribution_unavailable_count);
        tc_puts(terminal_tid, "\r\n");
}

static void print_track_owner_cell(int terminal_tid, int owner,
                                   int sensor_state_available, int active) {
        // Pipe是visible cell boundary，UNKNOWN markers也不会和相邻train id黏在一起。
        tc_put_char(terminal_tid, '|');
        // Marker独占one column：*是active，?是state unavailable，space是inactive。
        tc_put_char(terminal_tid, (unsigned char)(
                !sensor_state_available ? '?' : active ? '*' : ' '));
        // Cell固定four columns，single/double/triple-digit train ids都保持grid aligned。
        if (owner <= 0) {
                tc_puts(terminal_tid, "   .");
                return;
        }
        if (owner < 10) {
                tc_puts(terminal_tid, "  ");
        } else if (owner < 100) {
                tc_puts(terminal_tid, " ");
        }
        tc_puts(terminal_tid, "T");
        tc_put_uint(terminal_tid, (unsigned int)owner);
}

static void print_track_sensor_ruler(int terminal_tid, int bank, int half) {
        tc_puts(terminal_tid, "          ");
        for (int offset = 0; offset < 8; ++offset) {
                int number = half * 8 + offset + 1;
                // Ruler cell同样是six columns，和下面的marker/owner cell精确对齐。
                tc_put_char(terminal_tid, '|');
                tc_puts(terminal_tid, number < 10 ? "   " : "  ");
                tc_put_char(terminal_tid, (unsigned char)('A' + bank));
                tc_put_uint(terminal_tid, (unsigned int)number);
        }
        tc_puts(terminal_tid, "|\r\n");
}

static void print_track_range_label(int terminal_tid, int bank, int half) {
        int first = half * 8 + 1;
        int last = half * 8 + 8;
        tc_puts(terminal_tid, "  ");
        tc_put_char(terminal_tid, (unsigned char)('A' + bank));
        if (first < 10) tc_puts(terminal_tid, "0");
        tc_put_uint(terminal_tid, (unsigned int)first);
        tc_puts(terminal_tid, "-");
        tc_put_char(terminal_tid, (unsigned char)('A' + bank));
        if (last < 10) tc_puts(terminal_tid, "0");
        tc_put_uint(terminal_tid, (unsigned int)last);
        tc_puts(terminal_tid, ":");
}

static const char *tc2_failure_reason_name(int reason) {
        switch (reason) {
        case TC2_FAILURE_NONE: return "none";
        case TC2_FAILURE_ROUTE: return "route";
        case TC2_FAILURE_RESERVATION: return "reservation";
        case TC2_FAILURE_SENSOR_SERVICE: return "sensor-service";
        case TC2_FAILURE_SENSOR_SEQUENCE: return "sensor-sequence";
        case TC2_FAILURE_RESERVATION_SERVICE: return "reservation-service";
        case TC2_FAILURE_CAN_SERVICE: return "CAN-service";
        case TC2_FAILURE_CAN_HEALTH: return "CAN-health";
        case TC2_FAILURE_TURNOUT: return "turnout";
        case TC2_FAILURE_WATCHDOG: return "watchdog";
        case TC2_FAILURE_INTERNAL: return "internal";
        default: return "invalid";
        }
}

static const char *tc2_sensor_health_fault_name(int fault) {
        switch (fault) {
        case TC2_SENSOR_HEALTH_OK: return "ok";
        case TC2_SENSOR_HEALTH_SERVICE_UNAVAILABLE:
                return "service-unavailable";
        case TC2_SENSOR_HEALTH_HEARTBEAT_STALE:
                return "heartbeat-stale";
        case TC2_SENSOR_HEALTH_RECEIVE_FAILURE:
                return "receive-failure";
        case TC2_SENSOR_HEALTH_TIME_FAILURE:
                return "time-failure";
        default: return "invalid";
        }
}

static const char *tc2_job_start_label(const tc2_dispatch_job_snapshot *job) {
        return job->start_index == TC2_DISPATCH_START_CURRENT ?
                "CURRENT" : Tc2DispatchStartLabel(job->start_index);
}

static const char *tc2_job_anchor_sensor(
        const tc2_dispatch_job_snapshot *job) {
        if (job->selected_destination_side == 0) {
                return Tc2DispatchDestinationSensorA(job->destination_index);
        }
        if (job->selected_destination_side == 1) {
                return Tc2DispatchDestinationSensorB(job->destination_index);
        }
        return "pending";
}

static void print_optional_tick(int terminal_tid, int value) {
        if (value < 0) {
                tc_puts(terminal_tid, "-");
        } else {
                tc_put_int(terminal_tid, value);
        }
}

static void print_observation_tick(
        int terminal_tid, int valid, int value) {
        if (!valid) {
                tc_puts(terminal_tid, "-");
                return;
        }
        tc_put_uint(terminal_tid, (unsigned int)value);
}

static void print_optional_tick_delta(
        int terminal_tid, int valid,
        int actual, int predicted) {
        if (!valid) {
                tc_puts(terminal_tid, "-");
                return;
        }
        unsigned int forward =
                (unsigned int)actual -
                (unsigned int)predicted;
        if (forward == 0) {
                tc_puts(terminal_tid, "0");
        } else if (forward <= 0x7fffffffu) {
                tc_puts(terminal_tid, "+");
                tc_put_uint(terminal_tid, forward);
        } else if (forward == 0x80000000u) {
                tc_puts(terminal_tid, "ambiguous");
        } else {
                tc_puts(terminal_tid, "-");
                tc_put_uint(
                        terminal_tid,
                        (unsigned int)predicted -
                                (unsigned int)actual);
        }
}

static const char *tc2_stop_trigger_name(int trigger) {
        switch (trigger) {
        case TC2_STOP_TRIGGER_TIMER: return "prediction-timer";
        case TC2_STOP_TRIGGER_SENSOR: return "target-sensor-fallback";
        case TC2_STOP_TRIGGER_FAILSAFE: return "failsafe";
        default: return "none";
        }
}

static void print_dispatch_wait_reason(
        int terminal_tid, const tc2_dispatch_job_snapshot *job) {
        if (job->needs_stop_retry) {
                tc_puts(terminal_tid, "emergency stop retry");
                return;
        }
        if (job->state == TC2_JOB_FAILED ||
            job->failure_reason != TC2_FAILURE_NONE) {
                tc_puts(terminal_tid, tc2_failure_reason_name(
                        job->failure_reason));
                return;
        }
        if (job->state == TC2_JOB_STAGED) {
                tc_puts(terminal_tid, "awaiting go");
                return;
        }
        if (job->state == TC2_JOB_WAITING) {
                if (job->conflict_node >= 0) {
                        tc_puts(terminal_tid,
                                job->conflict_train > 0 ?
                                "reserved by T" : "occupied/blocked at ");
                        if (job->conflict_train > 0) {
                                tc_put_uint(
                                        terminal_tid,
                                        (unsigned int)job->conflict_train);
                                tc_puts(terminal_tid, " at ");
                        }
                        tc_puts(terminal_tid,
                                tc_track_name_or(job->conflict_node, "node?"));
                } else {
                        tc_puts(terminal_tid, "route/reservation queue");
                }
                return;
        }
        if (job->state == TC2_JOB_PREPARING) {
                tc_puts(terminal_tid,
                        "validating next/nextnext FIFO, direction, and safety");
                return;
        }
        if (job->state == TC2_JOB_READY) {
                tc_puts(terminal_tid, "batch barrier/turnout settle");
                return;
        }
        if (job->state == TC2_JOB_LAUNCHING) {
                tc_puts(terminal_tid, "awaiting CS3 batch confirmation");
                return;
        }
        if (job->state == TC2_JOB_REVERSING) {
                tc_puts(terminal_tid, "stopped reversal transition");
                return;
        }
        if (job->state == TC2_JOB_BRAKING) {
                tc_puts(terminal_tid, "braking hold");
                return;
        }
        if (job->state == TC2_JOB_TRAFFIC_HOLD) {
                if (job->traffic_reason ==
                            TC2_TRAFFIC_HEAD_ON) {
                        tc_puts(
                                terminal_tid,
                                "HEAD-ON HOLD with T");
                        tc_put_uint(
                                terminal_tid,
                                (unsigned int)
                                        job->traffic_peer_train);
                        tc_puts(
                                terminal_tid,
                                "; if no safe route is available, option 1: "
                                "cancel one train, wait "
                                "STOP_HOLD, physically remove it, then "
                                "remove it; option 2: reverse/reroute with "
                                "dispatch T");
                        tc_put_uint(
                                terminal_tid,
                                (unsigned int)job->train);
                        tc_puts(
                                terminal_tid,
                                " CURRENT <speed> <d1-d8>, then go");
                } else if (job->traffic_reason ==
                           TC2_TRAFFIC_FOLLOWING) {
                        tc_puts(
                                terminal_tid,
                                "same-direction rear within 350mm of "
                                "a speed-zero obstacle; next authority "
                                "remains reserved");
                } else if (job->traffic_reason ==
                           TC2_TRAFFIC_AUDIT) {
                        tc_puts(
                                terminal_tid,
                                "temporary topology/position audit hold");
                } else if (job->traffic_reason ==
                           TC2_TRAFFIC_AUTHORITY) {
                        tc_puts(
                                terminal_tid,
                                "rolling-authority/intersection wait");
                } else {
                        tc_puts(
                                terminal_tid,
                                "collision-safety hold");
                }
                return;
        }
        if (job->state == TC2_JOB_STOP_UNCONFIRMED) {
                tc_puts(terminal_tid,
                        "destination unconfirmed; cancel then remove");
                return;
        }
        if (job->state == TC2_JOB_CANCEL_BRAKING) {
                tc_puts(terminal_tid, "cancel braking/settle");
                return;
        }
        if (job->state == TC2_JOB_STOPPED) {
                tc_puts(terminal_tid,
                        "stopped/static; reroute or remove is available");
                return;
        }
        tc_puts(terminal_tid, "-");
}

static void print_can_health(int terminal_tid) {
        can_health_t health;
        tc_puts(terminal_tid, "CAN current: ");
        if (tc_can_server_tid < 0 ||
            CanGetHealth(tc_can_server_tid, &health) < 0) {
                tc_puts(terminal_tid, "UNAVAILABLE\r\n");
                return;
        }
        tc_puts(terminal_tid, "hw=");
        tc_puts(terminal_tid, health.hw_ready ? "READY" : "NOT_READY");
        tc_puts(terminal_tid, " tx-ok=");
        tc_put_uint(terminal_tid, health.tx_completed);
        tc_puts(terminal_tid, " tx-fail=");
        tc_put_uint(terminal_tid, health.tx_failed);
        tc_puts(terminal_tid, " tx-timeout=");
        tc_put_uint(terminal_tid, health.tx_timeout);
        tc_puts(terminal_tid, " rx=");
        tc_put_uint(terminal_tid, health.rx_received);
        tc_puts(terminal_tid, " dropped=");
        tc_put_uint(terminal_tid, health.rx_dropped);
        tc_puts(terminal_tid, " overflow=");
        tc_put_uint(terminal_tid, health.rx_overflow);
        tc_puts(terminal_tid, " unexpected-turnout=");
        tc_put_uint(terminal_tid, health.turnout_changes);
        tc_puts(terminal_tid, "\r\n");
}

static void print_dispatch_turnout_step(
        int terminal_tid, int action_count,
        const int switch_number[TC2_DISPATCH_TURNOUT_ACTIONS_PER_STEP],
        const char direction[TC2_DISPATCH_TURNOUT_ACTIONS_PER_STEP]) {
        if (action_count <= 0) {
                tc_puts(terminal_tid, "none");
                return;
        }
        if (action_count > TC2_DISPATCH_TURNOUT_ACTIONS_PER_STEP) {
                action_count = TC2_DISPATCH_TURNOUT_ACTIONS_PER_STEP;
        }
        for (int action = 0; action < action_count; ++action) {
                if (action > 0) tc_put_char(terminal_tid, '+');
                tc_puts(terminal_tid, "SW");
                tc_put_int(terminal_tid, switch_number[action]);
                char setting = '?';
                if (direction[action] == 'C' ||
                    direction[action] == 'S') {
                        setting = direction[action];
                }
                tc_put_char(terminal_tid, setting);
        }
}

static void print_dispatch_jobs(int terminal_tid) {
        tc2_dispatch_snapshot snapshot;

        tc_puts(terminal_tid,
                "\r\n----------------- MULTI-TRAIN DISPATCH ----------------\r\n");
        if (tc_dispatch_server_tid < 0 ||
            Tc2DispatchGetSnapshot(tc_dispatch_server_tid, &snapshot) < 0) {
                tc_puts(terminal_tid, "dispatch state unavailable\r\n");
                return;
        }

        tc_puts(terminal_tid, "scheduler=");
        tc_puts(terminal_tid,
                snapshot.scheduler_healthy ? "HEALTHY" : "UNAVAILABLE");
        tc_puts(terminal_tid, " | batch_epoch=");
        tc_put_uint(terminal_tid, snapshot.launch_epoch);
        tc_puts(terminal_tid, " | batch_ready_tick=");
        print_optional_tick(terminal_tid, snapshot.batch_ready_at_tick);
        tc_puts(terminal_tid, " | jobs=");
        tc_put_uint(terminal_tid, (unsigned int)snapshot.job_count);
        tc_puts(terminal_tid, " | blocked=");
        tc_put_uint(terminal_tid,
                    (unsigned int)snapshot.blocked_physical_count);
        tc_puts(terminal_tid, "\r\n");
        print_can_health(terminal_tid);

        int printed = 0;
        for (int i = 0; i < TC2_DISPATCH_MAX_JOBS; ++i) {
                tc2_dispatch_job_snapshot *job = &snapshot.jobs[i];
                if (job->state == TC2_JOB_EMPTY) continue;
                tc_puts(terminal_tid,
                        "------------------------------------------------------\r\nT");
                tc_put_uint(terminal_tid, (unsigned int)job->train);
                tc_puts(terminal_tid, " from=");
                tc_puts(terminal_tid, tc2_job_start_label(job));
                tc_puts(terminal_tid, " speed=");
                tc_put_int(terminal_tid, job->speed);
                tc_puts(terminal_tid, " destination=");
                tc_puts(terminal_tid,
                        Tc2DispatchDestinationLabel(job->destination_index));
                tc_puts(terminal_tid, " anchor_sensor=");
                tc_puts(terminal_tid, tc2_job_anchor_sensor(job));
                tc_puts(terminal_tid, " user_endpoint_offset=");
                if (job->selected_destination_side < 0) {
                        tc_puts(terminal_tid, "pending-route");
                } else {
                        tc_put_int(
                                terminal_tid,
                                job->destination_offset_mm);
                        tc_puts(terminal_tid, "mm(");
                        tc_puts(terminal_tid,
                                job->destination_offset_confirmed ?
                                "CONFIRMED" : "PROVISIONAL");
                        tc_puts(terminal_tid, ";base=");
                        tc_put_int(
                                terminal_tid,
                                job->destination_base_offset_mm);
                        tc_puts(terminal_tid, "mm speed_correction=");
                        tc_put_int(
                                terminal_tid,
                                job->destination_speed_correction_mm);
                        tc_puts(terminal_tid, "mm)");
                }
                tc_puts(terminal_tid, " state=");
                tc_puts(terminal_tid, Tc2DispatchStateName(job->state));
                tc_puts(terminal_tid, "\r\n  position=");
                if (job->current_node >= 0) {
                        tc_puts(terminal_tid,
                                tc_track_name_or(job->current_node, "node?"));
                        tc_puts(terminal_tid,
                                job->position_estimated ?
                                " (estimated)" : " (sensor-confirmed)");
                } else {
                        tc_puts(terminal_tid, "unknown");
                }
                tc_puts(terminal_tid, " | progress=");
                tc_put_int(terminal_tid, job->confirmed_distance_mm);
                tc_puts(terminal_tid, "/");
                tc_put_int(terminal_tid, job->route_distance_mm);
                tc_puts(terminal_tid, "mm | remaining=");
                tc_put_int(terminal_tid, job->remaining_distance_mm);
                tc_puts(terminal_tid, "mm | next_sensor=");
                tc_puts(terminal_tid,
                        tc_track_name_or(job->next_sensor_node, "none"));
                tc_puts(terminal_tid, "\r\n  turnout_plan: steps=");
                tc_put_int(terminal_tid, job->turnout_plan_step_count);
                tc_puts(terminal_tid, " actions=");
                tc_put_int(terminal_tid, job->turnout_plan_action_count);
                tc_puts(terminal_tid, " | next_turnout=");
                print_dispatch_turnout_step(
                        terminal_tid,
                        job->next_turnout_action_count,
                        job->next_turnout_switch,
                        job->next_turnout_direction);
                tc_puts(terminal_tid, " | next_next_turnout=");
                print_dispatch_turnout_step(
                        terminal_tid,
                        job->next_next_turnout_action_count,
                        job->next_next_turnout_switch,
                        job->next_next_turnout_direction);
                tc_puts(terminal_tid, "\r\n  route_offset=current:");
                tc_put_int(terminal_tid, job->current_route_offset);
                tc_puts(terminal_tid, " anchor:");
                tc_put_int(terminal_tid, job->target_route_offset);
                tc_puts(terminal_tid, " endpoint-ceiling:");
                tc_put_int(terminal_tid, job->destination_route_offset);
                tc_puts(terminal_tid, " | reversals=");
                tc_put_int(terminal_tid, job->reversals_completed);
                tc_puts(terminal_tid, "/");
                tc_put_int(terminal_tid, job->route_reversal_count);
                tc_puts(terminal_tid, " | leg=");
                tc_put_int(terminal_tid, job->current_leg);
                tc_puts(terminal_tid, "\r\n  sensors: missing=");
                tc_put_int(terminal_tid, job->missing_sensor_count);
                tc_puts(terminal_tid, " spurious=");
                tc_put_int(terminal_tid, job->spurious_sensor_count);
                tc_puts(terminal_tid, " duplicate=");
                tc_put_int(terminal_tid, job->duplicate_sensor_count);
                tc_puts(terminal_tid, " journal_lost=");
                tc_puts(terminal_tid, job->journal_lost ? "yes" : "no");
                tc_puts(terminal_tid, " health=");
                tc_puts(
                        terminal_tid,
                        tc2_sensor_health_fault_name(
                                job->sensor_health_fault));
                tc_puts(terminal_tid, "\r\n  reservation: generation=");
                tc_put_uint(terminal_tid, job->plan_generation);
                tc_puts(terminal_tid, " hold=");
                tc_puts(terminal_tid, job->hold_active ? "ACTIVE" : "no");
                tc_puts(terminal_tid, " | wait_ticks=");
                tc_put_int(terminal_tid, job->wait_age_ticks);
                tc_puts(terminal_tid, " queue=");
                tc_put_uint(terminal_tid, job->queue_sequence);
                tc_puts(terminal_tid, "\r\n  batch: epoch=");
                tc_put_uint(terminal_tid, job->job_launch_epoch);
                tc_puts(terminal_tid, " enqueue_tick=");
                print_optional_tick(terminal_tid, job->launch_tick);
                tc_puts(terminal_tid, " confirm_delay_ticks=");
                tc_put_int(terminal_tid, job->launch_skew_ticks);
                tc_puts(terminal_tid, " braking_tick=");
                print_optional_tick(terminal_tid, job->braking_at_tick);
                if (job->provisional_prediction) {
                        tc_puts(terminal_tid,
                                "\r\n  prediction=PROVISIONAL_T14_A_D7");
                        tc_puts(terminal_tid, " velocity=");
                        tc_put_int(
                                terminal_tid,
                                job->prediction_velocity_um_per_tick);
                        tc_puts(terminal_tid,
                                "um/10ms stop_distance=");
                        tc_put_int(
                                terminal_tid,
                                job->prediction_stop_distance_um);
                        tc_puts(terminal_tid, "um(rounded_display=");
                        tc_put_int(
                                terminal_tid,
                                job->prediction_stop_distance_mm);
                        tc_puts(terminal_tid, "mm)");
                        tc_puts(terminal_tid,
                                "\r\n  prediction_anchor=");
                        tc_puts(
                                terminal_tid,
                                tc_track_name_or(
                                        job->prediction_anchor_node,
                                        "pending"));
                        tc_puts(terminal_tid, " tick=");
                        print_observation_tick(
                                terminal_tid,
                                job->prediction_timing_valid,
                                job->prediction_anchor_tick);
                        tc_puts(terminal_tid, " distance=");
                        tc_put_int(
                                terminal_tid,
                                job->prediction_anchor_distance_mm);
                        tc_puts(terminal_tid,
                                "mm predicted_request_tick=");
                        print_observation_tick(
                                terminal_tid,
                                job->prediction_timing_valid,
                                job->prediction_command_at_tick);
                        tc_puts(terminal_tid,
                                "\r\n  actual_request_tick=");
                        print_observation_tick(
                                terminal_tid,
                                job->stop_timing_valid,
                                job->stop_request_tick);
                        tc_puts(terminal_tid,
                                " confirmed_tick=");
                        print_observation_tick(
                                terminal_tid,
                                job->stop_timing_valid,
                                job->stop_confirmed_tick);
                        tc_puts(terminal_tid,
                                " request_delta_ticks=");
                        print_optional_tick_delta(
                                terminal_tid,
                                job->prediction_timing_valid &&
                                        job->stop_timing_valid,
                                job->stop_request_tick,
                                job->prediction_command_at_tick);
                        tc_puts(terminal_tid,
                                " confirm_delay_ticks=");
                        print_optional_tick_delta(
                                terminal_tid,
                                job->stop_timing_valid,
                                job->stop_confirmed_tick,
                                job->stop_request_tick);
                        tc_puts(terminal_tid, " trigger=");
                        tc_puts(
                                terminal_tid,
                                tc2_stop_trigger_name(
                                        job->stop_trigger));
                }
                tc_puts(terminal_tid, "\r\n  CAN@job: tx-ok=");
                tc_put_uint(terminal_tid, job->can_tx_completed);
                tc_puts(terminal_tid, " fail=");
                tc_put_uint(terminal_tid, job->can_tx_failed);
                tc_puts(terminal_tid, " timeout=");
                tc_put_uint(terminal_tid, job->can_tx_timeout);
                tc_puts(terminal_tid, " rx-drop=");
                tc_put_uint(terminal_tid, job->can_rx_dropped);
                tc_puts(terminal_tid, " overflow=");
                tc_put_uint(terminal_tid, job->can_rx_overflow);
                tc_puts(terminal_tid, " turnout-change=");
                tc_put_uint(
                        terminal_tid,
                        job->can_turnout_changes);
                tc_puts(terminal_tid, "\r\n  wait/reason=");
                print_dispatch_wait_reason(terminal_tid, job);
                tc_puts(terminal_tid, "\r\n");
                printed = 1;
        }
        if (!printed) tc_puts(terminal_tid, "none staged\r\n");
        tc_puts(terminal_tid,
                "go submits all STAGED jobs as one batch; queued conflicts remain stopped.\r\n");
}

static int tc2_dispatch_is_active(void) {
        tc2_dispatch_snapshot snapshot;
        if (tc_dispatch_server_tid < 0 ||
            Tc2DispatchGetSnapshot(tc_dispatch_server_tid, &snapshot) < 0) {
                // In TC2, an unavailable scheduler cannot safely coexist with manual motion.
                return 1;
        }
        if (!snapshot.scheduler_healthy) {
                return 1;
        }
        for (int i = 0; i < TC2_DISPATCH_MAX_JOBS; ++i) {
                int job_state = snapshot.jobs[i].state;
                if (job_state != TC2_JOB_EMPTY) {
                        return 1;
                }
        }
        return 0;
}

static void print_blocked_nodes(int terminal_tid) {
        tc2_dispatch_blocked_snapshot snapshot;
        int printed = 0;

        tc_puts(terminal_tid, "\r\nblocked physical locations: ");
        if (tc_dispatch_server_tid < 0 ||
            Tc2DispatchGetBlockedSnapshot(tc_dispatch_server_tid,
                                          &snapshot) < 0) {
                tc_puts(terminal_tid, "UNAVAILABLE\r\n");
                return;
        }
        tc_put_uint(terminal_tid,
                    (unsigned int)snapshot.blocked_physical_count);
        tc_puts(terminal_tid, "\r\n");
        for (int node = 0; node < TRACK_MAX; ++node) {
                if (!snapshot.blocked_by_node[node]) continue;
                int reverse = -1;
                if (tc2_track[node].reverse) {
                        reverse = (int)(tc2_track[node].reverse - tc2_track);
                }
                if (reverse >= 0 && reverse < TRACK_MAX &&
                    snapshot.blocked_by_node[reverse] && reverse < node) {
                        continue;
                }
                tc_puts(terminal_tid, "  ");
                tc_puts(terminal_tid, tc_track_name_or(node, "node?"));
                if (reverse >= 0 && reverse < TRACK_MAX &&
                    snapshot.blocked_by_node[reverse]) {
                        tc_puts(terminal_tid, " / ");
                        tc_puts(terminal_tid,
                                tc_track_name_or(reverse, "node?"));
                }
                tc_puts(terminal_tid, "\r\n");
                printed = 1;
        }
        if (!printed) tc_puts(terminal_tid, "  none\r\n");
}

static void print_manual_control_blocked(int terminal_tid) {
        tc_puts(terminal_tid,
                "\r\nmanual control disabled in TC2: use "
                "dispatch/go/trips/cancel/remove so all motion remains protected\r\n");
}

static void print_dispatch_track(int terminal_tid) {
        char layout_error[96];
        char layout_line[TC2_TRACK_D_LAYOUT_COLUMNS + 1];

        tc_puts(terminal_tid,
                "\r\n================= TC2 DISPATCH TRACK D =================\r\n");
        if (Tc2TrackDLayoutValidate(
                    layout_error, sizeof(layout_error)) < 0) {
                tc_puts(terminal_tid, "layout validation failed: ");
                tc_puts(terminal_tid, layout_error);
                tc_puts(terminal_tid, "\r\n");
        } else {
                for (size_t row = 0;
                     row < Tc2TrackDLayoutLineCount(); ++row) {
                        if (Tc2TrackDLayoutRenderLine(
                                    row, layout_line,
                                    sizeof(layout_line)) < 0) {
                                tc_puts(terminal_tid,
                                        "[layout row unavailable]\r\n");
                                continue;
                        }
                        tc_puts(terminal_tid, layout_line);
                        tc_puts(terminal_tid, "\r\n");
                }
        }
        tc_puts(terminal_tid,
                "\r\nTRACK D CATALOG / MODEL EVIDENCE\r\n"
                " starts: A=EN5 B=EN4 C=EN7 D=EN10 E=EN9 F=EN3\r\n"
                " endpoint intervals: d1={A12,A15} d2={A4,B15} d3={C11,B6} d4={C10,B2}\r\n"
                "                     d5={E5,E6} d6={E13,E14} d7={E9,D6} d8={C15,D11}\r\n"
                " parts: d1=CAN-side 24064; d2/d3/d4/d8=24077; d5/d6=24664.\r\n"
                " measured directed offsets (mm): d1=381/381 d2=226/226 d3=89/254 d4=76/279\r\n"
                "                                d5=0/0 d6=0/0 d7=305/305 d8=203/203.\r\n"
                " train 14 uses the measured 8in body, centre pickup, and speed 14..120 stopping curve; all endpoint sides remain TRIAL until repeated.\r\n"
                " trips reports anchor_sensor and the separate user_endpoint_offset; sensor position and model endpoint are never treated as the same point.\r\n");
        print_dispatch_jobs(terminal_tid);
}

static void print_track_view(int terminal_tid) {
        track_reservation_snapshot snapshot;
        train_sensor_snapshot_t sensor_snapshot;
        int has_sensor_state = 0;
        int active_count = 0;
        int reserved_sensor_count = 0;
        int reserved_train_count = 0;

        print_dispatch_track(terminal_tid);
        print_blocked_nodes(terminal_tid);

        if (tc_reservation_server_tid < 0 ||
            TrackReservationServerSnapshot(tc_reservation_server_tid,
                                           &snapshot) < 0) {
                tc_puts(terminal_tid, "\r\ntrack view unavailable\r\n");
                return;
        }
        if (tc_sensor_server_tid >= 0 &&
            TrainSensorGetLatest(tc_sensor_server_tid, &sensor_snapshot) == 0) {
                has_sensor_state = 1;
        }

        // Reservation和sensor是sequential IPC snapshots，所以这里只呈现sample状态。
        for (int sensor = 0; sensor < TRAIN_SENSOR_COUNT; ++sensor) {
                if (snapshot.owner_by_node[sensor] > 0 &&
                    snapshot.owner_by_node[sensor] <= 255) {
                        ++reserved_sensor_count;
                }
                if (has_sensor_state && sensor_snapshot.sensor_state[sensor]) {
                        ++active_count;
                }
        }
        // reserved_trains按valid destinations计算，也包含没有sensor landmark的route。
        for (int train = 1; train <= 255; ++train) {
                int destination = snapshot.destination_by_train[train];
                if (destination >= 0 && destination < TRACK_MAX) {
                        ++reserved_train_count;
                }
        }

        tc_puts(terminal_tid,
                "\r\n============== TC2 TRACK B/D SAMPLE ==============\r\n");
        tc_puts(terminal_tid, " sensor_state=");
        tc_puts(terminal_tid, has_sensor_state ? "OK" : "UNAVAILABLE");
        tc_puts(terminal_tid, " | active=");
        if (has_sensor_state) {
                tc_put_uint(terminal_tid, (unsigned int)active_count);
        } else {
                tc_puts(terminal_tid, "?");
        }
        tc_puts(terminal_tid, " | reserved_sensors=");
        tc_put_uint(terminal_tid, (unsigned int)reserved_sensor_count);
        tc_puts(terminal_tid, " | reserved_trains=");
        tc_put_uint(terminal_tid, (unsigned int)reserved_train_count);
        tc_puts(terminal_tid,
                "\r\n--------------------------------------------------\r\n");
        // 每个bank分成two bounded rows，避免16个variable-width train ids挤在一行。
        for (int bank = 0; bank < 5; ++bank) {
                for (int half = 0; half < 2; ++half) {
                        int first = bank * 16 + half * 8;
                        int last = first + 7;
                        print_track_sensor_ruler(terminal_tid, bank, half);
                        print_track_range_label(terminal_tid, bank, half);
                        for (int sensor = first; sensor <= last; ++sensor) {
                                int owner = snapshot.owner_by_node[sensor];
                                int active = has_sensor_state ?
                                        sensor_snapshot.sensor_state[sensor] : 0;
                                print_track_owner_cell(terminal_tid, owner,
                                                       has_sensor_state, active);
                        }
                        tc_puts(terminal_tid, "|\r\n");
                }
        }
        tc_puts(terminal_tid,
                "legend: *=active, ?=sensor state unavailable\r\n"
                "        .=unreserved, Tn=reserved by train n\r\n");
        tc_puts(terminal_tid, "position candidates:\r\n");
        if (!has_sensor_state) {
                tc_puts(terminal_tid, "  unavailable\r\n");
                return;
        }

        int printed = 0;
        int columns = 0;
        for (int train = 1; train <= 255; ++train) {
                int sensor = sensor_snapshot.position_by_train[train];
                if (sensor < 0 || sensor >= TRAIN_SENSOR_COUNT) continue;
                if (columns == 0) {
                        tc_puts(terminal_tid, "  ");
                } else {
                        tc_puts(terminal_tid, " ");
                }
                tc_puts(terminal_tid, "T");
                tc_put_uint(terminal_tid, (unsigned int)train);
                tc_puts(terminal_tid, "=");
                tc_puts(terminal_tid, tc2_track[sensor].name);
                tc_puts(terminal_tid, position_matches_current_plan(
                        &sensor_snapshot, &snapshot, train) ?
                        "(current)" : "(stale)");
                ++printed;
                ++columns;
                if (columns == 4) {
                        tc_puts(terminal_tid, "\r\n");
                        columns = 0;
                }
        }
        if (!printed) {
                tc_puts(terminal_tid, "  none\r\n");
        } else if (columns != 0) {
                tc_puts(terminal_tid, "\r\n");
        }
}

static void preview_sensor_route(int terminal_tid, const char *destination_name) {
        train_sensor_snapshot_t snapshot;
        track_route route;
        track_turnout_plan turnout_plan;
        int destination;

        // 这个preview从latest occupied sensor开始算route，only reports并且不会move train。
        if (tc_sensor_server_tid < 0 ||
            TrainSensorGetLatest(tc_sensor_server_tid, &snapshot) < 0 ||
            !snapshot.has_occupied) {
                tc_puts(terminal_tid, "\r\nsensor route unavailable: no occupied sensor\r\n");
                return;
        }

        destination = TrackFindNodeByName(tc2_track, destination_name);
        if (destination < 0) {
                tc_puts(terminal_tid, "\r\nsensor route unavailable: unknown destination\r\n");
                return;
        }

        if (TrackFindShortestRouteWithReversals(tc2_track,
                                                snapshot.latest_occupied.sensor_index,
                                                destination,
                                                TC2_REVERSAL_PENALTY_MM,
                                                &route) < 0) {
                tc_puts(terminal_tid, "\r\nsensor route unavailable: no route\r\n");
                return;
        }

        if (TrackBuildTurnoutPlan(tc2_track, &route, &turnout_plan) < 0) {
                tc_puts(terminal_tid, "\r\nsensor route unavailable: invalid turnout plan\r\n");
                return;
        }

        tc_puts(terminal_tid, "\r\nsensor route preview: from=");
        tc_puts(terminal_tid, tc2_track[route.nodes[0]].name);
        tc_puts(terminal_tid, " to=");
        tc_puts(terminal_tid, tc2_track[route.nodes[route.node_count - 1]].name);
        tc_puts(terminal_tid, " distance_mm=");
        tc_put_uint(terminal_tid, (unsigned int)route.distance_mm);
        tc_puts(terminal_tid, " optimization_cost_mm=");
        tc_put_uint(terminal_tid, (unsigned int)route.optimization_cost_mm);
        tc_puts(terminal_tid, " reversals=");
        tc_put_uint(terminal_tid, (unsigned int)route.reversal_count);
        tc_puts(terminal_tid, " nodes=");
        tc_put_uint(terminal_tid, (unsigned int)route.node_count);
        tc_puts(terminal_tid, " turnout_actions=");
        tc_put_uint(terminal_tid, (unsigned int)turnout_plan.action_count);
        tc_puts(terminal_tid, "\r\n");
}

static void print_reservations(int terminal_tid) {
        track_reservation_snapshot snapshot;
        int printed = 0;

        if (tc_reservation_server_tid < 0 ||
            TrackReservationServerSnapshot(tc_reservation_server_tid, &snapshot) < 0) {
                tc_puts(terminal_tid, "\r\nreservations unavailable\r\n");
                return;
        }

        tc_puts(terminal_tid, "\r\nreservation owners:\r\n");
        for (int train = 1; train <= 255; ++train) {
                int count = 0;
                for (int node = 0; node < TRACK_MAX; ++node) {
                        if (snapshot.owner_by_node[node] == train) ++count;
                }
                if (count == 0) continue;
                tc_puts(terminal_tid, "  train=");
                tc_put_uint(terminal_tid, (unsigned int)train);
                tc_puts(terminal_tid, " reserved_nodes=");
                tc_put_uint(terminal_tid, (unsigned int)count);
                tc_puts(terminal_tid, " planned_destination=");
                int destination = snapshot.destination_by_train[train];
                tc_puts(terminal_tid, destination >= 0 && destination < TRACK_MAX ?
                        tc2_track[destination].name : "none");
                tc_puts(terminal_tid, " plan_generation=");
                tc_put_uint(terminal_tid, snapshot.generation_by_train[train]);
                tc_puts(terminal_tid, "\r\n    sensor_landmarks=");
                int sensor_count = 0;
                //前80个track nodes是sensors；列出landmarks让physical reservation可现场核对。
                for (int node = 0; node < TRAIN_SENSOR_COUNT; ++node) {
                        if (snapshot.owner_by_node[node] != train) continue;
                        if (sensor_count > 0) tc_puts(terminal_tid, ",");
                        tc_puts(terminal_tid, tc2_track[node].name);
                        ++sensor_count;
                }
                if (sensor_count == 0) tc_puts(terminal_tid, "none");
                tc_puts(terminal_tid, "\r\n");
                printed = 1;
        }
        if (!printed) tc_puts(terminal_tid, "  none\r\n");
}
#endif

static int apply_one_switch(int terminal_tid, int can_tid, int sw, char dir) {
        int ret;

        if (sw <= 0) {
                return 0;
        }

        tc_puts(terminal_tid, "  switch ");
        tc_put_uint(terminal_tid, (unsigned int)sw);
        tc_puts(terminal_tid, " -> ");
        tc_put_char(terminal_tid, (unsigned char)dir);

        ret = CanSwitch(can_tid, sw, dir);
        tc_puts(terminal_tid, ret == 0 ? " ok\r\n" : " failed\r\n");

        return ret;
}


static int apply_calibrated_route(int terminal_tid, int can_tid, const char *target_name) {
        int ret = 0;

#define APPLY_ROUTE_SWITCH(sw, dir) do { \
        if (apply_one_switch(terminal_tid, can_tid, (sw), (dir)) < 0) { \
                ret = -1; \
        } \
} while (0)

        /*
         * Full calibrated Track-D / Track-B-style routes from the fixed start:
         * start at the top 24977 piece, moving into A1 -> MR12.
         *
         * These route lists set both facing turnouts and important trailing
         * turnout alignments on the path so old switch states do not leak in.
         * Use physical turnout numbers from the official diagram.
         * Do NOT apply switch_no - 1 in CanSwitch().
         */

        if (tc_streq(target_name, "A5")) {
                /*
                 * From fixed start to A5:
                 * Observed required turnout is physical switch 8 -> S.
                 * CanSwitch() already maps physical label to CAN address by -1.
                 */
                APPLY_ROUTE_SWITCH(8, 'S');
                return ret;
        }

        if (tc_streq(target_name, "B7")) {
                /*
                 * From fixed start to B7:
                 * Observed required turnouts are:
                 *   8   -> S
                 *   17  -> S
                 *   155 -> S
                 */
                APPLY_ROUTE_SWITCH(8, 'S');
                APPLY_ROUTE_SWITCH(17, 'S');
                APPLY_ROUTE_SWITCH(155, 'S');
                return ret;
        }

        if (tc_streq(target_name, "C8")) {
                /*
                 * From fixed start to C8:
                 * 8 S and 17 S put the train on the C/D-side path,
                 * 156 S selects the C8/D10 side connection,
                 * 15 S selects the final C8 branch.
                 */
                APPLY_ROUTE_SWITCH(8, 'S');
                APPLY_ROUTE_SWITCH(17, 'S');
                APPLY_ROUTE_SWITCH(156, 'S');
                APPLY_ROUTE_SWITCH(15, 'S');
                return ret;
        }

        if (tc_streq(target_name, "D10")) {
                /*
                 * From fixed start to D10:
                 * 8 S and 17 S put the train on the C/D-side path,
                 * 156 S selects the D10-side route.
                 * Unlike C8, D10 does not need switch 15.
                 */
                APPLY_ROUTE_SWITCH(8, 'S');
                APPLY_ROUTE_SWITCH(17, 'S');
                APPLY_ROUTE_SWITCH(156, 'S');
                return ret;
        }

        if (tc_streq(target_name, "LOOP")) {
                /*
                 * From fixed start to LOOP:
                 * Observed required turnouts are:
                 *   8  -> C
                 *   7  -> S
                 *   18 -> C
                 *   3  -> S
                 *   2  -> C
                 *   1  -> C
                 */
                APPLY_ROUTE_SWITCH(8, 'C');
                APPLY_ROUTE_SWITCH(7, 'S');
                APPLY_ROUTE_SWITCH(18, 'C');
                APPLY_ROUTE_SWITCH(3, 'S');
                APPLY_ROUTE_SWITCH(2, 'C');
                APPLY_ROUTE_SWITCH(1, 'C');
                return ret;
        }

#undef APPLY_ROUTE_SWITCH

        return -2;
}

static int apply_route(int terminal_tid, int can_tid, const char *target_name) {
        tc_target_t *target = find_target(target_name);
        int ret = 0;
        int calibrated_ret;

        if (!target) {
                tc_puts(terminal_tid, "\r\nunknown target: ");
                tc_puts(terminal_tid, target_name);
                tc_puts(terminal_tid, "\r\n");
                print_targets(terminal_tid);
                return -1;
        }

        tc_puts(terminal_tid, "\r\nApplying route to ");
        tc_puts(terminal_tid, target->name);
        tc_puts(terminal_tid, "\r\n");

        calibrated_ret = apply_calibrated_route(terminal_tid, can_tid, target->name);
        if (calibrated_ret != -2) {
                return calibrated_ret;
        }

        if (apply_one_switch(terminal_tid, can_tid, target->switch_a, target->dir_a) < 0) {
                ret = -1;
        }

        if (apply_one_switch(terminal_tid, can_tid, target->switch_b, target->dir_b) < 0) {
                ret = -1;
        }

        if (apply_one_switch(terminal_tid, can_tid, target->switch_c, target->dir_c) < 0) {
                ret = -1;
        }

        /*
         * Generic route-table fallback.  Current TC1 destinations are
         * handled earlier by apply_calibrated_route(), so this path is
         * kept only as a simple safety fallback for future target entries.
         */
        if (streq(target->name, "LOOP")) {
                apply_one_switch(terminal_tid, can_tid, 3, 'S');
                apply_one_switch(terminal_tid, can_tid, 2, 'C');
                apply_one_switch(terminal_tid, can_tid, 1, 'C');
        }

        return ret;
}

static int predict_run_ticks(int terminal_tid, tc_target_t *target, int speed) {
        int velocity = tc_velocity_for_speed(speed);
        int stop_distance = tc_stop_distance_for_speed(speed);
        int travel_before_stop;

        if (velocity <= 0) {
                tc_puts(terminal_tid, "\r\ncannot predict for speed zero\r\n");
                return -1;
        }

        travel_before_stop = target->distance_mm - stop_distance;

        if (travel_before_stop < 0) {
                travel_before_stop = 0;
        }

        return travel_before_stop / velocity;
}


static int preview_stop_plan(int terminal_tid, const char *target_name, int speed) {
        tc_target_t *target;
        int run_ticks;
        int travel_before_stop;
        int predicted_ms;

        if (speed < 1 || speed > TC_MAX_USER_SPEED) {
                tc_puts(terminal_tid, "\r\ninvalid speed for plan\r\n");
                return -1;
        }

        target = find_target(target_name);
        if (!target) {
                tc_puts(terminal_tid, "\r\nunknown target: ");
                tc_puts(terminal_tid, target_name);
                tc_puts(terminal_tid, "\r\n");
                print_targets(terminal_tid);
                return -1;
        }

        run_ticks = predict_run_ticks(terminal_tid, target, speed);
        if (run_ticks < 0) {
                return -1;
        }

        if (run_ticks < 20) {
                run_ticks = 20;
        }

        /*
         * For the speed-120 demo, use target-specific timing calibrated
         * relative to A5.  A5 is the anchor target that already stops correctly.
         */
        if (speed == 120) {
                int a5_base_ticks;

                a5_base_ticks = predict_run_ticks(terminal_tid, find_target("A5"), speed);
                if (a5_base_ticks < 20) {
                        a5_base_ticks = 20;
                }

                run_ticks = tc_target120_ticks(target->name, a5_base_ticks);

                if (run_ticks < 20) {
                        run_ticks = 20;
                }
        }

        travel_before_stop =
                target->distance_mm - tc_stop_distance_for_speed(speed);
        if (travel_before_stop < 0) {
                travel_before_stop = 0;
        }

        predicted_ms = run_ticks * 10;

#ifdef MODE_TC2
        tc_puts(terminal_tid, "\r\nLegacy stop-plan preview:\r\n");
#else
        tc_puts(terminal_tid, "\r\nTC1 stop-plan preview:\r\n");
#endif
        tc_puts(terminal_tid, "  target=");
        tc_puts(terminal_tid, target->name);
        tc_puts(terminal_tid, "\r\n  speed=");
        tc_put_uint(terminal_tid, (unsigned int)speed);
        tc_puts(terminal_tid, "\r\n  target_distance_mm=");
        tc_put_uint(terminal_tid, (unsigned int)target->distance_mm);
        tc_puts(terminal_tid, "\r\n  stop_distance_mm=");
        tc_put_uint(terminal_tid,
                    (unsigned int)tc_stop_distance_for_speed(speed));
        tc_puts(terminal_tid, "\r\n  travel_before_stop_mm=");
        tc_put_uint(terminal_tid, (unsigned int)travel_before_stop);
        tc_puts(terminal_tid, "\r\n  velocity_mm_per_tick=");
        tc_put_uint(terminal_tid,
                    (unsigned int)tc_velocity_for_speed(speed));
        tc_puts(terminal_tid, "\r\n  predicted_run_ticks=");
        tc_put_uint(terminal_tid, (unsigned int)run_ticks);
        tc_puts(terminal_tid, "\r\n  predicted_run_ms=");
        tc_put_uint(terminal_tid, (unsigned int)predicted_ms);
        tc_puts(terminal_tid, "\r\n  route_switches=");

        print_route_switches(terminal_tid, target->name);

        tc_puts(terminal_tid, "\r\n");

        return 0;
}

static int compare_observation(int terminal_tid, const char *target_name, int speed, int actual_ticks) {
        tc_target_t *target;
        int predicted_ticks;
        int error_ticks;
        int error_mm;

        if (speed < 1 || speed > TC_MAX_USER_SPEED || actual_ticks < 0) {
                tc_puts(terminal_tid, "\r\ninvalid obs arguments\r\n");
                return -1;
        }

        target = find_target(target_name);
        if (!target) {
                tc_puts(terminal_tid, "\r\nunknown target: ");
                tc_puts(terminal_tid, target_name);
                tc_puts(terminal_tid, "\r\n");
                return -1;
        }

        predicted_ticks = predict_run_ticks(terminal_tid, target, speed);
        if (predicted_ticks < 20) {
                predicted_ticks = 20;
        }

        error_ticks = actual_ticks - predicted_ticks;
        error_mm = error_ticks * tc_velocity_for_speed(speed);

        state.last_target = target->name;
        state.last_run_ticks = predicted_ticks;
        state.last_stop_distance_mm = tc_stop_distance_for_speed(speed);
        state.last_prediction_error_ticks = error_ticks;

#ifdef MODE_TC2
        tc_puts(terminal_tid, "\r\nLegacy sensor/timing observation:\r\n");
#else
        tc_puts(terminal_tid, "\r\nTC1 sensor/timing observation:\r\n");
#endif
        tc_puts(terminal_tid, "  target=");
        tc_puts(terminal_tid, target->name);
        tc_puts(terminal_tid, "\r\n  speed=");
        tc_put_uint(terminal_tid, (unsigned int)speed);
        tc_puts(terminal_tid, "\r\n  predicted_ticks=");
        tc_put_uint(terminal_tid, (unsigned int)predicted_ticks);
        tc_puts(terminal_tid, "\r\n  actual_ticks=");
        tc_put_uint(terminal_tid, (unsigned int)actual_ticks);
        tc_puts(terminal_tid, "\r\n  error_ticks=");
        tc_put_int(terminal_tid, error_ticks);
        tc_puts(terminal_tid, "\r\n  approx_error_mm=");
        tc_put_int(terminal_tid, error_mm);
        tc_puts(terminal_tid, "\r\n");

        return 0;
}

static int run_stop_plan(int terminal_tid, int can_tid, const char *target_name, int train, int speed) {
        tc_target_t *target;
        int run_ticks;

        if (train < 1 || train > 255 || speed < 1 || speed > TC_MAX_USER_SPEED) {
                tc_puts(terminal_tid, "\r\ninvalid train or speed\r\n");
                return -1;
        }

        target = find_target(target_name);
        if (!target) {
                tc_puts(terminal_tid, "\r\nunknown target: ");
                tc_puts(terminal_tid, target_name);
                tc_puts(terminal_tid, "\r\n");
                print_targets(terminal_tid);
                return -1;
        }

        if (apply_route(terminal_tid, can_tid, target_name) < 0) {
                tc_puts(terminal_tid,
                        "route application failed; train remains stopped\r\n");
                return -1;
        }

        run_ticks = predict_run_ticks(terminal_tid, target, speed);
        if (run_ticks < 0) {
                return -1;
        }

        if (run_ticks < 20) {
                run_ticks = 20;
        }

        /*
         * Speed-120 demo calibration from fixed start.
         * A5 is already correct. D10 was observed stopping too early,
         * so extend only D10 first.
         */
        if (speed == 120) {
                if (tc_streq(target->name, "A5")) {
                        run_ticks = 19;
                } else if (tc_streq(target->name, "D10")) {
                        run_ticks = 28;
                } else if (tc_streq(target->name, "B7")) {
                        run_ticks = 28;
                } else if (tc_streq(target->name, "C8")) {
                        run_ticks = 37;
                } else if (tc_streq(target->name, "LOOP")) {
                        run_ticks = 32;
                }
        }

        state.train = train;
        state.speed = speed;
        state.last_target = target->name;
        state.last_run_ticks = run_ticks;
        state.last_stop_distance_mm = tc_stop_distance_for_speed(speed);
        state.last_prediction_error_ticks = 0;

        tc_puts(terminal_tid, "\r\nStop plan:\r\n");
        tc_puts(terminal_tid, "  target=");
        tc_puts(terminal_tid, target->name);
        tc_puts(terminal_tid, "\r\n  distance_mm=");
        tc_put_uint(terminal_tid, (unsigned int)target->distance_mm);
        tc_puts(terminal_tid, "\r\n  speed=");
        tc_put_uint(terminal_tid, (unsigned int)speed);
        tc_puts(terminal_tid, "\r\n  route_switches=");
        print_route_switches(terminal_tid, target->name);
        tc_puts(terminal_tid, "\r\n  velocity_mm_per_tick=");
        tc_put_uint(terminal_tid,
                    (unsigned int)tc_velocity_for_speed(speed));
        tc_puts(terminal_tid, "\r\n  stop_distance_mm=");
        tc_put_uint(terminal_tid,
                    (unsigned int)tc_stop_distance_for_speed(speed));
        tc_puts(terminal_tid, "\r\n  predicted_run_ticks=");
        tc_put_uint(terminal_tid, (unsigned int)run_ticks);
        if (speed == 120) {
                tc_puts(terminal_tid, "\r\n  speed120 calibrated_run_ticks=");
                tc_put_uint(terminal_tid, (unsigned int)run_ticks);
        }
        tc_puts(terminal_tid, "\r\n");

        tc_puts(terminal_tid, "Starting train...\r\n");
        if (CanTrainSetSpeed(can_tid, train, speed) < 0) {
                tc_puts(terminal_tid, "start command failed\r\n");
                return -1;
        }

        tc_busy_wait_ticks(run_ticks);

        tc_puts(terminal_tid, "Stopping train...\r\n");
        if (CanTrainSetSpeed(can_tid, train, 0) < 0) {
                tc_puts(terminal_tid, "stop command failed\r\n");
                return -1;
        }

        state.speed = 0;
        tc_puts(terminal_tid, "stop complete\r\n");
        tc_puts(terminal_tid, "Stop command sent. Returning to prompt.\r\n");

        return 0;
}

static void run_command(int terminal_tid, int can_tid, char *line) {
#ifdef MODE_TC2
        (void)can_tid;
        /*
         * The legacy implementation remains available to the TC1 build,
         * but no TC2 command path may call its busy-waiting motion plan.
         */
        (void)run_stop_plan;
#endif
        char *p = line;
        char target[16];
        int a;
        int b;
        int c;

        skip_spaces(&p);

        if (*p == 0) {
                return;
        }

#ifdef MODE_TC2
        /*
         * GTKTerm can inherit complete xHCI diagnostics from UART0 when it
         * attaches.  They are boot output, not operator commands, so consume
         * them without producing a repeating "unknown command" message.
         */
        if (tc2_boot_diagnostic_line(p)) {
                return;
        }
        if (tc2_demo_state.active &&
            !command_no_args(p, "returnbackstop") &&
            !command_no_args(p, "help")) {
                tc2_status_set(
                        "returnback active; use returnbackstop before another command");
                tc_puts(terminal_tid,
                        "\r\nreturnback active: fixed script owns its three trains; use returnbackstop first\r\n");
                return;
        }
        if (tc2_handle_offline_command(p, 1, terminal_tid)) {
                return;
        }
#endif

        if (command_no_args(p, "help")) {
                print_help(terminal_tid);
                return;
        }

        if (command_no_args(p, "cal")) {
                print_calibration(terminal_tid);
                return;
        }

        if (command_no_args(p, "targets")) {
                print_targets(terminal_tid);
                return;
        }

        if (command_with_args(p, "plan")) {
                p += 4;
                if (parse_word(&p, target, sizeof(target)) < 0 ||
                    parse_uint(&p, &a) < 0 ||
                    !command_args_complete(p)) {
                        tc_puts(terminal_tid, "\r\ninvalid plan command\r\n");
                        return;
                }
                preview_stop_plan(terminal_tid, target, a);
                return;
        }

        if (command_with_args(p, "obs")) {
                p += 3;
                if (parse_word(&p, target, sizeof(target)) < 0 ||
                    parse_uint(&p, &a) < 0 ||
                    parse_uint(&p, &b) < 0 ||
                    !command_args_complete(p)) {
                        tc_puts(terminal_tid, "\r\ninvalid obs command\r\n");
                        return;
                }
                compare_observation(terminal_tid, target, a, b);
                return;
        }

        if (command_no_args(p, "status")) {
                print_status(terminal_tid);
                return;
        }

#ifdef MODE_TC2
        if (command_token(p, "reroute")) {
                char destination[8];
                p += 7;
                if (parse_uint(&p, &a) < 0 ||
                    parse_uint(&p, &b) < 0 ||
                    parse_word(&p, destination,
                               sizeof(destination)) < 0 ||
                    !no_more_args(p)) {
                        tc_puts(terminal_tid,
                                "\r\nusage: reroute <train 14|15|17|18> <speed 1..120> <d1-d8>\r\n");
                        return;
                }
                int destination_index =
                        Tc2DispatchParseDestination(destination);
                int stage_status = -1;
                if (tc_dispatch_server_tid >= 0 &&
                    (a == 14 || a == 15 || a == 17 || a == 18) &&
                    b >= 1 && b <= TC_MAX_USER_SPEED &&
                    destination_index >= 0) {
                        stage_status =
                                Tc2DispatchStageReroute(
                                        tc_dispatch_server_tid,
                                        a, b,
                                        destination_index);
                }
                if (stage_status < 0) {
                        tc2_status_set(
                                "reroute rejected; train is absent, unsupported, or pending physical removal");
                        tc_puts(terminal_tid,
                                "\r\nreroute rejected: train is absent, unsupported, or pending physical removal\r\n");
                        return;
                }
                tc2_demo_state.active = 0;
                tc2_status_set(
                        "reroute staged; enter go to stop safely and replan from CURRENT");
                tc_puts(terminal_tid,
                        "\r\nreroute staged: train=");
                tc_put_uint(terminal_tid, (unsigned int)a);
                tc_puts(terminal_tid, " from=CURRENT speed=");
                tc_put_uint(terminal_tid, (unsigned int)b);
                tc_puts(terminal_tid, " destination=");
                tc_puts(terminal_tid,
                        Tc2DispatchDestinationLabel(
                                destination_index));
                tc_puts(terminal_tid, "\r\n");
                return;
        }

        if (command_token(p, "caldispatch")) {
                char start[8];
                char destination[8];
                p += 11;
                if (parse_uint(&p, &a) < 0 ||
                    parse_word(&p, start, sizeof(start)) < 0 ||
                    parse_uint(&p, &b) < 0 ||
                    parse_word(&p, destination, sizeof(destination)) < 0 ||
                    !no_more_args(p)) {
                        tc_puts(terminal_tid,
                                "\r\nusage: caldispatch 14 A <speed 1..120> d7\r\n");
                        return;
                }
                int start_index = Tc2DispatchParseStart(start);
                int destination_index =
                        Tc2DispatchParseDestination(destination);
                int stage_status = -1;
                if (tc_dispatch_server_tid >= 0 &&
                    a == 14 && start_index == 0 &&
                    b >= 1 && b <= TC_MAX_USER_SPEED &&
                    destination_index == 6) {
                        stage_status =
                                Tc2DispatchStageCalibration(
                                        tc_dispatch_server_tid,
                                        a, start_index, b,
                                        destination_index);
                }
                if (stage_status < 0) {
                        tc_puts(terminal_tid,
                                "\r\ncaldispatch rejected: require exactly T14, A, speed 1..120, d7, healthy scheduler, and no other managed job\r\n");
                        return;
                }
                tc_puts(terminal_tid,
                        "\r\ncaldispatch staged: PROVISIONAL train=14 from=A speed=");
                tc_put_uint(terminal_tid, (unsigned int)b);
                tc_puts(terminal_tid, " destination=d7 velocity=");
                tc_put_uint(
                        terminal_tid,
                        (unsigned int)
                                Tc2MotionProvisionalVelocityUmPerTick(
                                        b));
                tc_puts(terminal_tid, "um/10ms stop_distance=");
                tc_put_uint(
                        terminal_tid,
                        (unsigned int)
                                Tc2MotionProvisionalStopDistanceMm(
                                        b));
                tc_puts(terminal_tid,
                        "mm; use go, then inspect trips\r\n");
                return;
        }

        if (command_token(p, "dispatch")) {
                char start[8];
                char destination[8];
                p += 8;
                if (parse_uint(&p, &a) < 0 ||
                    parse_word(&p, start, sizeof(start)) < 0 ||
                    parse_uint(&p, &b) < 0 ||
                    parse_word(&p, destination, sizeof(destination)) < 0 ||
                    !no_more_args(p)) {
                        tc_puts(terminal_tid,
                                "\r\nusage: dispatch <train 14|15|17|18> <A-F|CURRENT> <speed> <d1-d8>\r\n");
                        return;
                }
                int start_index = Tc2DispatchParseStart(start);
                int destination_index =
                        Tc2DispatchParseDestination(destination);
                int stage_status = -1;
                if (tc_dispatch_server_tid >= 0 &&
                    (a == 14 || a == 15 || a == 17 || a == 18) &&
                    b >= 1 && b <= TC_MAX_USER_SPEED &&
                    start_index != TC2_DISPATCH_START_INVALID &&
                    destination_index >= 0) {
                        stage_status =
                                start_index == TC2_DISPATCH_START_CURRENT ?
                                Tc2DispatchStageFromCurrent(
                                        tc_dispatch_server_tid, a, b,
                                        destination_index) :
                                Tc2DispatchStage(
                                        tc_dispatch_server_tid, a,
                                        start_index, b, destination_index);
                }
                if (stage_status < 0) {
                        tc2_status_set(
                                "dispatch rejected; check train/start/speed/destination and scheduler");
                        tc_puts(terminal_tid,
                                "\r\ndispatch rejected: require train 14|15|17|18, A-F|CURRENT, speed 1..120, d1-d8, healthy scheduler, and available placement\r\n");
                        return;
                }
                tc2_demo_state.active = 0;
                tc2_status_set(
                        "dispatch staged; enter go to start the train");
                tc_puts(terminal_tid, "\r\ndispatch staged: train=");
                tc_put_uint(terminal_tid, (unsigned int)a);
                tc_puts(terminal_tid, " from=");
                tc_puts(terminal_tid,
                        start_index == TC2_DISPATCH_START_CURRENT ?
                        "CURRENT" : Tc2DispatchStartLabel(start_index));
                tc_puts(terminal_tid, " speed=");
                tc_put_uint(terminal_tid, (unsigned int)b);
                tc_puts(terminal_tid, " destination=");
                tc_puts(terminal_tid,
                        Tc2DispatchDestinationLabel(destination_index));
                tc_puts(terminal_tid, "\r\n");
                return;
        }

        if (command_only(p, "go")) {
                if (tc_dispatch_server_tid < 0 ||
                    Tc2DispatchStartAll(tc_dispatch_server_tid) < 0) {
                        tc2_status_set(
                                "go failed; no staged dispatch job");
                        tc_puts(terminal_tid,
                                "\r\ngo failed: no staged dispatch jobs\r\n");
                        return;
                }
                tc_puts(terminal_tid,
                        "\r\ngo submitted: inspect READY/RUNNING/WAIT_ROUTE with trips\r\n");
                print_dispatch_jobs(terminal_tid);
                {
                        int live_tick = Time();
                        Tc2LiveUiInitialize(
                                &tc2_live_ui_state,
                                live_tick < 0 ? 0u :
                                (uint32_t)live_tick);
                        tc2_ui_live = 1;
                        tc2_ui_active = 1;
                        tc2_ui_shutdown_requested = 0;
                        tc2_ui_shutdown_frame_queued = 0;
                        tc2_ui_next_render_tick = 0;
                        tc2_status_set("");
                }
                return;
        }

        if (command_only(p, "trips")) {
                print_dispatch_jobs(terminal_tid);
                return;
        }

                if (command_token(p, "cancel")) {
                p += 6;
                if (parse_uint(&p, &a) < 0 || !no_more_args(p) ||
                    a < 1 || a > 255 ||
                    tc_dispatch_server_tid < 0 ||
                    Tc2DispatchCancel(tc_dispatch_server_tid, a) < 0) {
                        tc2_status_set("cancel failed");
                        tc_puts(terminal_tid, "\r\ncancel failed\r\n");
                        return;
                }
                tc2_demo_state.active = 0;
                tc2_status_set(
                        "cancel accepted; wait for stop, then remove");
                tc_puts(terminal_tid, "\r\ncancel ok train=");
                tc_put_uint(terminal_tid, (unsigned int)a);
                tc_puts(terminal_tid,
                        " braking; wait for STOP_HOLD before remove\r\n");
                return;
        }

                if (command_token(p, "remove")) {
                p += 6;
                if (parse_uint(&p, &a) < 0 || !no_more_args(p) ||
                    a < 1 || a > 255 ||
                    tc_dispatch_server_tid < 0 ||
                    Tc2DispatchRemove(tc_dispatch_server_tid, a) < 0) {
                        tc2_status_set("remove failed");
                        tc_puts(terminal_tid, "\r\nremove failed\r\n");
                        return;
                }
                tc2_demo_state.active = 0;
                tc2_status_set(
                        "remove accepted; train may be dispatched again");
                tc_puts(terminal_tid, "\r\nremove ok train=");
                tc_put_uint(terminal_tid, (unsigned int)a);
                tc_puts(terminal_tid,
                        " reservation released (after cancel + operator removal)\r\n");
                return;
        }

        if (command_only(p, "blocks")) {
                print_blocked_nodes(terminal_tid);
                return;
        }

        if (command_token(p, "block")) {
                p += 5;
                if (parse_word(&p, target, sizeof(target)) < 0 ||
                    !no_more_args(p)) {
                        tc_puts(terminal_tid,
                                "\r\nusage: block <track-node>\r\n");
                        return;
                }
                uppercase_word(target);
                int node = TrackFindNodeByName(tc2_track, target);
                if (node < 0 || tc_dispatch_server_tid < 0 ||
                    Tc2DispatchBlockNode(tc_dispatch_server_tid, node) < 0) {
                        tc_puts(terminal_tid,
                                "\r\nblock failed: unknown node or scheduler unavailable\r\n");
                        return;
                }
                tc_puts(terminal_tid, "\r\nblocked physical location ");
                tc_puts(terminal_tid, tc_track_name_or(node, target));
                tc_puts(terminal_tid, "\r\n");
                return;
        }

        if (command_token(p, "unblock")) {
                p += 7;
                if (parse_word(&p, target, sizeof(target)) < 0 ||
                    !no_more_args(p)) {
                        tc_puts(terminal_tid,
                                "\r\nusage: unblock <track-node>\r\n");
                        return;
                }
                uppercase_word(target);
                int node = TrackFindNodeByName(tc2_track, target);
                if (node < 0 || tc_dispatch_server_tid < 0 ||
                    Tc2DispatchUnblockNode(tc_dispatch_server_tid, node) < 0) {
                        tc_puts(terminal_tid,
                                "\r\nunblock failed: unknown node or scheduler unavailable\r\n");
                        return;
                }
                tc_puts(terminal_tid, "\r\nunblocked physical location ");
                tc_puts(terminal_tid, tc_track_name_or(node, target));
                tc_puts(terminal_tid, "\r\n");
                return;
        }

        if (command_only(p, "positions")) {
                print_position_candidates(terminal_tid);
                return;
        }

        if (command_only(p, "trackview")) {
                print_track_view(terminal_tid);
                return;
        }

        if (command_only(p, "reservations")) {
                print_reservations(terminal_tid);
                return;
        }

        if (command_token(p, "reserve")) {
                char from[16];

                p += 7;
                if (parse_uint(&p, &a) < 0 ||
                    parse_word(&p, from, sizeof(from)) < 0 ||
                    parse_word(&p, target, sizeof(target)) < 0 ||
                    !no_more_args(p) || a < 1 || a > 255) {
                        tc_puts(terminal_tid,
                                "\r\nusage: reserve <train 1..255> <from-node> <to-node>\r\n");
                        return;
                }
                (void)from;
                print_manual_control_blocked(terminal_tid);
                return;
        }

        if (command_token(p, "release")) {
                p += 7;
                if (parse_uint(&p, &a) < 0 || !no_more_args(p) ||
                    a < 1 || a > 255) {
                        tc_puts(terminal_tid,
                                "\r\nusage: release <train 1..255>\r\n");
                        return;
                }
                print_manual_control_blocked(terminal_tid);
                return;
        }

        if (command_token(p, "sensorroute")) {
                p += 11;
                if (parse_word(&p, target, sizeof(target)) < 0 ||
                    !no_more_args(p)) {
                        tc_puts(terminal_tid,
                                "\r\nusage: sensorroute <track-node>\r\n");
                        return;
                }
                preview_sensor_route(terminal_tid, target);
                return;
        }

        if (command_only(p, "sensors")) {
                print_sensor_status(terminal_tid);
                return;
        }
#endif

        if (command_no_args(p, "returnbackstop")) {
#ifdef MODE_TC2
                int accepted = tc2_stop_physical_demo();
                tc_puts(terminal_tid,
                        "\r\nreturnbackstop requested; emergency-stop batch accepted for ");
                tc_put_uint(terminal_tid, (unsigned int)accepted);
                tc_puts(terminal_tid,
                        accepted > 0 ?
                        " scripted train(s); returnback UI state cleared\r\n" :
                        " scripted train(s); stop failed, returnback UI state preserved\r\n");
#else
                tc_puts(terminal_tid, "\r\nreturnbackstop is TC2-only\r\n");
#endif
                return;
        }

        if (command_token(p, "returnback")) {
#ifdef MODE_TC2
                char first[16];
                char second[16];
                char third[16];
                int now = Time();
                int profile_id = -1;
                p += 10;
                if (parse_word(&p, first, sizeof(first)) < 0 ||
                    parse_word(&p, second, sizeof(second)) < 0 ||
                    parse_word(&p, third, sizeof(third)) < 0 ||
                    !no_more_args(p)) {
                        tc_puts(terminal_tid,
                                "\r\nusage: returnback <intersection/destination> <intersection/destination> <intersection/destination> (FAKE fixed physical script; place T14@A, T15@C, T17@F)\r\n");
                        return;
                }
                if (tc_streq(first, "6") &&
                    tc_streq(second, "18") &&
                    tc_streq(third, "15")) {
                        profile_id = TC2_DEMO_PROFILE_INTERSECTIONS;
                } else if (tc_streq(first, "D4") &&
                           tc_streq(second, "D3") &&
                           tc_streq(third, "D1")) {
                        profile_id = TC2_DEMO_PROFILE_DESTINATIONS;
                }
                if (profile_id < 0) {
                        tc_puts(terminal_tid,
                                "\r\nreturnback rejected: unsupported fixed script arguments\r\n");
                        return;
                }
                if (tc2_start_physical_demo(
                            now < 0 ? 0u : (uint32_t)now,
                            14, 15, 17, profile_id) < 0) {
                        tc2_status_set(
                                "returnback rejected: scheduler, CAN, and sensors must be healthy with no managed jobs");
                        tc_puts(terminal_tid,
                                "\r\nreturnback rejected: place T14@A, T15@C, T17@F; scheduler/CAN/sensors must be healthy and trips must be empty\r\n");
                } else {
                        if (profile_id ==
                            TC2_DEMO_PROFILE_DESTINATIONS) {
                                tc_puts(terminal_tid,
                                        "\r\nreturnback FAKE demo started: T14 A->d4->A, T15 C->d3->C, T17 F->d1->F; speed=80; use returnbackstop to brake and clear UI\r\n");
                        } else {
                                tc_puts(terminal_tid,
                                        "\r\nreturnback FAKE demo started: T14@A -> junction 6 script, T15@C -> junction 18 script, T17@F -> junction 15 script; speed=60; use returnbackstop to brake and clear UI\r\n");
                        }
                }
#else
                tc_puts(terminal_tid, "\r\nreturnback is TC2-only\r\n");
#endif
                return;
        }

#ifdef MODE_TC2
        if (command_only(p, "q")) {
#else
        if (p[0] == 'q' && p[1] == 0) {
#endif
#ifdef MODE_TC2
                if (tc2_dispatch_is_active()) {
                        tc_puts(terminal_tid,
                                "\r\nq blocked: managed trains still hold protection; remove only after lifting them\r\n");
                        return;
                }
                tc_puts(terminal_tid, "\r\nTC2 command client exiting.\r\n");
#else
                tc_puts(terminal_tid, "\r\nTC1 command client exiting.\r\n");
#endif
                Exit();
        }

        if (command_with_args(p, "route")) {
                p += 5;
                if (parse_word(&p, target, sizeof(target)) < 0 ||
                    !command_args_complete(p)) {
                        tc_puts(terminal_tid, "\r\ninvalid route command\r\n");
                        return;
                }
#ifdef MODE_TC2
                print_manual_control_blocked(terminal_tid);
#else
                if (apply_route(terminal_tid, can_tid, target) == 0) {
                        state.last_target = find_target(target) ? find_target(target)->name : state.last_target;
                }
#endif
                return;
        }

        if (command_with_args(p, "stop")) {
                p += 4;
                if (parse_uint(&p, &a) < 0 ||
                    parse_word(&p, target, sizeof(target)) < 0 ||
                    parse_uint(&p, &b) < 0 ||
                    !command_args_complete(p)) {
                        tc_puts(terminal_tid, "\r\ninvalid stop command\r\n");
                        return;
                }

#ifdef MODE_TC2
                print_manual_control_blocked(terminal_tid);
#else
                run_stop_plan(terminal_tid, can_tid, target, a, b);
#endif
                return;
        }

        if (command_with_args(p, "tr")) {
                p += 2;
                if (parse_uint(&p, &a) < 0 ||
                    parse_uint(&p, &b) < 0 ||
                    !command_args_complete(p)
#ifdef MODE_TC2
                    || a < 1 || a > 255 || b < 0 ||
                    b > TC_MAX_USER_SPEED
#endif
                   ) {
                        tc_puts(terminal_tid, "\r\ninvalid tr command\r\n");
                        return;
                }

#ifdef MODE_TC2
                print_manual_control_blocked(terminal_tid);
#else
                if (CanTrainSetSpeed(can_tid, a, b) == 0) {
                        state.train = a;
                        state.speed = b;
                        tc_puts(terminal_tid, "\r\ntr ok train=");
                        tc_put_uint(terminal_tid, (unsigned int)a);
                        tc_puts(terminal_tid, " speed=");
                        tc_put_uint(terminal_tid, (unsigned int)b);
                        tc_puts(terminal_tid, "\r\n");
                } else {
                        tc_puts(terminal_tid, "\r\ntr failed\r\n");
                }
#endif
                return;
        }

        if (command_with_args(p, "sw")) {
                char dir;

                p += 2;
                if (parse_uint(&p, &c) < 0) {
                        tc_puts(terminal_tid, "\r\ninvalid sw command\r\n");
                        return;
                }

                skip_spaces(&p);
                dir = *p;

                if (!(dir == 'S' || dir == 's' || dir == 'C' || dir == 'c')) {
                        tc_puts(terminal_tid, "\r\ninvalid sw direction\r\n");
                        return;
                }
                ++p;
                if (!command_args_complete(p)) {
                        tc_puts(terminal_tid, "\r\ninvalid sw command\r\n");
                        return;
                }

#ifdef MODE_TC2
                print_manual_control_blocked(terminal_tid);
#else
                if (CanSwitch(can_tid, c, dir) == 0) {
                        tc_puts(terminal_tid, "\r\nsw ok switch=");
                        tc_put_uint(terminal_tid, (unsigned int)c);
                        tc_puts(terminal_tid, " dir=");
                        tc_put_char(terminal_tid, (unsigned char)dir);
                        tc_puts(terminal_tid, "\r\n");
                } else {
                        tc_puts(terminal_tid, "\r\nsw failed\r\n");
                }
#endif
                return;
        }

        tc_puts(terminal_tid, "\r\nunknown command\r\n");
}

#ifdef MODE_TC2
static void tc2_execute_live_dashboard_command(
        int terminal_tid, int can_tid, char *command) {
        int leave_dashboard =
                command_only(command, "uioff") ||
                command_only(command, "q");
        int preserve_text_output =
                command_only(command, "help") ||
                command_token(command, "returnback") ||
                command_only(command, "returnbackstop") ||
                command_only(command, "cal") ||
                command_only(command, "targets") ||
                command_only(command, "status") ||
                command_only(command, "trips") ||
                command_only(command, "blocks") ||
                command_token(command, "plan") ||
                command_token(command, "obs");
        /*
         * Dashboard rendering is presentation only. Execute every command
         * through the ordinary parser. Motion commands redraw the map;
         * verbose inspection commands stay in text mode until "uion", so
         * their output cannot be overwritten by a queued dashboard frame.
         */
        tc2_ui_active = 0;
        tc2_ui_shutdown_requested = 0;
        tc2_ui_shutdown_frame_queued = 0;
        tc2_ui_next_render_tick = 0;
        if (tc2_ui_live) {
                Tc2UiTxPumpInit(&tc2_live_ui_state.pump);
                Tc2UiRendererInvalidate(
                        &tc2_live_ui_state.renderer);
        } else {
                Tc2UiTxPumpInit(
                        &tc2_offline_controller_state.pump);
                Tc2UiRendererInvalidate(
                        &tc2_offline_controller_state.renderer);
        }
        tc_puts(
                terminal_tid,
                "\033[0m\033[?25h\033[H\033[2J\033[3J\033[H");
        run_command(terminal_tid, can_tid, command);
        if (!leave_dashboard && !preserve_text_output) {
                tc2_ui_active = 1;
                tc2_ui_live = 1;
                tc2_ui_shutdown_requested = 0;
                tc2_ui_shutdown_frame_queued = 0;
                tc2_ui_next_render_tick = 0;
                Tc2UiRendererInvalidate(
                        &tc2_live_ui_state.renderer);
        } else if (!tc2_ui_active) {
                if (leave_dashboard) tc2_ui_live = 0;
                tc_puts(
                        terminal_tid,
                        preserve_text_output ?
                        "\r\n> " :
                        "> ");
        }
}
#endif

static void automatic_validation(int terminal_tid) {
#ifdef MODE_TC2
        train_sensor_snapshot_t sensor_snapshot;
        track_reservation_snapshot reservation_snapshot;
        tc2_dispatch_snapshot dispatch_snapshot;
        can_health_t can_health;
        int graph_ready = TrackFindNodeByName(tc2_track, "A1") == 0 &&
                TrackFindNodeByName(tc2_track, "E16") == 79;
        int services_ready = tc_sensor_server_tid >= 0 &&
                tc_reservation_server_tid >= 0 &&
                tc_dispatch_server_tid >= 0 &&
                tc_can_server_tid >= 0 &&
                TrainSensorGetLatest(tc_sensor_server_tid,
                                     &sensor_snapshot) == 0 &&
                sensor_snapshot.sensor_server_registered &&
                sensor_snapshot.courier_created &&
                sensor_snapshot.courier_ready &&
                TrackReservationServerSnapshot(tc_reservation_server_tid,
                                               &reservation_snapshot) == 0 &&
                Tc2DispatchGetSnapshot(tc_dispatch_server_tid,
                                       &dispatch_snapshot) == 0 &&
                dispatch_snapshot.scheduler_healthy &&
                CanGetHealth(tc_can_server_tid, &can_health) == 0 &&
                can_health.hw_ready;

        // Startup check执行real IPC和graph lookup，不能用unconditional success糊弄demo。
        if (graph_ready && services_ready) {
                tc_puts(terminal_tid,
                        "\r\nautomatic_validation: TC2 graph and services ready\r\n");
        } else {
                tc_puts(terminal_tid,
                        "\r\nautomatic_validation: TC2 startup check FAILED\r\n");
        }
#else
        tc_puts(terminal_tid, "\r\nautomatic_validation: success\r\n");
#endif
}

void TrainControlTask(void) {
        int terminal_tid;
        int can_tid;
        char line[TC_MAX_LINE];
        int len = 0;

        terminal_tid = WhoIs(TERMINAL_SERVER_NAME);
        can_tid = WhoIs(CAN_SERVER_NAME);
#ifdef MODE_TC2
        tc_sensor_server_tid = WhoIs(TRAIN_SENSOR_SERVER_NAME);
        tc_reservation_server_tid = WhoIs(TRACK_RESERVATION_SERVER_NAME);
        tc_dispatch_server_tid = WhoIs(TC2_DISPATCH_SERVER_NAME);
        tc_can_server_tid = can_tid;
        // Track D复用official Track B topology，UI startup只初始化一次graph。
        init_trackb(tc2_track);
        {
                int initial_tick = Time();
                int controller_status = Tc2OfflineControllerInitialize(
                        &tc2_offline_controller_state,
                        initial_tick < 0 ? 0u : (uint32_t)initial_tick);
                tc2_offline_controller_ready =
                        controller_status == TC2_OFFLINE_CONTROLLER_OK;
                tc2_ui_active = 0;
                tc2_ui_live = 0;
                tc2_ui_shutdown_requested = 0;
                tc2_ui_shutdown_frame_queued = 0;
                tc2_ui_next_render_tick = 0;
                tc2_demo_reset_state();
                tc2_status_set(
                        tc2_offline_controller_ready ?
                        "offline simulator ready; stage A-F to d1-d8 trips" :
                        "offline simulator initialization FAILED");
        }
#endif

#ifdef MODE_TC2
        // 启动画面明确标记TC2，避免把sensor-enabled image误认为旧TC1 build。
        tc_puts(terminal_tid, "\r\nTC2 Train Control Application\r\n");
        tc_puts(terminal_tid,
                "Goal: batch-dispatch multiple trains by validated lowest-cost routes, "
                "wait on conflicts, and stop at d1-d8.\r\n");
#else
        tc_puts(terminal_tid, "\r\nTC1 Train Control Application\r\n");
        tc_puts(terminal_tid, "Goal: route one train and stop it at a chosen location.\r\n");
#endif

        automatic_validation(terminal_tid);

#ifdef MODE_TC2
        tc_puts(terminal_tid,
                "\r\nType `help`; live commands use dispatch/go, prediction-only "
                "commands use simdispatch/simgo.\r\n> ");
#else
        tc_puts(terminal_tid,
                "\r\nType `help` for commands.\r\n> ");
#endif

#ifdef MODE_TC2
        for (;;) {
                int raw_time = Time();
                uint32_t now_tick =
                        raw_time < 0 ? 0u : (uint32_t)raw_time;
                int step_status = TC2_OFFLINE_CONTROLLER_NOT_INITIALIZED;
                int ch;

                if (tc2_offline_controller_ready) {
                        step_status = Tc2OfflineControllerStep(
                                &tc2_offline_controller_state,
                                now_tick);
                        if (step_status < 0) {
                                tc2_status_set(
                                        "offline controller failed closed; use simreset");
                        }
                }
                tc2_step_physical_demo(now_tick);
                if (tc2_ui_active && tc2_ui_live &&
                    !tc2_demo_state.active &&
                    tc2_sync_live_ui(now_tick) < 0) {
                        tc2_status_set(
                                "live UI snapshot unavailable; dispatcher safety remains active");
                }

                ch = TryGetc(terminal_tid);

                if (tc2_ui_active) {
                        if (ch >= 0) {
                                if (ch == '\r' || ch == '\n') {
                                        line[len] = 0;
                                        if (len > 0) {
                                                char *command = line;
                                                skip_spaces(&command);
                                                if (tc2_ui_live) {
                                                        tc2_execute_live_dashboard_command(
                                                                terminal_tid,
                                                                can_tid,
                                                                command);
                                                } else if (!tc2_handle_offline_command(
                                                                   command, 0,
                                                                   terminal_tid)) {
                                                        if (command_only(
                                                                    command,
                                                                    "help")) {
                                                                tc2_status_set(
                                                                        "UI mode: returnback/simdispatch/simgo/simtrips/simremove/simreset/uioff");
                                                        } else {
                                                                tc2_status_set(
                                                                        "live command rejected while UI is active; use uioff first");
                                                        }
                                                }
                                        }
                                        len = 0;
                                        tc2_ui_next_render_tick = 0;
                                } else if (ch == 8 || ch == 127) {
                                        if (len > 0) len--;
                                        tc2_ui_next_render_tick = 0;
                                } else if (len < TC_MAX_LINE - 1) {
                                        line[len++] = (char)ch;
                                        tc2_ui_next_render_tick = 0;
                                } else {
                                        len = 0;
                                        tc2_status_set(
                                                "UI command line too long; input cleared");
                                        tc2_ui_next_render_tick = 0;
                                }
                        } else if (ch == TERMINAL_TRY_GETC_ERROR) {
                                tc2_status_set(
                                        "terminal input poll failed; simulation continues");
                        }

                        if (!tc2_ui_active) {
                                len = 0;
                                Yield();
                                continue;
                        }

                        /*
                         * One bounded, all-or-none chunk per scheduler turn.
                         * Input, prediction, and other tasks therefore keep
                         * making progress even while a full frame drains.
                         */
                        if (tc2_ui_live ?
                            Tc2LiveUiHasPendingOutput(
                                    &tc2_live_ui_state) :
                            Tc2OfflineControllerHasPendingOutput(
                                    &tc2_offline_controller_state)) {
                                tc2_ui_tx_drain_result drain_result;
                                int drain_status = tc2_ui_live ?
                                        Tc2LiveUiDrain(
                                                &tc2_live_ui_state,
                                                tc2_terminal_sink,
                                                &terminal_tid, 1,
                                                &drain_result) :
                                        Tc2OfflineControllerDrain(
                                                &tc2_offline_controller_state,
                                                tc2_terminal_sink,
                                                &terminal_tid, 1,
                                                &drain_result);
                                if (drain_status < 0) {
                                        tc2_status_set(
                                                "terminal output failed; full redraw scheduled");
                                        tc2_ui_next_render_tick = 0;
                                }
                                Yield();
                                continue;
                        }

                        if (tc2_ui_shutdown_requested &&
                            tc2_ui_shutdown_frame_queued) {
                                tc2_ui_active = 0;
                                tc2_ui_live = 0;
                                tc2_ui_shutdown_requested = 0;
                                tc2_ui_shutdown_frame_queued = 0;
                                len = 0;
                                Yield();
                                continue;
                        }

                        if (tc2_ui_next_render_tick == 0 ||
                            tc2_tick_due(now_tick,
                                         tc2_ui_next_render_tick)) {
                                size_t restore_length =
                                        tc2_build_prompt_restore(line, len);
                                int render_status;
                                if (tc2_ui_live) {
                                        tc2_live_ui_render_result
                                                render_result;
                                        render_status =
                                                Tc2LiveUiRender(
                                                        &tc2_live_ui_state,
                                                        tc2_ui_restore,
                                                        restore_length,
                                                        &render_result);
                                } else {
                                        tc2_offline_controller_render_result
                                                render_result;
                                        render_status =
                                                Tc2OfflineControllerRender(
                                                        &tc2_offline_controller_state,
                                                        tc2_ui_restore,
                                                        restore_length,
                                                        &render_result);
                                }
                                if (render_status ==
                                    TC2_OFFLINE_CONTROLLER_OK) {
                                        tc2_ui_next_render_tick =
                                                now_tick + 10u;
                                        if (tc2_ui_shutdown_requested) {
                                                tc2_ui_shutdown_frame_queued =
                                                        1;
                                        }
                                } else if (render_status ==
                                           TC2_OFFLINE_CONTROLLER_BACKPRESSURE ||
                                           render_status ==
                                           TC2_OFFLINE_CONTROLLER_NEEDS_FULL_REDRAW) {
                                        tc2_ui_next_render_tick = 0;
                                } else if (render_status < 0) {
                                        tc2_status_set(
                                                "dashboard render failed; simulation remains prediction-only");
                                        tc2_ui_next_render_tick =
                                                now_tick + 10u;
                                }
                        }
                        Yield();
                        continue;
                }

                if (ch == TERMINAL_TRY_GETC_EMPTY ||
                    ch == TERMINAL_TRY_GETC_ERROR) {
                        Yield();
                        continue;
                }

                if (ch == '\r' || ch == '\n') {
                        tc_put_char(terminal_tid, '\r');
                        tc_put_char(terminal_tid, '\n');
                        line[len] = 0;
                        run_command(terminal_tid, can_tid, line);
                        len = 0;
                        if (!tc2_ui_active) {
                                tc_puts(terminal_tid, "> ");
                        }
                        continue;
                }

                if (ch == 8 || ch == 127) {
                        if (len > 0) {
                                len--;
                                tc_puts(terminal_tid, "\b \b");
                        }
                        continue;
                }

                if (len < TC_MAX_LINE - 1) {
                        line[len++] = (char)ch;
                        tc_put_char(terminal_tid, (unsigned char)ch);
                } else {
                        tc_puts(terminal_tid,
                                "\r\nline too long\r\n> ");
                        len = 0;
                }
        }
#else
        for (;;) {
                int ch = Getc(terminal_tid);

                if (ch < 0) {
                        Yield();
                        continue;
                }

                if (ch == '\r' || ch == '\n') {
                        tc_put_char(terminal_tid, '\r');
                        tc_put_char(terminal_tid, '\n');
                        line[len] = 0;
                        run_command(terminal_tid, can_tid, line);
                        len = 0;
                        tc_puts(terminal_tid, "> ");
                        continue;
                }

                if (ch == 8 || ch == 127) {
                        if (len > 0) {
                                len--;
                                tc_puts(terminal_tid, "\b \b");
                        }
                        continue;
                }

                if (len < TC_MAX_LINE - 1) {
                        line[len++] = (char)ch;
                        tc_put_char(terminal_tid, (unsigned char)ch);
                } else {
                        tc_puts(terminal_tid, "\r\nline too long\r\n> ");
                        len = 0;
                }
        }
#endif
}
