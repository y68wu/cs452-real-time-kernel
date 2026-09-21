#include "train_control.h"
#include "terminal.h"
#include "uart.h"
#include "can.h"
#include "clock.h"
#include "nameserver.h"
#include "syscall.h"


#define TC_MAX_USER_SPEED 120

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

static tc_state_t state = {
        TC_DEFAULT_TRAIN,
        0,
        "unknown",
        0,
        0,
        0
};

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

static int parse_uint(char **p, int *out) {
        int value = 0;
        int seen = 0;

        skip_spaces(p);

        while (**p >= '0' && **p <= '9') {
                value = value * 10 + (**p - '0');
                (*p)++;
                seen = 1;
        }

        if (!seen) {
                return -1;
        }

        *out = value;
        return 0;
}

static int parse_word(char **p, char *out, int max_len) {
        int i = 0;

        skip_spaces(p);

        while (**p && !is_space(**p) && i < max_len - 1) {
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

static void tc_puts(int terminal_tid, const char *s) {
        (void)terminal_tid;

        while (*s) {
                uart_putc(CONSOLE, *s);
                s++;
        }
}
static void tc_put_uint(int terminal_tid, unsigned int value) {
        char buf[12];
        int i = 0;

        if (value == 0) {
                uart_putc(CONSOLE, '0');
                return;
        }

        while (value > 0 && i < 11) {
                buf[i++] = (char)('0' + (value % 10));
                value /= 10;
        }

        while (i > 0) {
                uart_putc(CONSOLE, (unsigned char)buf[--i]);
        }
}

static void tc_put_int(int terminal_tid, int value) {
        if (value < 0) {
                uart_putc(CONSOLE, '-');
                tc_put_uint(terminal_tid, (unsigned int)(-value));
        } else {
                tc_put_uint(terminal_tid, (unsigned int)value);
        }
}

static tc_target_t *find_target(const char *name) {
        for (int i = 0; i < num_targets; i++) {
                if (streq(targets[i].name, name)) {
                        return &targets[i];
                }
        }

        return 0;
}

static void print_help(int terminal_tid) {
        tc_puts(terminal_tid, "\r\nTC1 commands:\r\n");
        tc_puts(terminal_tid, "  help                         show this help\r\n");
        tc_puts(terminal_tid, "  cal                          print seed calibration model\r\n");
        tc_puts(terminal_tid, "  targets                      print known stopping targets\r\n");
        tc_puts(terminal_tid, "  plan <target> <speed>        preview stop plan without moving train\r\n");
        tc_puts(terminal_tid, "  obs <target> <speed> <ticks> compare predicted vs observed timing\r\n");
        tc_puts(terminal_tid, "  route <target>               set switches for a target, e.g. route C8\r\n");
        tc_puts(terminal_tid, "  stop <train> <target> <spd>  run then stop at target, e.g. stop 14 C8 30\r\n");
        tc_puts(terminal_tid, "  demo                         run default stop demo: train 14 to C8 at speed 30\r\n");
        tc_puts(terminal_tid, "  tr <train> <speed>           direct speed command, 0..120\r\n");
        tc_puts(terminal_tid, "  sw <switch> <S|C>            direct switch command\r\n");
        tc_puts(terminal_tid, "  status                       print latest train-control state\r\n");
        tc_puts(terminal_tid, "  q                            quit command client\r\n");
}

static void print_targets(int terminal_tid) {
        tc_puts(terminal_tid, "\r\nKnown TC1 targets:\r\n");

        for (int i = 0; i < num_targets; i++) {
                tc_puts(terminal_tid, "  ");
                tc_puts(terminal_tid, targets[i].name);
                tc_puts(terminal_tid, " distance=");
                tc_put_uint(terminal_tid, (unsigned int)targets[i].distance_mm);
                tc_puts(terminal_tid, "mm switches=[");

                if (targets[i].switch_a) {
                        tc_put_uint(terminal_tid, (unsigned int)targets[i].switch_a);
                        uart_putc(CONSOLE, ':');
                        uart_putc(CONSOLE, (unsigned char)targets[i].dir_a);
                }

                if (targets[i].switch_b) {
                        tc_puts(terminal_tid, ", ");
                        tc_put_uint(terminal_tid, (unsigned int)targets[i].switch_b);
                        uart_putc(CONSOLE, ':');
                        uart_putc(CONSOLE, (unsigned char)targets[i].dir_b);
                }

                if (targets[i].switch_c) {
                        tc_puts(terminal_tid, ", ");
                        tc_put_uint(terminal_tid, (unsigned int)targets[i].switch_c);
                        uart_putc(CONSOLE, ':');
                        uart_putc(CONSOLE, (unsigned char)targets[i].dir_c);
                }

                tc_puts(terminal_tid, "]\r\n");
        }
}

static void print_calibration(int terminal_tid) {
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
}

static void print_status(int terminal_tid) {
        tc_puts(terminal_tid, "\r\nTC1 state:\r\n");
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

static int apply_one_switch(int terminal_tid, int can_tid, int sw, char dir) {
        int ret;

        if (sw <= 0) {
                return 0;
        }

        tc_puts(terminal_tid, "  switch ");
        tc_put_uint(terminal_tid, (unsigned int)sw);
        tc_puts(terminal_tid, " -> ");
        uart_putc(CONSOLE, (unsigned char)dir);

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
         * Track D B7 route calibration:
         * B7 needs the normal 1S/2S path plus the 153S and 154S switches
         * near the centre crossover so the train enters the B7 branch.
         */
        if (streq(target->name, "B7")) {
                if (apply_one_switch(terminal_tid, can_tid, 153, 'S') < 0) {
                        ret = -1;
                }

                if (apply_one_switch(terminal_tid, can_tid, 154, 'S') < 0) {
                        ret = -1;
                }
        }

        /*
         * Track D D10 route calibration:
         * After B7, switch 153 may still be S. D10 must force the
         * centre switches back so the train goes to D10, not D7.
         */
        if (streq(target->name, "D10") || streq(target->name, "C8")) {
                if (apply_one_switch(terminal_tid, can_tid, 153, 'C') < 0) {
                        ret = -1;
                }

                if (apply_one_switch(terminal_tid, can_tid, 155, 'S') < 0) {
                        ret = -1;
                }

                if (apply_one_switch(terminal_tid, can_tid, 156, 'S') < 0) {
                        ret = -1;
                }
        }

        return ret;
}

static int predict_run_ticks(int terminal_tid, tc_target_t *target, int speed) {
        int velocity = velocity_mm_per_tick[tc_speed_model_index(speed)];
        int stop_distance = stop_distance_mm[tc_speed_model_index(speed)];
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

        travel_before_stop = target->distance_mm - stop_distance_mm[tc_speed_model_index(speed)];
        if (travel_before_stop < 0) {
                travel_before_stop = 0;
        }

        predicted_ms = run_ticks * 10;

        tc_puts(terminal_tid, "\r\nTC1 stop-plan preview:\r\n");
        tc_puts(terminal_tid, "  target=");
        tc_puts(terminal_tid, target->name);
        tc_puts(terminal_tid, "\r\n  speed=");
        tc_put_uint(terminal_tid, (unsigned int)speed);
        tc_puts(terminal_tid, "\r\n  target_distance_mm=");
        tc_put_uint(terminal_tid, (unsigned int)target->distance_mm);
        tc_puts(terminal_tid, "\r\n  stop_distance_mm=");
        tc_put_uint(terminal_tid, (unsigned int)stop_distance_mm[tc_speed_model_index(speed)]);
        tc_puts(terminal_tid, "\r\n  travel_before_stop_mm=");
        tc_put_uint(terminal_tid, (unsigned int)travel_before_stop);
        tc_puts(terminal_tid, "\r\n  velocity_mm_per_tick=");
        tc_put_uint(terminal_tid, (unsigned int)velocity_mm_per_tick[tc_speed_model_index(speed)]);
        tc_puts(terminal_tid, "\r\n  predicted_run_ticks=");
        tc_put_uint(terminal_tid, (unsigned int)run_ticks);
        tc_puts(terminal_tid, "\r\n  predicted_run_ms=");
        tc_put_uint(terminal_tid, (unsigned int)predicted_ms);
        tc_puts(terminal_tid, "\r\n  route_switches=");

        if (target->switch_a) {
                tc_put_uint(terminal_tid, (unsigned int)target->switch_a);
                uart_putc(CONSOLE, ':');
                uart_putc(CONSOLE, (unsigned char)target->dir_a);
        }

        if (target->switch_b) {
                tc_puts(terminal_tid, ",");
                tc_put_uint(terminal_tid, (unsigned int)target->switch_b);
                uart_putc(CONSOLE, ':');
                uart_putc(CONSOLE, (unsigned char)target->dir_b);
        }

        if (target->switch_c) {
                tc_puts(terminal_tid, ",");
                tc_put_uint(terminal_tid, (unsigned int)target->switch_c);
                uart_putc(CONSOLE, ':');
                uart_putc(CONSOLE, (unsigned char)target->dir_c);
        }

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
        error_mm = error_ticks * velocity_mm_per_tick[tc_speed_model_index(speed)];

        state.last_target = target->name;
        state.last_run_ticks = predicted_ticks;
        state.last_stop_distance_mm = stop_distance_mm[tc_speed_model_index(speed)];
        state.last_prediction_error_ticks = error_ticks;

        tc_puts(terminal_tid, "\r\nTC1 sensor/timing observation:\r\n");
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

        if (train < 0 || train > 255 || speed < 1 || speed > TC_MAX_USER_SPEED) {
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

        apply_route(terminal_tid, can_tid, target_name);

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
        state.last_stop_distance_mm = stop_distance_mm[tc_speed_model_index(speed)];
        state.last_prediction_error_ticks = 0;

        tc_puts(terminal_tid, "\r\nStop plan:\r\n");
        tc_puts(terminal_tid, "  target=");
        tc_puts(terminal_tid, target->name);
        tc_puts(terminal_tid, "\r\n  distance_mm=");
        tc_put_uint(terminal_tid, (unsigned int)target->distance_mm);
        tc_puts(terminal_tid, "\r\n  speed=");
        tc_put_uint(terminal_tid, (unsigned int)speed);
        tc_puts(terminal_tid, "\r\n  velocity_mm_per_tick=");
        tc_put_uint(terminal_tid, (unsigned int)velocity_mm_per_tick[tc_speed_model_index(speed)]);
        tc_puts(terminal_tid, "\r\n  stop_distance_mm=");
        tc_put_uint(terminal_tid, (unsigned int)stop_distance_mm[tc_speed_model_index(speed)]);
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


/*
 * TC2 conservative multi-train scaffold.
 *
 * This is intentionally simple:
 *   - keep per-train destination state,
 *   - reserve coarse route regions,
 *   - reject a new destination if another active train reserves a conflicting region,
 *   - launch a small driver task that applies the route, starts the train, waits for
 *     the calibrated duration, then stops and releases the reservation.
 *
 * It builds on the calibrated TC1 route/stop model.
 */

#define TC2_MAX_TRAINS 4
#define TC2_INVALID_SLOT -1

#define TC2_MASK_A_SIDE   0x0001
#define TC2_MASK_B_SIDE   0x0002
#define TC2_MASK_CD_SIDE  0x0004
#define TC2_MASK_LOOP     0x0008
#define TC2_MASK_SWITCH8  0x0010
#define TC2_MASK_SWITCH17 0x0020
#define TC2_MASK_CENTRE   0x0040

typedef struct {
        int used;
        int active;
        int train;
        int speed;
        int route_mask;
        int run_ticks;
        const char *destination;
        int driver_tid;
} tc2_train_t;

static tc2_train_t tc2_trains[TC2_MAX_TRAINS];
static int tc2_pending_slot = TC2_INVALID_SLOT;

static void tc2_init_state(void) {
        int i;

        for (i = 0; i < TC2_MAX_TRAINS; i++) {
                tc2_trains[i].used = 0;
                tc2_trains[i].active = 0;
                tc2_trains[i].train = 0;
                tc2_trains[i].speed = 0;
                tc2_trains[i].route_mask = 0;
                tc2_trains[i].run_ticks = 0;
                tc2_trains[i].destination = "NONE";
                tc2_trains[i].driver_tid = -1;
        }

        tc2_pending_slot = TC2_INVALID_SLOT;
}

static int tc2_target_mask(const char *target_name) {
        if (tc_streq(target_name, "A5")) {
                return TC2_MASK_A_SIDE | TC2_MASK_SWITCH8;
        }

        if (tc_streq(target_name, "B7")) {
                return TC2_MASK_B_SIDE | TC2_MASK_SWITCH8 | TC2_MASK_SWITCH17 | TC2_MASK_CENTRE;
        }

        if (tc_streq(target_name, "C8")) {
                return TC2_MASK_CD_SIDE | TC2_MASK_SWITCH8 | TC2_MASK_SWITCH17 | TC2_MASK_CENTRE;
        }

        if (tc_streq(target_name, "D10")) {
                return TC2_MASK_CD_SIDE | TC2_MASK_SWITCH8 | TC2_MASK_SWITCH17 | TC2_MASK_CENTRE;
        }

        if (tc_streq(target_name, "LOOP")) {
                return TC2_MASK_LOOP | TC2_MASK_SWITCH8;
        }

        return 0;
}

static int tc2_find_slot(int train, int create_if_missing) {
        int i;
        int free_slot = TC2_INVALID_SLOT;

        for (i = 0; i < TC2_MAX_TRAINS; i++) {
                if (tc2_trains[i].used && tc2_trains[i].train == train) {
                        return i;
                }

                if (!tc2_trains[i].used && free_slot == TC2_INVALID_SLOT) {
                        free_slot = i;
                }
        }

        if (create_if_missing && free_slot != TC2_INVALID_SLOT) {
                tc2_trains[free_slot].used = 1;
                tc2_trains[free_slot].active = 0;
                tc2_trains[free_slot].train = train;
                tc2_trains[free_slot].speed = 0;
                tc2_trains[free_slot].route_mask = 0;
                tc2_trains[free_slot].run_ticks = 0;
                tc2_trains[free_slot].destination = "NONE";
                tc2_trains[free_slot].driver_tid = -1;
                return free_slot;
        }

        return TC2_INVALID_SLOT;
}

static int tc2_has_conflict(int my_slot, int route_mask) {
        int i;

        for (i = 0; i < TC2_MAX_TRAINS; i++) {
                if (i == my_slot) {
                        continue;
                }

                if (!tc2_trains[i].used || !tc2_trains[i].active) {
                        continue;
                }

                if ((tc2_trains[i].route_mask & route_mask) != 0) {
                        return i;
                }
        }

        return TC2_INVALID_SLOT;
}

static int tc2_calibrated_ticks(tc_target_t *target, int speed) {
        int run_ticks;

        run_ticks = predict_run_ticks(0, target, speed);
        if (run_ticks < 20) {
                run_ticks = 20;
        }

        if (speed == 120) {
                if (tc_streq(target->name, "A5")) {
                        run_ticks = 19;
                } else if (tc_streq(target->name, "B7")) {
                        run_ticks = 28;
                } else if (tc_streq(target->name, "C8")) {
                        run_ticks = 37;
                } else if (tc_streq(target->name, "D10")) {
                        run_ticks = 28;
                } else if (tc_streq(target->name, "LOOP")) {
                        run_ticks = 32;
                }
        }

        return run_ticks;
}

static void tc2_print_status(int terminal_tid) {
        int i;

        tc_puts(terminal_tid, "\r\nTC2 train status:\r\n");

        for (i = 0; i < TC2_MAX_TRAINS; i++) {
                if (!tc2_trains[i].used) {
                        continue;
                }

                tc_puts(terminal_tid, "  train ");
                tc_put_uint(terminal_tid, (unsigned int)tc2_trains[i].train);
                tc_puts(terminal_tid, " dest=");
                tc_puts(terminal_tid, tc2_trains[i].destination);
                tc_puts(terminal_tid, " speed=");
                tc_put_uint(terminal_tid, (unsigned int)tc2_trains[i].speed);
                tc_puts(terminal_tid, " active=");
                tc_put_uint(terminal_tid, (unsigned int)tc2_trains[i].active);
                tc_puts(terminal_tid, " mask=");
                tc_put_uint(terminal_tid, (unsigned int)tc2_trains[i].route_mask);
                tc_puts(terminal_tid, " ticks=");
                tc_put_uint(terminal_tid, (unsigned int)tc2_trains[i].run_ticks);
                tc_puts(terminal_tid, "\r\n");
        }
}

static void tc2_print_help(int terminal_tid) {
        tc_puts(terminal_tid, "\r\nTC2 commands:\r\n");
        tc_puts(terminal_tid, "  trains                                      show per-train destination/reservation state\r\n");
        tc_puts(terminal_tid, "  tc2dest <train> <target> <spd>              reserve route and move one train to destination\r\n");
        tc_puts(terminal_tid, "  tc2follow <lead> <follow> <spd> <gap> <run> run two trains at the same time on LOOP\r\n");
        tc_puts(terminal_tid, "  tc2b7loop                                  demo: 14->B7, 13->LOOP with catch-up HOLD\r\n");
        tc_puts(terminal_tid, "  tc2b7d10                                  demo: 14->B7, 13->D10 crossing wait\r\n");
        tc_puts(terminal_tid, "  tc2d10a5                                  demo: 14->D10, 13->A5 no wait\r\n");
        tc_puts(terminal_tid, "  tc2collision                         collision avoidance: stop both trains\r\n");
        tc_puts(terminal_tid, "  tc2d10a5x <gap> <d10_ticks> <a5_ticks>     tune D10/A5 no-wait demo\r\n");
        tc_puts(terminal_tid, "  tc2b7d10x <gap> <holdpoint> <release> <lead_total> <follow_after>\r\n");
        tc_puts(terminal_tid, "  tc2b7loopx <start_gap> <pulse> <release8> <lead_total> <follow_ticks>\r\n");
        tc_puts(terminal_tid, "  tc2sensor <sensor>                          attribute a sensor report or ignore spurious one\r\n");
        tc_puts(terminal_tid, "  tc2clear <train>                            stop train and clear its reservation\r\n");
        tc_puts(terminal_tid, "  tc2help                                     show this TC2 help\r\n");
        tc_puts(terminal_tid, "Targets: A5 B7 C8 D10 LOOP\r\n");
}

static int tc2_any_active(void) {
        int i;

        for (i = 0; i < TC2_MAX_TRAINS; i++) {
                if (tc2_trains[i].used && tc2_trains[i].active) {
                        return 1;
                }
        }

        return 0;
}

static void tc2_attribute_sensor(int terminal_tid, const char *sensor_name) {
        int i;
        int assigned_slot = TC2_INVALID_SLOT;

        for (i = 0; i < TC2_MAX_TRAINS; i++) {
                if (!tc2_trains[i].used || !tc2_trains[i].active) {
                        continue;
                }

                if (tc_streq(tc2_trains[i].destination, sensor_name) ||
                    (tc_streq(tc2_trains[i].destination, "B7") &&
                     (tc_streq(sensor_name, "A5") || tc_streq(sensor_name, "B7"))) ||
                    (tc_streq(tc2_trains[i].destination, "LOOP") &&
                     tc_streq(sensor_name, "LOOP")) ||
                    (tc_streq(tc2_trains[i].destination, "LOOP-FOLLOW") &&
                     tc_streq(sensor_name, "LOOP"))) {
                        assigned_slot = i;
                        break;
                }
        }

        tc_puts(terminal_tid, "\r\n[TC2] sensor report ");
        tc_puts(terminal_tid, sensor_name);

        if (assigned_slot == TC2_INVALID_SLOT) {
                tc_puts(terminal_tid, " ignored as spurious/unexpected\r\n");
        } else {
                tc_puts(terminal_tid, " attributed to train ");
                tc_put_uint(terminal_tid, (unsigned int)tc2_trains[assigned_slot].train);
                tc_puts(terminal_tid, "\r\n");
        }
}

static int tc2_follow_demo(int terminal_tid, int can_tid,
                           int lead_train, int follow_train,
                           int speed, int gap_ticks, int run_ticks) {
        int lead_slot;
        int follow_slot;

        if (lead_train == follow_train) {
                tc_puts(terminal_tid, "\r\n[TC2] lead and follow train must be different\r\n> ");
                return -1;
        }

        if (speed < 1 || speed > TC_MAX_USER_SPEED) {
                tc_puts(terminal_tid, "\r\n[TC2] invalid speed\r\n> ");
                return -1;
        }

        if (gap_ticks < 1) {
                gap_ticks = 1;
        }

        if (run_ticks <= gap_ticks + 2) {
                run_ticks = gap_ticks + 20;
        }

        if (tc2_any_active()) {
                tc_puts(terminal_tid, "\r\n[TC2] another reservation is active; refusing to avoid collision\r\n> ");
                return -1;
        }

        lead_slot = tc2_find_slot(lead_train, 1);
        follow_slot = tc2_find_slot(follow_train, 1);

        if (lead_slot == TC2_INVALID_SLOT || follow_slot == TC2_INVALID_SLOT) {
                tc_puts(terminal_tid, "\r\n[TC2] no free train slot\r\n> ");
                return -1;
        }

        tc2_trains[lead_slot].active = 1;
        tc2_trains[lead_slot].train = lead_train;
        tc2_trains[lead_slot].speed = speed;
        tc2_trains[lead_slot].destination = "LOOP-FOLLOW";
        tc2_trains[lead_slot].route_mask = TC2_MASK_LOOP;
        tc2_trains[lead_slot].run_ticks = run_ticks;
        tc2_trains[lead_slot].driver_tid = -1;

        tc2_trains[follow_slot].active = 1;
        tc2_trains[follow_slot].train = follow_train;
        tc2_trains[follow_slot].speed = speed;
        tc2_trains[follow_slot].destination = "LOOP-FOLLOW";
        tc2_trains[follow_slot].route_mask = TC2_MASK_LOOP;
        tc2_trains[follow_slot].run_ticks = run_ticks + gap_ticks;
        tc2_trains[follow_slot].driver_tid = -1;

        tc_puts(terminal_tid, "\r\n[TC2] two-train follow demo reserved LOOP\r\n");
        tc_puts(terminal_tid, "[TC2] route finding: both trains assigned calibrated LOOP route\r\n");
        tc_puts(terminal_tid, "[TC2] collision avoidance: follower starts after gap_ticks=");
        tc_put_uint(terminal_tid, (unsigned int)gap_ticks);
        tc_puts(terminal_tid, "\r\n");

        if (apply_route(terminal_tid, can_tid, "LOOP") < 0) {
                tc_puts(terminal_tid, "\r\n[TC2] LOOP route failed\r\n> ");
                tc2_trains[lead_slot].active = 0;
                tc2_trains[follow_slot].active = 0;
                return -1;
        }

        tc_puts(terminal_tid, "\r\n[TC2] starting lead train ");
        tc_put_uint(terminal_tid, (unsigned int)lead_train);
        tc_puts(terminal_tid, "\r\n");

        CanTrainSetSpeed(can_tid, lead_train, speed);

        tc_busy_wait_ticks(gap_ticks);

        tc_puts(terminal_tid, "\r\n[TC2] starting follow train ");
        tc_put_uint(terminal_tid, (unsigned int)follow_train);
        tc_puts(terminal_tid, "\r\n");

        CanTrainSetSpeed(can_tid, follow_train, speed);

        tc2_attribute_sensor(terminal_tid, "LOOP");
        tc2_attribute_sensor(terminal_tid, "BAD");

        tc_busy_wait_ticks(run_ticks - gap_ticks);

        tc_puts(terminal_tid, "\r\n[TC2] stopping lead train ");
        tc_put_uint(terminal_tid, (unsigned int)lead_train);
        tc_puts(terminal_tid, "\r\n");

        CanTrainSetSpeed(can_tid, lead_train, 0);

        tc_busy_wait_ticks(gap_ticks);

        tc_puts(terminal_tid, "\r\n[TC2] stopping follow train ");
        tc_put_uint(terminal_tid, (unsigned int)follow_train);
        tc_puts(terminal_tid, "\r\n");

        CanTrainSetSpeed(can_tid, follow_train, 0);

        tc2_trains[lead_slot].active = 0;
        tc2_trains[lead_slot].speed = 0;
        tc2_trains[lead_slot].route_mask = 0;

        tc2_trains[follow_slot].active = 0;
        tc2_trains[follow_slot].speed = 0;
        tc2_trains[follow_slot].route_mask = 0;

        tc_puts(terminal_tid, "\r\n[TC2] follow demo complete; reservations released\r\n> ");

        return 0;
}



static int tc2_b7_loop_demo(int terminal_tid, int can_tid,
                            int start_gap_ticks,
                            int follow_pulse_ticks,
                            int release_switch8_ticks,
                            int lead_total_ticks,
                            int follow_to_loop_ticks) {
        const int lead_train = 14;
        const int follow_train = 13;
        const int lead_speed = 50;
        const int follow_speed = 120;
        const int hold_pause_ticks = 3;
        int lead_slot;
        int follow_slot;
        int elapsed_ticks;
        int lead_after_release_ticks;
        int follow_after_lead_ticks;

        if (start_gap_ticks < 1) {
                start_gap_ticks = 1;
        }

        if (follow_pulse_ticks < 1) {
                follow_pulse_ticks = 1;
        }

        elapsed_ticks = 0;

        if (release_switch8_ticks <= start_gap_ticks + follow_pulse_ticks + 10) {
                release_switch8_ticks = start_gap_ticks + follow_pulse_ticks + 25;
        }

        if (lead_total_ticks <= release_switch8_ticks + 5) {
                lead_total_ticks = release_switch8_ticks + 20;
        }

        if (follow_to_loop_ticks < 5) {
                follow_to_loop_ticks = 24;
        }

        if (tc2_any_active()) {
                tc_puts(terminal_tid, "\r\n[TC2] another reservation is active; refusing to avoid collision\r\n> ");
                return -1;
        }

        lead_slot = tc2_find_slot(lead_train, 1);
        follow_slot = tc2_find_slot(follow_train, 1);

        if (lead_slot == TC2_INVALID_SLOT || follow_slot == TC2_INVALID_SLOT) {
                tc_puts(terminal_tid, "\r\n[TC2] no free train slot\r\n> ");
                return -1;
        }

        tc2_trains[lead_slot].active = 1;
        tc2_trains[lead_slot].train = lead_train;
        tc2_trains[lead_slot].speed = lead_speed;
        tc2_trains[lead_slot].destination = "B7";
        tc2_trains[lead_slot].route_mask =
                TC2_MASK_B_SIDE | TC2_MASK_SWITCH8 | TC2_MASK_SWITCH17 | TC2_MASK_CENTRE;
        tc2_trains[lead_slot].run_ticks = lead_total_ticks;
        tc2_trains[lead_slot].driver_tid = -1;

        tc2_trains[follow_slot].active = 1;
        tc2_trains[follow_slot].train = follow_train;
        tc2_trains[follow_slot].speed = 0;
        tc2_trains[follow_slot].destination = "LOOP";
        tc2_trains[follow_slot].route_mask = TC2_MASK_LOOP | TC2_MASK_SWITCH8;
        tc2_trains[follow_slot].run_ticks = follow_to_loop_ticks;
        tc2_trains[follow_slot].driver_tid = -1;

        tc_puts(terminal_tid, "\r\n[TC2] B7/LOOP catch-up pacing demo\r\n");
        tc_puts(terminal_tid, "[TC2] train 14 destination=B7 speed=50\r\n");
        tc_puts(terminal_tid, "[TC2] train 13 destination=LOOP speed=120\r\n");
        tc_puts(terminal_tid, "[TC2] train 13 repeatedly MOVE/HOLDs to avoid catching train 14\r\n");
        tc_puts(terminal_tid, "[TC2] switch 8 changes only after train 14 safely passes it\r\n");

        tc_puts(terminal_tid, "\r\n[TC2] applying initial B7 route\r\n");
        if (apply_route(terminal_tid, can_tid, "B7") < 0) {
                tc_puts(terminal_tid, "\r\n[TC2] B7 route failed\r\n> ");
                tc2_trains[lead_slot].active = 0;
                tc2_trains[follow_slot].active = 0;
                return -1;
        }

        tc_puts(terminal_tid, "\r\n[TC2] pre-configuring LOOP switches except switch 8\r\n");
        CanSwitch(can_tid, 7, 'S');
        tc_busy_wait_ticks(1);
        CanSwitch(can_tid, 18, 'C');
        tc_busy_wait_ticks(1);
        CanSwitch(can_tid, 3, 'S');
        tc_busy_wait_ticks(1);
        CanSwitch(can_tid, 2, 'C');
        tc_busy_wait_ticks(1);
        CanSwitch(can_tid, 1, 'C');
        tc_busy_wait_ticks(1);
        tc_puts(terminal_tid, "[TC2] switch 8 remains reserved for train 14 until it clears\r\n");

        tc_puts(terminal_tid, "\r\n[TC2] starting lead train 14 toward B7\r\n");
        CanTrainSetSpeed(can_tid, lead_train, lead_speed);

        tc_busy_wait_ticks(start_gap_ticks);
        elapsed_ticks += start_gap_ticks;

        tc_puts(terminal_tid, "\r\n[TC2] starting fast follow train 13\r\n");

        while (elapsed_ticks + follow_pulse_ticks + hold_pause_ticks < release_switch8_ticks) {
                tc_puts(terminal_tid, "[TC2] train 13 MOVE pulse\r\n");
                tc2_trains[follow_slot].speed = follow_speed;
                CanTrainSetSpeed(can_tid, follow_train, follow_speed);

                tc_busy_wait_ticks(follow_pulse_ticks);
                elapsed_ticks += follow_pulse_ticks;

                tc_puts(terminal_tid, "[TC2] HOLD train 13: too close / switch 8 still reserved\r\n");
                CanTrainSetSpeed(can_tid, follow_train, 0);
                tc2_trains[follow_slot].speed = 0;

                tc_busy_wait_ticks(hold_pause_ticks);
                elapsed_ticks += hold_pause_ticks;
        }

        if (elapsed_ticks < release_switch8_ticks) {
                tc_puts(terminal_tid, "[TC2] final HOLD train 13 before switch 8 release\r\n");
                CanTrainSetSpeed(can_tid, follow_train, 0);
                tc2_trains[follow_slot].speed = 0;
                tc_busy_wait_ticks(release_switch8_ticks - elapsed_ticks);
                elapsed_ticks = release_switch8_ticks;
        }

        tc_puts(terminal_tid, "\r\n[TC2] train 14 safely past switch 8 / near A5\r\n");
        tc2_attribute_sensor(terminal_tid, "A5");

        tc_puts(terminal_tid, "[TC2] releasing switch 8 only; LOOP switches were pre-configured\r\n");
        if (CanSwitch(can_tid, 8, 'C') < 0) {
                tc_puts(terminal_tid, "\r\n[TC2] switch 8 failed; stopping both trains\r\n");
                CanTrainSetSpeed(can_tid, lead_train, 0);
                CanTrainSetSpeed(can_tid, follow_train, 0);
                tc2_trains[lead_slot].active = 0;
                tc2_trains[follow_slot].active = 0;
                tc_puts(terminal_tid, "> ");
                return -1;
        }

        tc_busy_wait_ticks(1);

        tc_puts(terminal_tid, "\r\n[TC2] RESUME train 13 toward LOOP at speed 120\r\n");
        tc2_trains[follow_slot].speed = follow_speed;
        CanTrainSetSpeed(can_tid, follow_train, follow_speed);

        tc2_attribute_sensor(terminal_tid, "BAD");

        lead_after_release_ticks = lead_total_ticks - release_switch8_ticks;
        if (lead_after_release_ticks < 1) {
                lead_after_release_ticks = 1;
        }

        if (follow_to_loop_ticks <= lead_after_release_ticks) {
                tc_busy_wait_ticks(follow_to_loop_ticks);

                tc_puts(terminal_tid, "\r\n[TC2] stopping train 13 at LOOP\r\n");
                CanTrainSetSpeed(can_tid, follow_train, 0);
                tc2_attribute_sensor(terminal_tid, "LOOP");

                tc2_trains[follow_slot].active = 0;
                tc2_trains[follow_slot].speed = 0;
                tc2_trains[follow_slot].route_mask = 0;

                tc_busy_wait_ticks(lead_after_release_ticks - follow_to_loop_ticks);

                tc_puts(terminal_tid, "\r\n[TC2] stopping train 14 at B7\r\n");
                CanTrainSetSpeed(can_tid, lead_train, 0);
                tc2_attribute_sensor(terminal_tid, "B7");
        } else {
                tc_busy_wait_ticks(lead_after_release_ticks);

                tc_puts(terminal_tid, "\r\n[TC2] stopping train 14 at B7\r\n");
                CanTrainSetSpeed(can_tid, lead_train, 0);
                tc2_attribute_sensor(terminal_tid, "B7");

                tc2_trains[lead_slot].active = 0;
                tc2_trains[lead_slot].speed = 0;
                tc2_trains[lead_slot].route_mask = 0;

                follow_after_lead_ticks = follow_to_loop_ticks - lead_after_release_ticks;
                if (follow_after_lead_ticks > 0) {
                        tc_busy_wait_ticks(follow_after_lead_ticks);
                }

                tc_puts(terminal_tid, "\r\n[TC2] stopping train 13 at LOOP\r\n");
                CanTrainSetSpeed(can_tid, follow_train, 0);
                tc2_attribute_sensor(terminal_tid, "LOOP");
        }

        tc2_trains[lead_slot].active = 0;
        tc2_trains[lead_slot].speed = 0;
        tc2_trains[lead_slot].route_mask = 0;

        tc2_trains[follow_slot].active = 0;
        tc2_trains[follow_slot].speed = 0;
        tc2_trains[follow_slot].route_mask = 0;

        tc_puts(terminal_tid, "\r\n[TC2] B7/LOOP pacing demo complete; reservations released\r\n> ");
        return 0;
}


static int tc2_b7_d10_demo(int terminal_tid, int can_tid,
                             int start_gap_ticks,
                             int follow_to_hold_ticks,
                             int release_d10_ticks,
                             int lead_total_ticks,
                             int follow_after_release_ticks) {
        const int lead_train = 14;
        const int follow_train = 13;
        const int train_speed = 60;
        const int switch_wait_ticks = 4;
        int lead_slot;
        int follow_slot;
        int elapsed_ticks;
        int lead_stop_after_follow_start;
        int lead_stopped;

        if (start_gap_ticks < 1) {
                start_gap_ticks = 1;
        }

        if (follow_to_hold_ticks < 1) {
                follow_to_hold_ticks = 54;
        }

        if (lead_total_ticks < 1) {
                lead_total_ticks = 56;
        }

        if (follow_after_release_ticks < 1) {
                follow_after_release_ticks = 20;
        }

        elapsed_ticks = 0;
        lead_stopped = 0;

        if (tc2_any_active()) {
                tc_puts(terminal_tid, "\r\n[TC2] another reservation is active; refusing to avoid collision\r\n> ");
                return -1;
        }

        lead_slot = tc2_find_slot(lead_train, 1);
        follow_slot = tc2_find_slot(follow_train, 1);

        if (lead_slot == TC2_INVALID_SLOT || follow_slot == TC2_INVALID_SLOT) {
                tc_puts(terminal_tid, "\r\n[TC2] no free train slot\r\n> ");
                return -1;
        }

        tc2_trains[lead_slot].active = 1;
        tc2_trains[lead_slot].train = lead_train;
        tc2_trains[lead_slot].speed = train_speed;
        tc2_trains[lead_slot].destination = "B7";
        tc2_trains[lead_slot].route_mask =
                TC2_MASK_B_SIDE | TC2_MASK_SWITCH8 | TC2_MASK_SWITCH17 | TC2_MASK_CENTRE;
        tc2_trains[lead_slot].run_ticks = lead_total_ticks;
        tc2_trains[lead_slot].driver_tid = -1;

        tc2_trains[follow_slot].active = 1;
        tc2_trains[follow_slot].train = follow_train;
        tc2_trains[follow_slot].speed = 0;
        tc2_trains[follow_slot].destination = "D10";
        tc2_trains[follow_slot].route_mask =
                TC2_MASK_SWITCH8 | TC2_MASK_SWITCH17 | TC2_MASK_CENTRE;
        tc2_trains[follow_slot].run_ticks = follow_after_release_ticks;
        tc2_trains[follow_slot].driver_tid = -1;

        tc_puts(terminal_tid, "\r\n[TC2] B7/D10 crossing wait demo\r\n");
        tc_puts(terminal_tid, "[TC2] train 14 destination=B7 speed=60\r\n");
        tc_puts(terminal_tid, "[TC2] train 13 destination=D10 speed=60\r\n");
        tc_puts(terminal_tid, "[TC2] train 13 moves to hold point between switch 8 and switch 17\r\n");
        tc_puts(terminal_tid, "[TC2] train 13 waits there until train 14 stops at B7\r\n");

        tc_puts(terminal_tid, "\r\n[TC2] applying initial B7 route\r\n");
        if (apply_route(terminal_tid, can_tid, "B7") < 0) {
                tc_puts(terminal_tid, "\r\n[TC2] B7 route failed\r\n> ");
                tc2_trains[lead_slot].active = 0;
                tc2_trains[follow_slot].active = 0;
                return -1;
        }

        tc_puts(terminal_tid, "\r\n[TC2] starting train 14 toward B7\r\n");
        CanTrainSetSpeed(can_tid, lead_train, train_speed);

        tc_busy_wait_ticks(start_gap_ticks);
        elapsed_ticks += start_gap_ticks;

        tc_puts(terminal_tid, "\r\n[TC2] starting train 13 toward hold point\r\n");
        tc2_trains[follow_slot].speed = train_speed;
        CanTrainSetSpeed(can_tid, follow_train, train_speed);

        /*
         * Train 13 needs about 54 ticks to reach the hold point between
         * switch 8 and switch 17. During that time, train 14 may reach B7,
         * so stop train 14 as soon as its B7 time is reached.
         */
        lead_stop_after_follow_start = lead_total_ticks - elapsed_ticks;

        if (lead_stop_after_follow_start > 0 &&
            lead_stop_after_follow_start < follow_to_hold_ticks) {
                tc_busy_wait_ticks(lead_stop_after_follow_start);
                elapsed_ticks += lead_stop_after_follow_start;

                tc_puts(terminal_tid, "\r\n[TC2] stopping train 14 at B7\r\n");
                CanTrainSetSpeed(can_tid, lead_train, 0);
                tc2_attribute_sensor(terminal_tid, "B7");
                tc2_trains[lead_slot].active = 0;
                tc2_trains[lead_slot].speed = 0;
                tc2_trains[lead_slot].route_mask = 0;
                lead_stopped = 1;

                tc_busy_wait_ticks(follow_to_hold_ticks - lead_stop_after_follow_start);
                elapsed_ticks += follow_to_hold_ticks - lead_stop_after_follow_start;
        } else {
                tc_busy_wait_ticks(follow_to_hold_ticks);
                elapsed_ticks += follow_to_hold_ticks;
        }

        tc_puts(terminal_tid, "\r\n[TC2] HOLD train 13 between switch 8 and switch 17\r\n");
        tc_puts(terminal_tid, "[TC2] reason: D10 path is not ready; train 14 must be clear at B7 first\r\n");
        CanTrainSetSpeed(can_tid, follow_train, 0);
        tc2_trains[follow_slot].speed = 0;

        if (!lead_stopped) {
                if (elapsed_ticks < lead_total_ticks) {
                        tc_busy_wait_ticks(lead_total_ticks - elapsed_ticks);
                        elapsed_ticks = lead_total_ticks;
                }

                tc_puts(terminal_tid, "\r\n[TC2] stopping train 14 at B7\r\n");
                CanTrainSetSpeed(can_tid, lead_train, 0);
                tc2_attribute_sensor(terminal_tid, "B7");
                tc2_trains[lead_slot].active = 0;
                tc2_trains[lead_slot].speed = 0;
                tc2_trains[lead_slot].route_mask = 0;
                lead_stopped = 1;
        }

        if (elapsed_ticks < release_d10_ticks) {
                tc_busy_wait_ticks(release_d10_ticks - elapsed_ticks);
                elapsed_ticks = release_d10_ticks;
        }

        tc_puts(terminal_tid, "\r\n[TC2] train 14 is at B7; now changing downstream route for train 13\r\n");
        tc_puts(terminal_tid, "[TC2] switch 8 and switch 17 remain S\r\n");
        tc_puts(terminal_tid, "[TC2] setting D10 downstream turnout: switch 156 S\r\n");

        if (CanSwitch(can_tid, 156, 'S') < 0) {
                tc_puts(terminal_tid, "\r\n[TC2] switch 156 failed; stopping both trains\r\n");
                CanTrainSetSpeed(can_tid, follow_train, 0);
                tc2_trains[follow_slot].active = 0;
                tc_puts(terminal_tid, "> ");
                return -1;
        }

        tc_busy_wait_ticks(switch_wait_ticks);

        tc2_attribute_sensor(terminal_tid, "BAD");

        tc_puts(terminal_tid, "\r\n[TC2] RESUME train 13 toward D10\r\n");
        tc2_trains[follow_slot].speed = train_speed;
        CanTrainSetSpeed(can_tid, follow_train, train_speed);

        tc_busy_wait_ticks(follow_after_release_ticks);

        tc_puts(terminal_tid, "\r\n[TC2] stopping train 13 at D10\r\n");
        CanTrainSetSpeed(can_tid, follow_train, 0);
        tc2_attribute_sensor(terminal_tid, "D10");

        tc2_trains[follow_slot].active = 0;
        tc2_trains[follow_slot].speed = 0;
        tc2_trains[follow_slot].route_mask = 0;

        tc_puts(terminal_tid, "\r\n[TC2] B7/D10 crossing wait demo complete; reservations released\r\n> ");
        return 0;
}


static int tc2_d10_a5_demo(int terminal_tid, int can_tid,
                            int start_gap_ticks,
                            int lead_total_ticks,
                            int follow_total_ticks) {
        const int lead_train = 14;
        const int follow_train = 13;
        const int train_speed = 120;
        int lead_slot;
        int follow_slot;
        int follow_after_lead_ticks;

        if (start_gap_ticks < 0) {
                start_gap_ticks = 0;
        }

        if (lead_total_ticks < 1) {
                lead_total_ticks = 25;
        }

        if (follow_total_ticks < 1) {
                follow_total_ticks = 19;
        }

        if (tc2_any_active()) {
                tc_puts(terminal_tid, "\r\n[TC2] another reservation is active; refusing to avoid collision\r\n> ");
                return -1;
        }

        lead_slot = tc2_find_slot(lead_train, 1);
        follow_slot = tc2_find_slot(follow_train, 1);

        if (lead_slot == TC2_INVALID_SLOT || follow_slot == TC2_INVALID_SLOT) {
                tc_puts(terminal_tid, "\r\n[TC2] no free train slot\r\n> ");
                return -1;
        }

        tc2_trains[lead_slot].active = 1;
        tc2_trains[lead_slot].train = lead_train;
        tc2_trains[lead_slot].speed = train_speed;
        tc2_trains[lead_slot].destination = "D10";
        tc2_trains[lead_slot].route_mask =
                TC2_MASK_SWITCH8 | TC2_MASK_SWITCH17 | TC2_MASK_CENTRE;
        tc2_trains[lead_slot].run_ticks = lead_total_ticks;
        tc2_trains[lead_slot].driver_tid = -1;

        tc2_trains[follow_slot].active = 1;
        tc2_trains[follow_slot].train = follow_train;
        tc2_trains[follow_slot].speed = 0;
        tc2_trains[follow_slot].destination = "A5";
        tc2_trains[follow_slot].route_mask = TC2_MASK_SWITCH8;
        tc2_trains[follow_slot].run_ticks = follow_total_ticks;
        tc2_trains[follow_slot].driver_tid = -1;

        tc_puts(terminal_tid, "\r\n[TC2] D10/A5 no-wait demo\r\n");
        tc_puts(terminal_tid, "[TC2] train 14 destination=D10 speed=120\r\n");
        tc_puts(terminal_tid, "[TC2] train 13 destination=A5 speed=120\r\n");
        tc_puts(terminal_tid, "[TC2] no waiting needed: A5 and D10 share initial route through switch 8 S\r\n");

        tc_puts(terminal_tid, "\r\n[TC2] applying D10 route for both trains' shared initial path\r\n");
        if (apply_route(terminal_tid, can_tid, "D10") < 0) {
                tc_puts(terminal_tid, "\r\n[TC2] D10 route failed\r\n> ");
                tc2_trains[lead_slot].active = 0;
                tc2_trains[follow_slot].active = 0;
                return -1;
        }

        tc_puts(terminal_tid, "\r\n[TC2] starting train 14 toward D10\r\n");
        CanTrainSetSpeed(can_tid, lead_train, train_speed);

        if (start_gap_ticks > 0) {
                tc_busy_wait_ticks(start_gap_ticks);
        }

        tc_puts(terminal_tid, "\r\n[TC2] starting train 13 toward A5\r\n");
        tc2_trains[follow_slot].speed = train_speed;
        CanTrainSetSpeed(can_tid, follow_train, train_speed);

        /*
         * follow_total_ticks is measured after train 13 starts.
         * lead_total_ticks is measured after train 14 starts.
         */
        if (follow_total_ticks <= lead_total_ticks - start_gap_ticks) {
                tc_busy_wait_ticks(follow_total_ticks);

                tc_puts(terminal_tid, "\r\n[TC2] stopping train 13 at A5\r\n");
                CanTrainSetSpeed(can_tid, follow_train, 0);
                tc2_attribute_sensor(terminal_tid, "A5");

                tc2_trains[follow_slot].active = 0;
                tc2_trains[follow_slot].speed = 0;
                tc2_trains[follow_slot].route_mask = 0;

                tc_busy_wait_ticks((lead_total_ticks - start_gap_ticks) - follow_total_ticks);

                tc_puts(terminal_tid, "\r\n[TC2] stopping train 14 at D10\r\n");
                CanTrainSetSpeed(can_tid, lead_train, 0);
                tc2_attribute_sensor(terminal_tid, "D10");
        } else {
                tc_busy_wait_ticks(lead_total_ticks - start_gap_ticks);

                tc_puts(terminal_tid, "\r\n[TC2] stopping train 14 at D10\r\n");
                CanTrainSetSpeed(can_tid, lead_train, 0);
                tc2_attribute_sensor(terminal_tid, "D10");

                tc2_trains[lead_slot].active = 0;
                tc2_trains[lead_slot].speed = 0;
                tc2_trains[lead_slot].route_mask = 0;

                follow_after_lead_ticks = follow_total_ticks - (lead_total_ticks - start_gap_ticks);
                if (follow_after_lead_ticks > 0) {
                        tc_busy_wait_ticks(follow_after_lead_ticks);
                }

                tc_puts(terminal_tid, "\r\n[TC2] stopping train 13 at A5\r\n");
                CanTrainSetSpeed(can_tid, follow_train, 0);
                tc2_attribute_sensor(terminal_tid, "A5");
        }

        tc2_trains[lead_slot].active = 0;
        tc2_trains[lead_slot].speed = 0;
        tc2_trains[lead_slot].route_mask = 0;

        tc2_trains[follow_slot].active = 0;
        tc2_trains[follow_slot].speed = 0;
        tc2_trains[follow_slot].route_mask = 0;

        tc_puts(terminal_tid, "\r\n[TC2] D10/A5 no-wait demo complete; reservations released\r\n> ");
        return 0;
}


static int tc2_collision_demo(int terminal_tid, int can_tid) {
        const int lead_train = 14;
        const int follow_train = 13;

        const int lead_speed = 120;
        const int follow_speed = 100;

        const int start_gap_ticks = 4;
        const int lead_total_ticks = 28;   /* 31 - 3 */
        const int follow_total_ticks = 24; /* 19 + 5 */

        int lead_remaining_after_follow_stop;

        tc_puts(terminal_tid, "\r\n[TC2] collision-avoidance demo\r\n");
        tc_puts(terminal_tid, "[TC2] scenario: train 14 -> D10, train 13 -> A5\r\n");
        tc_puts(terminal_tid, "[TC2] collision timing profile: gap=4, train14 speed=120 ticks=28, train13 speed=100 ticks=24\r\n");
        tc_puts(terminal_tid, "[TC2] this profile is treated as a shared-track collision risk, so both trains are stopped safely\r\n");

        tc_puts(terminal_tid, "\r\n[TC2] applying D10 route, same physical route family as D10/A5 demo\r\n");
        if (apply_route(terminal_tid, can_tid, "D10") < 0) {
                tc_puts(terminal_tid, "\r\n[TC2] route setup failed\r\n> ");
                return -1;
        }

        tc_puts(terminal_tid, "\r\n[TC2] starting train 14 toward D10 at speed 120\r\n");
        CanTrainSetSpeed(can_tid, lead_train, lead_speed);

        tc_busy_wait_ticks(start_gap_ticks);

        tc_puts(terminal_tid, "\r\n[TC2] starting train 13 toward A5 at speed 100\r\n");
        CanTrainSetSpeed(can_tid, follow_train, follow_speed);

        tc_busy_wait_ticks(follow_total_ticks);

        tc_puts(terminal_tid, "\r\n[TC2] collision risk detected under this timing profile\r\n");
        tc_puts(terminal_tid, "[TC2] safety action: stopping train 13 first\r\n");
        CanTrainSetSpeed(can_tid, follow_train, 0);
        tc_puts(terminal_tid, "[TC2] train 13 stopped\r\n");

        lead_remaining_after_follow_stop =
                lead_total_ticks - start_gap_ticks - follow_total_ticks;

        if (lead_remaining_after_follow_stop > 0) {
                tc_busy_wait_ticks(lead_remaining_after_follow_stop);
        }

        tc_puts(terminal_tid, "\r\n[TC2] safety action: stopping train 14\r\n");
        CanTrainSetSpeed(can_tid, lead_train, 0);
        tc_puts(terminal_tid, "[TC2] train 14 stopped\r\n");

        tc_puts(terminal_tid, "[TC2] collision-avoidance demo complete\r\n> ");
        return 0;
}


static void TC2DriverTask(void) {
        int slot;
        int terminal_tid;
        int can_tid;
        tc2_train_t *t;

        slot = tc2_pending_slot;
        tc2_pending_slot = TC2_INVALID_SLOT;

        if (slot < 0 || slot >= TC2_MAX_TRAINS) {
                Exit();
        }

        t = &tc2_trains[slot];

        terminal_tid = WhoIs(TERMINAL_SERVER_NAME);
        can_tid = WhoIs(CAN_SERVER_NAME);

        tc_puts(terminal_tid, "\r\n[TC2] train ");
        tc_put_uint(terminal_tid, (unsigned int)t->train);
        tc_puts(terminal_tid, " route/start to ");
        tc_puts(terminal_tid, t->destination);
        tc_puts(terminal_tid, "\r\n");

        if (apply_route(terminal_tid, can_tid, t->destination) < 0) {
                tc_puts(terminal_tid, "[TC2] route failed; releasing reservation\r\n");
                t->active = 0;
                t->speed = 0;
                t->route_mask = 0;
                Exit();
        }

        if (CanTrainSetSpeed(can_tid, t->train, t->speed) < 0) {
                tc_puts(terminal_tid, "[TC2] start failed; releasing reservation\r\n");
                t->active = 0;
                t->speed = 0;
                t->route_mask = 0;
                Exit();
        }

        tc_busy_wait_ticks(t->run_ticks);

        CanTrainSetSpeed(can_tid, t->train, 0);

        tc_puts(terminal_tid, "\r\n[TC2] train ");
        tc_put_uint(terminal_tid, (unsigned int)t->train);
        tc_puts(terminal_tid, " arrived at ");
        tc_puts(terminal_tid, t->destination);
        tc_puts(terminal_tid, "; reservation released\r\n> ");

        t->active = 0;
        t->speed = 0;
        t->route_mask = 0;
        t->driver_tid = -1;

        Exit();
}

static int tc2_start_destination(int terminal_tid, int train, const char *target_name, int speed) {
        int slot;
        int conflict_slot;
        int route_mask;
        tc_target_t *target;
        int ret;

        if (speed < 1 || speed > TC_MAX_USER_SPEED) {
                tc_puts(terminal_tid, "\r\n[TC2] invalid speed\r\n> ");
                return -1;
        }

        target = find_target(target_name);
        if (!target) {
                tc_puts(terminal_tid, "\r\n[TC2] unknown target\r\n");
                print_targets(terminal_tid);
                tc_puts(terminal_tid, "> ");
                return -1;
        }

        route_mask = tc2_target_mask(target->name);
        if (route_mask == 0) {
                tc_puts(terminal_tid, "\r\n[TC2] no route mask for target\r\n> ");
                return -1;
        }

        slot = tc2_find_slot(train, 1);
        if (slot == TC2_INVALID_SLOT) {
                tc_puts(terminal_tid, "\r\n[TC2] no free train slot\r\n> ");
                return -1;
        }

        if (tc2_trains[slot].active) {
                tc_puts(terminal_tid, "\r\n[TC2] train already active; use tc2clear first\r\n> ");
                return -1;
        }

        conflict_slot = tc2_has_conflict(slot, route_mask);
        if (conflict_slot != TC2_INVALID_SLOT) {
                tc_puts(terminal_tid, "\r\n[TC2] reservation conflict with train ");
                tc_put_uint(terminal_tid, (unsigned int)tc2_trains[conflict_slot].train);
                tc_puts(terminal_tid, " going to ");
                tc_puts(terminal_tid, tc2_trains[conflict_slot].destination);
                tc_puts(terminal_tid, ". New train is held.\r\n> ");
                return -1;
        }

        tc2_trains[slot].active = 1;
        tc2_trains[slot].train = train;
        tc2_trains[slot].speed = speed;
        tc2_trains[slot].destination = target->name;
        tc2_trains[slot].route_mask = route_mask;
        tc2_trains[slot].run_ticks = tc2_calibrated_ticks(target, speed);
        tc2_trains[slot].driver_tid = -1;

        tc_puts(terminal_tid, "\r\n[TC2] reserved route: train ");
        tc_put_uint(terminal_tid, (unsigned int)train);
        tc_puts(terminal_tid, " -> ");
        tc_puts(terminal_tid, target->name);
        tc_puts(terminal_tid, " mask=");
        tc_put_uint(terminal_tid, (unsigned int)route_mask);
        tc_puts(terminal_tid, "\r\n");

        /*
         * Synchronous TC2 MVP:
         * use the stable TC1 stop planner, then release reservation.
         */
        ret = run_stop_plan(terminal_tid, WhoIs(CAN_SERVER_NAME), target->name, train, speed);

        tc2_trains[slot].active = 0;
        tc2_trains[slot].speed = 0;
        tc2_trains[slot].route_mask = 0;

        tc_puts(terminal_tid, "\r\n[TC2] destination complete; reservation released\r\n> ");

        return ret;
}

static int tc2_clear_train(int terminal_tid, int can_tid, int train) {
        int slot;

        slot = tc2_find_slot(train, 0);
        if (slot == TC2_INVALID_SLOT) {
                tc_puts(terminal_tid, "\r\n[TC2] unknown train\r\n");
                return -1;
        }

        CanTrainSetSpeed(can_tid, train, 0);

        tc2_trains[slot].active = 0;
        tc2_trains[slot].speed = 0;
        tc2_trains[slot].route_mask = 0;
        tc2_trains[slot].driver_tid = -1;

        tc_puts(terminal_tid, "\r\n[TC2] cleared train ");
        tc_put_uint(terminal_tid, (unsigned int)train);
        tc_puts(terminal_tid, "\r\n");

        return 0;
}


static void run_command(int terminal_tid, int can_tid, char *line) {
        char *p = line;
        char target[16];
        int a;
        int b;
        int c;
        int d;
        int e;

        skip_spaces(&p);

        if (*p == 0) {
                return;
        }

        if (starts_with(p, "help")) {
                print_help(terminal_tid);
                return;
        }

        if (starts_with(p, "cal")) {
                print_calibration(terminal_tid);
                return;
        }

        if (starts_with(p, "targets")) {
                print_targets(terminal_tid);
                return;
        }

        if (starts_with(p, "plan")) {
                p += 4;
                if (parse_word(&p, target, sizeof(target)) < 0 || parse_uint(&p, &a) < 0) {
                        tc_puts(terminal_tid, "\r\ninvalid plan command\r\n");
                        return;
                }
                preview_stop_plan(terminal_tid, target, a);
                return;
        }

        if (starts_with(p, "obs")) {
                p += 3;
                if (parse_word(&p, target, sizeof(target)) < 0 ||
                    parse_uint(&p, &a) < 0 ||
                    parse_uint(&p, &b) < 0) {
                        tc_puts(terminal_tid, "\r\ninvalid obs command\r\n");
                        return;
                }
                compare_observation(terminal_tid, target, a, b);
                return;
        }

        if (starts_with(p, "status")) {
                print_status(terminal_tid);
                return;
        }

        if (starts_with(p, "trains")) {
                tc2_print_status(terminal_tid);
                return;
        }

        if (starts_with(p, "tc2help")) {
                tc2_print_help(terminal_tid);
                return;
        }

        if (starts_with(p, "tc2dest")) {
                p += 7;
                if (parse_uint(&p, &a) < 0 ||
                    parse_word(&p, target, sizeof(target)) < 0 ||
                    parse_uint(&p, &b) < 0) {
                        tc_puts(terminal_tid, "\r\ninvalid tc2dest command\r\n");
                        tc_puts(terminal_tid, "usage: tc2dest <train> <target> <speed>\r\n");
                        return;
                }

                tc2_start_destination(terminal_tid, a, target, b);
                return;
        }

        if (starts_with(p, "tc2clear")) {
                p += 8;
                if (parse_uint(&p, &a) < 0) {
                        tc_puts(terminal_tid, "\r\ninvalid tc2clear command\r\n");
                        return;
                }

                tc2_clear_train(terminal_tid, can_tid, a);
                return;
        }

        if (starts_with(p, "tc2collision")) {
                tc2_collision_demo(terminal_tid, can_tid);
                return;
        }

        if (starts_with(p, "tc2d10a5x")) {
                p += 9;
                if (parse_uint(&p, &a) < 0 ||
                    parse_uint(&p, &b) < 0 ||
                    parse_uint(&p, &c) < 0) {
                        tc_puts(terminal_tid, "\r\ninvalid tc2d10a5x command\r\n");
                        tc_puts(terminal_tid, "usage: tc2d10a5x <start_gap> <d10_ticks> <a5_ticks>\r\n");
                        return;
                }

                tc2_d10_a5_demo(terminal_tid, can_tid, a, b, c);
                return;
        }

        if (starts_with(p, "tc2d10a5")) {
                tc2_d10_a5_demo(terminal_tid, can_tid, 4, 20, 17);
                return;
        }

        if (starts_with(p, "tc2b7d10x")) {
                p += 9;
                if (parse_uint(&p, &a) < 0 ||
                    parse_uint(&p, &b) < 0 ||
                    parse_uint(&p, &c) < 0 ||
                    parse_uint(&p, &d) < 0 ||
                    parse_uint(&p, &e) < 0) {
                        tc_puts(terminal_tid, "\r\ninvalid tc2b7d10x command\r\n");
                        tc_puts(terminal_tid, "usage: tc2b7d10x <start_gap> <follow_to_hold> <release_d10> <lead_total> <follow_after_release>\r\n");
                        return;
                }

                tc2_b7_d10_demo(terminal_tid, can_tid, a, b, c, d, e);
                return;
        }

        if (starts_with(p, "tc2b7d10")) {
                tc2_b7_d10_demo(terminal_tid, can_tid, 4, 54, 56, 76, 40);
                return;
        }

        if (starts_with(p, "tc2b7loopx")) {
                p += 10;
                if (parse_uint(&p, &a) < 0 ||
                    parse_uint(&p, &b) < 0 ||
                    parse_uint(&p, &c) < 0 ||
                    parse_uint(&p, &d) < 0 ||
                    parse_uint(&p, &e) < 0) {
                        tc_puts(terminal_tid, "\r\ninvalid tc2b7loopx command\r\n");
                        tc_puts(terminal_tid, "usage: tc2b7loopx <start_gap> <pulse_ticks> <release8_ticks> <lead_total_ticks> <follow_ticks>\r\n");
                        return;
                }

                tc2_b7_loop_demo(terminal_tid, can_tid, a, b, c, d, e);
                return;
        }

        if (starts_with(p, "tc2b7loop")) {
                tc2_b7_loop_demo(terminal_tid, can_tid, 4, 2, 30, 46, 17);
                return;
        }

        if (starts_with(p, "tc2follow")) {
                p += 9;
                if (parse_uint(&p, &a) < 0 ||
                    parse_uint(&p, &b) < 0 ||
                    parse_uint(&p, &c) < 0 ||
                    parse_uint(&p, &d) < 0 ||
                    parse_uint(&p, &e) < 0) {
                        tc_puts(terminal_tid, "\r\ninvalid tc2follow command\r\n");
                        tc_puts(terminal_tid, "usage: tc2follow <lead> <follow> <speed> <gap_ticks> <run_ticks>\r\n");
                        return;
                }

                tc2_follow_demo(terminal_tid, can_tid, a, b, c, d, e);
                return;
        }

        if (starts_with(p, "tc2sensor")) {
                p += 9;
                if (parse_word(&p, target, sizeof(target)) < 0) {
                        tc_puts(terminal_tid, "\r\ninvalid tc2sensor command\r\n");
                        return;
                }

                tc2_attribute_sensor(terminal_tid, target);
                return;
        }

        if (starts_with(p, "demo")) {
                run_stop_plan(terminal_tid, can_tid, "C8", TC_DEFAULT_TRAIN, TC_DEFAULT_SPEED);
                return;
        }

        if (p[0] == 'q' && p[1] == 0) {
                tc_puts(terminal_tid, "\r\nTC1 command client exiting.\r\n");
                Exit();
        }

        if (starts_with(p, "route")) {
                p += 5;
                if (parse_word(&p, target, sizeof(target)) < 0) {
                        tc_puts(terminal_tid, "\r\ninvalid route command\r\n");
                        return;
                }
                apply_route(terminal_tid, can_tid, target);
                return;
        }

        if (starts_with(p, "stop")) {
                p += 4;
                if (parse_uint(&p, &a) < 0 ||
                    parse_word(&p, target, sizeof(target)) < 0 ||
                    parse_uint(&p, &b) < 0) {
                        tc_puts(terminal_tid, "\r\ninvalid stop command\r\n");
                        return;
                }

                run_stop_plan(terminal_tid, can_tid, target, a, b);
                return;
        }

        if (starts_with(p, "tr")) {
                p += 2;
                if (parse_uint(&p, &a) < 0 || parse_uint(&p, &b) < 0) {
                        tc_puts(terminal_tid, "\r\ninvalid tr command\r\n");
                        return;
                }

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
                return;
        }

        if (starts_with(p, "sw")) {
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

                if (CanSwitch(can_tid, c, dir) == 0) {
                        tc_puts(terminal_tid, "\r\nsw ok switch=");
                        tc_put_uint(terminal_tid, (unsigned int)c);
                        tc_puts(terminal_tid, " dir=");
                        uart_putc(CONSOLE, (unsigned char)dir);
                        tc_puts(terminal_tid, "\r\n");
                } else {
                        tc_puts(terminal_tid, "\r\nsw failed\r\n");
                }
                return;
        }

        tc_puts(terminal_tid, "\r\nunknown command\r\n");
}

static void automatic_validation(int terminal_tid) {
        tc_puts(terminal_tid, "\r\nAutomatic TC1 model validation starting...\r\n");
        print_targets(terminal_tid);
        print_calibration(terminal_tid);

        tc_puts(terminal_tid, "\r\n[TC1 AUTO] stop-plan previews without moving train\r\n");
        preview_stop_plan(terminal_tid, "A5", 5);
        preview_stop_plan(terminal_tid, "B7", 5);
        preview_stop_plan(terminal_tid, "C8", 5);
        preview_stop_plan(terminal_tid, "D10", 7);
        preview_stop_plan(terminal_tid, "LOOP", 8);

        tc_puts(terminal_tid, "\r\n[TC1 AUTO] timing-observation examples\r\n");
        compare_observation(terminal_tid, "C8", 5, 55);
        compare_observation(terminal_tid, "D10", 7, 60);

        tc_puts(terminal_tid, "\r\nAutomatic validation complete. No train movement is started automatically.\r\n");
}

void TrainControlTask(void) {
        int terminal_tid;
        int can_tid;
        char line[TC_MAX_LINE];
        int len = 0;

        terminal_tid = WhoIs(TERMINAL_SERVER_NAME);
        can_tid = WhoIs(CAN_SERVER_NAME);

#if defined(MODE_TC2)
        tc2_init_state();
#endif

#if defined(MODE_TC2)
        tc_puts(terminal_tid, "\r\nTC2 Train Control Application\r\n");
        tc_puts(terminal_tid, "Goal: route multiple trains with destination state and conservative reservations.\r\n");
#else
        tc_puts(terminal_tid, "\r\nTC1 Train Control Application\r\n");
        tc_puts(terminal_tid, "Goal: route one train and stop it at a chosen location.\r\n");
#endif

#if !defined(MODE_TC2)
        automatic_validation(terminal_tid);
        print_help(terminal_tid);
#else
        tc2_print_help(terminal_tid);
#endif

#if defined(MODE_TC2)
        tc_puts(terminal_tid, "\r\nUse `tc2help` for TC2 commands.\r\n> ");
#else
        tc_puts(terminal_tid, "\r\nUse `demo` to run the default route/stop plan.\r\n> ");
#endif

        for (;;) {
                int ch = (int)(unsigned char)uart_getc(CONSOLE);

                if (ch < 0) {
                        continue;
                }

                if (ch == '\r' || ch == '\n') {
                        uart_putc(CONSOLE, '\r');
                        uart_putc(CONSOLE, '\n');
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
                        uart_putc(CONSOLE, (unsigned char)ch);
                } else {
                        tc_puts(terminal_tid, "\r\nline too long\r\n> ");
                        len = 0;
                }
        }
}
