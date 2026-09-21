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

static void run_command(int terminal_tid, int can_tid, char *line) {
        char *p = line;
        char target[16];
        int a;
        int b;
        int c;

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

        tc_puts(terminal_tid, "\r\nTC1 Train Control Application\r\n");
        tc_puts(terminal_tid, "Goal: route one train and stop it at a chosen location.\r\n");

        automatic_validation(terminal_tid);
        print_help(terminal_tid);

        tc_puts(terminal_tid, "\r\nUse `demo` to run the default route/stop plan.\r\n> ");

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
