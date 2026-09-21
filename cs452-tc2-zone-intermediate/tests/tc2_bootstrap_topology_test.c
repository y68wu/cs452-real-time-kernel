#include <stdarg.h>
#include <stddef.h>
#include <stdio.h>

#define MODE_TC2 1
#include "../tasks.c"

#define EXPECTED_CREATE_COUNT 9

_Static_assert(
        TC2_TRAIN_CONTROL_PRIORITY ==
                TC2_DISPATCH_TICKER_PRIORITY,
        "TC2 UI and dispatcher ticker must share a priority");
_Static_assert(
        TC2_SENSOR_COURIER_PRIORITY ==
                TC2_CONTROL_PLANE_PRIORITY,
        "TC2 sensor courier must share the control-plane priority");
_Static_assert(
        TC2_TERMINAL_NOTIFIER_PRIORITY ==
                TC2_CONTROL_PLANE_PRIORITY,
        "TC2 terminal notifiers must share the control-plane priority");

static int create_count;
static int create_priorities[EXPECTED_CREATE_COUNT];
static void (*create_entries[EXPECTED_CREATE_COUNT])(void);
static int exit_calls;
static int marker_seen;

static int text_equal(const char *left, const char *right) {
        while (*left && *right && *left == *right) {
                ++left;
                ++right;
        }
        return *left == *right;
}

int Create(int priority, void (*function)(void)) {
        if (create_count < EXPECTED_CREATE_COUNT) {
                create_priorities[create_count] = priority;
                create_entries[create_count] = function;
        }
        ++create_count;
        return create_count + 1;
}

void Exit(void) {
        ++exit_calls;
}

int MyTid(void) {
        return 1;
}

int MyParentTid(void) {
        return 0;
}

void Yield(void) {
}

void uart_puts(size_t line, const char *text) {
        (void)line;
        if (text_equal(text, TC2_BUILD_MARKER)) {
                marker_seen = 1;
        }
}

void uart_printf(size_t line, const char *format, ...) {
        (void)line;
        (void)format;
}

void NameServerTask(void) {
}

void ClockServerTask(void) {
}

void CanServerTask(void) {
}

void TerminalServerTask(void) {
}

void TrackReservationServerTask(void) {
}

void TrainSensorServerTask(void) {
}

void Tc2DispatchServerTask(void) {
}

void IdleTask(void) {
}

void TrainControlTask(void) {
}

int main(void) {
        const int expected_priorities[EXPECTED_CREATE_COUNT] = {
                1, 2, 3, 4, 4, 4, 4, 15,
                TC2_TRAIN_CONTROL_PRIORITY
        };
        void (*expected_entries[EXPECTED_CREATE_COUNT])(void) = {
                NameServerTask,
                ClockServerTask,
                CanServerTask,
                TerminalServerTask,
                TrackReservationServerTask,
                TrainSensorServerTask,
                Tc2DispatchServerTask,
                IdleTask,
                TrainControlTask
        };

        FirstUserTask();

        if (!marker_seen || exit_calls != 1 ||
            create_count != EXPECTED_CREATE_COUNT) {
                puts("TC2 bootstrap marker/count/exit validation failed");
                return 1;
        }
        for (int index = 0; index < EXPECTED_CREATE_COUNT; ++index) {
                if (create_priorities[index] !=
                            expected_priorities[index] ||
                    create_entries[index] != expected_entries[index]) {
                        printf("TC2 bootstrap topology mismatch at %d\n",
                               index);
                        return 1;
                }
        }
        if (create_priorities[EXPECTED_CREATE_COUNT - 1] !=
                    TC2_CONTROL_PLANE_PRIORITY) {
                puts("TC2 UI does not share the control-plane priority");
                return 1;
        }

        puts("validated TC2 bootstrap topology and control-plane priority");
        return 0;
}
