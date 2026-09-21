#include <stdio.h>
#include <setjmp.h>
#include "tasks.h"
#include "train_sensor.h"

static jmp_buf courier_escape;
enum courier_test_mode {
        COURIER_TEST_NONE = 0,
        COURIER_TEST_CAN_FAILURE,
        COURIER_TEST_CONTINUOUS_INVALID,
        COURIER_TEST_CONTINUOUS_SENSOR,
        COURIER_TEST_SERVER_STARTUP
};
static enum courier_test_mode courier_test;
static int await_event_calls;
static int yield_calls;
static int receive_poll_calls;
static int forwarded_sensor_event_calls;
static int mock_whois_tid = -1;
static int created_priority = -1;
static void (*created_function)(void);

/*
 * Link-only stubs keep this host test focused on the pure sensor-state
 * helpers; none of the task/server paths are called.
 */
int CanReceive(int tid, can_frame_t *frame) {
        (void)tid;
        (void)frame;
        return -1;
}
int CanReceivePoll(int tid, can_frame_t *frame) {
        (void)tid;
        if (courier_test == COURIER_TEST_CONTINUOUS_INVALID ||
            courier_test == COURIER_TEST_CONTINUOUS_SENSOR) {
                ++receive_poll_calls;
                frame->id = 0;
                frame->extended = 0;
                frame->dlc = 0;
                for (int index = 0; index < 8; ++index) {
                        frame->data[index] = 0;
                }
                if (courier_test == COURIER_TEST_CONTINUOUS_SENSOR) {
                        frame->id = (0x11u << 17) | 0x10000u;
                        frame->extended = 1;
                        frame->dlc = 8;
                        frame->data[3] = 1;
                        frame->data[5] = 1;
                }
                return 0;
        }
        return CanReceive(tid, frame);
}
int AwaitEvent(int event_type) {
        (void)event_type;
        ++await_event_calls;
        if ((courier_test == COURIER_TEST_CAN_FAILURE &&
             await_event_calls >= 3) ||
            ((courier_test == COURIER_TEST_CONTINUOUS_INVALID ||
              courier_test == COURIER_TEST_CONTINUOUS_SENSOR) &&
             await_event_calls >= 1)) {
                longjmp(courier_escape, 1);
        }
        return 0;
}
int Create(int priority, void (*function)(void)) {
        created_priority = priority;
        created_function = function;
        if (courier_test == COURIER_TEST_SERVER_STARTUP) {
                return 16;
        }
        return -1;
}
int MyParentTid(void) { return -1; }
int Receive(int *tid, char *msg, int msglen) {
        (void)tid;
        (void)msg;
        (void)msglen;
        if (courier_test == COURIER_TEST_SERVER_STARTUP) {
                longjmp(courier_escape, 1);
        }
        return -1;
}
int RegisterAs(const char *name) {
        (void)name;
        if (courier_test == COURIER_TEST_SERVER_STARTUP) {
                return 0;
        }
        return -1;
}
int Reply(int tid, const char *reply, int rplen) {
        (void)tid;
        (void)reply;
        (void)rplen;
        return -1;
}
int Send(int tid, const char *msg, int msglen, char *reply, int rplen) {
        (void)tid;
        if ((courier_test == COURIER_TEST_CONTINUOUS_INVALID ||
             courier_test == COURIER_TEST_CONTINUOUS_SENSOR) &&
            reply && rplen >= (int)sizeof(int)) {
                if (msg && msglen >= (int)sizeof(int) &&
                    *(const int *)msg == 2) {
                        ++forwarded_sensor_event_calls;
                }
                *(int *)reply = 0;
                return (int)sizeof(int);
        }
        return -1;
}
int TrackReservationServerLookupNode(int tid, int node_index,
                                     int *owner_train,
                                     int *destination_node,
                                     unsigned int *generation) {
        (void)tid;
        (void)node_index;
        (void)owner_train;
        (void)destination_node;
        (void)generation;
        return -1;
}
int WhoIs(const char *name) {
        (void)name;
        return mock_whois_tid;
}
int Time(void) { return 1234; }
void Yield(void) { ++yield_calls; }
void uart_puts(size_t line, const char *buf) {
        (void)line;
        (void)buf;
}
void uart_printf(size_t line, const char *format, ...) {
        (void)line;
        (void)format;
}

static int attribute(train_sensor_snapshot_t *snapshot, int sensor,
                     int train, unsigned int generation) {
        snapshot->has_occupied = 1;
        snapshot->latest_occupied.sensor_index = sensor;
        snapshot->latest_occupied.new_state = 1;
        snapshot->sensor_state[sensor] = 1;
        snapshot->occupied_count++;
        int status = TrainSensorRecordAttribution(
                snapshot, sensor, train, generation);
        snapshot->sensor_state[sensor] = 0;
        return status;
}

int main(void) {
        train_sensor_snapshot_t snapshot;
        TrainSensorInitSnapshot(&snapshot);

        if (snapshot.sensor_server_registered ||
            snapshot.courier_created ||
            snapshot.courier_ready ||
            snapshot.courier_last_heartbeat_tick != -1 ||
            snapshot.courier_last_receive_tick != -1 ||
            snapshot.courier_heartbeat_count != 0 ||
            snapshot.courier_receive_count != 0 ||
            snapshot.courier_receive_failure_count != 0 ||
            snapshot.time_failure_count != 0) {
                fprintf(stderr, "sensor runtime health was not initialized\n");
                return 1;
        }
        if (TrainSensorRecordCourierHeartbeat(
                    &snapshot, 1, 0, 40) < 0 ||
            !snapshot.courier_ready ||
            snapshot.courier_last_heartbeat_tick != 40 ||
            snapshot.courier_last_receive_tick != -1 ||
            snapshot.courier_heartbeat_count != 1 ||
            snapshot.courier_receive_count != 0 ||
            TrainSensorRecordCourierHeartbeat(
                    &snapshot, 1, 1, 41) < 0 ||
            snapshot.courier_last_heartbeat_tick != 41 ||
            snapshot.courier_last_receive_tick != 41 ||
            snapshot.courier_heartbeat_count != 2 ||
            snapshot.courier_receive_count != 1 ||
            TrainSensorRecordCourierHeartbeat(
                    &snapshot, 0, 0, 42) < 0 ||
            snapshot.courier_ready ||
            snapshot.courier_heartbeat_count != 3 ||
            TrainSensorRecordCourierHeartbeat(
                    &snapshot, 0, 1, 43) != -1 ||
            TrainSensorRecordCourierHeartbeat(
                    &snapshot, 1, 0, -1) != -1) {
                fprintf(stderr, "sensor courier health accounting failed\n");
                return 1;
        }

        /*
         * A failed CanReceive used to Yield-spin forever at priority 3 and
         * starve every priority-4 safety server. Escape after three timer
         * waits to prove that repeated failures block on the hardware timer
         * and never fall back to Yield.
         */
        courier_test = COURIER_TEST_CAN_FAILURE;
        await_event_calls = 0;
        yield_calls = 0;
        mock_whois_tid = 1;
        if (setjmp(courier_escape) == 0) {
                TrainSensorCourierTask();
                fprintf(stderr, "sensor courier unexpectedly returned\n");
                return 1;
        }
        courier_test = COURIER_TEST_NONE;
        mock_whois_tid = -1;
        if (await_event_calls != 3 || yield_calls != 0) {
                fprintf(stderr,
                        "CAN-unavailable courier did not block safely\n");
                return 1;
        }

        /*
         * Sustained valid or unrelated CAN traffic must also block after a
         * bounded raw-frame burst.  Decode failures used to continue before
         * any wait and could starve FirstUserTask at boot.
         */
        for (int sensor_frames = 0; sensor_frames <= 1; ++sensor_frames) {
                courier_test = sensor_frames ?
                        COURIER_TEST_CONTINUOUS_SENSOR :
                        COURIER_TEST_CONTINUOUS_INVALID;
                await_event_calls = 0;
                yield_calls = 0;
                receive_poll_calls = 0;
                forwarded_sensor_event_calls = 0;
                mock_whois_tid = 1;
                if (setjmp(courier_escape) == 0) {
                        TrainSensorCourierTask();
                        fprintf(stderr,
                                "continuous CAN courier unexpectedly returned\n");
                        return 1;
                }
                if (await_event_calls != 1 || yield_calls != 0 ||
                    receive_poll_calls !=
                            TRAIN_SENSOR_COURIER_DRAIN_BUDGET ||
                    forwarded_sensor_event_calls !=
                            (sensor_frames ?
                                     TRAIN_SENSOR_COURIER_DRAIN_BUDGET :
                                     0)) {
                        fprintf(stderr,
                                "continuous CAN courier did not process then pause at its drain budget\n");
                        return 1;
                }
        }
        courier_test = COURIER_TEST_NONE;
        mock_whois_tid = -1;

        /*
         * The sensor courier must share the priority-4 control-plane class.
         * A higher-priority courier can keep only its sensor server runnable
         * under continuous CAN traffic and starve dispatch/UI startup.
         */
        courier_test = COURIER_TEST_SERVER_STARTUP;
        created_priority = -1;
        created_function = 0;
        mock_whois_tid = 6;
        if (setjmp(courier_escape) == 0) {
                TrainSensorServerTask();
                fprintf(stderr,
                        "sensor server unexpectedly returned\n");
                return 1;
        }
        if (created_priority !=
                    TC2_SENSOR_COURIER_PRIORITY ||
            created_priority != TC2_CONTROL_PLANE_PRIORITY ||
            created_function != TrainSensorCourierTask) {
                fprintf(stderr,
                        "sensor courier priority contract failed\n");
                return 1;
        }
        courier_test = COURIER_TEST_NONE;
        mock_whois_tid = -1;

        if (attribute(&snapshot, 11, 77, 3) < 0 ||
            attribute(&snapshot, 20, 77, 3) < 0 ||
            snapshot.position_by_train[77] != 20 ||
            snapshot.last_attributed_train_by_sensor[11] != 77 ||
            snapshot.last_attributed_generation_by_sensor[11] != 3 ||
            snapshot.last_attributed_sequence_by_sensor[11] != 1 ||
            snapshot.last_attributed_sequence_by_sensor[20] != 2) {
                fprintf(stderr, "attributed sensor latch lost an earlier target\n");
                return 1;
        }

        if (attribute(&snapshot, 11, 78, 4) < 0 ||
            snapshot.last_attributed_train_by_sensor[11] != 78 ||
            snapshot.last_attributed_generation_by_sensor[11] != 4 ||
            snapshot.last_attributed_sequence_by_sensor[11] != 3) {
                fprintf(stderr, "attributed sensor latch did not advance\n");
                return 1;
        }

        printf("validated sensor courier health/fairness and per-sensor attributed-event latch\n");
        return 0;
}
