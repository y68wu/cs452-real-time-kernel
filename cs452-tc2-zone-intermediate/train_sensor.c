#include "train_sensor.h"
#include "can.h"
#include "clock.h"
#include "events.h"
#include "nameserver.h"
#include "syscall.h"
#include "tasks.h"
#include "track_reservation_server.h"
#include "uart.h"

#define MARKLIN_SENSOR_COMMAND 0x11u
#define MARKLIN_RESPONSE_BIT 0x10000u
#define SENSOR_MSG_GET_LATEST 1
#define SENSOR_MSG_EVENT 2
#define SENSOR_MSG_GET_ATTRIBUTED_EVENTS 3
#define SENSOR_MSG_COURIER_HEALTH 4
#define SENSOR_FORWARD_RETRY_COUNT 3

#define SENSOR_COURIER_STARTED 1
#define SENSOR_COURIER_READY 2
#define SENSOR_COURIER_RECEIVED 3
#define SENSOR_COURIER_CAN_FAILED 4
#define SENSOR_COURIER_FORWARD_FAILED 5

typedef struct {
        int type;
        train_sensor_event_t event;
        int train;
        unsigned int after_sequence;
        int courier_status;
} train_sensor_request_t;

/*
 * Attributed event sequence zero is reserved as the "no cursor" sentinel.
 * All other values form a wrapping uint32 sequence.  As with TCP sequence
 * numbers, ordering is unambiguous while producer/consumer distance stays
 * below half of the unsigned range (the journal only retains 64 events).
 */
static int attributed_sequence_after(unsigned int candidate,
                                     unsigned int reference) {
        if (candidate == 0 || candidate == reference) return 0;
        if (reference == 0) return 1;
        return candidate - reference < (1u << 31);
}

static unsigned int next_attributed_sequence(unsigned int sequence) {
        ++sequence;
        return sequence == 0 ? 1 : sequence;
}

static void clear_sensor_request(train_sensor_request_t *req, int type) {
        unsigned char *bytes = (unsigned char *)req;
        for (unsigned int index = 0; index < sizeof(*req); ++index) {
                bytes[index] = 0;
        }
        req->type = type;
        req->event.sensor_index = -1;
}

static int report_courier_health(
        int server_tid, train_sensor_request_t *req, int status) {
        int ack;
        clear_sensor_request(req, SENSOR_MSG_COURIER_HEALTH);
        req->courier_status = status;
        int ret = Send(server_tid, (const char *)req, sizeof(*req),
                       (char *)&ack, sizeof(ack));
        return ret == (int)sizeof(ack) && ack >= 0 ? 0 : -1;
}

int TrainSensorGetLatest(int tid, train_sensor_snapshot_t *snapshot) {
        train_sensor_request_t req;
        int ret;

        // 这个client API uses Send/Reply，UI不直接读取sensor task的shared memory。
        if (!snapshot) {
                return -1;
        }

        clear_sensor_request(&req, SENSOR_MSG_GET_LATEST);
        ret = Send(tid,
                   (const char *)&req,
                   sizeof(req),
                   (char *)snapshot,
                   sizeof(*snapshot));
        return ret == (int)sizeof(*snapshot) ? 0 : -1;
}

int TrainSensorGetAttributedEvents(int tid, int train,
                                   unsigned int after_sequence,
                                   train_sensor_event_batch_t *batch) {
        train_sensor_request_t req;
        int ret;
        if (!batch || train < 1 || train > 255) return -1;
        clear_sensor_request(&req, SENSOR_MSG_GET_ATTRIBUTED_EVENTS);
        req.train = train;
        req.after_sequence = after_sequence;
        ret = Send(tid, (const char *)&req, sizeof(req),
                   (char *)batch, sizeof(*batch));
        return ret == (int)sizeof(*batch) ? 0 : -1;
}

int TrainSensorDecode(const can_frame_t *frame, train_sensor_event_t *event) {
        unsigned int command;
        int sensor_number;

        // 这个decoder只accepts documented Märklin sensor responses，拒绝普通CAN traffic。
        if (!frame || !event || !frame->extended || frame->dlc != 8) {
                return -1;
        }

        command = (frame->id >> 17) & 0xffu;
        if (command != MARKLIN_SENSOR_COMMAND ||
            (frame->id & MARKLIN_RESPONSE_BIT) == 0) {
                return -1;
        }

        sensor_number = ((int)frame->data[2] << 8) | frame->data[3];
        if (sensor_number < 1 || sensor_number > TRAIN_SENSOR_COUNT ||
            frame->data[4] > 1 || frame->data[5] > 1) {
                return -1;
        }

        // sensor_index使用zero-based 0..79，直接对应track graph中的A1..E16 nodes。
        event->sensor_index = sensor_number - 1;
        event->old_state = frame->data[4];
        event->new_state = frame->data[5];
        event->timestamp = ((int)frame->data[6] << 8) | frame->data[7];
        return 0;
}

int TrainSensorLabel(int sensor_index, char *bank, int *number) {
        // 这个helper centralizes the A1..E16 mapping并rejects invalid graph indexes。
        if (sensor_index < 0 || sensor_index >= TRAIN_SENSOR_COUNT ||
            !bank || !number) {
                return -1;
        }

        *bank = (char)('A' + sensor_index / 16);
        *number = sensor_index % 16 + 1;
        return 0;
}

void TrainSensorInitSnapshot(train_sensor_snapshot_t *snapshot) {
        if (!snapshot) return;
        snapshot->latest.sensor_index = -1;
        snapshot->latest.old_state = 0;
        snapshot->latest.new_state = 0;
        snapshot->latest.timestamp = 0;
        snapshot->latest_occupied = snapshot->latest;
        snapshot->event_count = 0;
        snapshot->occupied_count = 0;
        snapshot->inconsistent_count = 0;
        snapshot->attributed_count = 0;
        snapshot->unattributed_count = 0;
        snapshot->attribution_unavailable_count = 0;
        snapshot->resynchronized_rising_count = 0;
        snapshot->duplicate_timestamp_count = 0;
        snapshot->out_of_order_timestamp_count = 0;
        snapshot->sensor_server_registered = 0;
        snapshot->courier_created = 0;
        snapshot->courier_ready = 0;
        snapshot->courier_last_heartbeat_tick = -1;
        snapshot->courier_last_receive_tick = -1;
        snapshot->courier_heartbeat_count = 0;
        snapshot->courier_receive_count = 0;
        snapshot->courier_receive_failure_count = 0;
        snapshot->time_failure_count = 0;
        snapshot->has_event = 0;
        snapshot->has_occupied = 0;
        // 显式initialize every sensor byte，IPC snapshot不会携带uninitialized state。
        for (int i = 0; i < TRAIN_SENSOR_COUNT; ++i) {
                snapshot->sensor_state[i] = 0;
                snapshot->has_timestamp_by_sensor[i] = 0;
                snapshot->last_timestamp_by_sensor[i] = 0;
                snapshot->last_attributed_train_by_sensor[i] = 0;
                snapshot->last_attributed_generation_by_sensor[i] = 0;
                snapshot->last_attributed_sequence_by_sensor[i] = 0;
        }
        for (int train = 0; train < 256; ++train) {
                snapshot->position_by_train[train] = -1;
                snapshot->position_sequence_by_train[train] = 0;
                snapshot->position_timestamp_by_train[train] = 0;
                snapshot->position_generation_by_train[train] = 0;
                snapshot->attribution_failure_by_train[train] = 0;
        }
}

int TrainSensorApplyEvent(train_sensor_snapshot_t *snapshot,
                          const train_sensor_event_t *event) {
        int previous_state;
        int new_occupied;
        unsigned short timestamp;

        if (!snapshot || !event ||
            event->sensor_index < 0 || event->sensor_index >= TRAIN_SENSOR_COUNT ||
            event->old_state < 0 || event->old_state > 1 ||
            event->new_state < 0 || event->new_state > 1 ||
            event->timestamp < 0 || event->timestamp > 0xffff) {
                return -1;
        }

        previous_state = snapshot->sensor_state[event->sensor_index];
        timestamp = (unsigned short)event->timestamp;
        /*
         * Bytes 6/7 are opaque and cannot identify retransmissions. The only
         * safely ignorable report is an explicit 1->1 while our current
         * state is already high. In contrast, a delivered protocol 0->1 while
         * current state is high may be the first evidence after a lost 1->0,
         * even if its opaque bytes equal or wrap past the previous report.
         */
        if (previous_state == 1 &&
            event->old_state == 1 && event->new_state == 1) {
                snapshot->duplicate_timestamp_count++;
                return 0;
        }
        /*
         * The course Märklin protocol notes explicitly classify bytes 6/7
         * as an ignorable timestamp.  In particular, a 16-bit modular value
         * cannot order two reports from a sensor that has been quiet for
         * more than half a wrap.  CAN delivery order is authoritative here;
         * the timestamp is retained for diagnostics only.
         */

        new_occupied = previous_state == 0 && event->new_state == 1;
        if (!new_occupied && previous_state == 1 &&
            event->old_state == 0 && event->new_state == 1) {
                /*
                 * A lost release leaves the server at 1. A later delivered
                 * protocol 0->1 proves a new occupation and repairs that
                 * state without turning an exact duplicate into two events.
                 */
                new_occupied = 1;
                snapshot->resynchronized_rising_count++;
        }
        if (previous_state != event->old_state) {
                // Protocol old_state与server history不一致时仍accept new state，并记录diagnostic。
                snapshot->inconsistent_count++;
        }
        // Snapshot保留全部80个live sensor states，给attribution和track UI统一读取。
        snapshot->latest = *event;
        snapshot->sensor_state[event->sensor_index] =
                (unsigned char)event->new_state;
        snapshot->has_timestamp_by_sensor[event->sensor_index] = 1;
        snapshot->last_timestamp_by_sensor[event->sensor_index] = timestamp;
        snapshot->event_count++;
        snapshot->has_event = 1;
        // 只按server state识别0->1，duplicate rising frames不会制造第二次position event。
        if (new_occupied) {
                snapshot->latest_occupied = *event;
                snapshot->occupied_count++;
                snapshot->has_occupied = 1;
        }
        return new_occupied;
}

void TrainSensorJournalInit(train_sensor_journal_t *journal) {
        if (!journal) return;
        journal->head = 0;
        journal->count = 0;
        for (int i = 0; i < TRAIN_SENSOR_JOURNAL_CAPACITY; ++i) {
                journal->entries[i].sensor_index = -1;
                journal->entries[i].train = 0;
                journal->entries[i].generation = 0;
                journal->entries[i].sequence = 0;
                journal->entries[i].timestamp = 0;
                journal->entries[i].arrival_tick = 0;
        }
        for (int train = 0; train < 256; ++train) {
                journal->dropped_sequence_by_train[train] = 0;
                journal->newest_sequence_by_train[train] = 0;
        }
}

int TrainSensorJournalRecord(train_sensor_journal_t *journal,
                             const train_sensor_attributed_event_t *event) {
        if (!journal || !event || event->sensor_index < 0 ||
            event->sensor_index >= TRAIN_SENSOR_COUNT ||
            event->train < 1 || event->train > 255 ||
            event->generation == 0 || event->sequence == 0 ||
            event->timestamp < 0 || event->timestamp > 0xffff ||
            event->arrival_tick < 0) {
                return -1;
        }

        int write_index;
        if (journal->count < TRAIN_SENSOR_JOURNAL_CAPACITY) {
                write_index =
                        (journal->head + journal->count) %
                        TRAIN_SENSOR_JOURNAL_CAPACITY;
                journal->count++;
        } else {
                train_sensor_attributed_event_t *dropped =
                        &journal->entries[journal->head];
                if (dropped->train >= 1 && dropped->train <= 255) {
                        journal->dropped_sequence_by_train[dropped->train] =
                                dropped->sequence;
                }
                write_index = journal->head;
                journal->head =
                        (journal->head + 1) %
                        TRAIN_SENSOR_JOURNAL_CAPACITY;
        }
        journal->entries[write_index] = *event;
        journal->newest_sequence_by_train[event->train] = event->sequence;
        return 0;
}

int TrainSensorJournalQuery(const train_sensor_journal_t *journal, int train,
                            unsigned int after_sequence,
                            train_sensor_event_batch_t *batch) {
        if (!journal || !batch || train < 1 || train > 255) return -1;
        batch->count = 0;
        batch->has_more = 0;
        batch->lost =
                journal->dropped_sequence_by_train[train] != 0 &&
                attributed_sequence_after(
                        journal->dropped_sequence_by_train[train],
                        after_sequence);
        batch->newest_sequence =
                journal->newest_sequence_by_train[train];

        for (int offset = 0; offset < journal->count; ++offset) {
                int index =
                        (journal->head + offset) %
                        TRAIN_SENSOR_JOURNAL_CAPACITY;
                const train_sensor_attributed_event_t *event =
                        &journal->entries[index];
                if (event->train != train ||
                    !attributed_sequence_after(event->sequence,
                                               after_sequence)) {
                        continue;
                }
                if (batch->count >= TRAIN_SENSOR_EVENT_BATCH_CAPACITY) {
                        batch->has_more = 1;
                        break;
                }
                batch->events[batch->count++] = *event;
        }
        return 0;
}

int TrainSensorRecordAttribution(train_sensor_snapshot_t *snapshot,
                                 int sensor_index, int owner_train,
                                 unsigned int reservation_generation) {
        if (!snapshot || sensor_index < 0 || sensor_index >= TRAIN_SENSOR_COUNT ||
            owner_train < 0 || owner_train > 255 ||
            (owner_train > 0 && reservation_generation == 0) ||
            !snapshot->has_occupied ||
            snapshot->latest_occupied.sensor_index != sensor_index ||
            snapshot->sensor_state[sensor_index] != 1) {
                return -1;
        }
        if (owner_train == 0) {
                // Unreserved rising event保持unattributed，不能猜测成任意train position。
                snapshot->unattributed_count++;
                return 0;
        }

        snapshot->position_by_train[owner_train] = sensor_index;
        snapshot->position_sequence_by_train[owner_train] =
                snapshot->occupied_count;
        snapshot->position_timestamp_by_train[owner_train] =
                snapshot->latest_occupied.timestamp;
        snapshot->position_generation_by_train[owner_train] =
                reservation_generation;
        snapshot->attributed_count =
                next_attributed_sequence(snapshot->attributed_count);
        snapshot->last_attributed_train_by_sensor[sensor_index] = owner_train;
        snapshot->last_attributed_generation_by_sensor[sensor_index] =
                reservation_generation;
        snapshot->last_attributed_sequence_by_sensor[sensor_index] =
                snapshot->attributed_count;
        return 0;
}

int TrainSensorRecordCourierHeartbeat(
        train_sensor_snapshot_t *snapshot, int ready,
        int received_frame, int tick) {
        if (!snapshot || (ready != 0 && ready != 1) ||
            (received_frame != 0 && received_frame != 1) ||
            (received_frame && !ready) || tick < 0) {
                return -1;
        }
        snapshot->courier_ready = ready;
        snapshot->courier_last_heartbeat_tick = tick;
        ++snapshot->courier_heartbeat_count;
        if (received_frame) {
                snapshot->courier_last_receive_tick = tick;
                ++snapshot->courier_receive_count;
        }
        return 0;
}

#ifdef TRAIN_SENSOR_DEBUG_RAW
static void print_raw_frame(const can_frame_t *frame) {
        // 这个debug output preserves the raw CAN payload，后续用真实数据确认sensor protocol。
        uart_printf(CONSOLE,
                    "\r\nsensor raw id=%x dlc=%u data=",
                    (unsigned int)frame->id,
                    (unsigned int)frame->dlc);

        for (int i = 0; i < 8; i++) {
                uart_printf(CONSOLE, "%x", (unsigned int)frame->data[i]);
                if (i < 7) {
                        uart_putc(CONSOLE, ' ');
                }
        }
        uart_puts(CONSOLE, "\r\n");
}

static void print_sensor_event(const train_sensor_event_t *event) {
        char bank;
        int number;

        if (TrainSensorLabel(event->sensor_index, &bank, &number) < 0) {
                uart_puts(CONSOLE, "sensor event has invalid index\r\n");
                return;
        }

        uart_printf(CONSOLE,
                    "sensor event=%c%d old=%d new=%d protocol_raw_6_7=%d\r\n",
                    bank,
                    number,
                    event->old_state,
                    event->new_state,
                    event->timestamp);
}
#endif

void TrainSensorCourierTask(void) {
        int server_tid = MyParentTid();
        int can_tid = WhoIs(CAN_SERVER_NAME);
        can_frame_t frame;
        train_sensor_request_t req;
        int ack;
        int ret;
        int raw_frames_since_pause = 0;

        // Courier独占CanReceive并forwards decoded events，sensor server可以同时answer UI queries。
        if (report_courier_health(
                    server_tid, &req, SENSOR_COURIER_STARTED) < 0) {
                (void)AwaitEvent(EVENT_TIMER);
        }
        if (can_tid >= 0 &&
            report_courier_health(
                    server_tid, &req, SENSOR_COURIER_READY) < 0) {
                (void)AwaitEvent(EVENT_TIMER);
        }
        for (;;) {
                if (can_tid < 0) {
                        can_tid = WhoIs(CAN_SERVER_NAME);
                        if (can_tid < 0) {
                                (void)report_courier_health(
                                        server_tid, &req,
                                        SENSOR_COURIER_CAN_FAILED);
                                (void)AwaitEvent(EVENT_TIMER);
                                continue;
                        }
                        if (report_courier_health(
                                    server_tid, &req,
                                    SENSOR_COURIER_READY) < 0) {
                                (void)AwaitEvent(EVENT_TIMER);
                        }
                }
                int receive_status =
                        CanReceivePoll(can_tid, &frame);
                if (receive_status < 0) {
                        raw_frames_since_pause = 0;
                        can_tid = -1;
                        (void)report_courier_health(
                                server_tid, &req,
                                SENSOR_COURIER_CAN_FAILED);
                        (void)AwaitEvent(EVENT_TIMER);
                        continue;
                }
                if (receive_status > 0) {
                        raw_frames_since_pause = 0;
                        (void)report_courier_health(
                                server_tid, &req,
                                SENSOR_COURIER_READY);
                        (void)AwaitEvent(EVENT_TIMER);
                        continue;
                }

                int pause_after_frame = 0;
                /*
                 * CAN carries status, bootloader, turnout, and train frames
                 * as well as sensor reports.  Under sustained unrelated
                 * traffic, a priority-3 courier that immediately polls again
                 * can otherwise keep every lower-priority bootstrap/UI task
                 * from running.  Account every raw frame before decode and
                 * block after processing the fourth, so invalid frames cannot
                 * bypass the fairness point.
                 */
                ++raw_frames_since_pause;
                if (raw_frames_since_pause >=
                            TRAIN_SENSOR_COURIER_DRAIN_BUDGET) {
                        raw_frames_since_pause = 0;
                        pause_after_frame = 1;
                }

                if (report_courier_health(
                            server_tid, &req,
                            SENSOR_COURIER_RECEIVED) < 0) {
                        (void)AwaitEvent(EVENT_TIMER);
                }

#ifdef TRAIN_SENSOR_DEBUG_RAW
                // Raw trace只在debug build启用，default hot path不会被console output拖慢。
                print_raw_frame(&frame);
#endif
                clear_sensor_request(&req, SENSOR_MSG_EVENT);
                if (TrainSensorDecode(&frame, &req.event) < 0) {
                        if (pause_after_frame) {
                                (void)AwaitEvent(EVENT_TIMER);
                        }
                        continue;
                }

#ifdef TRAIN_SENSOR_DEBUG_RAW
                print_sensor_event(&req.event);
#endif
                ret = -1;
                for (int attempt = 0;
                     attempt < SENSOR_FORWARD_RETRY_COUNT; ++attempt) {
                        ret = Send(server_tid,
                                   (const char *)&req,
                                   sizeof(req),
                                   (char *)&ack,
                                   sizeof(ack));
                        if (ret == (int)sizeof(ack) && ack >= 0) break;
                        (void)AwaitEvent(EVENT_TIMER);
                }
                // Courier verifies the full ack，避免silent sensor event loss被当成成功。
                if (ret != (int)sizeof(ack) || ack < 0) {
                        /*
                         * Do not use the direct UART from the sensor safety
                         * path. A missing event is handled by ordered-route
                         * monitoring/watchdogs; blocking on a full FIFO would
                         * prevent every other safety task from progressing.
                         */
                        (void)report_courier_health(
                                server_tid, &req,
                                SENSOR_COURIER_FORWARD_FAILED);
                        (void)AwaitEvent(EVENT_TIMER);
                }
                if (pause_after_frame) {
                        (void)AwaitEvent(EVENT_TIMER);
                }
        }
}

void TrainSensorServerTask(void) {
        train_sensor_snapshot_t snapshot;
        train_sensor_journal_t journal;
        train_sensor_event_batch_t event_batch;
        train_sensor_request_t req;
        int sender_tid;
        int request_len;
        int ack;
        int reservation_tid;
        int courier_tid;

        uart_puts(CONSOLE,
                  "TrainSensorServer: initializing state\r\n");
        TrainSensorInitSnapshot(&snapshot);
        TrainSensorJournalInit(&journal);
        uart_puts(CONSOLE,
                  "TrainSensorServer: state initialized\r\n");

        snapshot.sensor_server_registered =
                RegisterAs(TRAIN_SENSOR_SERVER_NAME) >= 0;
        uart_printf(CONSOLE,
                    "TrainSensorServer: registered=%d\r\n",
                    snapshot.sensor_server_registered);
        reservation_tid = WhoIs(TRACK_RESERVATION_SERVER_NAME);
        uart_printf(CONSOLE,
                    "TrainSensorServer: reservation_tid=%d\r\n",
                    reservation_tid);
        courier_tid = snapshot.sensor_server_registered ?
                Create(TC2_SENSOR_COURIER_PRIORITY,
                       TrainSensorCourierTask) : -1;
        snapshot.courier_created = courier_tid >= 0;
        uart_printf(CONSOLE,
                    "TrainSensorServer: courier_tid=%d\r\n",
                    courier_tid);

        // 这个server owns sensor state，后续attribution和UI都通过messages访问snapshot。
        for (;;) {
                request_len = Receive(&sender_tid, (char *)&req, sizeof(req));
                if (request_len != (int)sizeof(req)) {
                        ack = -1;
                        Reply(sender_tid, (const char *)&ack, sizeof(ack));
                        continue;
                }

                if (req.type == SENSOR_MSG_EVENT) {
                        if (sender_tid != courier_tid) {
                                ack = -1;
                                Reply(sender_tid,
                                      (const char *)&ack,
                                      sizeof(ack));
                                continue;
                        }
                        /*
                         * Capture arrival before reservation lookup or any
                         * other blocking IPC. This is the event time used by
                         * route plausibility checks; protocol bytes 6/7 are
                         * opaque and must never substitute for it.
                         */
                        int arrival_tick = Time();
                        if (arrival_tick < 0) {
                                ++snapshot.time_failure_count;
                                snapshot.courier_ready = 0;
                        }
                        ack = TrainSensorApplyEvent(&snapshot, &req.event);
                        if (ack > 0) {
                                // 只为new rising event读取atomic reservation snapshot。
                                if (reservation_tid < 0) {
                                        reservation_tid =
                                                WhoIs(TRACK_RESERVATION_SERVER_NAME);
                                }
                                int owner;
                                unsigned int generation;
                                if (reservation_tid >= 0 &&
                                    TrackReservationServerLookupNode(
                                            reservation_tid,
                                            req.event.sensor_index,
                                            &owner,
                                            0,
                                            &generation) == 0) {
                                        // Query是one-way dependency，reservation server不会callback sensor server。
                                        if (arrival_tick < 0) {
                                                snapshot
                                                        .attribution_unavailable_count++;
                                                if (owner >= 1 &&
                                                    owner <= 255) {
                                                        snapshot
                                                                .attribution_failure_by_train[
                                                                        owner]++;
                                                } else {
                                                        snapshot
                                                                .unattributed_count++;
                                                }
                                        } else if (TrainSensorRecordAttribution(
                                                    &snapshot,
                                                    req.event.sensor_index,
                                                    owner,
                                                    generation) < 0) {
                                                snapshot.attribution_unavailable_count++;
                                                if (owner >= 1 && owner <= 255) {
                                                        snapshot
                                                                .attribution_failure_by_train[
                                                                        owner]++;
                                                }
                                        } else if (owner > 0) {
                                                train_sensor_attributed_event_t
                                                        attributed;
                                                attributed.sensor_index =
                                                        req.event.sensor_index;
                                                attributed.train = owner;
                                                attributed.generation =
                                                        generation;
                                                attributed.sequence =
                                                        snapshot.attributed_count;
                                                attributed.timestamp =
                                                        req.event.timestamp;
                                                attributed.arrival_tick =
                                                        arrival_tick;
                                                if (TrainSensorJournalRecord(
                                                            &journal,
                                                            &attributed) < 0) {
                                                        snapshot
                                                                .attribution_unavailable_count++;
                                                        snapshot
                                                                .attribution_failure_by_train[
                                                                        owner]++;
                                                }
                                        }
                                } else {
                                        snapshot.attribution_unavailable_count++;
                                }
                        }
                        Reply(sender_tid, (const char *)&ack, sizeof(ack));
                } else if (req.type ==
                           SENSOR_MSG_COURIER_HEALTH) {
                        if (sender_tid != courier_tid) {
                                ack = -1;
                                Reply(sender_tid,
                                      (const char *)&ack,
                                      sizeof(ack));
                                continue;
                        }
                        int tick = Time();
                        int ready =
                                req.courier_status ==
                                                SENSOR_COURIER_READY ||
                                        req.courier_status ==
                                                SENSOR_COURIER_RECEIVED;
                        int received =
                                req.courier_status ==
                                SENSOR_COURIER_RECEIVED;
                        int valid_status =
                                req.courier_status ==
                                                SENSOR_COURIER_STARTED ||
                                        ready ||
                                        req.courier_status ==
                                                SENSOR_COURIER_CAN_FAILED ||
                                        req.courier_status ==
                                                SENSOR_COURIER_FORWARD_FAILED;
                        if (!valid_status) {
                                ack = -1;
                        } else if (tick < 0) {
                                ++snapshot.time_failure_count;
                                snapshot.courier_ready = 0;
                                ack = 0;
                        } else {
                                ack = TrainSensorRecordCourierHeartbeat(
                                        &snapshot, ready, received,
                                        tick);
                                if (req.courier_status ==
                                            SENSOR_COURIER_CAN_FAILED ||
                                    req.courier_status ==
                                            SENSOR_COURIER_FORWARD_FAILED) {
                                        ++snapshot
                                                .courier_receive_failure_count;
                                }
                        }
                        Reply(sender_tid,
                              (const char *)&ack, sizeof(ack));
                } else if (req.type == SENSOR_MSG_GET_LATEST) {
                        Reply(sender_tid,
                              (const char *)&snapshot,
                              sizeof(snapshot));
                } else if (req.type ==
                           SENSOR_MSG_GET_ATTRIBUTED_EVENTS) {
                        if (TrainSensorJournalQuery(
                                    &journal, req.train,
                                    req.after_sequence, &event_batch) < 0) {
                                event_batch.count = 0;
                                event_batch.has_more = 0;
                                event_batch.lost = 1;
                                event_batch.newest_sequence = 0;
                        }
                        Reply(sender_tid, (const char *)&event_batch,
                              sizeof(event_batch));
                } else {
                        ack = -1;
                        Reply(sender_tid, (const char *)&ack, sizeof(ack));
                }
        }
}
