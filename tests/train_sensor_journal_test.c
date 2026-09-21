#include <limits.h>
#include <stdio.h>

#include "train_sensor.h"

int CanReceive(int tid, can_frame_t *frame) {
        (void)tid;
        (void)frame;
        return -1;
}
int CanReceivePoll(int tid, can_frame_t *frame) {
        return CanReceive(tid, frame);
}
int AwaitEvent(int event_type) {
        (void)event_type;
        return 0;
}
int Create(int priority, void (*function)(void)) {
        (void)priority;
        (void)function;
        return -1;
}
int MyParentTid(void) { return -1; }
int Receive(int *tid, char *msg, int msglen) {
        (void)tid;
        (void)msg;
        (void)msglen;
        return -1;
}
int RegisterAs(const char *name) {
        (void)name;
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
        (void)msg;
        (void)msglen;
        (void)reply;
        (void)rplen;
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
        return -1;
}
int Time(void) { return 1234; }
void Yield(void) {}
void uart_puts(size_t line, const char *buf) {
        (void)line;
        (void)buf;
}
void uart_printf(size_t line, const char *format, ...) {
        (void)line;
        (void)format;
}

static can_frame_t sensor_frame(int sensor, int old_state,
                                int new_state, int timestamp) {
        can_frame_t frame = {0};
        frame.id = (0x11u << 17) | 0x10000u;
        frame.extended = 1;
        frame.dlc = 8;
        frame.data[2] = (unsigned char)(sensor >> 8);
        frame.data[3] = (unsigned char)sensor;
        frame.data[4] = (unsigned char)old_state;
        frame.data[5] = (unsigned char)new_state;
        frame.data[6] = (unsigned char)(timestamp >> 8);
        frame.data[7] = (unsigned char)timestamp;
        return frame;
}

static int record(train_sensor_journal_t *journal, int train,
                  unsigned int sequence, int sensor) {
        train_sensor_attributed_event_t event;
        event.sensor_index = sensor;
        event.train = train;
        event.generation = 3;
        event.sequence = sequence;
        event.timestamp = (int)(sequence & 0xffffu);
        event.arrival_tick = 1000 + (int)(sequence & 0xffffu) * 7;
        return TrainSensorJournalRecord(journal, &event);
}

static unsigned int next_sequence(unsigned int sequence) {
        ++sequence;
        return sequence == 0 ? 1 : sequence;
}

int main(void) {
        train_sensor_event_t event;
        train_sensor_snapshot_t snapshot;
        can_frame_t frame = sensor_frame(1, 0, 1, 10);

        if (TrainSensorDecode(&frame, &event) != 0 ||
            event.sensor_index != 0 || event.old_state != 0 ||
            event.new_state != 1 || event.timestamp != 10) {
                fprintf(stderr, "valid sensor frame decode failed\n");
                return 1;
        }
        frame.extended = 0;
        if (TrainSensorDecode(&frame, &event) != -1) {
                fprintf(stderr, "standard frame was accepted\n");
                return 1;
        }
        frame.extended = 1;
        frame.dlc = 7;
        if (TrainSensorDecode(&frame, &event) != -1) {
                fprintf(stderr, "bad DLC was accepted\n");
                return 1;
        }

        TrainSensorInitSnapshot(&snapshot);
        event.sensor_index = 0;
        event.old_state = 0;
        event.new_state = 1;
        event.timestamp = 10;
        if (TrainSensorApplyEvent(&snapshot, &event) != 1 ||
            TrainSensorApplyEvent(&snapshot, &event) != 1 ||
            snapshot.resynchronized_rising_count != 1 ||
            snapshot.occupied_count != 2) {
                fprintf(stderr,
                        "same opaque timestamp swallowed lost-release resync\n");
                return 1;
        }
        event.old_state = 1;
        event.new_state = 1;
        event.timestamp = 65535;
        if (TrainSensorApplyEvent(&snapshot, &event) != 0 ||
            snapshot.duplicate_timestamp_count != 1) {
                fprintf(stderr, "current-high repeat handling failed\n");
                return 1;
        }

        /* The release at timestamp 11 is intentionally missing. */
        event.old_state = 0;
        event.new_state = 1;
        event.timestamp = 20;
        if (TrainSensorApplyEvent(&snapshot, &event) != 1 ||
            snapshot.resynchronized_rising_count != 2 ||
            snapshot.occupied_count != 3) {
                fprintf(stderr, "missing-release resynchronization failed\n");
                return 1;
        }
        event.old_state = 1;
        event.new_state = 0;
        event.timestamp = 15;
        if (TrainSensorApplyEvent(&snapshot, &event) != 0 ||
            snapshot.sensor_state[0] != 0 ||
            snapshot.out_of_order_timestamp_count != 0) {
                fprintf(stderr, "CAN arrival order was overridden by an opaque timestamp\n");
                return 1;
        }

        event.sensor_index = 2;
        event.old_state = 0;
        event.new_state = 1;
        event.timestamp = 1000;
        if (TrainSensorApplyEvent(&snapshot, &event) != 1) return 1;
        event.old_state = 1;
        event.new_state = 0;
        event.timestamp = 41000;
        if (TrainSensorApplyEvent(&snapshot, &event) != 0 ||
            snapshot.sensor_state[2] != 0) {
                fprintf(stderr, "long-interval sensor transition was rejected\n");
                return 1;
        }

        event.sensor_index = 1;
        event.old_state = 0;
        event.new_state = 1;
        event.timestamp = 65530;
        if (TrainSensorApplyEvent(&snapshot, &event) != 1) return 1;
        event.old_state = 1;
        event.new_state = 0;
        event.timestamp = 3;
        if (TrainSensorApplyEvent(&snapshot, &event) != 0 ||
            snapshot.sensor_state[1] != 0) {
                fprintf(stderr, "16-bit timestamp wrap was rejected\n");
                return 1;
        }

        train_sensor_journal_t journal;
        train_sensor_event_batch_t batch;
        TrainSensorJournalInit(&journal);
        for (unsigned int sequence = 1; sequence <= 64; ++sequence) {
                if (record(&journal, 78, sequence,
                           (int)(sequence % TRAIN_SENSOR_COUNT)) < 0) {
                        return 1;
                }
        }
        if (TrainSensorJournalQuery(&journal, 77, 0, &batch) < 0 ||
            batch.lost || batch.count != 0) {
                fprintf(stderr, "other-train overflow caused false loss\n");
                return 1;
        }

        TrainSensorJournalInit(&journal);
        for (unsigned int sequence = 1; sequence <= 20; ++sequence) {
                if (record(&journal, 77, sequence,
                           (int)(sequence % TRAIN_SENSOR_COUNT)) < 0) {
                        return 1;
                }
        }
        if (TrainSensorJournalQuery(&journal, 77, 0, &batch) < 0 ||
            batch.lost || batch.count != TRAIN_SENSOR_EVENT_BATCH_CAPACITY ||
            !batch.has_more || batch.events[0].sequence != 1 ||
            batch.events[15].sequence != 16 ||
            batch.events[0].arrival_tick != 1007 ||
            batch.events[15].arrival_tick != 1112) {
                fprintf(stderr, "journal first page is incorrect\n");
                return 1;
        }
        if (TrainSensorJournalQuery(&journal, 77, 16, &batch) < 0 ||
            batch.lost || batch.count != 4 || batch.has_more ||
            batch.events[0].sequence != 17 ||
            batch.events[3].sequence != 20 ||
            batch.events[0].arrival_tick != 1119 ||
            batch.events[3].arrival_tick != 1140) {
                fprintf(stderr, "journal continuation is incorrect\n");
                return 1;
        }

        for (unsigned int sequence = 21; sequence <= 85; ++sequence) {
                if (record(&journal, 78, sequence,
                           (int)(sequence % TRAIN_SENSOR_COUNT)) < 0) {
                        return 1;
                }
        }
        if (TrainSensorJournalQuery(&journal, 77, 0, &batch) < 0 ||
            !batch.lost) {
                fprintf(stderr, "overwritten train event loss was hidden\n");
                return 1;
        }

        /*
         * Sequence zero is an API sentinel.  The producer must skip it when
         * UINT_MAX wraps, and journal cursors must still order the new values.
         */
        TrainSensorInitSnapshot(&snapshot);
        event.sensor_index = 3;
        event.old_state = 0;
        event.new_state = 1;
        event.timestamp = 12;
        snapshot.attributed_count = UINT_MAX;
        if (TrainSensorApplyEvent(&snapshot, &event) != 1 ||
            TrainSensorRecordAttribution(&snapshot, 3, 77, 9) < 0 ||
            snapshot.attributed_count != 1 ||
            snapshot.last_attributed_sequence_by_sensor[3] != 1) {
                fprintf(stderr, "attributed sequence emitted zero at wrap\n");
                return 1;
        }

        TrainSensorJournalInit(&journal);
        unsigned int sequence = UINT_MAX - 3u;
        for (int index = 0; index < 18; ++index) {
                if (record(&journal, 77, sequence,
                           index % TRAIN_SENSOR_COUNT) < 0) {
                        return 1;
                }
                sequence = next_sequence(sequence);
        }
        if (TrainSensorJournalQuery(&journal, 77, UINT_MAX - 2u,
                                    &batch) < 0 ||
            batch.lost || batch.count != TRAIN_SENSOR_EVENT_BATCH_CAPACITY ||
            batch.has_more ||
            batch.events[0].sequence != UINT_MAX - 1u ||
            batch.events[1].sequence != UINT_MAX ||
            batch.events[2].sequence != 1 ||
            batch.events[15].sequence != 14) {
                fprintf(stderr, "journal paging did not cross sequence wrap\n");
                return 1;
        }
        if (TrainSensorJournalQuery(&journal, 77, UINT_MAX, &batch) < 0 ||
            batch.lost || batch.count != 14 || batch.has_more ||
            batch.events[0].sequence != 1 ||
            batch.events[13].sequence != 14) {
                fprintf(stderr, "post-wrap journal cursor is incorrect\n");
                return 1;
        }

        /*
         * A dropped post-wrap event is newer than a UINT_MAX cursor.  The
         * converse case must not report loss once the cursor has crossed wrap.
         */
        TrainSensorJournalInit(&journal);
        for (unsigned int value = 1; value <= 65; ++value) {
                if (record(&journal, 77, value,
                           (int)(value % TRAIN_SENSOR_COUNT)) < 0) {
                        return 1;
                }
        }
        if (TrainSensorJournalQuery(&journal, 77, UINT_MAX, &batch) < 0 ||
            !batch.lost) {
                fprintf(stderr, "post-wrap overwritten event loss was hidden\n");
                return 1;
        }

        TrainSensorJournalInit(&journal);
        if (record(&journal, 77, UINT_MAX, 0) < 0) return 1;
        for (unsigned int value = 1; value <= 64; ++value) {
                if (record(&journal, 77, value,
                           (int)(value % TRAIN_SENSOR_COUNT)) < 0) {
                        return 1;
                }
        }
        if (TrainSensorJournalQuery(&journal, 77, 1, &batch) < 0 ||
            batch.lost || batch.count == 0 ||
            batch.events[0].sequence != 2) {
                fprintf(stderr, "pre-wrap drop caused false post-wrap loss\n");
                return 1;
        }

        puts("validated opaque protocol bytes, same-value resync, arrival ticks, isolated journal paging, and uint32 sequence wrap");
        return 0;
}
