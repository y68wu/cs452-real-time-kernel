#ifndef _train_sensor_h_
#define _train_sensor_h_ 1

#include "can.h"

#define TRAIN_SENSOR_SERVER_NAME "train-sensors"
#define TRAIN_SENSOR_COUNT 80
#define TRAIN_SENSOR_JOURNAL_CAPACITY 64
#define TRAIN_SENSOR_EVENT_BATCH_CAPACITY 16
#define TRAIN_SENSOR_COURIER_DRAIN_BUDGET 4

typedef struct {
        int sensor_index;
        int old_state;
        int new_state;
        int timestamp;
} train_sensor_event_t;

typedef struct {
        train_sensor_event_t latest;
        train_sensor_event_t latest_occupied;
        unsigned int event_count;
        unsigned int occupied_count;
        unsigned int inconsistent_count;
        unsigned int attributed_count;
        unsigned int unattributed_count;
        unsigned int attribution_unavailable_count;
        unsigned int resynchronized_rising_count;
        /* Legacy name: counts safely ignored current-high 1->1 reports. */
        unsigned int duplicate_timestamp_count;
        /* Reserved diagnostic; protocol timestamps are not ordering keys. */
        unsigned int out_of_order_timestamp_count;
        /*
         * Runtime health is part of the server-owned snapshot so dispatch
         * clients can distinguish an empty, healthy track from a sensor
         * courier that was never created or lost access to CAN.
         */
        int sensor_server_registered;
        int courier_created;
        int courier_ready;
        int courier_last_heartbeat_tick;
        int courier_last_receive_tick;
        unsigned int courier_heartbeat_count;
        unsigned int courier_receive_count;
        unsigned int courier_receive_failure_count;
        unsigned int time_failure_count;
        int has_event;
        int has_occupied;
        unsigned char sensor_state[TRAIN_SENSOR_COUNT];
        /* Diagnostic presence/value of opaque protocol bytes 6/7. */
        unsigned char has_timestamp_by_sensor[TRAIN_SENSOR_COUNT];
        unsigned short last_timestamp_by_sensor[TRAIN_SENSOR_COUNT];
        /*
         * Per-sensor attributed-event latch prevents a target rising event
         * from disappearing when the same train later reaches another sensor.
         */
        int last_attributed_train_by_sensor[TRAIN_SENSOR_COUNT];
        unsigned int last_attributed_generation_by_sensor[TRAIN_SENSOR_COUNT];
        unsigned int last_attributed_sequence_by_sensor[TRAIN_SENSOR_COUNT];
        // Per-train candidate保留last attributed sensor，不代表continuous exact position。
        int position_by_train[256];
        unsigned int position_sequence_by_train[256];
        /* Raw protocol bytes 6/7 for diagnostics; not a clock timestamp. */
        int position_timestamp_by_train[256];
        unsigned int position_generation_by_train[256];
        unsigned int attribution_failure_by_train[256];
} train_sensor_snapshot_t;

typedef struct {
        int sensor_index;
        int train;
        unsigned int generation;
        unsigned int sequence;
        /* Raw protocol bytes 6/7; opaque diagnostic, never an ordering key. */
        int timestamp;
        /* Clock-server tick captured when the sensor server receives EVENT. */
        int arrival_tick;
} train_sensor_attributed_event_t;

typedef struct {
        train_sensor_attributed_event_t
                entries[TRAIN_SENSOR_JOURNAL_CAPACITY];
        int head;
        int count;
        unsigned int dropped_sequence_by_train[256];
        unsigned int newest_sequence_by_train[256];
} train_sensor_journal_t;

typedef struct {
        train_sensor_attributed_event_t
                events[TRAIN_SENSOR_EVENT_BATCH_CAPACITY];
        int count;
        int has_more;
        int lost;
        unsigned int newest_sequence;
} train_sensor_event_batch_t;

int TrainSensorDecode(const can_frame_t *frame, train_sensor_event_t *event);
int TrainSensorLabel(int sensor_index, char *bank, int *number);
void TrainSensorInitSnapshot(train_sensor_snapshot_t *snapshot);
// Apply returns 1 for a new rising event，0 for another valid transition，-1 invalid。
int TrainSensorApplyEvent(train_sensor_snapshot_t *snapshot,
                          const train_sensor_event_t *event);
int TrainSensorRecordAttribution(train_sensor_snapshot_t *snapshot,
                                 int sensor_index, int owner_train,
                                 unsigned int reservation_generation);
int TrainSensorRecordCourierHeartbeat(
        train_sensor_snapshot_t *snapshot, int ready,
        int received_frame, int tick);
void TrainSensorJournalInit(train_sensor_journal_t *journal);
int TrainSensorJournalRecord(train_sensor_journal_t *journal,
                             const train_sensor_attributed_event_t *event);
int TrainSensorJournalQuery(const train_sensor_journal_t *journal, int train,
                            unsigned int after_sequence,
                            train_sensor_event_batch_t *batch);
int TrainSensorGetLatest(int tid, train_sensor_snapshot_t *snapshot);
int TrainSensorGetAttributedEvents(int tid, int train,
                                   unsigned int after_sequence,
                                   train_sensor_event_batch_t *batch);
void TrainSensorServerTask(void);
void TrainSensorCourierTask(void);

#endif
