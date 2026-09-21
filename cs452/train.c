#include <stdint.h>

#include "train.h"
#include "mcp2515.h"

#define CAN_HASH 0x4711u

#define CAN_ID_LOK_SPEED 0x08u
#define CAN_ID_LOK_DIRECTION 0x0Au
#define CAN_ID_ACCESSORY 0x16u

#define MM_LOCO_BASE 0x0000u
#define MM2_ACCESSORY_BASE 0x3000u

#define FRAME_QUEUE_SIZE 64
#define MAX_TRACKED_TRAINS 256
#define MAX_PENDING_REVERSES 16

#define REVERSE_DELAY_US 5000000u

typedef struct {
        unsigned int active;
        unsigned int train;
        unsigned int restore_speed;
        uint32_t deadline_us;
} reverse_state_t;

static can_frame_t frame_queue[FRAME_QUEUE_SIZE];
static unsigned int q_head = 0;
static unsigned int q_tail = 0;

static unsigned int train_speeds[MAX_TRACKED_TRAINS];
static unsigned int train_directions[MAX_TRACKED_TRAINS];
static reverse_state_t pending_reverses[MAX_PENDING_REVERSES];

static uint32_t cs2_can_id(uint32_t command_in_can_id) {
        return (command_in_can_id << 16) | CAN_HASH;
}

static int queue_empty(void) {
        return q_head == q_tail;
}

static int queue_full(void) {
        return ((q_tail + 1) % FRAME_QUEUE_SIZE) == q_head;
}

static unsigned int queue_count(void) {
        if (q_tail >= q_head) return q_tail - q_head;
        return FRAME_QUEUE_SIZE - q_head + q_tail;
}

static unsigned int queue_free_slots(void) {
        return FRAME_QUEUE_SIZE - 1 - queue_count();
}

static int enqueue_frame(const can_frame_t *frame) {
        if (queue_full()) return 0;
        frame_queue[q_tail] = *frame;
        q_tail = (q_tail + 1) % FRAME_QUEUE_SIZE;
        return 1;
}

static uint32_t make_loco_loc_id(unsigned int train) {
        return MM_LOCO_BASE | (train & 0x03ffu);
}

static uint32_t make_accessory_loc_id(unsigned int sw) {
        return MM2_ACCESSORY_BASE | (sw & 0x03ffu);
}

static void put_loc_id(can_frame_t *frame, uint32_t loc_id) {
        frame->data[0] = (uint8_t)(loc_id >> 24);
        frame->data[1] = (uint8_t)(loc_id >> 16);
        frame->data[2] = (uint8_t)(loc_id >> 8);
        frame->data[3] = (uint8_t)(loc_id);
}

static unsigned int scale_speed_14_to_cs2(unsigned int speed) {
        if (speed == 0) return 0;
        if (speed > 14) speed = 14;

        /*
         * CS2 uses a system speed in roughly 0..1000.
         * For 14 speed steps the protocol table gives step size 77.
         */
        return 1u + (speed - 1u) * 77u;
}

static can_frame_t make_speed_frame(unsigned int train, unsigned int speed) {
        can_frame_t frame;
        unsigned int cs2_speed = scale_speed_14_to_cs2(speed);

        frame.id = cs2_can_id(CAN_ID_LOK_SPEED);
        frame.extended = 1;
        frame.dlc = 6;

        put_loc_id(&frame, make_loco_loc_id(train));
        frame.data[4] = (uint8_t)(cs2_speed >> 8);
        frame.data[5] = (uint8_t)(cs2_speed);

        for (unsigned int i = 6; i < CAN_MAX_DATA_LEN; i++) {
                frame.data[i] = 0;
        }

        return frame;
}

static can_frame_t make_direction_toggle_frame(unsigned int train) {
        can_frame_t frame;
        unsigned int next_direction = 2;

        if (train < MAX_TRACKED_TRAINS) {
                next_direction = (train_directions[train] == 2) ? 1 : 2;
                train_directions[train] = next_direction;
        }

        frame.id = cs2_can_id(CAN_ID_LOK_DIRECTION);
        frame.extended = 1;
        frame.dlc = 5;

        put_loc_id(&frame, make_loco_loc_id(train));
        frame.data[4] = (uint8_t)next_direction;

        for (unsigned int i = 5; i < CAN_MAX_DATA_LEN; i++) {
                frame.data[i] = 0;
        }

        return frame;
}

static can_frame_t make_switch_frame(unsigned int sw, char direction) {
        can_frame_t frame;
        uint8_t position = 0;

        if (direction >= 'a' && direction <= 'z') {
                direction = (char)(direction - 'a' + 'A');
        }

        /*
         * In the accessory command, position 1 corresponds to green/straight,
         * while 0 corresponds to red/curved for the usual Marklin mapping.
         */
        position = (direction == 'S') ? 1u : 0u;

        frame.id = cs2_can_id(CAN_ID_ACCESSORY);
        frame.extended = 1;
        frame.dlc = 6;

        put_loc_id(&frame, make_accessory_loc_id(sw));
        frame.data[4] = position;
        frame.data[5] = 1; // power on; CS3 default timeout should switch it off.

        for (unsigned int i = 6; i < CAN_MAX_DATA_LEN; i++) {
                frame.data[i] = 0;
        }

        return frame;
}

void train_init(void) {
        q_head = 0;
        q_tail = 0;

        for (unsigned int i = 0; i < MAX_TRACKED_TRAINS; i++) {
                train_speeds[i] = 0;
                train_directions[i] = 1;
        }

        for (unsigned int i = 0; i < MAX_PENDING_REVERSES; i++) {
                pending_reverses[i].active = 0;
                pending_reverses[i].train = 0;
                pending_reverses[i].restore_speed = 0;
                pending_reverses[i].deadline_us = 0;
        }
}

int train_set_speed(unsigned int train, unsigned int speed) {
        can_frame_t frame;

        if (speed > 14) return 0;

        frame = make_speed_frame(train, speed);
        if (!enqueue_frame(&frame)) return 0;

        if (train < MAX_TRACKED_TRAINS) {
                train_speeds[train] = speed;
        }

        return 1;
}

int train_reverse(unsigned int train, uint32_t now_us) {
        unsigned int restore_speed = 0;
        unsigned int slot = MAX_PENDING_REVERSES;

        if (train < MAX_TRACKED_TRAINS) {
                restore_speed = train_speeds[train];
        }

        if (!train_set_speed(train, 0)) return 0;

        for (unsigned int i = 0; i < MAX_PENDING_REVERSES; i++) {
                if (pending_reverses[i].active && pending_reverses[i].train == train) {
                        slot = i;
                        break;
                }
        }

        if (slot == MAX_PENDING_REVERSES) {
                for (unsigned int i = 0; i < MAX_PENDING_REVERSES; i++) {
                        if (!pending_reverses[i].active) {
                                slot = i;
                                break;
                        }
                }
        }

        if (slot == MAX_PENDING_REVERSES) return 0;

        pending_reverses[slot].active = 1;
        pending_reverses[slot].train = train;
        pending_reverses[slot].restore_speed = restore_speed;
        pending_reverses[slot].deadline_us = now_us + REVERSE_DELAY_US;

        return 1;
}

int train_switch(unsigned int sw, char direction) {
        can_frame_t frame;

        if (direction >= 'a' && direction <= 'z') {
                direction = (char)(direction - 'a' + 'A');
        }

        if (!(direction == 'S' || direction == 'C')) return 0;

        frame = make_switch_frame(sw, direction);
        return enqueue_frame(&frame);
}

void train_poll(uint32_t now_us) {
        for (unsigned int i = 0; i < MAX_PENDING_REVERSES; i++) {
                if (!pending_reverses[i].active) continue;

                if ((int32_t)(now_us - pending_reverses[i].deadline_us) >= 0) {
                        unsigned int needed = 1;
                        can_frame_t direction_frame;
                        can_frame_t speed_frame;

                        if (pending_reverses[i].restore_speed > 0) {
                                needed = 2;
                        }

                        if (queue_free_slots() < needed) {
                                continue;
                        }

                        direction_frame = make_direction_toggle_frame(pending_reverses[i].train);
                        enqueue_frame(&direction_frame);

                        if (pending_reverses[i].restore_speed > 0) {
                                speed_frame = make_speed_frame(pending_reverses[i].train, pending_reverses[i].restore_speed);
                                enqueue_frame(&speed_frame);
                        }

                        pending_reverses[i].active = 0;
                }
        }

        if (!queue_empty()) {
                if (mcp2515_try_send(&frame_queue[q_head])) {
                        q_head = (q_head + 1) % FRAME_QUEUE_SIZE;
                }
        }
}
