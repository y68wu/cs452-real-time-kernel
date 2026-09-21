#include "can.h"
#include "mcp2515.h"
#include "spi.h"
#include "syscall.h"
#include "events.h"
#include "nameserver.h"
#include "uart.h"

static int can_hw_initialized = 0;

static int can_hw_init_once(void) {
        if (can_hw_initialized) {
                return 0;
        }

        spi_init();

        if (mcp2515_init() != 0) {
                return -1;
        }

        /*
         * Important: RAW14 test worked after init had time to settle before
         * the user pressed g.  In kernel mode, do the same explicitly.
         */
        for (volatile int d = 0; d < 100000000; d++) {
        }

        can_hw_initialized = 1;
        uart_puts(CONSOLE, "CAN: hw init ok and settled\r\n");
        return 0;
}


#define CAN_MSG_SEND     1
#define CAN_MSG_RECV     2
#define CAN_MSG_RX_FRAME 3
#define CAN_MSG_TX_POLL  4
#define CAN_MSG_HEALTH   5
#define CAN_MSG_RX_OVERFLOW 6
#define CAN_MSG_RX_DROPPED  7
#define CAN_MSG_HW_FAILED   8
#define CAN_MSG_SEND_CONFIRMED 9
#define CAN_MSG_SPEED_BATCH 10
#define CAN_MSG_BATCH_STATUS 11
#define CAN_MSG_BATCH_CANCEL 12
#define CAN_MSG_EMERGENCY_STOP_BATCH 13
#define CAN_MSG_REGISTER_TURNOUT_AUTHORITY 14
#define CAN_MSG_RECV_POLL 15
#define CAN_MSG_RX_HEARTBEAT 16
#define CAN_MSG_TRAIN_CONTROL_PRIORITY 17

#define CAN_TX_Q_SIZE 32
#define CAN_RX_Q_SIZE 32
#define CAN_WAIT_RECV_SIZE 16
#define CAN_TX_TIMEOUT_POLLS 25
#define CAN_RESPONSE_TIMEOUT_POLLS 25
#define CAN_RX_DRAIN_BUDGET 4
#define CAN_TX_PRIORITY_NORMAL 0
#define CAN_TX_PRIORITY_CONTROL 1
#define CAN_TX_PRIORITY_STOP 2
/*
 * A batch can remain pending for as long as its first frame is ahead of up
 * to CAN_TX_Q_SIZE - 1 other batches in the TX queue.  Keep a second ring
 * worth of terminal records so completing that oldest batch cannot make its
 * status slot the very next allocation target.
 */
#define CAN_BATCH_STATUS_SLOTS (CAN_TX_Q_SIZE * 2)

typedef struct {
 int type;
        unsigned int value;
        can_frame_t frame;
        int frame_count;
        can_frame_t frames[CAN_COMMAND_BATCH_MAX];
} can_request_t;

typedef struct {
        int result;
        can_frame_t frame;
} can_reply_t;

typedef struct {
        int sender_tid;
        int require_response;
        unsigned int batch_token;
        int emergency;
        can_frame_t frame;
} tx_item_t;

static tx_item_t tx_q[CAN_TX_Q_SIZE];
static int tx_head = 0;
static int tx_tail = 0;
static int tx_count = 0;

static can_frame_t rx_q[CAN_RX_Q_SIZE];
static int rx_head = 0;
static int rx_tail = 0;
static int rx_count = 0;

static int waiting_recv[CAN_WAIT_RECV_SIZE];
static int wait_head = 0;
static int wait_tail = 0;
static int wait_count = 0;
static int can_rx_notifier_tid = -1;
static int can_tx_notifier_tid = -1;
static unsigned int tx_busy_polls = 0;
static unsigned int response_wait_polls = 0;
static int tx_waiting_response = 0;
static int quarantined_response_valid = 0;
static can_frame_t quarantined_response_request;
static can_tx_poll_state_t tx_poll_state;
static can_health_t can_health;
static can_batch_status_t
        batch_status[CAN_BATCH_STATUS_SLOTS];
static unsigned int next_batch_token;
static int next_batch_status_slot;
static int can_turnout_authority_tid = -1;

void CanTxPollStateInit(can_tx_poll_state_t *state,
                        unsigned int poll_limit) {
        if (!state) return;
        state->polls = 0;
        state->poll_limit = poll_limit > 0 ? poll_limit : 1;
        state->active = 0;
}

void CanTxPollStateStart(can_tx_poll_state_t *state) {
        if (!state) return;
        state->polls = 0;
        if (state->poll_limit == 0) state->poll_limit = 1;
        state->active = 1;
}

int CanTxPollStateObserve(can_tx_poll_state_t *state,
                          int controller_status) {
        if (!state || !state->active) {
                return CAN_TX_POLL_FAILED;
        }
        if (controller_status == MCP2515_TX_COMPLETE) {
                state->active = 0;
                return CAN_TX_POLL_COMPLETE;
        }
        if (controller_status == MCP2515_TX_FAILED) {
                state->active = 0;
                return CAN_TX_POLL_FAILED;
        }
        if (controller_status != MCP2515_TX_PENDING) {
                state->active = 0;
                return CAN_TX_POLL_FAILED;
        }

        state->polls++;
        if (state->polls >= state->poll_limit) {
                state->active = 0;
                return CAN_TX_POLL_TIMEOUT;
        }
        return CAN_TX_POLL_PENDING;
}

static void clear_can_request(can_request_t *request, int type) {
        unsigned char *bytes = (unsigned char *)request;
        for (unsigned int i = 0; i < sizeof(*request); ++i) bytes[i] = 0;
        request->type = type;
        request->frame.extended = 1;
}

static int can_frame_valid(const can_frame_t *frame) {
        return frame && frame->extended <= 1 && frame->dlc <= 8 &&
                ((frame->extended && frame->id <= 0x1fffffffu) ||
                 (!frame->extended && frame->id <= 0x7ffu));
}

static void reset_can_state(void) {
        tx_head = 0;
        tx_tail = 0;
        tx_count = 0;
        rx_head = 0;
        rx_tail = 0;
        rx_count = 0;
        wait_head = 0;
        wait_tail = 0;
        wait_count = 0;
        can_rx_notifier_tid = -1;
        can_tx_notifier_tid = -1;
        tx_busy_polls = 0;
        response_wait_polls = 0;
        tx_waiting_response = 0;
        quarantined_response_valid = 0;
        CanTxPollStateInit(&tx_poll_state, CAN_TX_TIMEOUT_POLLS);
        can_health.hw_ready = 0;
        can_health.tx_completed = 0;
        can_health.tx_failed = 0;
        can_health.tx_timeout = 0;
        can_health.rx_received = 0;
        can_health.rx_dropped = 0;
        can_health.rx_overflow = 0;
        can_health.turnout_changes = 0;
        can_health.tx_notifier_heartbeat = 0;
        can_health.rx_notifier_heartbeat = 0;
        next_batch_token = 0;
        next_batch_status_slot = 0;
        can_turnout_authority_tid = -1;
        for (int slot = 0;
             slot < CAN_BATCH_STATUS_SLOTS; ++slot) {
                batch_status[slot].token = 0;
                batch_status[slot].state = CAN_BATCH_FAILED;
                batch_status[slot].total = 0;
                batch_status[slot].confirmed = 0;
        }
}

static void clear_can_reply(can_reply_t *reply) {
        // Byte-wise zero包含struct padding，所有CAN IPC reply都保持deterministic。
        unsigned char *bytes = (unsigned char *)reply;
        for (unsigned int i = 0; i < sizeof(*reply); ++i) bytes[i] = 0;
}

static can_frame_t to_can_frame(const mcp2515_frame_t *m) {
        can_frame_t c;
        c.id = m->id;
        c.extended = m->extended;
        c.dlc = m->dlc;
        for (int i = 0; i < 8; i++) {
                c.data[i] = m->data[i];
        }
        return c;
}

static mcp2515_frame_t to_mcp_frame(const can_frame_t *c) {
        mcp2515_frame_t m;
        m.id = c->id;
        m.extended = c->extended;
        m.dlc = c->dlc;
        for (int i = 0; i < 8; i++) {
                m.data[i] = c->data[i];
        }
        return m;
}

static tx_item_t *tx_peek(void);
static void reply_send_result(int tid, int result);
static void fail_batch(unsigned int token);

static void quarantine_active_response(const tx_item_t *item) {
        if (!item || !item->require_response) return;
        quarantined_response_request = item->frame;
        quarantined_response_valid = 1;
}

static int tx_push(int tid, const can_frame_t *frame,
                   int require_response,
                   unsigned int batch_token, int emergency) {
        if (tx_count >= CAN_TX_Q_SIZE) {
                return -1;
        }

        tx_q[tx_tail].sender_tid = tid;
        tx_q[tx_tail].require_response = require_response;
        tx_q[tx_tail].batch_token = batch_token;
        tx_q[tx_tail].emergency = emergency;
        tx_q[tx_tail].frame = *frame;
        tx_tail = (tx_tail + 1) % CAN_TX_Q_SIZE;
        tx_count++;
        return 0;
}

static int tx_push_priority(
        int tid, const can_frame_t *frame,
        int require_response,
        unsigned int batch_token, int priority) {
        if (tx_count >= CAN_TX_Q_SIZE) return -1;
        if (priority <= CAN_TX_PRIORITY_NORMAL ||
            priority > CAN_TX_PRIORITY_STOP) {
                return -1;
        }
        int active =
                tx_count > 0 &&
                (tx_poll_state.active ||
                 tx_waiting_response);
        int insert_offset = active ? 1 : 0;
        while (insert_offset < tx_count) {
                int index =
                        (tx_head + insert_offset) %
                        CAN_TX_Q_SIZE;
                if (tx_q[index].emergency < priority) break;
                ++insert_offset;
        }
        for (int offset = tx_count;
             offset > insert_offset; --offset) {
                int destination =
                        (tx_head + offset) %
                        CAN_TX_Q_SIZE;
                int source =
                        (tx_head + offset - 1) %
                        CAN_TX_Q_SIZE;
                tx_q[destination] = tx_q[source];
        }
        int insert =
                (tx_head + insert_offset) %
                CAN_TX_Q_SIZE;
        tx_q[insert].sender_tid = tid;
        tx_q[insert].require_response = require_response;
        tx_q[insert].batch_token = batch_token;
        tx_q[insert].emergency = priority;
        tx_q[insert].frame = *frame;
        ++tx_count;
        tx_tail = (tx_head + tx_count) % CAN_TX_Q_SIZE;
        return 0;
}

static void cancel_lower_priority_tx(
        int minimum_priority, int record_failure) {
        int original_count = tx_count;
        int write_count = 0;
        int preserve_active =
                original_count > 0 &&
                (tx_poll_state.active ||
                 tx_waiting_response);
        for (int offset = 0; offset < original_count; ++offset) {
                int read_index =
                        (tx_head + offset) % CAN_TX_Q_SIZE;
                tx_item_t item = tx_q[read_index];
                if (item.emergency < minimum_priority &&
                    !(preserve_active && offset == 0)) {
                        if (item.sender_tid > 0) {
                                reply_send_result(
                                        item.sender_tid, -1);
                        }
                        fail_batch(item.batch_token);
                        if (record_failure) {
                                ++can_health.tx_failed;
                        }
                        continue;
                }
                int write_index =
                        (tx_head + write_count) %
                        CAN_TX_Q_SIZE;
                tx_q[write_index] = item;
                ++write_count;
        }
        tx_count = write_count;
        tx_tail = (tx_head + tx_count) % CAN_TX_Q_SIZE;
}

static can_batch_status_t *find_batch_status(
        unsigned int token) {
        if (token == 0) return 0;
        for (int slot = 0;
             slot < CAN_BATCH_STATUS_SLOTS; ++slot) {
                if (batch_status[slot].token == token) {
                        return &batch_status[slot];
                }
        }
        return 0;
}

static can_batch_status_t *allocate_batch_status(int total) {
        int selected = -1;
        for (int offset = 0;
             offset < CAN_BATCH_STATUS_SLOTS; ++offset) {
                int slot = (next_batch_status_slot + offset) %
                        CAN_BATCH_STATUS_SLOTS;
                if (batch_status[slot].token == 0 ||
                    batch_status[slot].state !=
                            CAN_BATCH_PENDING) {
                        selected = slot;
                        break;
                }
        }
        if (selected < 0 || total < 1) return 0;
        for (int attempt = 0;
             attempt <= CAN_BATCH_STATUS_SLOTS; ++attempt) {
                ++next_batch_token;
                if (next_batch_token == 0 ||
                    next_batch_token > 0x7fffffffu) {
                        next_batch_token = 1;
                }
                if (!find_batch_status(next_batch_token)) break;
        }
        if (find_batch_status(next_batch_token)) return 0;
        batch_status[selected].token = next_batch_token;
        batch_status[selected].state = CAN_BATCH_PENDING;
        batch_status[selected].total = total;
        batch_status[selected].confirmed = 0;
        next_batch_status_slot =
                (selected + 1) % CAN_BATCH_STATUS_SLOTS;
        return &batch_status[selected];
}

static void confirm_batch_item(unsigned int token) {
        can_batch_status_t *status = find_batch_status(token);
        if (!status || status->state != CAN_BATCH_PENDING) return;
        ++status->confirmed;
        if (status->confirmed >= status->total) {
                status->confirmed = status->total;
                status->state = CAN_BATCH_COMPLETE;
        }
}

static void fail_batch(unsigned int token) {
        can_batch_status_t *status = find_batch_status(token);
        if (status && status->state == CAN_BATCH_PENDING) {
                status->state = CAN_BATCH_FAILED;
        }
}

static void drop_queued_batch_items(unsigned int token) {
        if (token == 0 || tx_count <= 0) return;
        int original_count = tx_count;
        int write_count = 0;
        for (int offset = 0; offset < original_count; ++offset) {
                int read_index =
                        (tx_head + offset) % CAN_TX_Q_SIZE;
                tx_item_t item = tx_q[read_index];
                if (item.batch_token == token) continue;
                int write_index =
                        (tx_head + write_count) %
                        CAN_TX_Q_SIZE;
                tx_q[write_index] = item;
                ++write_count;
        }
        tx_count = write_count;
        tx_tail = (tx_head + tx_count) % CAN_TX_Q_SIZE;
}

static tx_item_t *tx_peek(void) {
        if (tx_count <= 0) {
                return 0;
        }

        return &tx_q[tx_head];
}

static void tx_pop(void) {
        if (tx_count > 0) {
                tx_head = (tx_head + 1) % CAN_TX_Q_SIZE;
                tx_count--;
        }
}

static int rx_push(const can_frame_t *frame) {
        if (rx_count >= CAN_RX_Q_SIZE) {
                return -1;
        }

        rx_q[rx_tail] = *frame;
        rx_tail = (rx_tail + 1) % CAN_RX_Q_SIZE;
        rx_count++;
        return 0;
}

static int rx_pop(can_frame_t *frame) {
        if (rx_count <= 0) {
                return -1;
        }

        *frame = rx_q[rx_head];
        rx_head = (rx_head + 1) % CAN_RX_Q_SIZE;
        rx_count--;
        return 0;
}

static int wait_push(int tid) {
        if (wait_count >= CAN_WAIT_RECV_SIZE) {
                return -1;
        }

        waiting_recv[wait_tail] = tid;
        wait_tail = (wait_tail + 1) % CAN_WAIT_RECV_SIZE;
        wait_count++;
        return 0;
}

static int wait_pop(int *tid) {
        if (wait_count <= 0) {
                return -1;
        }

        *tid = waiting_recv[wait_head];
        wait_head = (wait_head + 1) % CAN_WAIT_RECV_SIZE;
        wait_count--;
        return 0;
}

static void reply_send_result(int tid, int result) {
        can_reply_t reply;

        // 这个helper initializes the full reply，避免把未初始化的stack bytes回传给client。
        clear_can_reply(&reply);
        reply.result = result;
        reply.frame.extended = 1;
        if (tid > 0) {
                Reply(tid, (const char *)&reply, sizeof(reply));
        }
}

static int reply_recv_frame(int tid, const can_frame_t *frame) {
        can_reply_t reply;
        clear_can_reply(&reply);
        reply.result = 0;
        reply.frame.id = frame->id;
        reply.frame.extended = frame->extended;
        reply.frame.dlc = frame->dlc;
        for (int i = 0; i < 8; ++i) reply.frame.data[i] = frame->data[i];
        return Reply(tid, (const char *)&reply, sizeof(reply));
}

static void fail_all_queued_tx(void) {
        while (tx_count > 0) {
                tx_item_t *item = tx_peek();
                fail_batch(item->batch_token);
                if (item->sender_tid > 0) {
                        reply_send_result(item->sender_tid, -1);
                }
                can_health.tx_failed++;
                tx_pop();
        }
        tx_poll_state.active = 0;
        tx_busy_polls = 0;
        tx_waiting_response = 0;
        response_wait_polls = 0;
}

static void fail_all_waiting_recv(void) {
        int tid;

        while (wait_pop(&tid) == 0) {
                reply_send_result(tid, -1);
        }
}

static void start_queued_tx(void) {
        while (can_health.hw_ready && tx_count > 0 &&
               !tx_poll_state.active && !tx_waiting_response) {
                tx_item_t *item = tx_peek();
                mcp2515_frame_t m = to_mcp_frame(&item->frame);
                int status = mcp2515_send(&m);

                if (status == 0) {
                        CanTxPollStateStart(&tx_poll_state);
                        tx_busy_polls = 0;
                        return;
                }
                if (status == MCP2515_SEND_BUSY) {
                        tx_busy_polls++;
                        if (tx_busy_polls < CAN_TX_TIMEOUT_POLLS) {
                                return;
                        }
                        if (mcp2515_tx_abort() == MCP2515_IO_ERROR) {
                                can_health.hw_ready = 0;
                        }
                        can_health.tx_timeout++;
                } else if (status == MCP2515_SEND_NOT_READY ||
                           status == MCP2515_IO_ERROR) {
                        can_health.hw_ready = 0;
                }

                if (item->sender_tid > 0) {
                        reply_send_result(item->sender_tid, -1);
                }
                unsigned int failed_batch = item->batch_token;
                fail_batch(failed_batch);
                can_health.tx_failed++;
                tx_pop();
                drop_queued_batch_items(failed_batch);
                tx_busy_polls = 0;
        }

        if (!can_health.hw_ready) {
                fail_all_queued_tx();
                fail_all_waiting_recv();
        }
}

static void progress_queued_tx(int timed_poll) {
        if (!can_health.hw_ready) {
                fail_all_queued_tx();
                return;
        }

        if (tx_waiting_response) {
                if (timed_poll) ++response_wait_polls;
                if (timed_poll &&
                    response_wait_polls >=
                    CAN_RESPONSE_TIMEOUT_POLLS) {
                        tx_item_t *item = tx_peek();
                        if (item && item->sender_tid > 0) {
                                reply_send_result(
                                        item->sender_tid, -1);
                        }
                        if (item) {
                                quarantine_active_response(item);
                                unsigned int failed_batch =
                                        item->batch_token;
                                can_health.tx_failed++;
                                can_health.tx_timeout++;
                                fail_batch(failed_batch);
                                tx_pop();
                                drop_queued_batch_items(
                                        failed_batch);
                        }
                        tx_waiting_response = 0;
                        response_wait_polls = 0;
                }
        } else if (tx_poll_state.active) {
                int controller_status = mcp2515_tx_status();
                int transition =
                        !timed_poll &&
                                controller_status ==
                                        MCP2515_TX_PENDING ?
                        CAN_TX_POLL_PENDING :
                        CanTxPollStateObserve(
                                &tx_poll_state,
                                controller_status);
                if (controller_status == MCP2515_IO_ERROR) {
                        can_health.hw_ready = 0;
                }
                if (transition == CAN_TX_POLL_COMPLETE) {
                        tx_item_t *item = tx_peek();
                        if (item) {
                                if (item->require_response) {
                                        tx_waiting_response = 1;
                                        response_wait_polls = 0;
                                } else {
                                        if (item->sender_tid > 0) {
                                                reply_send_result(
                                                        item->sender_tid,
                                                        0);
                                        }
                                        can_health.tx_completed++;
                                        confirm_batch_item(
                                                item->batch_token);
                                        tx_pop();
                                }
                        }
                } else if (transition == CAN_TX_POLL_FAILED ||
                           transition == CAN_TX_POLL_TIMEOUT) {
                        tx_item_t *item = tx_peek();
                        if (mcp2515_tx_abort() == MCP2515_IO_ERROR) {
                                can_health.hw_ready = 0;
                        }
                        if (item) {
                                /*
                                 * A controller error after TXREQ was raised
                                 * can still leave a delayed CS3 response on
                                 * the bus. It must not confirm a later,
                                 * byte-identical safety command.
                                 */
                                quarantine_active_response(item);
                                unsigned int failed_batch =
                                        item->batch_token;
                                if (item->sender_tid > 0) {
                                        reply_send_result(
                                                item->sender_tid, -1);
                                }
                                can_health.tx_failed++;
                                fail_batch(failed_batch);
                                if (transition == CAN_TX_POLL_TIMEOUT) {
                                        can_health.tx_timeout++;
                                }
                                tx_pop();
                                drop_queued_batch_items(
                                        failed_batch);
                        }
                }
        }

        if (!can_health.hw_ready) {
                fail_all_queued_tx();
                fail_all_waiting_recv();
                return;
        }
        start_queued_tx();
}

static uint32_t cs2_id(uint8_t can_id_value) {
        return (((uint32_t)can_id_value) << 17) | 0x4711u;
}

int CanFrameMatchesResponse(const can_frame_t *request,
                            const can_frame_t *response) {
        const uint32_t response_bit = 0x00010000u;
        int normalized_direction_response;
        if (!can_frame_valid(request) || !can_frame_valid(response) ||
            !request->extended || !response->extended ||
            (request->id & response_bit) != 0 ||
            (response->id & response_bit) == 0 ||
            ((request->id >> 17) & 0xffu) !=
                    ((response->id >> 17) & 0xffu) ||
            request->dlc != response->dlc) {
                return 0;
        }
        /*
         * Direction value 3 is a toggle request.  The CS3 acknowledges it
         * with the resulting absolute direction (1 or 2), not by echoing 3.
         * Every address/other payload byte must still match.
         */
        normalized_direction_response =
                ((request->id >> 17) & 0xffu) == 0x05u &&
                request->dlc == 5 && request->data[4] == 3 &&
                (response->data[4] == 1 || response->data[4] == 2);
        for (int byte = 0; byte < request->dlc; ++byte) {
                if (request->data[byte] != response->data[byte] &&
                    !(normalized_direction_response && byte == 4)) {
                        return 0;
                }
        }
        return 1;
}

int CanFrameIsTurnoutChange(const can_frame_t *frame) {
        return can_frame_valid(frame) && frame->extended &&
                ((frame->id >> 17) & 0xffu) == 0x0bu &&
                frame->dlc == 6 && frame->data[5] == 1;
}

static int can_frame_is_train_control(
        const can_frame_t *frame) {
        if (!can_frame_valid(frame) || !frame->extended ||
            (frame->id & 0x00010000u) != 0) {
                return 0;
        }
        unsigned int command =
                (frame->id >> 17) & 0xffu;
        return command == 0x04u || command == 0x05u;
}

static int can_frame_is_absolute_idempotent(
        const can_frame_t *frame) {
        if (!can_frame_valid(frame) || !frame->extended ||
            (frame->id & 0x00010000u) != 0) {
                return 0;
        }
        unsigned int command =
                (frame->id >> 17) & 0xffu;
        return command == 0x04u ||
                (command == 0x0bu &&
                 frame->dlc == 6 &&
                 frame->data[5] == 1);
}

static int can_send_with_type(int tid, const can_frame_t *frame,
                              int type) {
        can_request_t req;
        can_reply_t reply;
        int ret;

        if (tid < 1 || !can_frame_valid(frame)) {
                return -1;
        }
        clear_can_request(&req, type);
        req.frame = *frame;
        ret = Send(tid, (const char *)&req, sizeof(req),
                   (char *)&reply, sizeof(reply));
        if (ret != (int)sizeof(reply)) return -1;
        return reply.result;
}

int CanSend(int tid, const can_frame_t *frame) {
        return can_send_with_type(tid, frame, CAN_MSG_SEND);
}

int CanSendConfirmed(int tid, const can_frame_t *frame) {
        return can_send_with_type(
                tid, frame, CAN_MSG_SEND_CONFIRMED);
}

int CanReceive(int tid, can_frame_t *frame) {
        can_request_t req;
        can_reply_t reply;
        int ret;

        // 这个API validates its output buffer and full reply，避免制造假的sensor frame。
        if (tid < 1 || !frame) {
                return -1;
        }

        clear_can_request(&req, CAN_MSG_RECV);

        ret = Send(tid, (const char *)&req, sizeof(req), (char *)&reply, sizeof(reply));
        if (ret != (int)sizeof(reply) || reply.result < 0 ||
            !can_frame_valid(&reply.frame)) {
                return -1;
        }

        *frame = reply.frame;
        return 0;
}

int CanReceivePoll(int tid, can_frame_t *frame) {
        can_request_t req;
        can_reply_t reply;
        if (tid < 1 || !frame) return -1;
        clear_can_request(&req, CAN_MSG_RECV_POLL);
        int ret = Send(tid, (const char *)&req, sizeof(req),
                       (char *)&reply, sizeof(reply));
        if (ret != (int)sizeof(reply)) return -1;
        if (reply.result == 1) return 1;
        if (reply.result < 0 ||
            !can_frame_valid(&reply.frame)) {
                return -1;
        }
        *frame = reply.frame;
        return 0;
}

int CanGetHealth(int tid, can_health_t *health) {
        can_request_t req;
        int ret;

        if (tid < 1 || !health) {
                return -1;
        }

        clear_can_request(&req, CAN_MSG_HEALTH);
        ret = Send(tid, (const char *)&req, sizeof(req),
                   (char *)health, sizeof(*health));
        return ret == (int)sizeof(*health) ? 0 : -1;
}

int CanRegisterTurnoutAuthority(int tid) {
        can_request_t req;
        can_reply_t reply;
        if (tid < 1) return -1;
        clear_can_request(
                &req, CAN_MSG_REGISTER_TURNOUT_AUTHORITY);
        int ret = Send(tid, (const char *)&req, sizeof(req),
                       (char *)&reply, sizeof(reply));
        return ret == (int)sizeof(reply) ? reply.result : -1;
}

static int build_train_speed_frame(int train, int speed,
                                   can_frame_t *f) {
        uint16_t value;

        if (!f || train < 1 || train > 255 ||
            speed < 0 || speed > 120) {
                return -1;
        }

        if (speed == 0) {
                value = 0x0000;
        } else {
                value = (uint16_t)((speed * 1000) / 120);
        }

        f->id = 0x00085772;
        f->extended = 1;
        f->dlc = 6;
        f->data[0] = 0x00;
        f->data[1] = 0x00;
        f->data[2] = 0x00;
        f->data[3] = (uint8_t)train;
        f->data[4] = (uint8_t)(value >> 8);
        f->data[5] = (uint8_t)value;
        f->data[6] = 0x00;
        f->data[7] = 0x00;
        return 0;
}

int CanTrainSetSpeed(int tid, int train, int speed) {
        can_frame_t f;
        if (build_train_speed_frame(train, speed, &f) < 0) {
                return -1;
        }

        return CanSendConfirmed(tid, &f);
}

int CanTrainSetSpeedPriority(int tid, int train, int speed) {
        can_frame_t f;
        if (build_train_speed_frame(train, speed, &f) < 0) {
                return -1;
        }
        return can_send_with_type(
                tid, &f,
                CAN_MSG_TRAIN_CONTROL_PRIORITY);
}

int CanTrainSetSpeedBatch(int tid, const int *trains,
                          const int *speeds, int count) {
        can_request_t req;
        can_reply_t reply;
        if (tid < 1 || !trains || !speeds || count < 1 ||
            count > CAN_COMMAND_BATCH_MAX) {
                return -1;
        }
        clear_can_request(&req, CAN_MSG_SPEED_BATCH);
        req.frame_count = count;
        for (int index = 0; index < count; ++index) {
                if (build_train_speed_frame(
                            trains[index], speeds[index],
                            &req.frames[index]) < 0) {
                        return -1;
                }
        }
        int ret = Send(tid, (const char *)&req, sizeof(req),
                       (char *)&reply, sizeof(reply));
        return ret == (int)sizeof(reply) ? reply.result : -1;
}

int CanTrainEmergencyStopBatch(int tid, const int *trains,
                               int count) {
        can_request_t req;
        can_reply_t reply;
        if (tid < 1 || !trains || count < 1 ||
            count > CAN_COMMAND_BATCH_MAX) {
                return -1;
        }
        clear_can_request(&req, CAN_MSG_EMERGENCY_STOP_BATCH);
        req.frame_count = count;
        for (int index = 0; index < count; ++index) {
                if (build_train_speed_frame(
                            trains[index], 0,
                            &req.frames[index]) < 0) {
                        return -1;
                }
        }
        int ret = Send(tid, (const char *)&req, sizeof(req),
                       (char *)&reply, sizeof(reply));
        return ret == (int)sizeof(reply) ? reply.result : -1;
}

int CanGetBatchStatus(int tid, unsigned int token,
                      can_batch_status_t *status) {
        can_request_t req;
        int ret;
        if (tid < 1 || token == 0 || !status) return -1;
        clear_can_request(&req, CAN_MSG_BATCH_STATUS);
        req.value = token;
        ret = Send(tid, (const char *)&req, sizeof(req),
                   (char *)status, sizeof(*status));
        return ret == (int)sizeof(*status) &&
                status->token == token &&
                (status->state == CAN_BATCH_PENDING ||
                 status->state == CAN_BATCH_COMPLETE ||
                 status->state == CAN_BATCH_FAILED) ? 0 : -1;
}

int CanCancelBatch(int tid, unsigned int token) {
        can_request_t req;
        can_reply_t reply;
        if (tid < 1 || token == 0) return -1;
        clear_can_request(&req, CAN_MSG_BATCH_CANCEL);
        req.value = token;
        int ret = Send(tid, (const char *)&req, sizeof(req),
                       (char *)&reply, sizeof(reply));
        return ret == (int)sizeof(reply) ? reply.result : -1;
}

static int build_train_reverse_frame(int train, can_frame_t *f) {
        if (!f || train < 1 || train > 255) {
                return -1;
        }

        /*
         * The 8-bit Märklin command is 0x05.  The raw identifier field
         * therefore contains 0x0A after the protocol's response-bit
         * position; cs2_id() performs that shift and must receive 0x05,
         * not the already shifted 0x0A value.
         *
         * Speed control for the lab locomotive uses the MM address
         * 00 00 00 <train>, so direction must address the same locomotive
         * rather than silently switching to a DCC 00 00 C0 namespace.
         */
        f->id = cs2_id(0x05);
        f->extended = 1;
        f->dlc = 5;
        f->data[0] = 0;
        f->data[1] = 0;
        f->data[2] = 0;
        f->data[3] = (uint8_t)train;
        f->data[4] = 3;
        f->data[5] = 0;
        f->data[6] = 0;
        f->data[7] = 0;
        return 0;
}

int CanTrainReverse(int tid, int train) {
        can_frame_t f;
        if (build_train_reverse_frame(train, &f) < 0) {
                return -1;
        }
        return CanSendConfirmed(tid, &f);
}

int CanTrainReversePriority(int tid, int train) {
        can_frame_t f;
        if (build_train_reverse_frame(train, &f) < 0) {
                return -1;
        }
        return can_send_with_type(
                tid, &f,
                CAN_MSG_TRAIN_CONTROL_PRIORITY);
}

static int build_switch_frame(int switch_no, char direction,
                              can_frame_t *f) {
    int can_switch_no;
    int protocol_position;

    /*
     * User-facing switch numbers use the physical labels printed on
     * the Track D diagram.  The CAN accessory address observed on the
     * physical track is one less than the printed label, so the mapping
     * is done here in the CAN layer instead of inside every route table.
     *
     * Example:
     *   user command: sw 8 S
     *   CAN address:  7
     */
    if (!f || switch_no <= 0 || switch_no > 256) {
        return -1;
    }

    if (direction == 'C' || direction == 'c') {
        protocol_position = 0;
    } else if (direction == 'S' || direction == 's') {
        protocol_position = 1;
    } else {
        return -1;
    }

    can_switch_no = switch_no - 1;

    f->id = cs2_id(0x0B);
    f->extended = 1;
    f->dlc = 6;

    f->data[0] = 0x00;
    f->data[1] = 0x00;
    f->data[2] = 0x30;
    f->data[3] = (uint8_t)can_switch_no;
    /*
     * Märklin accessory byte 4 is 0 for curved and 1 for straight.
     * Keep the user-facing C/S spelling at this API boundary.
     */
    f->data[4] = (uint8_t)protocol_position;
    f->data[5] = 1;
    f->data[6] = 0;
    f->data[7] = 0;
    return 0;
}

int CanSwitch(int tid, int switch_no, char direction) {
    can_frame_t f;
    if (build_switch_frame(switch_no, direction, &f) < 0) {
        return -1;
    }

    return CanSendConfirmed(tid, &f);
}

int CanSwitchBatch(int tid, const int *switch_numbers,
                   const char *directions, int count) {
        can_request_t req;
        can_reply_t reply;
        if (tid < 1 || !switch_numbers || !directions ||
            count < 1 || count > CAN_COMMAND_BATCH_MAX) {
                return -1;
        }
        clear_can_request(&req, CAN_MSG_SPEED_BATCH);
        req.frame_count = count;
        for (int index = 0; index < count; ++index) {
                if (build_switch_frame(
                            switch_numbers[index],
                            directions[index],
                            &req.frames[index]) < 0) {
                        return -1;
                }
        }
        int ret = Send(tid, (const char *)&req, sizeof(req),
                       (char *)&reply, sizeof(reply));
        return ret == (int)sizeof(reply) ? reply.result : -1;
}

void CanRxNotifierTask(void) {
        int server_tid = MyParentTid();
        can_request_t req;
        can_reply_t reply;
        mcp2515_frame_t m;
        int ret;

        for (;;) {
                int frames_processed = 0;
                if (AwaitEvent(EVENT_TIMER) < 0) {
                        Yield();
                        continue;
                }
                clear_can_request(
                        &req, CAN_MSG_RX_HEARTBEAT);
                ret = Send(server_tid, (const char *)&req,
                           sizeof(req), (char *)&reply,
                           sizeof(reply));
                if (ret != (int)sizeof(reply) ||
                    reply.result < 0) {
                        continue;
                }

                for (int pass = 0; pass < 2; ++pass) {
                        int overflow = mcp2515_take_rx_overflow();
                        if (overflow == MCP2515_IO_ERROR) {
                                clear_can_request(&req, CAN_MSG_HW_FAILED);
                                ret = Send(server_tid, (const char *)&req,
                                           sizeof(req), (char *)&reply,
                                           sizeof(reply));
                                (void)ret;
                                break;
                        }
                        if (overflow > 0) {
                                clear_can_request(&req, CAN_MSG_RX_OVERFLOW);
                                req.value = (unsigned int)overflow;
                                ret = Send(server_tid, (const char *)&req,
                                           sizeof(req), (char *)&reply,
                                           sizeof(reply));
                                (void)ret;
                        }

                        while (frames_processed <
                               CAN_RX_DRAIN_BUDGET) {
                                int receive_status = mcp2515_recv(&m);
                                if (receive_status == MCP2515_RECV_NONE) {
                                        break;
                                }

                                if (receive_status ==
                                    MCP2515_RECV_INVALID) {
                                        clear_can_request(
                                                &req,
                                                CAN_MSG_RX_DROPPED);
                                        req.value = 1;
                                } else if (receive_status ==
                                           MCP2515_IO_ERROR) {
                                        clear_can_request(
                                                &req,
                                                CAN_MSG_HW_FAILED);
                                        ret = Send(
                                                server_tid,
                                                (const char *)&req,
                                                sizeof(req),
                                                (char *)&reply,
                                                sizeof(reply));
                                        (void)ret;
                                        frames_processed =
                                                CAN_RX_DRAIN_BUDGET;
                                        break;
                                } else if (receive_status == 0) {
                                        clear_can_request(
                                                &req,
                                                CAN_MSG_RX_FRAME);
                                        req.frame = to_can_frame(&m);
                                } else {
                                        break;
                                }

                                ++frames_processed;
                                ret = Send(server_tid,
                                           (const char *)&req,
                                           sizeof(req),
                                           (char *)&reply,
                                           sizeof(reply));
                                /*
                                 * The server records queue, frame, and
                                 * hardware-overflow failures in can_health.
                                 * Never print from this safety hot path: a
                                 * full UART FIFO would freeze the cooperative
                                 * kernel while additional CAN frames arrive.
                                 */
                                (void)ret;
                        }

                        /*
                         * The second pass catches overflow asserted while both
                         * receive buffers were being drained.
                         */
                        if (pass == 0 &&
                            frames_processed <
                                    CAN_RX_DRAIN_BUDGET) {
                                continue;
                        }
                        break;
                }
        }
}

void CanTxNotifierTask(void) {
        int server_tid = MyParentTid();
        can_request_t req;
        can_reply_t reply;

        for (;;) {
                if (AwaitEvent(EVENT_TIMER) < 0) {
                        Yield();
                        continue;
                }

                clear_can_request(&req, CAN_MSG_TX_POLL);

                int ret = Send(server_tid, (const char *)&req, sizeof(req),
                               (char *)&reply, sizeof(reply));
                if (ret != (int)sizeof(reply) || reply.result < 0) {
                        Yield();
                }
        }
}

void CanServerTask(void) {
        int sender_tid;
        can_request_t req;

        reset_can_state();
        if (RegisterAs(CAN_SERVER_NAME) < 0) {
                uart_puts(CONSOLE,
                          "CanServer: failed to register as can0\r\n");
        } else {
                uart_puts(CONSOLE,
                          "CanServer: registered as can0\r\n");
        }
        uart_puts(CONSOLE, "CanServer: initializing hardware\r\n");

        if (can_hw_init_once() != 0) {
                uart_puts(CONSOLE, "CanServer: MCP2515 init failed\r\n");
                uart_puts(CONSOLE, "CanServer: unavailable\r\n");
        } else {
                can_health.hw_ready = 1;
                can_rx_notifier_tid = Create(2, CanRxNotifierTask);
                can_tx_notifier_tid = Create(3, CanTxNotifierTask);
                if (can_rx_notifier_tid < 0 || can_tx_notifier_tid < 0) {
                        can_health.hw_ready = 0;
                        uart_puts(CONSOLE,
                                  "CanServer: notifier creation failed; unavailable\r\n");
                } else {
                        uart_puts(CONSOLE, "CanServer: ready\r\n");
                }
        }

        for (;;) {
                int request_len = Receive(&sender_tid, (char *)&req, sizeof(req));

                // 这个guard rejects truncated CAN requests，避免用旧的stack bytes解析sensor消息。
                if (request_len != (int)sizeof(req)) {
                        reply_send_result(sender_tid, -1);
                        continue;
                }
                /*
                 * Notifier heartbeats, RX delivery, health queries, and
                 * safety requests all provide bounded opportunities to
                 * advance TX. A single dead TX notifier therefore cannot
                 * freeze a launch or emergency stop indefinitely.
                 */
                if (req.type != CAN_MSG_TX_POLL) {
                        progress_queued_tx(0);
                }

                if (req.type == CAN_MSG_RECV) {
                        can_frame_t frame;

                        if (!can_health.hw_ready) {
                                reply_send_result(sender_tid, -1);
                        } else if (rx_pop(&frame) == 0) {
                                reply_recv_frame(sender_tid, &frame);
                        } else if (wait_push(sender_tid) < 0) {
                                reply_send_result(sender_tid, -1);
                        }
                        continue;
                }

                if (req.type == CAN_MSG_RECV_POLL) {
                        can_frame_t frame;
                        if (!can_health.hw_ready) {
                                reply_send_result(sender_tid, -1);
                        } else if (rx_pop(&frame) == 0) {
                                reply_recv_frame(sender_tid, &frame);
                        } else {
                                reply_send_result(sender_tid, 1);
                        }
                        continue;
                }

                if (req.type == CAN_MSG_RX_HEARTBEAT) {
                        if (sender_tid != can_rx_notifier_tid) {
                                reply_send_result(sender_tid, -1);
                        } else {
                                ++can_health
                                        .rx_notifier_heartbeat;
                                reply_send_result(sender_tid, 0);
                        }
                        continue;
                }

                if (req.type == CAN_MSG_RX_FRAME) {
                        int receiver_tid;

                        if (sender_tid != can_rx_notifier_tid ||
                            !can_frame_valid(&req.frame)) {
                                if (sender_tid == can_rx_notifier_tid) {
                                        can_health.rx_dropped++;
                                }
                                reply_send_result(sender_tid, -1);
                                continue;
                        }
                        can_health.rx_received++;

                        tx_item_t *item = tx_peek();
                        if (quarantined_response_valid &&
                            CanFrameMatchesResponse(
                                    &quarantined_response_request,
                                    &req.frame)) {
                                quarantined_response_valid = 0;
                                /*
                                 * A matching acknowledgement also proves an
                                 * identical absolute speed or turnout-state
                                 * retry reached the requested state. Direction
                                 * command 0x05 is a toggle, so it deliberately
                                 * remains quarantined and the ambiguous retry
                                 * fails through the bounded timeout path.
                                 */
                                if (!(item &&
                                      item->require_response &&
                                      tx_waiting_response &&
                                      can_frame_is_absolute_idempotent(
                                              &item->frame) &&
                                      CanFrameMatchesResponse(
                                              &item->frame,
                                              &req.frame))) {
                                        reply_send_result(sender_tid, 0);
                                        continue;
                                }
                        }
                        if (item && item->require_response &&
                            tx_waiting_response &&
                            CanFrameMatchesResponse(
                                    &item->frame, &req.frame)) {
                                tx_poll_state.active = 0;
                                tx_waiting_response = 0;
                                response_wait_polls = 0;
                                if (item->sender_tid > 0) {
                                        reply_send_result(
                                                item->sender_tid, 0);
                                }
                                can_health.tx_completed++;
                                confirm_batch_item(
                                        item->batch_token);
                                tx_pop();
                                start_queued_tx();
                                reply_send_result(sender_tid, 0);
                                continue;
                        }

                        if (CanFrameIsTurnoutChange(
                                    &req.frame)) {
                                ++can_health.turnout_changes;
                        }
                        if (wait_pop(&receiver_tid) == 0) {
                                if (reply_recv_frame(receiver_tid,
                                                     &req.frame) < 0) {
                                        can_health.rx_dropped++;
                                }
                        } else {
                                // Reply result准确反映queue full，notifier才能报告frame loss。
                                int push_status = rx_push(&req.frame);
                                if (push_status < 0) {
                                        can_health.rx_dropped++;
                                }
                                reply_send_result(sender_tid, push_status);
                                continue;
                        }

                        reply_send_result(sender_tid, 0);
                        continue;
                }

                if (req.type == CAN_MSG_HW_FAILED) {
                        if (sender_tid != can_rx_notifier_tid) {
                                reply_send_result(sender_tid, -1);
                                continue;
                        }
                        can_health.hw_ready = 0;
                        fail_all_queued_tx();
                        fail_all_waiting_recv();
                        reply_send_result(sender_tid, 0);
                        continue;
                }

                if (req.type == CAN_MSG_RX_OVERFLOW ||
                    req.type == CAN_MSG_RX_DROPPED) {
                        if (sender_tid != can_rx_notifier_tid) {
                                reply_send_result(sender_tid, -1);
                                continue;
                        }

                        if (req.type == CAN_MSG_RX_OVERFLOW) {
                                unsigned int flags = req.value &
                                        (MCP2515_RX_OVERFLOW_0 |
                                         MCP2515_RX_OVERFLOW_1);
                                unsigned int count =
                                        ((flags & MCP2515_RX_OVERFLOW_0) ?
                                         1u : 0u) +
                                        ((flags & MCP2515_RX_OVERFLOW_1) ?
                                         1u : 0u);
                                can_health.rx_overflow += count;
                                can_health.rx_dropped += count;
                        } else {
                                unsigned int count =
                                        req.value > 0 ? req.value : 1u;
                                can_health.rx_dropped += count;
                        }
                        reply_send_result(sender_tid, 0);
                        continue;
                }

                if (req.type == CAN_MSG_SEND) {
                        if (!can_health.hw_ready ||
                            !can_frame_valid(&req.frame) ||
                            (CanFrameIsTurnoutChange(&req.frame) &&
                             can_turnout_authority_tid >= 0 &&
                             sender_tid !=
                                     can_turnout_authority_tid) ||
                            tx_push(sender_tid, &req.frame,
                                    0, 0, 0) < 0) {
                                can_health.tx_failed++;
                                reply_send_result(sender_tid, -1);
                        } else {
                                start_queued_tx();
                        }
                        continue;
                }

                if (req.type == CAN_MSG_SEND_CONFIRMED ||
                    req.type ==
                            CAN_MSG_TRAIN_CONTROL_PRIORITY) {
                        /*
                         * A braking, creep, or reverse command is part of an
                         * already moving train's safety envelope. Never let
                         * it wait behind speculative launch/turnout setup.
                         * Expected cancellation is carried by batch status,
                         * not recorded as a transport fault for the active
                         * train that requested this priority command.
                         */
                        if (req.type ==
                                    CAN_MSG_TRAIN_CONTROL_PRIORITY &&
                            tx_count >= CAN_TX_Q_SIZE) {
                                cancel_lower_priority_tx(
                                        CAN_TX_PRIORITY_CONTROL, 0);
                        }
                        int valid = can_health.hw_ready &&
                            can_frame_valid(&req.frame) &&
                            (req.type ==
                                     CAN_MSG_SEND_CONFIRMED ||
                             can_frame_is_train_control(
                                     &req.frame)) &&
                            !(CanFrameIsTurnoutChange(&req.frame) &&
                             can_turnout_authority_tid >= 0 &&
                             sender_tid !=
                                     can_turnout_authority_tid);
                        if (!valid) {
                                can_health.tx_failed++;
                                reply_send_result(sender_tid, -1);
                                continue;
                        }
                        int pushed =
                                req.type ==
                                             CAN_MSG_TRAIN_CONTROL_PRIORITY ?
                                     tx_push_priority(
                                             sender_tid,
                                             &req.frame, 1, 0,
                                             CAN_TX_PRIORITY_CONTROL) :
                                     tx_push(
                                             sender_tid,
                                             &req.frame,
                                             1, 0, 0);
                        if (pushed < 0) {
                                reply_send_result(
                                        sender_tid,
                                        CAN_SEND_BUSY);
                        } else {
                                start_queued_tx();
                        }
                        continue;
                }

                if (req.type == CAN_MSG_SPEED_BATCH) {
                        int valid = can_health.hw_ready &&
                                req.frame_count >= 1 &&
                                req.frame_count <=
                                        CAN_COMMAND_BATCH_MAX;
                        for (int index = 0;
                             valid && index < req.frame_count;
                             ++index) {
                                valid = can_frame_valid(
                                        &req.frames[index]);
                                if (valid &&
                                    CanFrameIsTurnoutChange(
                                            &req.frames[index]) &&
                                    can_turnout_authority_tid >= 0 &&
                                    sender_tid !=
                                            can_turnout_authority_tid) {
                                        valid = 0;
                                }
                        }
                        if (!valid) {
                                can_health.tx_failed++;
                                reply_send_result(sender_tid, -1);
                                continue;
                        }
                        if (req.frame_count >
                            CAN_TX_Q_SIZE - tx_count) {
                                reply_send_result(
                                        sender_tid,
                                        CAN_SEND_BUSY);
                                continue;
                        }
                        can_batch_status_t *status =
                                allocate_batch_status(
                                        req.frame_count);
                        if (!status) {
                                can_health.tx_failed++;
                                reply_send_result(sender_tid, -1);
                                continue;
                        }
                        for (int index = 0;
                             index < req.frame_count; ++index) {
                                if (tx_push(
                                            0, &req.frames[index],
                                            1,
                                            status->token, 0) < 0) {
                                        /*
                                         * Capacity and frames were checked
                                         * before mutation, so this branch
                                         * indicates internal corruption.
                                         */
                                        can_health.hw_ready = 0;
                                        break;
                                }
                        }
                        if (!can_health.hw_ready) {
                                fail_all_queued_tx();
                                reply_send_result(sender_tid, -1);
                        } else {
                                start_queued_tx();
                                reply_send_result(
                                        sender_tid,
                                        (int)status->token);
                        }
                        continue;
                }

                if (req.type ==
                    CAN_MSG_REGISTER_TURNOUT_AUTHORITY) {
                        if (can_turnout_authority_tid < 0 ||
                            can_turnout_authority_tid ==
                                    sender_tid) {
                                can_turnout_authority_tid =
                                        sender_tid;
                                reply_send_result(sender_tid, 0);
                        } else {
                                reply_send_result(sender_tid, -1);
                        }
                        continue;
                }

                if (req.type ==
                    CAN_MSG_EMERGENCY_STOP_BATCH) {
                        int valid = can_health.hw_ready &&
                                req.frame_count >= 1 &&
                                req.frame_count <=
                                        CAN_COMMAND_BATCH_MAX;
                        for (int index = 0;
                             valid && index < req.frame_count;
                             ++index) {
                                can_frame_t *frame =
                                        &req.frames[index];
                                valid = can_frame_valid(frame) &&
                                        frame->extended &&
                                        frame->id == 0x00085772u &&
                                        frame->dlc == 6 &&
                                        frame->data[0] == 0 &&
                                        frame->data[1] == 0 &&
                                        frame->data[2] == 0 &&
                                        frame->data[3] > 0 &&
                                        frame->data[4] == 0 &&
                                        frame->data[5] == 0;
                        }
                        if (!valid) {
                                can_health.tx_failed++;
                                reply_send_result(sender_tid, -1);
                                continue;
                        }

                        /*
                         * A safety stop must not wait behind route setup,
                         * launch traffic, or queued creep/reverse/resume
                         * controls. Protocol correctness still requires the
                         * one command already on the wire to finish (or hit
                         * its bounded timeout) before another CS3 command is
                         * sent. Every stop is inserted immediately after
                         * that in-flight head and ahead of lower priorities.
                         */
                        cancel_lower_priority_tx(
                                CAN_TX_PRIORITY_STOP, 0);
                        if (!can_health.hw_ready) {
                                can_health.tx_failed++;
                                fail_all_queued_tx();
                                fail_all_waiting_recv();
                                reply_send_result(sender_tid, -1);
                                continue;
                        }
                        if (req.frame_count >
                            CAN_TX_Q_SIZE - tx_count) {
                                /*
                                 * Existing emergency stops are more valuable
                                 * than a duplicate retry. Reject this request
                                 * without flushing the already active queue.
                                 */
                                start_queued_tx();
                                reply_send_result(
                                        sender_tid,
                                        CAN_SEND_BUSY);
                                continue;
                        }

                        can_batch_status_t *status =
                                allocate_batch_status(
                                        req.frame_count);
                        if (!status) {
                                can_health.tx_failed++;
                                reply_send_result(sender_tid, -1);
                                continue;
                        }
                        for (int index = 0;
                             index < req.frame_count; ++index) {
                                if (tx_push_priority(
                                            0,
                                            &req.frames[index],
                                            1,
                                            status->token,
                                            CAN_TX_PRIORITY_STOP) < 0) {
                                        can_health.hw_ready = 0;
                                        break;
                                }
                        }
                        if (!can_health.hw_ready) {
                                fail_all_queued_tx();
                                fail_all_waiting_recv();
                                reply_send_result(sender_tid, -1);
                        } else {
                                start_queued_tx();
                                reply_send_result(
                                        sender_tid,
                                        (int)status->token);
                        }
                        continue;
                }

                if (req.type == CAN_MSG_BATCH_STATUS) {
                        can_batch_status_t *status =
                                find_batch_status(req.value);
                        if (!status) {
                                can_batch_status_t missing;
                                missing.token = req.value;
                                missing.state = CAN_BATCH_FAILED;
                                missing.total = 0;
                                missing.confirmed = 0;
                                Reply(sender_tid,
                                      (const char *)&missing,
                                      sizeof(missing));
                        } else {
                                Reply(sender_tid,
                                      (const char *)status,
                                      sizeof(*status));
                        }
                        continue;
                }

                if (req.type == CAN_MSG_BATCH_CANCEL) {
                        can_batch_status_t *status =
                                find_batch_status(req.value);
                        if (!status ||
                            status->state != CAN_BATCH_PENDING) {
                                reply_send_result(sender_tid, -1);
                                continue;
                        }
                        tx_item_t *item = tx_peek();
                        if (item &&
                            item->batch_token == req.value) {
                                if (tx_poll_state.active ||
                                    tx_waiting_response) {
                                        quarantine_active_response(
                                                item);
                                }
                                if (tx_poll_state.active &&
                                    mcp2515_tx_abort() ==
                                            MCP2515_IO_ERROR) {
                                        can_health.hw_ready = 0;
                                }
                                tx_poll_state.active = 0;
                                tx_waiting_response = 0;
                                response_wait_polls = 0;
                                tx_busy_polls = 0;
                        }
                        fail_batch(req.value);
                        drop_queued_batch_items(req.value);
                        if (!can_health.hw_ready) {
                                fail_all_queued_tx();
                                fail_all_waiting_recv();
                                reply_send_result(sender_tid, -1);
                        } else {
                                start_queued_tx();
                                reply_send_result(sender_tid, 0);
                        }
                        continue;
                }

                if (req.type == CAN_MSG_TX_POLL) {
                        if (sender_tid != can_tx_notifier_tid) {
                                reply_send_result(sender_tid, -1);
                                continue;
                        }
                        ++can_health.tx_notifier_heartbeat;
                        progress_queued_tx(1);
                        reply_send_result(sender_tid, 0);
                        continue;
                }

                if (req.type == CAN_MSG_HEALTH) {
                        Reply(sender_tid, (const char *)&can_health,
                              sizeof(can_health));
                        continue;
                }

                reply_send_result(sender_tid, -1);
        }
}
