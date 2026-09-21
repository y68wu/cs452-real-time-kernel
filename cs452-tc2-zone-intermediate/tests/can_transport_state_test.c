#include <stddef.h>
#include <stdint.h>
#include <setjmp.h>
#include <stdio.h>
#include <string.h>

#include "can.h"
#include "mcp2515.h"

typedef struct {
        int result;
        can_frame_t frame;
} mirrored_can_reply_t;

typedef struct {
        int type;
        unsigned int value;
        can_frame_t frame;
        int frame_count;
        can_frame_t frames[CAN_COMMAND_BATCH_MAX];
} mirrored_can_request_t;

static int send_calls;
static int force_short_reply;
static can_frame_t last_sent_frame;
static mirrored_can_request_t last_request;
static int run_server_script;
static int server_script_index;
static int server_script_count;
static int captured_status_valid;
static can_batch_status_t captured_status;
static int captured_health_valid;
static can_health_t captured_health;
static int captured_result_valid;
static int captured_result;
static jmp_buf server_script_done;
static mcp2515_frame_t sent_trace[8];
static int sent_trace_count;

#define SERVER_SCRIPT_CAPACITY 40

typedef struct {
        int sender_tid;
        mirrored_can_request_t request;
} scripted_server_request_t;

static scripted_server_request_t
        server_script[SERVER_SCRIPT_CAPACITY];

int Send(int tid, const char *msg, int msglen, char *reply, int rplen) {
        (void)tid;
        send_calls++;
        memset(&last_request, 0, sizeof(last_request));
        if (msg && msglen == (int)sizeof(last_request)) {
                memcpy(&last_request, msg, sizeof(last_request));
        }
        if (msg && msglen >= (int)(sizeof(int) + sizeof(unsigned int) +
                                   sizeof(can_frame_t))) {
                const unsigned char *bytes = (const unsigned char *)msg;
                memcpy(&last_sent_frame,
                       bytes + sizeof(int) + sizeof(unsigned int),
                       sizeof(last_sent_frame));
        }

        if (rplen == (int)sizeof(can_batch_status_t)) {
                can_batch_status_t status;
                status.token = last_request.value;
                status.state = CAN_BATCH_COMPLETE;
                status.total = 2;
                status.confirmed = 2;
                memcpy(reply, &status, sizeof(status));
        } else if (rplen == (int)sizeof(can_health_t)) {
                can_health_t health;
                health.hw_ready = 1;
                health.tx_completed = 11;
                health.tx_failed = 2;
                health.tx_timeout = 1;
                health.rx_received = 20;
                health.rx_dropped = 3;
                health.rx_overflow = 2;
                health.turnout_changes = 4;
                health.tx_notifier_heartbeat = 9;
                health.rx_notifier_heartbeat = 10;
                memcpy(reply, &health, sizeof(health));
        } else {
                mirrored_can_reply_t response;
                memset(&response, 0, sizeof(response));
                response.result =
                        (last_request.type == 10 ||
                         last_request.type == 13) ? 7 :
                        last_request.type == 15 ? 1 : 0;
                memcpy(reply, &response,
                       (size_t)(rplen < (int)sizeof(response) ?
                                rplen : (int)sizeof(response)));
        }
        return force_short_reply ? rplen - 1 : rplen;
}

int Reply(int tid, const char *reply, int rplen) {
        if (run_server_script && tid == 77 && reply &&
            rplen == (int)sizeof(can_batch_status_t)) {
                memcpy(&captured_status, reply,
                       sizeof(captured_status));
                captured_status_valid = 1;
        } else if (run_server_script && tid == 78 && reply &&
                   rplen == (int)sizeof(can_health_t)) {
                memcpy(&captured_health, reply,
                       sizeof(captured_health));
                captured_health_valid = 1;
        } else if (run_server_script && tid == 79 && reply &&
                   rplen == (int)sizeof(mirrored_can_reply_t)) {
                mirrored_can_reply_t result;
                memcpy(&result, reply, sizeof(result));
                captured_result = result.result;
                captured_result_valid = 1;
        }
        return rplen;
}

int Receive(int *tid, char *msg, int msglen) {
        if (run_server_script) {
                if (server_script_index >= server_script_count) {
                        longjmp(server_script_done, 1);
                }
                scripted_server_request_t *step =
                        &server_script[server_script_index++];
                if (msglen < (int)sizeof(step->request)) {
                        return -1;
                }
                *tid = step->sender_tid;
                memcpy(msg, &step->request,
                       sizeof(step->request));
                return (int)sizeof(step->request);
        }
        (void)tid;
        (void)msg;
        return msglen;
}

int AwaitEvent(int event_type) {
        (void)event_type;
        return 0;
}

int MyParentTid(void) { return 1; }
void Yield(void) {}

int Create(int priority, void (*function)(void)) {
        (void)priority;
        (void)function;
        return 1;
}

int RegisterAs(const char *name) {
        (void)name;
        return 0;
}

void uart_puts(size_t line, const char *text) {
        (void)line;
        (void)text;
}

void spi_init(void) {}
int mcp2515_init(void) { return 0; }
int mcp2515_tx_ready(void) { return 1; }
int mcp2515_send(const mcp2515_frame_t *frame) {
        if (frame && sent_trace_count <
                     (int)(sizeof(sent_trace) /
                           sizeof(sent_trace[0]))) {
                sent_trace[sent_trace_count++] = *frame;
        }
        return 0;
}
int mcp2515_tx_status(void) { return MCP2515_TX_COMPLETE; }
int mcp2515_tx_abort(void) { return 0; }
int mcp2515_recv(mcp2515_frame_t *frame) {
        (void)frame;
        return MCP2515_RECV_NONE;
}
int mcp2515_take_rx_overflow(void) { return 0; }

static void append_server_request(
        int sender_tid, const mirrored_can_request_t *request) {
        if (server_script_count >= SERVER_SCRIPT_CAPACITY) {
                return;
        }
        server_script[server_script_count].sender_tid = sender_tid;
        server_script[server_script_count].request = *request;
        ++server_script_count;
}

static mirrored_can_request_t make_single_speed_batch(int train) {
        mirrored_can_request_t request;
        memset(&request, 0, sizeof(request));
        request.type = 10;
        request.frame_count = 1;
        request.frames[0].id = 0x00085772u;
        request.frames[0].extended = 1;
        request.frames[0].dlc = 6;
        request.frames[0].data[3] = (uint8_t)train;
        request.frames[0].data[4] = 0;
        request.frames[0].data[5] = 100;
        return request;
}

static int test_completed_batch_retention(void) {
        mirrored_can_request_t first =
                make_single_speed_batch(1);

        server_script_index = 0;
        server_script_count = 0;
        captured_status_valid = 0;

        /*
         * Fill all 32 TX positions with distinct pending batches. The first
         * one then completes, freeing exactly one TX position and making its
         * status terminal. The following allocation used to scan from slot
         * zero and overwrite token 1 before its owner could query it.
         */
        append_server_request(42, &first);
        for (int train = 2; train <= 32; ++train) {
                mirrored_can_request_t pending =
                        make_single_speed_batch(train);
                append_server_request(42, &pending);
        }

        mirrored_can_request_t response;
        memset(&response, 0, sizeof(response));
        response.type = 3;
        response.frame = first.frames[0];
        response.frame.id =
                (response.frame.id & ~0xffffu) |
                0x00010000u | 0xc300u;
        append_server_request(1, &response);

        mirrored_can_request_t replacement =
                make_single_speed_batch(33);
        append_server_request(42, &replacement);

        mirrored_can_request_t query;
        memset(&query, 0, sizeof(query));
        query.type = 11;
        query.value = 1;
        append_server_request(77, &query);

        run_server_script = 1;
        if (setjmp(server_script_done) == 0) {
                CanServerTask();
        }
        run_server_script = 0;

        if (!captured_status_valid ||
            captured_status.token != 1 ||
            captured_status.state != CAN_BATCH_COMPLETE ||
            captured_status.total != 1 ||
            captured_status.confirmed != 1) {
                fprintf(stderr,
                        "completed batch token was overwritten by the next allocation\n");
                return -1;
        }
        return 0;
}

static int test_capacity_backpressure_is_not_health_failure(void) {
        server_script_index = 0;
        server_script_count = 0;
        captured_result_valid = 0;
        captured_health_valid = 0;
        sent_trace_count = 0;

        for (int train = 1; train <= 32; ++train) {
                mirrored_can_request_t pending =
                        make_single_speed_batch(train);
                append_server_request(42, &pending);
        }
        mirrored_can_request_t busy =
                make_single_speed_batch(33);
        append_server_request(79, &busy);

        mirrored_can_request_t health;
        memset(&health, 0, sizeof(health));
        health.type = 5;
        append_server_request(78, &health);

        run_server_script = 1;
        if (setjmp(server_script_done) == 0) {
                CanServerTask();
        }
        run_server_script = 0;

        if (!captured_result_valid ||
            captured_result != CAN_SEND_BUSY ||
            !captured_health_valid ||
            captured_health.tx_failed != 0 ||
            sent_trace_count != 1) {
                fprintf(stderr,
                        "temporary TX capacity pressure became a transport "
                        "failure\n");
                return -1;
        }
        return 0;
}

static int test_priority_train_control_preempts_setup(void) {
        mirrored_can_request_t request;
        server_script_index = 0;
        server_script_count = 0;
        captured_status_valid = 0;
        captured_health_valid = 0;
        sent_trace_count = 0;

        memset(&request, 0, sizeof(request));
        request.type = 14;
        append_server_request(42, &request);

        memset(&request, 0, sizeof(request));
        request.type = 10;
        request.frame_count = 2;
        request.frames[0].id = 0x00164711u;
        request.frames[0].extended = 1;
        request.frames[0].dlc = 6;
        request.frames[0].data[2] = 0x30;
        request.frames[0].data[3] = 7;
        request.frames[0].data[4] = 1;
        request.frames[0].data[5] = 1;
        request.frames[1] = request.frames[0];
        request.frames[1].data[3] = 8;
        append_server_request(42, &request);
        can_frame_t first_turnout = request.frames[0];
        can_frame_t second_turnout = request.frames[1];

        memset(&request, 0, sizeof(request));
        request.type = 17;
        request.frame.id = 0x00085772u;
        request.frame.extended = 1;
        request.frame.dlc = 6;
        request.frame.data[3] = 14;
        append_server_request(42, &request);
        can_frame_t priority_speed = request.frame;

        memset(&request, 0, sizeof(request));
        request.type = 3;
        request.frame = first_turnout;
        request.frame.id =
                (request.frame.id & ~0xffffu) |
                0x00010000u | 0xc300u;
        append_server_request(1, &request);

        memset(&request, 0, sizeof(request));
        request.type = 3;
        request.frame = priority_speed;
        request.frame.id =
                (request.frame.id & ~0xffffu) |
                0x00010000u | 0xc300u;
        append_server_request(1, &request);

        memset(&request, 0, sizeof(request));
        request.type = 3;
        request.frame = second_turnout;
        request.frame.id =
                (request.frame.id & ~0xffffu) |
                0x00010000u | 0xc300u;
        append_server_request(1, &request);

        memset(&request, 0, sizeof(request));
        request.type = 11;
        request.value = 1;
        append_server_request(77, &request);

        memset(&request, 0, sizeof(request));
        request.type = 5;
        append_server_request(78, &request);

        run_server_script = 1;
        if (setjmp(server_script_done) == 0) {
                CanServerTask();
        }
        run_server_script = 0;

        if (!captured_status_valid ||
            captured_status.token != 1 ||
            captured_status.state != CAN_BATCH_COMPLETE ||
            captured_status.total != 2 ||
            captured_status.confirmed != 2 ||
            sent_trace_count != 3 ||
            sent_trace[0].id != first_turnout.id ||
            sent_trace[0].data[3] != first_turnout.data[3] ||
            sent_trace[1].id != priority_speed.id ||
            sent_trace[1].data[3] != priority_speed.data[3] ||
            sent_trace[2].id != second_turnout.id ||
            sent_trace[2].data[3] != second_turnout.data[3] ||
            !captured_health_valid ||
            captured_health.tx_failed != 0) {
                fprintf(stderr,
                        "priority train control did not run immediately "
                        "after the in-flight turnout\n");
                return -1;
        }
        return 0;
}

static int test_emergency_stop_precedes_queued_control(void) {
        mirrored_can_request_t request;
        server_script_index = 0;
        server_script_count = 0;
        captured_status_valid = 0;
        captured_health_valid = 0;
        sent_trace_count = 0;

        memset(&request, 0, sizeof(request));
        request.type = 14;
        append_server_request(42, &request);

        memset(&request, 0, sizeof(request));
        request.type = 10;
        request.frame_count = 2;
        request.frames[0].id = 0x00164711u;
        request.frames[0].extended = 1;
        request.frames[0].dlc = 6;
        request.frames[0].data[2] = 0x30;
        request.frames[0].data[3] = 7;
        request.frames[0].data[4] = 1;
        request.frames[0].data[5] = 1;
        request.frames[1] = request.frames[0];
        request.frames[1].data[3] = 8;
        append_server_request(42, &request);
        can_frame_t active_turnout = request.frames[0];

        memset(&request, 0, sizeof(request));
        request.type = 17;
        request.frame.id = 0x000a4711u;
        request.frame.extended = 1;
        request.frame.dlc = 5;
        request.frame.data[3] = 14;
        request.frame.data[4] = 3;
        append_server_request(42, &request);
        can_frame_t queued_reverse = request.frame;

        memset(&request, 0, sizeof(request));
        request.type = 13;
        request.frame_count = 1;
        request.frames[0].id = 0x00085772u;
        request.frames[0].extended = 1;
        request.frames[0].dlc = 6;
        request.frames[0].data[3] = 14;
        append_server_request(42, &request);
        can_frame_t emergency_stop = request.frames[0];

        memset(&request, 0, sizeof(request));
        request.type = 3;
        request.frame = active_turnout;
        request.frame.id =
                (request.frame.id & ~0xffffu) |
                0x00010000u | 0xc300u;
        append_server_request(1, &request);

        memset(&request, 0, sizeof(request));
        request.type = 3;
        request.frame = emergency_stop;
        request.frame.id =
                (request.frame.id & ~0xffffu) |
                0x00010000u | 0xc300u;
        append_server_request(1, &request);

        memset(&request, 0, sizeof(request));
        request.type = 3;
        request.frame = queued_reverse;
        request.frame.id =
                (request.frame.id & ~0xffffu) |
                0x00010000u | 0xc300u;
        append_server_request(1, &request);

        memset(&request, 0, sizeof(request));
        request.type = 11;
        request.value = 2;
        append_server_request(77, &request);

        memset(&request, 0, sizeof(request));
        request.type = 5;
        append_server_request(78, &request);

        run_server_script = 1;
        if (setjmp(server_script_done) == 0) {
                CanServerTask();
        }
        run_server_script = 0;

        if (sent_trace_count != 2 ||
            sent_trace[0].id != active_turnout.id ||
            sent_trace[1].id != emergency_stop.id ||
            !captured_status_valid ||
            captured_status.token != 2 ||
            captured_status.state != CAN_BATCH_COMPLETE ||
            captured_status.confirmed != 1 ||
            !captured_health_valid ||
            captured_health.tx_failed != 0) {
                fprintf(stderr,
                        "emergency stop did not cancel queued direction "
                        "and setup traffic\n");
                return -1;
        }
        return 0;
}

static int test_idempotent_stop_retry_survives_quarantine(void) {
        mirrored_can_request_t request;
        server_script_index = 0;
        server_script_count = 0;
        captured_status_valid = 0;
        sent_trace_count = 0;

        memset(&request, 0, sizeof(request));
        request.type = 13;
        request.frame_count = 1;
        request.frames[0].id = 0x00085772u;
        request.frames[0].extended = 1;
        request.frames[0].dlc = 6;
        request.frames[0].data[3] = 14;
        append_server_request(42, &request);
        can_frame_t stop = request.frames[0];

        memset(&request, 0, sizeof(request));
        request.type = 4;
        for (int poll = 0; poll < 26; ++poll) {
                append_server_request(1, &request);
        }

        memset(&request, 0, sizeof(request));
        request.type = 13;
        request.frame_count = 1;
        request.frames[0] = stop;
        append_server_request(42, &request);

        memset(&request, 0, sizeof(request));
        request.type = 3;
        request.frame = stop;
        request.frame.id =
                (request.frame.id & ~0xffffu) |
                0x00010000u | 0xc300u;
        append_server_request(1, &request);

        memset(&request, 0, sizeof(request));
        request.type = 11;
        request.value = 2;
        append_server_request(77, &request);

        run_server_script = 1;
        if (setjmp(server_script_done) == 0) {
                CanServerTask();
        }
        run_server_script = 0;

        if (sent_trace_count != 2 ||
            sent_trace[0].id != stop.id ||
            sent_trace[1].id != stop.id ||
            !captured_status_valid ||
            captured_status.token != 2 ||
            captured_status.state != CAN_BATCH_COMPLETE ||
            captured_status.confirmed != 1) {
                fprintf(stderr,
                        "stale-response quarantine poisoned an identical "
                        "emergency-stop retry\n");
                return -1;
        }
        return 0;
}

static int test_poll_state(void) {
        can_tx_poll_state_t state;

        CanTxPollStateInit(&state, 3);
        if (state.active || state.poll_limit != 3 ||
            CanTxPollStateObserve(&state, MCP2515_TX_PENDING) !=
                    CAN_TX_POLL_FAILED) {
                fprintf(stderr, "inactive poll state was accepted\n");
                return -1;
        }

        CanTxPollStateStart(&state);
        if (CanTxPollStateObserve(&state, MCP2515_TX_PENDING) !=
                    CAN_TX_POLL_PENDING ||
            CanTxPollStateObserve(&state, MCP2515_TX_PENDING) !=
                    CAN_TX_POLL_PENDING ||
            CanTxPollStateObserve(&state, MCP2515_TX_PENDING) !=
                    CAN_TX_POLL_TIMEOUT ||
            state.active) {
                fprintf(stderr, "pending TX did not reach bounded timeout\n");
                return -1;
        }

        CanTxPollStateStart(&state);
        if (CanTxPollStateObserve(&state, MCP2515_TX_COMPLETE) !=
                    CAN_TX_POLL_COMPLETE ||
            state.active) {
                fprintf(stderr, "TX completion transition failed\n");
                return -1;
        }

        CanTxPollStateStart(&state);
        if (CanTxPollStateObserve(&state, MCP2515_TX_FAILED) !=
                    CAN_TX_POLL_FAILED ||
            state.active) {
                fprintf(stderr, "TX controller error transition failed\n");
                return -1;
        }

        CanTxPollStateStart(&state);
        if (CanTxPollStateObserve(&state, MCP2515_IO_ERROR) !=
                    CAN_TX_POLL_FAILED ||
            state.active) {
                fprintf(stderr, "SPI I/O failure remained pending\n");
                return -1;
        }
        return 0;
}

static int test_client_validation(void) {
        can_frame_t frame;
        can_health_t health;
        can_batch_status_t batch_status;

        memset(&frame, 0, sizeof(frame));
        frame.id = 0x321u;
        frame.extended = 0;
        frame.dlc = 1;

        send_calls = 0;
        if (CanSend(0, &frame) != -1 || send_calls != 0) {
                fprintf(stderr, "invalid CAN tid reached IPC\n");
                return -1;
        }
        frame.dlc = 9;
        if (CanSend(1, &frame) != -1 || send_calls != 0) {
                fprintf(stderr, "invalid DLC reached IPC\n");
                return -1;
        }
        frame.dlc = 1;
        frame.id = 0x800u;
        if (CanSend(1, &frame) != -1 || send_calls != 0) {
                fprintf(stderr, "oversized standard id reached IPC\n");
                return -1;
        }
        frame.id = 0x321u;
        frame.extended = 2;
        if (CanSend(1, &frame) != -1 || send_calls != 0) {
                fprintf(stderr, "invalid frame format reached IPC\n");
                return -1;
        }

        frame.extended = 0;
        force_short_reply = 0;
        if (CanSend(1, &frame) != 0 || send_calls != 1 ||
            last_sent_frame.id != frame.id ||
            last_sent_frame.extended != 0) {
                fprintf(stderr, "valid standard CAN send failed\n");
                return -1;
        }
        force_short_reply = 1;
        if (CanSend(1, &frame) != -1) {
                fprintf(stderr, "short CAN reply was accepted\n");
                return -1;
        }
        force_short_reply = 0;
        if (CanReceivePoll(1, &frame) != 1) {
                fprintf(stderr,
                        "empty nonblocking CAN receive was malformed\n");
                return -1;
        }

        send_calls = 0;
        if (CanTrainSetSpeed(1, 0, 0) != -1 ||
            CanTrainSetSpeed(1, 256, 0) != -1 ||
            CanTrainSetSpeedPriority(1, 0, 0) != -1 ||
            CanTrainReverse(1, 0) != -1 ||
            CanTrainReversePriority(1, 0) != -1 ||
            send_calls != 0) {
                fprintf(stderr, "train address outside 1..255 reached IPC\n");
                return -1;
        }
        if (CanTrainSetSpeed(1, 1, 0) != 0 || send_calls != 1 ||
            last_sent_frame.id != 0x00085772u ||
            !last_sent_frame.extended || last_sent_frame.dlc != 6 ||
            last_sent_frame.data[0] != 0 ||
            last_sent_frame.data[1] != 0 ||
            last_sent_frame.data[2] != 0 ||
            last_sent_frame.data[3] != 1 ||
            last_sent_frame.data[4] != 0 ||
            last_sent_frame.data[5] != 0) {
                fprintf(stderr, "valid MM train stop frame was malformed\n");
                return -1;
        }
        if (CanTrainSetSpeedPriority(1, 1, 0) != 0 ||
            last_request.type != 17) {
                fprintf(stderr,
                        "priority train stop IPC was malformed\n");
                return -1;
        }
        if (CanTrainSetSpeed(1, 255, 120) != 0 ||
            last_sent_frame.data[3] != 255 ||
            last_sent_frame.data[4] != 0x03 ||
            last_sent_frame.data[5] != 0xe8) {
                fprintf(stderr, "maximum train speed frame was malformed\n");
                return -1;
        }
        if (CanTrainReverse(1, 14) != 0 ||
            last_sent_frame.id != 0x000a4711u ||
            !last_sent_frame.extended || last_sent_frame.dlc != 5 ||
            last_sent_frame.data[0] != 0 ||
            last_sent_frame.data[1] != 0 ||
            last_sent_frame.data[2] != 0 ||
            last_sent_frame.data[3] != 14 ||
            last_sent_frame.data[4] != 3) {
                fprintf(stderr,
                        "Märklin direction command/address was malformed\n");
                return -1;
        }
        if (CanTrainReversePriority(1, 14) != 0 ||
            last_request.type != 17 ||
            last_sent_frame.id != 0x000a4711u) {
                fprintf(stderr,
                        "priority direction IPC was malformed\n");
                return -1;
        }
        {
                can_frame_t toggle_request = last_sent_frame;
                can_frame_t absolute_response = toggle_request;
                absolute_response.id =
                        (toggle_request.id & ~0xffffu) |
                        0x00010000u | 0xc300u;
                absolute_response.data[4] = 2;
                if (!CanFrameMatchesResponse(
                            &toggle_request,
                            &absolute_response)) {
                        fprintf(stderr,
                                "normalized reverse acknowledgement was rejected\n");
                        return -1;
                }
                absolute_response.data[4] = 1;
                if (!CanFrameMatchesResponse(
                            &toggle_request,
                            &absolute_response)) {
                        fprintf(stderr,
                                "normalized forward acknowledgement was rejected\n");
                        return -1;
                }
                absolute_response.data[4] = 0;
                if (CanFrameMatchesResponse(
                            &toggle_request,
                            &absolute_response)) {
                        fprintf(stderr,
                                "invalid direction acknowledgement was accepted\n");
                        return -1;
                }
                absolute_response.data[4] = 2;
                absolute_response.data[3] = 15;
                if (CanFrameMatchesResponse(
                            &toggle_request,
                            &absolute_response)) {
                        fprintf(stderr,
                                "another train's direction acknowledgement was accepted\n");
                        return -1;
                }
        }
        if (CanSwitch(1, 8, 'S') != 0 ||
            last_sent_frame.id != 0x00164711u ||
            last_sent_frame.dlc != 6 ||
            last_sent_frame.data[2] != 0x30 ||
            last_sent_frame.data[3] != 7 ||
            last_sent_frame.data[4] != 1 ||
            last_sent_frame.data[5] != 1) {
                fprintf(stderr, "turnout frame was malformed\n");
                return -1;
        }
        if (CanSwitch(1, 8, 'C') != 0 ||
            last_sent_frame.data[4] != 0) {
                fprintf(stderr, "curved turnout position was reversed\n");
                return -1;
        }
        int switch_numbers[3] = {153, 154, 8};
        char switch_directions[3] = {'S', 'C', 'S'};
        if (CanSwitchBatch(
                    1, switch_numbers, switch_directions, 3) != 7 ||
            last_request.type != 10 ||
            last_request.frame_count != 3 ||
            last_request.frames[0].data[3] != 152 ||
            last_request.frames[0].data[4] != 1 ||
            last_request.frames[1].data[3] != 153 ||
            last_request.frames[1].data[4] != 0 ||
            last_request.frames[2].data[3] != 7 ||
            last_request.frames[2].data[4] != 1) {
                fprintf(stderr,
                        "asynchronous turnout batch was malformed\n");
            return -1;
        }
        last_sent_frame = last_request.frames[2];

        can_frame_t request = last_sent_frame;
        can_frame_t response = request;
        response.id =
                (request.id & ~0xffffu) |
                0x00010000u | 0xc300u;
        if (!CanFrameMatchesResponse(&request, &response)) {
                fprintf(stderr, "matching CS3 response was rejected\n");
                return -1;
        }
        response.data[4] ^= 1u;
        if (CanFrameMatchesResponse(&request, &response)) {
                fprintf(stderr, "wrong response payload was accepted\n");
                return -1;
        }
        response = request;
        if (CanFrameMatchesResponse(&request, &response)) {
                fprintf(stderr, "request without response bit matched\n");
                return -1;
        }
        response.id =
                (request.id & ~0xffffu) |
                0x00010000u | 0xc300u;
        if (!CanFrameIsTurnoutChange(&request) ||
            !CanFrameIsTurnoutChange(&response)) {
                fprintf(stderr,
                        "engaged turnout traffic was not classified\n");
                return -1;
        }
        response.data[5] = 0;
        if (CanFrameMatchesResponse(&request, &response)) {
                fprintf(stderr,
                        "switch solenoid-off event matched confirmation\n");
                return -1;
        }
        if (CanFrameIsTurnoutChange(&response)) {
                fprintf(stderr,
                        "turnout solenoid-off event counted as a change\n");
                return -1;
        }

        int trains[2] = {14, 15};
        int speeds[2] = {30, 60};
        send_calls = 0;
        if (CanTrainSetSpeedBatch(
                    1, trains, speeds, 2) != 7 ||
            send_calls != 1 ||
            last_request.frame_count != 2 ||
            last_request.frames[0].data[3] != 14 ||
            last_request.frames[1].data[3] != 15 ||
            last_request.frames[0].data[4] != 0 ||
            last_request.frames[0].data[5] != 0xfa ||
            last_request.frames[1].data[4] != 0x01 ||
            last_request.frames[1].data[5] != 0xf4) {
                fprintf(stderr,
                        "atomic speed batch was malformed\n");
                return -1;
        }
        if (CanGetBatchStatus(1, 7, &batch_status) < 0 ||
            batch_status.state != CAN_BATCH_COMPLETE ||
            batch_status.total != 2 ||
            batch_status.confirmed != 2 ||
            CanCancelBatch(1, 7) < 0) {
                fprintf(stderr, "batch status/cancel IPC failed\n");
                return -1;
        }
        send_calls = 0;
        if (CanTrainEmergencyStopBatch(1, trains, 2) != 7 ||
            send_calls != 1 ||
            last_request.type != 13 ||
            last_request.frame_count != 2 ||
            last_request.frames[0].id != 0x00085772u ||
            last_request.frames[0].data[3] != 14 ||
            last_request.frames[1].data[3] != 15 ||
            last_request.frames[0].data[4] != 0 ||
            last_request.frames[0].data[5] != 0 ||
            last_request.frames[1].data[4] != 0 ||
            last_request.frames[1].data[5] != 0) {
                fprintf(stderr,
                        "priority emergency-stop batch was malformed\n");
                return -1;
        }
        int calls_before_invalid = send_calls;
        if (CanTrainSetSpeedBatch(1, trains, speeds, 0) != -1 ||
            CanTrainEmergencyStopBatch(1, trains, 0) != -1 ||
            CanGetBatchStatus(1, 0, &batch_status) != -1 ||
            CanCancelBatch(1, 0) != -1 ||
            send_calls != calls_before_invalid) {
                fprintf(stderr,
                        "invalid batch input reached IPC\n");
                return -1;
        }

        if (CanGetHealth(0, &health) != -1 ||
            CanGetHealth(1, 0) != -1) {
                fprintf(stderr, "invalid health request was accepted\n");
                return -1;
        }
        send_calls = 0;
        if (CanRegisterTurnoutAuthority(1) != 0 ||
            send_calls != 1 || last_request.type != 14 ||
            CanRegisterTurnoutAuthority(0) != -1 ||
            send_calls != 1) {
                fprintf(stderr,
                        "turnout-authority registration IPC failed\n");
                return -1;
        }
        if (CanGetHealth(1, &health) != 0 ||
            health.hw_ready != 1 || health.tx_completed != 11 ||
            health.tx_timeout != 1 || health.rx_overflow != 2 ||
            health.turnout_changes != 4 ||
            health.tx_notifier_heartbeat != 9 ||
            health.rx_notifier_heartbeat != 10) {
                fprintf(stderr, "CAN health snapshot reply failed\n");
                return -1;
        }
        force_short_reply = 1;
        if (CanGetHealth(1, &health) != -1) {
                fprintf(stderr, "short CAN health reply was accepted\n");
                return -1;
        }
        force_short_reply = 0;
        return 0;
}

int main(void) {
        if (test_poll_state() < 0 ||
            test_client_validation() < 0 ||
            test_priority_train_control_preempts_setup() < 0 ||
            test_emergency_stop_precedes_queued_control() < 0 ||
            test_idempotent_stop_retry_survives_quarantine() < 0 ||
            test_capacity_backpressure_is_not_health_failure() < 0 ||
            test_completed_batch_retention() < 0) {
                return 1;
        }
        printf("validated CAN timeout/quarantine, exact responses, stop priority, queue backpressure, retained batches, frame bytes, and health IPC\n");
        return 0;
}
