#include <setjmp.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include "events.h"
#include "terminal.h"
#include "tasks.h"

#define TERM_MSG_GETC     1
#define TERM_MSG_PUTC     2
#define TERM_MSG_RX_READY 3
#define TERM_MSG_TX_READY 4
#define TERM_MSG_TRY_GETC 5
#define TERM_MSG_TRY_WRITE 6

#define TERM_RESULT_OK    0
#define TERM_RESULT_EMPTY 1
#define TERM_RESULT_WOULD_BLOCK 2
#define EMPTY_POLL_COUNT 17

typedef struct {
        int type;
        unsigned char ch;
} mirrored_terminal_request_t;

typedef struct {
        int result;
        unsigned char ch;
} mirrored_terminal_reply_t;

typedef struct {
        int type;
        unsigned int length;
        unsigned char bytes[TERMINAL_TRY_WRITE_CAPACITY];
} mirrored_terminal_try_write_request_t;

typedef union {
        mirrored_terminal_request_t basic;
        mirrored_terminal_try_write_request_t try_write;
} mirrored_terminal_any_request_t;

#define TRY_WRITE_HEADER_SIZE \
        ((int)offsetof(mirrored_terminal_try_write_request_t, bytes))

typedef struct {
        int tid;
        mirrored_terminal_reply_t reply;
} reply_record_t;

enum server_scenario {
        SERVER_SCENARIO_BASE,
        SERVER_SCENARIO_BULK
};

#define BULK_FIRST_LENGTH 192
#define BULK_REJECTED_LENGTH 65
#define BULK_FILL_LENGTH 64
#define BULK_FIRST_DRAIN_LENGTH 128
#define BULK_WRAP_LENGTH 128
#define BULK_FINAL_DRAIN_LENGTH 256
#define BULK_MALFORMED_COUNT 4
#define BULK_REQUEST_COUNT \
        (4 + BULK_FIRST_DRAIN_LENGTH + 1 + \
         BULK_FINAL_DRAIN_LENGTH + BULK_MALFORMED_COUNT)

static const struct {
        int tid;
        int length;
        mirrored_terminal_request_t request;
} server_tail_requests[] = {
        {10, (int)sizeof(mirrored_terminal_request_t),
         {TERM_MSG_GETC, 0}},
        {12, (int)sizeof(mirrored_terminal_request_t),
         {TERM_MSG_GETC, 0}},
        {40, (int)sizeof(mirrored_terminal_request_t),
         {TERM_MSG_TRY_GETC, 0}},
        {44, (int)sizeof(mirrored_terminal_request_t),
         {TERM_MSG_TRY_GETC, 0}},
        {101, (int)sizeof(mirrored_terminal_request_t),
         {TERM_MSG_RX_READY, 'Q'}},
        {101, (int)sizeof(mirrored_terminal_request_t),
         {TERM_MSG_RX_READY, 'R'}},
        {101, (int)sizeof(mirrored_terminal_request_t),
         {TERM_MSG_RX_READY, 0x00}},
        {41, (int)sizeof(mirrored_terminal_request_t),
         {TERM_MSG_TRY_GETC, 0}},
        {101, (int)sizeof(mirrored_terminal_request_t),
         {TERM_MSG_RX_READY, 0xff}},
        {42, (int)sizeof(mirrored_terminal_request_t),
         {TERM_MSG_TRY_GETC, 0}},
        {43, (int)sizeof(mirrored_terminal_request_t),
         {TERM_MSG_TRY_GETC, 0}},
        {102, (int)sizeof(mirrored_terminal_request_t),
         {TERM_MSG_TX_READY, 0}},
        {11, (int)sizeof(mirrored_terminal_request_t),
         {TERM_MSG_PUTC, 'X'}},
        {13, 1, {TERM_MSG_GETC, 0}}
};

static jmp_buf server_done;
static enum server_scenario active_server_scenario;
static int receive_index;
static reply_record_t replies[512];
static int reply_count;
static int create_count;
static int client_short_reply;
static int client_reply_result;
static unsigned char client_reply_ch;
static int notifier_failure_mode;
static int notifier_send_calls;
static int notifier_timer_waits;
static int notifier_wrong_event;
static int notifier_yields;
static int create_wrong_priority;
static int client_last_request_type;
static int client_last_request_length;
static int client_send_calls;
static unsigned int client_last_bulk_length;
static unsigned char client_last_bulk_bytes[TERMINAL_TRY_WRITE_CAPACITY];

#ifdef MODE_TC2
#define EXPECTED_TERMINAL_NOTIFIER_PRIORITY \
        TC2_TERMINAL_NOTIFIER_PRIORITY
#else
#define EXPECTED_TERMINAL_NOTIFIER_PRIORITY 3
#endif

int RegisterAs(const char *name) {
        return name && strcmp(name, TERMINAL_SERVER_NAME) == 0 ? 0 : -1;
}

int Create(int priority, void (*function)(void)) {
        (void)function;
        if (priority != EXPECTED_TERMINAL_NOTIFIER_PRIORITY) {
                create_wrong_priority = 1;
                return -1;
        }
        create_count++;
        return create_count == 1 ? 101 : 102;
}

static unsigned char first_bulk_byte(int index) {
        return (unsigned char)(0x10 + (index % 181));
}

static unsigned char fill_bulk_byte(int index) {
        return (unsigned char)(0xa0 + (index % 47));
}

static unsigned char wrap_bulk_byte(int index) {
        return (unsigned char)(0x40 + (index % 113));
}

static int receive_bulk_request(
        int *tid, char *message, int message_length) {
        mirrored_terminal_any_request_t request;
        int request_length;
        int i;
        int index = receive_index;
        int first_drain_end = 4 + BULK_FIRST_DRAIN_LENGTH;
        int wrap_index = first_drain_end;
        int final_drain_start = wrap_index + 1;
        int final_drain_end =
                final_drain_start + BULK_FINAL_DRAIN_LENGTH;

        memset(&request, 0, sizeof(request));
        if (index >= BULK_REQUEST_COUNT) {
                longjmp(server_done, 1);
        }
        receive_index++;

        if (index == 0) {
                *tid = 30;
                request.try_write.type = TERM_MSG_TRY_WRITE;
                request.try_write.length = BULK_FIRST_LENGTH;
                for (i = 0; i < BULK_FIRST_LENGTH; i++) {
                        request.try_write.bytes[i] = first_bulk_byte(i);
                }
                request_length =
                        TRY_WRITE_HEADER_SIZE + BULK_FIRST_LENGTH;
        } else if (index == 1) {
                *tid = 31;
                request.try_write.type = TERM_MSG_TRY_WRITE;
                request.try_write.length = BULK_REJECTED_LENGTH;
                memset(request.try_write.bytes, 0xee,
                       BULK_REJECTED_LENGTH);
                request_length =
                        TRY_WRITE_HEADER_SIZE + BULK_REJECTED_LENGTH;
        } else if (index == 2) {
                *tid = 32;
                request.try_write.type = TERM_MSG_TRY_WRITE;
                request.try_write.length = BULK_FILL_LENGTH;
                for (i = 0; i < BULK_FILL_LENGTH; i++) {
                        request.try_write.bytes[i] = fill_bulk_byte(i);
                }
                request_length =
                        TRY_WRITE_HEADER_SIZE + BULK_FILL_LENGTH;
        } else if (index == 3) {
                *tid = 33;
                request.try_write.type = TERM_MSG_TRY_WRITE;
                request.try_write.length = 1;
                request.try_write.bytes[0] = 0xef;
                request_length = TRY_WRITE_HEADER_SIZE + 1;
        } else if (index < first_drain_end) {
                *tid = 102;
                request.basic.type = TERM_MSG_TX_READY;
                request_length = (int)sizeof(request.basic);
        } else if (index == wrap_index) {
                *tid = 34;
                request.try_write.type = TERM_MSG_TRY_WRITE;
                request.try_write.length = BULK_WRAP_LENGTH;
                for (i = 0; i < BULK_WRAP_LENGTH; i++) {
                        request.try_write.bytes[i] = wrap_bulk_byte(i);
                }
                request_length =
                        TRY_WRITE_HEADER_SIZE + BULK_WRAP_LENGTH;
        } else if (index < final_drain_end) {
                *tid = 102;
                request.basic.type = TERM_MSG_TX_READY;
                request_length = (int)sizeof(request.basic);
        } else {
                int malformed = index - final_drain_end;
                *tid = 40 + malformed;
                request.try_write.type = TERM_MSG_TRY_WRITE;
                if (malformed == 0) {
                        request_length = (int)sizeof(int);
                } else if (malformed == 1) {
                        request.try_write.length = 0;
                        request_length = TRY_WRITE_HEADER_SIZE;
                } else if (malformed == 2) {
                        request.try_write.length =
                                TERMINAL_TRY_WRITE_CAPACITY + 1U;
                        request_length = TRY_WRITE_HEADER_SIZE;
                } else {
                        request.try_write.length = 10;
                        request_length = TRY_WRITE_HEADER_SIZE + 9;
                }
        }

        memcpy(message, &request,
               (size_t)(request_length < message_length ?
                        request_length : message_length));
        return request_length;
}

int Receive(int *tid, char *message, int message_length) {
        int tail_index;
        mirrored_terminal_any_request_t request;
        int request_length;

        if (active_server_scenario == SERVER_SCENARIO_BULK) {
                return receive_bulk_request(tid, message, message_length);
        }

        memset(&request, 0, sizeof(request));
        if (receive_index < EMPTY_POLL_COUNT) {
                *tid = 20 + receive_index;
                request.basic.type = TERM_MSG_TRY_GETC;
                request.basic.ch = 0;
                request_length = (int)sizeof(request.basic);
                receive_index++;
        } else {
                tail_index = receive_index - EMPTY_POLL_COUNT;
                if (tail_index >=
                    (int)(sizeof(server_tail_requests) /
                          sizeof(server_tail_requests[0]))) {
                        longjmp(server_done, 1);
                }
                *tid = server_tail_requests[tail_index].tid;
                request.basic =
                        server_tail_requests[tail_index].request;
                request_length = server_tail_requests[tail_index].length;
                receive_index++;
        }

        if (request_length > message_length) {
                memcpy(message, &request, (size_t)message_length);
        } else {
                memcpy(message, &request, (size_t)request_length);
        }
        return request_length;
}

int Reply(int tid, const char *reply, int reply_length) {
        if (reply_count >= (int)(sizeof(replies) / sizeof(replies[0])) ||
            reply_length != (int)sizeof(mirrored_terminal_reply_t)) {
                return -1;
        }
        replies[reply_count].tid = tid;
        memcpy(&replies[reply_count].reply, reply,
               sizeof(replies[reply_count].reply));
        reply_count++;
        return reply_length;
}

int Send(int tid, const char *message, int message_length,
         char *reply, int reply_length) {
        (void)tid;
        ++client_send_calls;
        client_last_request_type = message_length >= (int)sizeof(int) ?
                ((const mirrored_terminal_request_t *)message)->type : -1;
        client_last_request_length = message_length;
        client_last_bulk_length = 0;
        if (client_last_request_type == TERM_MSG_TRY_WRITE &&
            message_length >= TRY_WRITE_HEADER_SIZE) {
                const mirrored_terminal_try_write_request_t *bulk =
                        (const mirrored_terminal_try_write_request_t *)
                        message;
                unsigned int copy_length = bulk->length;
                client_last_bulk_length = bulk->length;
                if (copy_length > TERMINAL_TRY_WRITE_CAPACITY) {
                        copy_length = TERMINAL_TRY_WRITE_CAPACITY;
                }
                if (message_length - TRY_WRITE_HEADER_SIZE <
                    (int)copy_length) {
                        copy_length = (unsigned int)(
                                message_length -
                                TRY_WRITE_HEADER_SIZE);
                }
                memcpy(client_last_bulk_bytes, bulk->bytes,
                       copy_length);
        }
        if (notifier_failure_mode) {
                ++notifier_send_calls;
                if (notifier_send_calls > 3) {
                        longjmp(server_done, 3);
                }
                return -1;
        }
        mirrored_terminal_reply_t response;
        response.result = client_reply_result;
        response.ch = client_reply_ch;
        if (reply_length >= (int)sizeof(response)) {
                memcpy(reply, &response, sizeof(response));
        }
        return client_short_reply ? reply_length - 1 : reply_length;
}

int MyParentTid(void) { return 1; }
int AwaitEvent(int event_type) {
        if (notifier_failure_mode) {
                if (event_type == EVENT_TIMER) {
                        ++notifier_timer_waits;
                } else {
                        ++notifier_wrong_event;
                }
        }
        return 0;
}
void Yield(void) {
        if (notifier_failure_mode) ++notifier_yields;
}
void Exit(void) { longjmp(server_done, 2); }

void uart_puts(size_t line, const char *text) {
        (void)line;
        (void)text;
}
int uart_try_getc(size_t line, char *ch) {
        (void)line;
        (void)ch;
        return -1;
}
int uart_try_putc(size_t line, char ch) {
        (void)line;
        (void)ch;
        return 0;
}

int WhoIs(const char *name) {
        (void)name;
        return 1;
}
int CanTrainSetSpeed(int tid, int train, int speed) {
        (void)tid;
        (void)train;
        (void)speed;
        return 0;
}
int CanTrainReverse(int tid, int train) {
        (void)tid;
        (void)train;
        return 0;
}
int CanSwitch(int tid, int switch_number, char direction) {
        (void)tid;
        (void)switch_number;
        (void)direction;
        return 0;
}

static int test_server_queueing(void) {
        int i;

        active_server_scenario = SERVER_SCENARIO_BASE;
        receive_index = 0;
        reply_count = 0;
        create_count = 0;
        create_wrong_priority = 0;

        if (setjmp(server_done) == 0) {
                TerminalServerTask();
                fprintf(stderr, "terminal server returned unexpectedly\n");
                return -1;
        }

        if (create_count != 2 || create_wrong_priority ||
            receive_index != EMPTY_POLL_COUNT +
                    (int)(sizeof(server_tail_requests) /
                          sizeof(server_tail_requests[0])) ||
            reply_count != 31) {
                fprintf(stderr,
                        "terminal server did not create notifiers/process requests\n");
                return -1;
        }

        /*
         * More empty polls than the 16-slot blocking waiter queue all return
         * immediately.  GETC(10), GETC(12), and TX_READY(102) intentionally
         * receive no immediate reply.  Empty TRY_GETC polls do not disturb the
         * blocking waiter FIFO: successive RX_READY messages wake 10 then 12.
         * PUTC later wakes 102 before acknowledging 11.
         */
        for (i = 0; i < EMPTY_POLL_COUNT; i++) {
                if (replies[i].tid != 20 + i ||
                    replies[i].reply.result != TERM_RESULT_EMPTY ||
                    replies[i].reply.ch != 0) {
                        fprintf(stderr,
                                "empty TRY_GETC blocked or consumed a waiter slot\n");
                        return -1;
                }
        }
        if (replies[17].tid != 40 ||
            replies[17].reply.result != TERM_RESULT_EMPTY ||
            replies[18].tid != 44 ||
            replies[18].reply.result != TERM_RESULT_EMPTY ||
            replies[19].tid != 10 ||
            replies[19].reply.result != TERM_RESULT_OK ||
            replies[19].reply.ch != 'Q' ||
            replies[20].tid != 101 ||
            replies[20].reply.result != TERM_RESULT_OK ||
            replies[21].tid != 12 ||
            replies[21].reply.result != TERM_RESULT_OK ||
            replies[21].reply.ch != 'R' ||
            replies[22].tid != 101 ||
            replies[22].reply.result != TERM_RESULT_OK ||
            replies[23].tid != 101 ||
            replies[23].reply.result != TERM_RESULT_OK ||
            replies[24].tid != 41 ||
            replies[24].reply.result != TERM_RESULT_OK ||
            replies[24].reply.ch != 0x00 ||
            replies[25].tid != 101 ||
            replies[25].reply.result != TERM_RESULT_OK ||
            replies[26].tid != 42 ||
            replies[26].reply.result != TERM_RESULT_OK ||
            replies[26].reply.ch != 0xff ||
            replies[27].tid != 43 ||
            replies[27].reply.result != TERM_RESULT_EMPTY ||
            replies[28].tid != 102 ||
            replies[28].reply.result != TERM_RESULT_OK ||
            replies[28].reply.ch != 'X' ||
            replies[29].tid != 11 ||
            replies[29].reply.result != TERM_RESULT_OK ||
            replies[30].tid != 13 || replies[30].reply.result != -1) {
                fprintf(stderr,
                        "terminal poll/waiter/RX/TX ordering behavior failed\n");
                return -1;
        }
        return 0;
}

static int test_bulk_write_atomic_queueing(void) {
        int i;
        int reply_index;

        active_server_scenario = SERVER_SCENARIO_BULK;
        receive_index = 0;
        reply_count = 0;
        create_count = 0;
        create_wrong_priority = 0;

        if (setjmp(server_done) == 0) {
                TerminalServerTask();
                fprintf(stderr,
                        "terminal bulk server returned unexpectedly\n");
                return -1;
        }

        if (create_count != 2 || create_wrong_priority ||
            receive_index != BULK_REQUEST_COUNT ||
            reply_count != BULK_REQUEST_COUNT) {
                fprintf(stderr,
                        "terminal bulk scenario did not process every "
                        "request\n");
                return -1;
        }

        if (replies[0].tid != 30 ||
            replies[0].reply.result != TERM_RESULT_OK ||
            replies[1].tid != 31 ||
            replies[1].reply.result != TERM_RESULT_WOULD_BLOCK ||
            replies[2].tid != 32 ||
            replies[2].reply.result != TERM_RESULT_OK ||
            replies[3].tid != 33 ||
            replies[3].reply.result != TERM_RESULT_WOULD_BLOCK) {
                fprintf(stderr,
                        "bulk enqueue did not distinguish accept from "
                        "all-or-none backpressure\n");
                return -1;
        }

        reply_index = 4;
        for (i = 0; i < BULK_FIRST_DRAIN_LENGTH; i++, reply_index++) {
                if (replies[reply_index].tid != 102 ||
                    replies[reply_index].reply.result != TERM_RESULT_OK ||
                    replies[reply_index].reply.ch != first_bulk_byte(i)) {
                        fprintf(stderr,
                                "first bulk drain lost FIFO order at %d\n",
                                i);
                        return -1;
                }
        }
        if (replies[reply_index].tid != 34 ||
            replies[reply_index].reply.result != TERM_RESULT_OK) {
                fprintf(stderr,
                        "wrapped bulk enqueue was not accepted atomically\n");
                return -1;
        }
        reply_index++;

        for (i = BULK_FIRST_DRAIN_LENGTH;
             i < BULK_FIRST_LENGTH; i++, reply_index++) {
                if (replies[reply_index].tid != 102 ||
                    replies[reply_index].reply.result != TERM_RESULT_OK ||
                    replies[reply_index].reply.ch != first_bulk_byte(i)) {
                        fprintf(stderr,
                                "wrapped FIFO lost original suffix at %d\n",
                                i);
                        return -1;
                }
        }
        for (i = 0; i < BULK_FILL_LENGTH; i++, reply_index++) {
                if (replies[reply_index].tid != 102 ||
                    replies[reply_index].reply.result != TERM_RESULT_OK ||
                    replies[reply_index].reply.ch != fill_bulk_byte(i)) {
                        fprintf(stderr,
                                "rejected chunk partially entered FIFO\n");
                        return -1;
                }
        }
        for (i = 0; i < BULK_WRAP_LENGTH; i++, reply_index++) {
                if (replies[reply_index].tid != 102 ||
                    replies[reply_index].reply.result != TERM_RESULT_OK ||
                    replies[reply_index].reply.ch != wrap_bulk_byte(i)) {
                        fprintf(stderr,
                                "wrapped bulk bytes lost FIFO order at %d\n",
                                i);
                        return -1;
                }
        }
        for (i = 0; i < BULK_MALFORMED_COUNT; i++, reply_index++) {
                if (replies[reply_index].tid != 40 + i ||
                    replies[reply_index].reply.result !=
                            TERMINAL_TRY_WRITE_ERROR) {
                        fprintf(stderr,
                                "malformed bulk request was accepted\n");
                        return -1;
                }
        }
        if (reply_index != reply_count) {
                fprintf(stderr,
                        "bulk FIFO validation did not consume all replies\n");
                return -1;
        }
        return 0;
}

static int test_client_reply_lengths(void) {
        static const char bulk_bytes[] = {
                'A', '\0', (char)0xff, 'Z'
        };
        char max_bulk[TERMINAL_TRY_WRITE_CAPACITY];
        int i;

        client_send_calls = 0;
        client_short_reply = 0;
        client_reply_result = 0;
        client_reply_ch = 'Z';
        if (Getc(0) != -1 ||
            TryGetc(0) != TERMINAL_TRY_GETC_ERROR ||
            Putc(0, 'x') != -1 ||
            Getc(1) != 'Z' || Putc(1, 'x') != 0) {
                fprintf(stderr, "terminal client validation failed\n");
                return -1;
        }
        client_reply_ch = 0xff;
        if (TryGetc(1) != 255 ||
            client_last_request_type != TERM_MSG_TRY_GETC ||
            client_last_request_length !=
                    (int)sizeof(mirrored_terminal_request_t)) {
                fprintf(stderr,
                        "TRY_GETC did not preserve an unsigned byte/request ABI\n");
                return -1;
        }
        client_reply_result = TERM_RESULT_EMPTY;
        if (TryGetc(1) != TERMINAL_TRY_GETC_EMPTY) {
                fprintf(stderr, "TRY_GETC empty state was not distinguished\n");
                return -1;
        }
        client_reply_result = 99;
        if (TryGetc(1) != TERMINAL_TRY_GETC_ERROR) {
                fprintf(stderr,
                        "TRY_GETC accepted an unknown positive result\n");
                return -1;
        }

        {
                int before_invalid = client_send_calls;
                if (TerminalTryWrite(0, bulk_bytes, 4) !=
                            TERMINAL_TRY_WRITE_ERROR ||
                    TerminalTryWrite(1, NULL, 4) !=
                            TERMINAL_TRY_WRITE_ERROR ||
                    TerminalTryWrite(1, bulk_bytes, 0) !=
                            TERMINAL_TRY_WRITE_ERROR ||
                    TerminalTryWrite(
                            1, bulk_bytes,
                            TERMINAL_TRY_WRITE_CAPACITY + 1) !=
                            TERMINAL_TRY_WRITE_ERROR ||
                    client_send_calls != before_invalid) {
                        fprintf(stderr,
                                "bulk client accepted invalid arguments "
                                "or sent IPC\n");
                        return -1;
                }
        }
        client_reply_result = TERM_RESULT_OK;
        if (TerminalTryWrite(1, bulk_bytes, 4) !=
                    TERMINAL_TRY_WRITE_ACCEPTED ||
            client_last_request_type != TERM_MSG_TRY_WRITE ||
            client_last_request_length != TRY_WRITE_HEADER_SIZE + 4 ||
            client_last_bulk_length != 4 ||
            memcmp(client_last_bulk_bytes, bulk_bytes, 4) != 0) {
                fprintf(stderr,
                        "bulk client request ABI/payload was not exact\n");
                return -1;
        }
        for (i = 0; i < TERMINAL_TRY_WRITE_CAPACITY; i++) {
                max_bulk[i] = (char)(i ^ 0x5a);
        }
        if (TerminalTryWrite(
                    1, max_bulk, TERMINAL_TRY_WRITE_CAPACITY) !=
                    TERMINAL_TRY_WRITE_ACCEPTED ||
            client_last_request_type != TERM_MSG_TRY_WRITE ||
            client_last_request_length !=
                    TRY_WRITE_HEADER_SIZE +
                            TERMINAL_TRY_WRITE_CAPACITY ||
            client_last_bulk_length !=
                    TERMINAL_TRY_WRITE_CAPACITY ||
            memcmp(client_last_bulk_bytes, max_bulk,
                   TERMINAL_TRY_WRITE_CAPACITY) != 0) {
                fprintf(stderr,
                        "bulk client maximum request was not exact\n");
                return -1;
        }
        client_reply_result = TERM_RESULT_WOULD_BLOCK;
        if (TerminalTryWrite(1, bulk_bytes, 4) !=
                    TERMINAL_TRY_WRITE_WOULD_BLOCK) {
                fprintf(stderr,
                        "bulk client lost WOULD_BLOCK distinction\n");
                return -1;
        }
        client_reply_result = 99;
        if (TerminalTryWrite(1, bulk_bytes, 4) !=
                    TERMINAL_TRY_WRITE_ERROR) {
                fprintf(stderr,
                        "bulk client accepted unknown server result\n");
                return -1;
        }

        client_short_reply = 1;
        client_reply_result = TERM_RESULT_OK;
        if (Getc(1) != -1 ||
            TryGetc(1) != TERMINAL_TRY_GETC_ERROR ||
            Putc(1, 'x') != -1 ||
            TerminalTryWrite(1, bulk_bytes, 4) !=
                    TERMINAL_TRY_WRITE_ERROR) {
                fprintf(stderr, "short terminal reply was accepted\n");
                return -1;
        }
        client_short_reply = 0;
        client_reply_result = -1;
        if (Getc(1) != -1 ||
            TryGetc(1) != TERMINAL_TRY_GETC_ERROR ||
            Putc(1, 'x') != -1 ||
            TerminalTryWrite(1, bulk_bytes, 4) !=
                    TERMINAL_TRY_WRITE_ERROR) {
                fprintf(stderr, "negative terminal reply was accepted\n");
                return -1;
        }
        return 0;
}

static int test_tx_notifier_failure_backoff(void) {
        notifier_failure_mode = 1;
        notifier_send_calls = 0;
        notifier_timer_waits = 0;
        notifier_wrong_event = 0;
        notifier_yields = 0;
        int jump = setjmp(server_done);
        if (jump == 0) {
                TerminalTxNotifierTask();
                fprintf(stderr,
                        "terminal TX notifier returned unexpectedly\n");
                notifier_failure_mode = 0;
                return -1;
        }
        notifier_failure_mode = 0;
        if (jump != 3 || notifier_send_calls != 4 ||
            notifier_timer_waits != 3 ||
            notifier_wrong_event != 0 ||
            notifier_yields != 0) {
                fprintf(stderr,
                        "terminal TX failure path did not timer-block "
                        "without Yield spinning\n");
                return -1;
        }
        return 0;
}

int main(void) {
        if (test_server_queueing() < 0 ||
            test_bulk_write_atomic_queueing() < 0 ||
            test_client_reply_lengths() < 0 ||
            test_tx_notifier_failure_backoff() < 0) {
                return 1;
        }
        printf("validated nonblocking TRY_GETC, atomic nonblocking bulk TX "
               "with wrap/backpressure, blocking GETC FIFO waiters, TX "
               "notifier queue, IPC lengths, priority, and timer-backed "
               "failure retry\n");
        return 0;
}
