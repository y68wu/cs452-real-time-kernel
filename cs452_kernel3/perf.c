#include "perf.h"
#include "syscall.h"
#include "timer.h"
#include "uart.h"

#define PERF_MAX_MSG 256

static int perf_receiver_tid = -1;
static int perf_msg_size = 4;
static int perf_iterations = 1000;
static const char *perf_label = "perf";

static char perf_send_buf[PERF_MAX_MSG];
static char perf_reply_buf[PERF_MAX_MSG];

void perf_configure_case(int receiver_tid, int msg_size, int iterations, const char *label) {
        perf_receiver_tid = receiver_tid;
        perf_msg_size = msg_size;
        perf_iterations = iterations;
        perf_label = label;
}

static void fill_message(char *buf, int len, char base) {
        for (int i = 0; i < len; i++) {
                buf[i] = (char)(base + (i % 23));
        }
}

void PerfReceiverTask(void) {
        int sender_tid;
        char msg[PERF_MAX_MSG];
        char reply[PERF_MAX_MSG];

        fill_message(reply, PERF_MAX_MSG, 'a');

        for (int i = 0; i < perf_iterations; i++) {
                Receive(&sender_tid, msg, perf_msg_size);
                Reply(sender_tid, reply, perf_msg_size);
        }

        Exit();
}

void PerfSenderTask(void) {
        uint64_t start;
        uint64_t end;
        uint64_t total;
        int avg;

        fill_message(perf_send_buf, PERF_MAX_MSG, 'A');

        start = timer_get_usec();

        for (int i = 0; i < perf_iterations; i++) {
                Send(perf_receiver_tid, perf_send_buf, perf_msg_size, perf_reply_buf, perf_msg_size);
        }

        end = timer_get_usec();
        total = end - start;

        if (perf_iterations > 0) {
                avg = (int)(total / (uint64_t)perf_iterations);
        } else {
                avg = 0;
        }

        uart_printf(CONSOLE, "PERF %s size=%d iter=%d avg_us=%d\r\n",
                    perf_label, perf_msg_size, perf_iterations, avg);

        Exit();
}
