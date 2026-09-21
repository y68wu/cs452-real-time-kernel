#include <assert.h>
#include <stdint.h>
#include <stdio.h>

#include "task.h"

#define TEST_PRIORITY 4
#define RESULT_SENTINEL UINT64_C(0x123456789abcdef0)

void task_exit_trampoline(void) {
}

static void dummy_entry(void) {
}

static task_descriptor_t *create_task(void) {
        task_descriptor_t *task =
                task_create_descriptor(TEST_PRIORITY, dummy_entry, 0);
        assert(task);
        return task;
}

static void expect_scheduled(task_descriptor_t *expected) {
        task_descriptor_t *actual = task_schedule();
        assert(actual == expected);
        assert(actual->state == TASK_ACTIVE);
}

static void configure_ipc_wait(task_descriptor_t *waiter,
                               task_descriptor_t *receiver,
                               task_state_t state,
                               const char *message,
                               char *reply) {
        assert(state == TASK_SEND_BLOCKED || state == TASK_REPLY_BLOCKED);

        waiter->state = state;
        waiter->send_msg = message;
        waiter->send_msglen = 4;
        waiter->reply_buf = reply;
        waiter->reply_len = 4;
        waiter->ipc_receiver = receiver;
        waiter->sp->x[0] = RESULT_SENTINEL;
}

static void assert_ipc_wait_cleared(const task_descriptor_t *task) {
        assert(task->send_next == 0);
        assert(task->send_msg == 0);
        assert(task->send_msglen == 0);
        assert(task->reply_buf == 0);
        assert(task->reply_len == 0);
        assert(task->ipc_receiver == 0);
}

static void test_exit_wakes_all_owned_ipc_waiters(void) {
        static const char queued1_message[] = "q1";
        static const char queued2_message[] = "q2";
        static const char delivered_message[] = "reply";
        static const char unrelated_message[] = "other";
        static const char nested_message[] = "nested";
        char queued1_reply[4];
        char queued2_reply[4];
        char delivered_reply[4];
        char unrelated_reply[4];
        char nested_reply[4];
        task_descriptor_t *receiver;
        task_descriptor_t *queued1;
        task_descriptor_t *queued2;
        task_descriptor_t *delivered;
        task_descriptor_t *other_receiver;
        task_descriptor_t *unrelated;
        task_descriptor_t *nested;
        task_descriptor_t *replacement;
        int exited_tid;

        task_system_init();
        receiver = create_task();
        queued1 = create_task();
        queued2 = create_task();
        delivered = create_task();
        other_receiver = create_task();
        unrelated = create_task();
        nested = create_task();

        expect_scheduled(receiver);
        expect_scheduled(queued1);
        expect_scheduled(queued2);
        expect_scheduled(delivered);
        expect_scheduled(other_receiver);
        expect_scheduled(unrelated);
        expect_scheduled(nested);
        assert(task_schedule() == 0);

        configure_ipc_wait(queued1, receiver, TASK_SEND_BLOCKED,
                           queued1_message, queued1_reply);
        configure_ipc_wait(queued2, receiver, TASK_SEND_BLOCKED,
                           queued2_message, queued2_reply);
        configure_ipc_wait(delivered, receiver, TASK_REPLY_BLOCKED,
                           delivered_message, delivered_reply);
        configure_ipc_wait(unrelated, other_receiver, TASK_REPLY_BLOCKED,
                           unrelated_message, unrelated_reply);
        configure_ipc_wait(nested, queued1, TASK_SEND_BLOCKED,
                           nested_message, nested_reply);

        receiver->send_head = queued1;
        receiver->send_tail = queued2;
        queued1->send_next = queued2;
        queued2->send_next = 0;

        /*
         * A task can be a server with queued callers while its own Send is
         * blocked. Waking queued1 must preserve this independent inbound FIFO.
         */
        queued1->send_head = nested;
        queued1->send_tail = nested;
        nested->send_next = 0;

        exited_tid = receiver->tid;
        assert(task_exit_descriptor(receiver, TASK_IPC_PEER_EXITED) == 3);

        assert(receiver->state == TASK_EXITED);
        assert(task_get(exited_tid) == receiver);
        assert(receiver->send_head == 0);
        assert(receiver->send_tail == 0);
        assert_ipc_wait_cleared(receiver);

        assert(queued1->state == TASK_READY);
        assert(queued2->state == TASK_READY);
        assert(delivered->state == TASK_READY);
        assert((int64_t)queued1->sp->x[0] == TASK_IPC_PEER_EXITED);
        assert((int64_t)queued2->sp->x[0] == TASK_IPC_PEER_EXITED);
        assert((int64_t)delivered->sp->x[0] == TASK_IPC_PEER_EXITED);
        assert_ipc_wait_cleared(queued1);
        assert_ipc_wait_cleared(queued2);
        assert_ipc_wait_cleared(delivered);

        assert(queued1->send_head == nested);
        assert(queued1->send_tail == nested);
        assert(nested->state == TASK_SEND_BLOCKED);
        assert(nested->ipc_receiver == queued1);
        assert(nested->sp->x[0] == RESULT_SENTINEL);

        assert(unrelated->state == TASK_REPLY_BLOCKED);
        assert(unrelated->ipc_receiver == other_receiver);
        assert(unrelated->sp->x[0] == RESULT_SENTINEL);

        expect_scheduled(queued1);
        expect_scheduled(queued2);
        expect_scheduled(delivered);
        assert(task_schedule() == 0);

        replacement = create_task();
        assert(replacement == receiver);
        assert(replacement->tid != exited_tid);
        assert(task_get(exited_tid) == 0);
        assert(task_get(replacement->tid) == replacement);
        assert(replacement->send_head == 0);
        assert(replacement->send_tail == 0);
        assert_ipc_wait_cleared(replacement);
        assert(unrelated->ipc_receiver == other_receiver);
        assert(nested->ipc_receiver == queued1);

        expect_scheduled(replacement);
        assert(task_exit_descriptor(replacement, TASK_IPC_PEER_EXITED) == 0);
        assert(unrelated->state == TASK_REPLY_BLOCKED);
        assert(nested->state == TASK_SEND_BLOCKED);
}

static void test_exit_normalizes_nonnegative_ipc_error(void) {
        static const char message[] = "wait";
        char reply[4];
        task_descriptor_t *receiver;
        task_descriptor_t *waiter;

        task_system_init();
        receiver = create_task();
        waiter = create_task();
        expect_scheduled(receiver);
        expect_scheduled(waiter);

        configure_ipc_wait(waiter, receiver, TASK_SEND_BLOCKED,
                           message, reply);
        receiver->send_head = waiter;
        receiver->send_tail = waiter;

        assert(task_exit_descriptor(receiver, 0) == 1);
        assert(waiter->state == TASK_READY);
        assert((int64_t)waiter->sp->x[0] == TASK_IPC_PEER_EXITED);
        assert_ipc_wait_cleared(waiter);
}

static void test_only_active_descriptor_can_exit(void) {
        task_descriptor_t *ready;

        task_system_init();
        ready = create_task();

        assert(ready->state == TASK_READY);
        assert(task_exit_descriptor(ready, TASK_IPC_PEER_EXITED) == -1);
        assert(ready->state == TASK_READY);
        expect_scheduled(ready);
}

static void test_equal_priority_control_plane_round_robin(void) {
        task_descriptor_t *ui;
        task_descriptor_t *ticker;

        task_system_init();
        ui = create_task();
        ticker = create_task();

        for (int cycle = 0; cycle < 128; ++cycle) {
                expect_scheduled(ui);
                /*
                 * A full terminal queue makes the UI Yield and become ready
                 * again. It must rejoin the tail, leaving the equal-priority
                 * dispatcher ticker at the head.
                 */
                task_make_ready(ui);
                expect_scheduled(ticker);
                task_make_ready(ticker);
        }
        assert(task_schedule() == ui);
}

int main(void) {
        test_exit_wakes_all_owned_ipc_waiters();
        test_exit_normalizes_nonnegative_ipc_error();
        test_only_active_descriptor_can_exit();
        test_equal_priority_control_plane_round_robin();

        puts("task IPC cleanup and equal-priority round-robin tests passed");
        return 0;
}
