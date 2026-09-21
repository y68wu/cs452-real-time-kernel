#include "kernel.h"
#include "syscall.h"
#include "task.h"
#include "uart.h"
#include "events.h"
#include "timer.h"

extern void install_exception_vectors(void);
extern void context_restore(context_frame_t *frame);

static task_descriptor_t *active_task = 0;
static task_descriptor_t *timer_wait_head = 0;
static unsigned long long timer_next_wakeup_usec = 0;

static void kernel_halt(void) {
        uart_puts(CONSOLE, "\r\nKernel: no ready tasks; halting.\r\n");
        for (;;) {
        }
}

static int min_int(int a, int b) {
        return a < b ? a : b;
}


static void timer_wait_insert(task_descriptor_t *task, unsigned long long wakeup_time) {
        task_descriptor_t **cur = &timer_wait_head;

        task->event_type = EVENT_TIMER;
        task->wakeup_time_usec = wakeup_time;
        task->event_next = 0;

        while (*cur && (*cur)->wakeup_time_usec <= wakeup_time) {
                cur = &((*cur)->event_next);
        }

        task->event_next = *cur;
        *cur = task;

        if (timer_wait_head) {
                timer_next_wakeup_usec = timer_wait_head->wakeup_time_usec;
        }
}

static void timer_release_ready_tasks(void) {
        unsigned long long now = timer_get_usec();

        while (timer_wait_head && timer_wait_head->wakeup_time_usec <= now) {
                task_descriptor_t *task = timer_wait_head;
                timer_wait_head = timer_wait_head->event_next;

                task->event_next = 0;
                task->event_type = -1;
                task->sp->x[0] = 0;
                task_make_ready(task);
        }

        if (timer_wait_head) {
                timer_next_wakeup_usec = timer_wait_head->wakeup_time_usec;
        } else {
                timer_next_wakeup_usec = 0;
        }
}

static int kernel_has_timer_waiters(void) {
        return timer_wait_head != 0;
}

static void kernel_idle_until_timer_event(void) {
        while (kernel_has_timer_waiters()) {
                timer_release_ready_tasks();

                active_task = task_schedule();
                if (active_task) {
                        return;
                }
        }

        kernel_halt();
}


static void copy_bytes(char *dst, const char *src, int len) {
        for (int i = 0; i < len; i++) {
                dst[i] = src[i];
        }
}

static int task_exists_for_message(task_descriptor_t *task) {
        return task && task->state != TASK_UNUSED && task->state != TASK_EXITED;
}

static void enqueue_sender(task_descriptor_t *receiver, task_descriptor_t *sender) {
        sender->send_next = 0;

        if (receiver->send_tail) {
                receiver->send_tail->send_next = sender;
        } else {
                receiver->send_head = sender;
        }

        receiver->send_tail = sender;
}

static task_descriptor_t *dequeue_sender(task_descriptor_t *receiver) {
        task_descriptor_t *sender = receiver->send_head;

        if (!sender) {
                return 0;
        }

        receiver->send_head = sender->send_next;
        if (!receiver->send_head) {
                receiver->send_tail = 0;
        }

        sender->send_next = 0;
        return sender;
}

static void deliver_message(task_descriptor_t *receiver, task_descriptor_t *sender) {
        int copied = min_int(sender->send_msglen, receiver->recv_len);

        if (copied > 0) {
                copy_bytes(receiver->recv_buf, sender->send_msg, copied);
        }

        if (receiver->recv_tid_ptr) {
                *(receiver->recv_tid_ptr) = sender->tid;
        }

        receiver->sp->x[0] = (uint64_t)sender->send_msglen;
        sender->state = TASK_REPLY_BLOCKED;
}

void kernel_run(void (*first_task)(), int first_priority) {
        task_descriptor_t *first;

        install_exception_vectors();
        task_system_init();

        first = task_create_descriptor(first_priority, first_task, 0);
        if (!first) {
                uart_puts(CONSOLE, "Kernel: failed to create first user task.\r\n");
                kernel_halt();
        }

        active_task = task_schedule();
        if (!active_task) {
                kernel_halt();
        }

        context_restore(active_task->sp);
}

context_frame_t *kernel_handle_exception(context_frame_t *frame) {
        long request = (long)frame->x[0];
        long arg0 = (long)frame->x[1];
        long arg1 = (long)frame->x[2];
        long arg2 = (long)frame->x[3];
        long arg3 = (long)frame->x[4];
        long arg4 = (long)frame->x[5];

        if (active_task) {
                active_task->sp = frame;
        }

        switch (request) {
        case SYSCALL_CREATE: {
                task_descriptor_t *task;
                int priority = (int)arg0;
                void (*entry)() = (void (*)())arg1;
                int parent_tid = active_task ? active_task->tid : 0;

                if (!task_priority_valid(priority)) {
                        frame->x[0] = (uint64_t)-1;
                        break;
                }

                task = task_create_descriptor(priority, entry, parent_tid);
                if (!task) {
                        frame->x[0] = (uint64_t)-2;
                        break;
                }

                frame->x[0] = (uint64_t)task->tid;
                break;
        }

        case SYSCALL_MY_TID:
                frame->x[0] = active_task ? (uint64_t)active_task->tid : (uint64_t)-1;
                break;

        case SYSCALL_MY_PARENT_TID:
                frame->x[0] = active_task ? (uint64_t)active_task->parent_tid : (uint64_t)-1;
                break;

        case SYSCALL_YIELD:
                frame->x[0] = 0;
                break;

        case SYSCALL_EXIT:
                if (active_task) {
                        active_task->state = TASK_EXITED;
                }
                break;

        case SYSCALL_SEND: {
                int tid = (int)arg0;
                const char *msg = (const char *)arg1;
                int msglen = (int)arg2;
                char *reply = (char *)arg3;
                int rplen = (int)arg4;
                task_descriptor_t *receiver = task_get(tid);

                if (!task_exists_for_message(receiver)) {
                        frame->x[0] = (uint64_t)-1;
                        break;
                }

                active_task->send_msg = msg;
                active_task->send_msglen = msglen;
                active_task->reply_buf = reply;
                active_task->reply_len = rplen;

                if (receiver->state == TASK_RECEIVE_BLOCKED) {
                        deliver_message(receiver, active_task);
                        task_make_ready(receiver);
                } else {
                        active_task->state = TASK_SEND_BLOCKED;
                        enqueue_sender(receiver, active_task);
                }

                break;
        }

        case SYSCALL_RECEIVE: {
                int *sender_tid = (int *)arg0;
                char *msg = (char *)arg1;
                int msglen = (int)arg2;
                task_descriptor_t *sender = dequeue_sender(active_task);

                active_task->recv_tid_ptr = sender_tid;
                active_task->recv_buf = msg;
                active_task->recv_len = msglen;

                if (sender) {
                        deliver_message(active_task, sender);
                } else {
                        active_task->state = TASK_RECEIVE_BLOCKED;
                }

                break;
        }

        case SYSCALL_AWAIT_EVENT: {
                int event_type = (int)arg0;

                if (event_type != EVENT_TIMER) {
                        frame->x[0] = (uint64_t)-1;
                        break;
                }

                active_task->state = TASK_EVENT_BLOCKED;
                timer_wait_insert(active_task, timer_get_usec() + 10000u);
                break;
        }

        case SYSCALL_REPLY: {
                int tid = (int)arg0;
                const char *reply = (const char *)arg1;
                int rplen = (int)arg2;
                task_descriptor_t *sender = task_get(tid);

                if (!task_exists_for_message(sender)) {
                        frame->x[0] = (uint64_t)-1;
                        break;
                }

                if (sender->state != TASK_REPLY_BLOCKED) {
                        frame->x[0] = (uint64_t)-2;
                        break;
                }

                int copied = min_int(rplen, sender->reply_len);

                if (copied > 0) {
                        copy_bytes(sender->reply_buf, reply, copied);
                }

                sender->sp->x[0] = (uint64_t)copied;
                frame->x[0] = (uint64_t)copied;

                task_make_ready(sender);
                break;
        }

        default:
                frame->x[0] = (uint64_t)-999;
                break;
        }

        if (active_task && active_task->state == TASK_ACTIVE) {
                task_make_ready(active_task);
        }

        timer_release_ready_tasks();

        active_task = task_schedule();
        if (!active_task) {
                kernel_idle_until_timer_event();
        }

        return active_task->sp;
}

long kernel_handle_syscall(long request, long arg0, long arg1, long arg2, long arg3, long arg4) {
        (void)request;
        (void)arg0;
        (void)arg1;
        (void)arg2;
        (void)arg3;
        (void)arg4;
        return -999;
}
