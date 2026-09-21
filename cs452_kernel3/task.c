#include "task.h"

static task_descriptor_t tasks[MAX_TASKS];
static unsigned char stacks[MAX_TASKS][TASK_STACK_SIZE] __attribute__((aligned(16)));

static task_descriptor_t *ready_head[MAX_PRIORITIES];
static task_descriptor_t *ready_tail[MAX_PRIORITIES];

static int next_tid = 1;

static void clear_task_message_fields(task_descriptor_t *task) {
        task->send_next = 0;
        task->send_head = 0;
        task->send_tail = 0;
        task->send_msg = 0;
        task->send_msglen = 0;
        task->reply_buf = 0;
        task->reply_len = 0;
        task->recv_tid_ptr = 0;
        task->recv_buf = 0;
        task->recv_len = 0;
        task->event_type = -1;
        task->wakeup_time_usec = 0;
        task->event_next = 0;
}

int task_priority_valid(int priority) {
        return priority >= 0 && priority < MAX_PRIORITIES;
}

void task_system_init(void) {
        next_tid = 1;

        for (int i = 0; i < MAX_TASKS; i++) {
                tasks[i].tid = 0;
                tasks[i].parent_tid = -1;
                tasks[i].priority = -1;
                tasks[i].state = TASK_UNUSED;
                tasks[i].sp = 0;
                tasks[i].entry = 0;
                tasks[i].ready_next = 0;
                clear_task_message_fields(&tasks[i]);
        }

        for (int p = 0; p < MAX_PRIORITIES; p++) {
                ready_head[p] = 0;
                ready_tail[p] = 0;
        }
}

static task_descriptor_t *find_free_descriptor(void) {
        for (int i = 0; i < MAX_TASKS; i++) {
                if (tasks[i].state == TASK_UNUSED || tasks[i].state == TASK_EXITED) {
                        return &tasks[i];
                }
        }

        return 0;
}

static void clear_context_frame(context_frame_t *frame) {
        uint64_t *p = (uint64_t *)frame;
        for (unsigned int i = 0; i < sizeof(context_frame_t) / sizeof(uint64_t); i++) {
                p[i] = 0;
        }
}

task_descriptor_t *task_create_descriptor(int priority, void (*entry)(), int parent_tid) {
        task_descriptor_t *task;

        if (!task_priority_valid(priority)) {
                return 0;
        }

        task = find_free_descriptor();
        if (!task) {
                return 0;
        }

        task->tid = next_tid++;
        task->parent_tid = parent_tid;
        task->priority = priority;
        task->state = TASK_READY;
        task->entry = entry;
        task->ready_next = 0;
        clear_task_message_fields(task);

        uintptr_t top = (uintptr_t)&stacks[task - tasks][TASK_STACK_SIZE];
        top &= ~(uintptr_t)0xFu;
        top -= sizeof(context_frame_t);

        task->sp = (context_frame_t *)top;
        clear_context_frame(task->sp);

        task->sp->elr = (uint64_t)entry;
        task->sp->spsr = SPSR_EL1H_MASKED;
        task->sp->x[30] = (uint64_t)task_exit_trampoline;

        task_make_ready(task);
        return task;
}

void task_make_ready(task_descriptor_t *task) {
        int p;

        if (!task) return;

        p = task->priority;
        if (!task_priority_valid(p)) return;

        task->state = TASK_READY;
        task->ready_next = 0;

        if (ready_tail[p]) {
                ready_tail[p]->ready_next = task;
        } else {
                ready_head[p] = task;
        }

        ready_tail[p] = task;
}

task_descriptor_t *task_schedule(void) {
        for (int p = 0; p < MAX_PRIORITIES; p++) {
                task_descriptor_t *task = ready_head[p];

                if (task) {
                        ready_head[p] = task->ready_next;
                        if (!ready_head[p]) {
                                ready_tail[p] = 0;
                        }

                        task->ready_next = 0;
                        task->state = TASK_ACTIVE;
                        return task;
                }
        }

        return 0;
}

task_descriptor_t *task_get(int tid) {
        for (int i = 0; i < MAX_TASKS; i++) {
                if (tasks[i].state != TASK_UNUSED && tasks[i].tid == tid) {
                        return &tasks[i];
                }
        }

        return 0;
}
