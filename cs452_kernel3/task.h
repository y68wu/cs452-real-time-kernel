#ifndef _task_h_
#define _task_h_ 1

#include <stdint.h>

#define MAX_TASKS 64
#define MAX_PRIORITIES 16
#define TASK_STACK_SIZE 65536

#define SPSR_EL1H_MASKED 0x3c5u

typedef enum {
        TASK_UNUSED = 0,
        TASK_READY,
        TASK_ACTIVE,
        TASK_EXITED,
        TASK_SEND_BLOCKED,
        TASK_RECEIVE_BLOCKED,
        TASK_REPLY_BLOCKED,
        TASK_EVENT_BLOCKED
} task_state_t;

typedef struct context_frame {
        uint64_t x[31];
        uint64_t elr;
        uint64_t spsr;
        uint64_t pad;
} context_frame_t;

typedef struct task_descriptor task_descriptor_t;

struct task_descriptor {
        int tid;
        int parent_tid;
        int priority;
        task_state_t state;

        context_frame_t *sp;
        void (*entry)();

        task_descriptor_t *ready_next;

        task_descriptor_t *send_next;
        task_descriptor_t *send_head;
        task_descriptor_t *send_tail;

        const char *send_msg;
        int send_msglen;
        char *reply_buf;
        int reply_len;

        int *recv_tid_ptr;
        char *recv_buf;
        int recv_len;

        int event_type;
        unsigned long long wakeup_time_usec;
        task_descriptor_t *event_next;
};

void task_system_init(void);

task_descriptor_t *task_create_descriptor(int priority, void (*entry)(), int parent_tid);
void task_make_ready(task_descriptor_t *task);
task_descriptor_t *task_schedule(void);

task_descriptor_t *task_get(int tid);
int task_priority_valid(int priority);

void task_exit_trampoline(void);

#endif
