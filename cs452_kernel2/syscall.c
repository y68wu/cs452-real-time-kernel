#include "syscall.h"

extern long syscall_invoke(long request, long arg0, long arg1, long arg2, long arg3, long arg4);

int Create(int priority, void (*function)()) {
        return (int)syscall_invoke(SYSCALL_CREATE, (long)priority, (long)function, 0, 0, 0);
}

int MyTid(void) {
        return (int)syscall_invoke(SYSCALL_MY_TID, 0, 0, 0, 0, 0);
}

int MyParentTid(void) {
        return (int)syscall_invoke(SYSCALL_MY_PARENT_TID, 0, 0, 0, 0, 0);
}

void Yield(void) {
        (void)syscall_invoke(SYSCALL_YIELD, 0, 0, 0, 0, 0);
}

void Exit(void) {
        (void)syscall_invoke(SYSCALL_EXIT, 0, 0, 0, 0, 0);
        for (;;) {
        }
}

int Send(int tid, const char *msg, int msglen, char *reply, int rplen) {
        return (int)syscall_invoke(SYSCALL_SEND, (long)tid, (long)msg, (long)msglen, (long)reply, (long)rplen);
}

int Receive(int *tid, char *msg, int msglen) {
        return (int)syscall_invoke(SYSCALL_RECEIVE, (long)tid, (long)msg, (long)msglen, 0, 0);
}

int Reply(int tid, const char *reply, int rplen) {
        return (int)syscall_invoke(SYSCALL_REPLY, (long)tid, (long)reply, (long)rplen, 0, 0);
}
