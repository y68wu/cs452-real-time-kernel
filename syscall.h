#ifndef _syscall_h_
#define _syscall_h_ 1

#define SYSCALL_CREATE        1
#define SYSCALL_MY_TID        2
#define SYSCALL_MY_PARENT_TID 3
#define SYSCALL_YIELD         4
#define SYSCALL_EXIT          5
#define SYSCALL_SEND          6
#define SYSCALL_RECEIVE       7
#define SYSCALL_REPLY         8
#define SYSCALL_AWAIT_EVENT   9

int Create(int priority, void (*function)());
int MyTid(void);
int MyParentTid(void);
void Yield(void);
void Exit(void);

int Send(int tid, const char *msg, int msglen, char *reply, int rplen);
int Receive(int *tid, char *msg, int msglen);
int Reply(int tid, const char *reply, int rplen);

int AwaitEvent(int eventType);

#endif
