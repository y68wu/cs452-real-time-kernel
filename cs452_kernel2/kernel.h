#ifndef _kernel_h_
#define _kernel_h_ 1

#include "task.h"

void kernel_run(void (*first_task)(), int first_priority);
context_frame_t *kernel_handle_exception(context_frame_t *frame);
long kernel_handle_syscall(long request, long arg0, long arg1, long arg2);

#endif
