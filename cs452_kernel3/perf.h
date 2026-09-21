#ifndef _perf_h_
#define _perf_h_ 1

void PerfReceiverTask(void);
void PerfSenderTask(void);

void perf_configure_case(int receiver_tid, int msg_size, int iterations, const char *label);

#endif
