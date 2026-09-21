#ifndef _terminal_h_
#define _terminal_h_ 1

#define TERMINAL_SERVER_NAME "terminal0"

#define TERMINAL_TRY_GETC_EMPTY (-1)
#define TERMINAL_TRY_GETC_ERROR (-2)

/*
 * TerminalTryWrite is the nonblocking, all-or-none transport used by the
 * incremental TC2 UI pump.  The fixed maximum keeps the IPC request bounded
 * and matches the pump's largest drain chunk.
 */
#define TERMINAL_TRY_WRITE_CAPACITY 192
#define TERMINAL_TRY_WRITE_ACCEPTED 0
#define TERMINAL_TRY_WRITE_WOULD_BLOCK 1
#define TERMINAL_TRY_WRITE_ERROR (-1)

int Getc(int tid);
/*
 * Poll one byte without waiting for future UART input.
 *
 * Returns 0..255 for a byte already buffered by TerminalServer,
 * TERMINAL_TRY_GETC_EMPTY when no byte is currently available, and
 * TERMINAL_TRY_GETC_ERROR for an invalid tid or IPC/server failure.
 */
int TryGetc(int tid);
int Putc(int tid, unsigned char ch);
/*
 * Attempt to enqueue exactly length bytes without waiting for TX FIFO space.
 *
 * ACCEPTED means every byte was enqueued in order.  WOULD_BLOCK means the
 * FIFO did not have room and no byte was enqueued.  ERROR means invalid
 * arguments or an IPC/server failure.  length must be in 1..192.
 */
int TerminalTryWrite(int tid, const char *bytes, int length);

void TerminalServerTask(void);
void TerminalRxNotifierTask(void);
void TerminalTxNotifierTask(void);
void K4TerminalTestTask(void);

#endif
