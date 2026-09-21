#ifndef _terminal_h_
#define _terminal_h_ 1

#define TERMINAL_SERVER_NAME "terminal0"

int Getc(int tid);
int Putc(int tid, unsigned char ch);

void TerminalServerTask(void);
void TerminalRxNotifierTask(void);
void TerminalTxNotifierTask(void);
void K4TerminalTestTask(void);

#endif
