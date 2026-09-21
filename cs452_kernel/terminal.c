#include "terminal.h"
#include "syscall.h"
#include "uart.h"
#include "events.h"
#include "nameserver.h"
#include "can.h"

#define TERM_RX_BUF_SIZE 128
#define TERM_TX_BUF_SIZE 256
#define TERM_WAITING_GETC_SIZE 16

#define TERM_MSG_GETC     1
#define TERM_MSG_PUTC     2
#define TERM_MSG_RX_READY 3
#define TERM_MSG_TX_READY 4

typedef struct {
        int type;
        unsigned char ch;
} terminal_request_t;

typedef struct {
        int result;
        unsigned char ch;
} terminal_reply_t;

static int rx_head = 0;
static int rx_tail = 0;
static int rx_count = 0;
static unsigned char rx_buf[TERM_RX_BUF_SIZE];

static int tx_head = 0;
static int tx_tail = 0;
static int tx_count = 0;
static unsigned char tx_buf[TERM_TX_BUF_SIZE];

static int waiting_getc[TERM_WAITING_GETC_SIZE];
static int waiting_getc_head = 0;
static int waiting_getc_tail = 0;
static int waiting_getc_count = 0;

static int tx_notifier_tid = -1;

static int rx_push(unsigned char ch) {
        if (rx_count >= TERM_RX_BUF_SIZE) {
                return -1;
        }

        rx_buf[rx_tail] = ch;
        rx_tail = (rx_tail + 1) % TERM_RX_BUF_SIZE;
        rx_count++;
        return 0;
}

static int rx_pop(unsigned char *ch) {
        if (rx_count <= 0) {
                return -1;
        }

        *ch = rx_buf[rx_head];
        rx_head = (rx_head + 1) % TERM_RX_BUF_SIZE;
        rx_count--;
        return 0;
}

static int tx_push(unsigned char ch) {
        if (tx_count >= TERM_TX_BUF_SIZE) {
                return -1;
        }

        tx_buf[tx_tail] = ch;
        tx_tail = (tx_tail + 1) % TERM_TX_BUF_SIZE;
        tx_count++;
        return 0;
}

static int tx_pop(unsigned char *ch) {
        if (tx_count <= 0) {
                return -1;
        }

        *ch = tx_buf[tx_head];
        tx_head = (tx_head + 1) % TERM_TX_BUF_SIZE;
        tx_count--;
        return 0;
}

static int waiting_getc_push(int tid) {
        if (waiting_getc_count >= TERM_WAITING_GETC_SIZE) {
                return -1;
        }

        waiting_getc[waiting_getc_tail] = tid;
        waiting_getc_tail = (waiting_getc_tail + 1) % TERM_WAITING_GETC_SIZE;
        waiting_getc_count++;
        return 0;
}

static int waiting_getc_pop(int *tid) {
        if (waiting_getc_count <= 0) {
                return -1;
        }

        *tid = waiting_getc[waiting_getc_head];
        waiting_getc_head = (waiting_getc_head + 1) % TERM_WAITING_GETC_SIZE;
        waiting_getc_count--;
        return 0;
}

static void reply_getc_client(int tid, unsigned char ch) {
        terminal_reply_t reply;

        reply.result = 0;
        reply.ch = ch;
        Reply(tid, (const char *)&reply, sizeof(reply));
}

static void maybe_unblock_tx_notifier(void) {
        terminal_reply_t reply;
        unsigned char ch;

        if (tx_notifier_tid < 0) {
                return;
        }

        if (tx_pop(&ch) < 0) {
                return;
        }

        reply.result = 0;
        reply.ch = ch;
        Reply(tx_notifier_tid, (const char *)&reply, sizeof(reply));
        tx_notifier_tid = -1;
}

int Getc(int tid) {
        terminal_request_t req;
        terminal_reply_t reply;
        int ret;

        req.type = TERM_MSG_GETC;
        req.ch = 0;

        ret = Send(tid, (const char *)&req, sizeof(req), (char *)&reply, sizeof(reply));
        if (ret < 0 || reply.result < 0) {
                return -1;
        }

        return (int)reply.ch;
}

int Putc(int tid, unsigned char ch) {
        terminal_request_t req;
        terminal_reply_t reply;
        int ret;

        req.type = TERM_MSG_PUTC;
        req.ch = ch;

        ret = Send(tid, (const char *)&req, sizeof(req), (char *)&reply, sizeof(reply));
        if (ret < 0 || reply.result < 0) {
                return -1;
        }

        return 0;
}

void TerminalRxNotifierTask(void) {
        int server_tid = MyParentTid();
        terminal_request_t req;
        terminal_reply_t reply;
        unsigned char ch;

        for (;;) {
                AwaitEvent(EVENT_TIMER);

                while (uart_try_getc(CONSOLE, (char *)&ch) == 0) {
                        req.type = TERM_MSG_RX_READY;
                        req.ch = ch;
                        Send(server_tid, (const char *)&req, sizeof(req), (char *)&reply, sizeof(reply));
                }
        }
}

void TerminalTxNotifierTask(void) {
        int server_tid = MyParentTid();
        terminal_request_t req;
        terminal_reply_t reply;

        for (;;) {
                req.type = TERM_MSG_TX_READY;
                req.ch = 0;

                Send(server_tid, (const char *)&req, sizeof(req), (char *)&reply, sizeof(reply));

                if (reply.result == 0) {
                        AwaitEvent(EVENT_UART0_TX);
                        while (uart_try_putc(CONSOLE, (char)reply.ch) != 0) {
                                AwaitEvent(EVENT_UART0_TX);
                        }
                }
        }
}

void TerminalServerTask(void) {
        int sender_tid;
        terminal_request_t req;
        terminal_reply_t reply;
        unsigned char ch;
        int waiting_tid;

        RegisterAs(TERMINAL_SERVER_NAME);

        uart_puts(CONSOLE, "TerminalServer: registered as terminal0\r\n");


        for (;;) {
                Receive(&sender_tid, (char *)&req, sizeof(req));

                switch (req.type) {
                case TERM_MSG_GETC:
                        if (rx_pop(&ch) == 0) {
                                reply_getc_client(sender_tid, ch);
                        } else {
                                while (uart_try_getc(CONSOLE, (char *)&ch) != 0) {
                                        Yield();
                                }
                                reply_getc_client(sender_tid, ch);
                        }
                        break;

                case TERM_MSG_PUTC:
                        uart_putc(CONSOLE, (char)req.ch);
                        reply.result = 0;
                        reply.ch = 0;
                        Reply(sender_tid, (const char *)&reply, sizeof(reply));
                        break;

                case TERM_MSG_RX_READY:
                        if (waiting_getc_pop(&waiting_tid) == 0) {
                                reply_getc_client(waiting_tid, req.ch);
                        } else {
                                rx_push(req.ch);
                        }

                        reply.result = 0;
                        reply.ch = 0;
                        Reply(sender_tid, (const char *)&reply, sizeof(reply));
                        break;

                case TERM_MSG_TX_READY:
                        if (tx_pop(&ch) == 0) {
                                reply.result = 0;
                                reply.ch = ch;
                                Reply(sender_tid, (const char *)&reply, sizeof(reply));
                        } else {
                                tx_notifier_tid = sender_tid;
                        }
                        break;

                default:
                        reply.result = -1;
                        reply.ch = 0;
                        Reply(sender_tid, (const char *)&reply, sizeof(reply));
                        break;
                }
        }
}

static void term_puts(int tid, const char *s) {
        while (*s) {
                Putc(tid, (unsigned char)*s);
                s++;
        }
}


static int is_space(char c) {
        return c == ' ' || c == '\t';
}

static void skip_spaces(char **p) {
        while (**p && is_space(**p)) {
                (*p)++;
        }
}

static int parse_uint(char **p, int *out) {
        int value = 0;
        int seen = 0;

        skip_spaces(p);

        while (**p >= '0' && **p <= '9') {
                value = value * 10 + (**p - '0');
                (*p)++;
                seen = 1;
        }

        if (!seen) {
                return -1;
        }

        *out = value;
        return 0;
}

static int starts_with(const char *s, const char *prefix) {
        while (*prefix) {
                if (*s != *prefix) {
                        return 0;
                }
                s++;
                prefix++;
        }

        return 1;
}

static void term_put_uint(int tid, unsigned int value) {
        char buf[12];
        int i = 0;
        int j;

        if (value == 0) {
                Putc(tid, '0');
                return;
        }

        while (value > 0 && i < 11) {
                buf[i++] = (char)('0' + (value % 10));
                value /= 10;
        }

        for (j = i - 1; j >= 0; j--) {
                Putc(tid, (unsigned char)buf[j]);
        }
}

static void print_help(int terminal_tid) {
        term_puts(terminal_tid, "\r\nCommands:\r\n");
        term_puts(terminal_tid, "  tr <train> <speed>   set train speed 0..14, example: tr 14 5\r\n");
        term_puts(terminal_tid, "  rv <train>           reverse train direction, example: rv 14\r\n");
        term_puts(terminal_tid, "  sw <switch> <S|C>    throw switch straight/curved, example: sw 1 S\r\n");
        term_puts(terminal_tid, "  help                 show this help\r\n");
        term_puts(terminal_tid, "  q                    quit command client\r\n");
}

static void run_command(int terminal_tid, int can_tid, char *line) {
        char *p = line;
        int a;
        int b;
        int result;

        skip_spaces(&p);

        if (*p == 0) {
                return;
        }

        if (starts_with(p, "help")) {
                print_help(terminal_tid);
                return;
        }

        if (p[0] == 'q' && p[1] == 0) {
                term_puts(terminal_tid, "\r\nK4 train command client done.\r\n");
                Exit();
        }

        if (starts_with(p, "tr")) {
                p += 2;
                if (parse_uint(&p, &a) < 0 || parse_uint(&p, &b) < 0) {
                        term_puts(terminal_tid, "\r\ninvalid tr command\r\n");
                        return;
                }

                result = CanTrainSetSpeed(can_tid, a, b);
                term_puts(terminal_tid, "\r\ntr sent: train=");
                term_put_uint(terminal_tid, (unsigned int)a);
                term_puts(terminal_tid, " speed=");
                term_put_uint(terminal_tid, (unsigned int)b);
                term_puts(terminal_tid, result == 0 ? " ok\r\n" : " failed\r\n");
                return;
        }

        if (starts_with(p, "rv")) {
                p += 2;
                if (parse_uint(&p, &a) < 0) {
                        term_puts(terminal_tid, "\r\ninvalid rv command\r\n");
                        return;
                }

                result = CanTrainReverse(can_tid, a);
                term_puts(terminal_tid, "\r\nrv sent: train=");
                term_put_uint(terminal_tid, (unsigned int)a);
                term_puts(terminal_tid, result == 0 ? " ok\r\n" : " failed\r\n");
                return;
        }

        if (starts_with(p, "sw")) {
                char dir;

                p += 2;
                if (parse_uint(&p, &a) < 0) {
                        term_puts(terminal_tid, "\r\ninvalid sw command\r\n");
                        return;
                }

                skip_spaces(&p);
                dir = *p;
                if (!(dir == 'S' || dir == 's' || dir == 'C' || dir == 'c')) {
                        term_puts(terminal_tid, "\r\ninvalid sw direction\r\n");
                        return;
                }

                result = CanSwitch(can_tid, a, dir);
                term_puts(terminal_tid, "\r\nsw sent: switch=");
                term_put_uint(terminal_tid, (unsigned int)a);
                term_puts(terminal_tid, " dir=");
                Putc(terminal_tid, (unsigned char)dir);
                term_puts(terminal_tid, result == 0 ? " ok\r\n" : " failed\r\n");
                return;
        }

        term_puts(terminal_tid, "\r\nunknown command\r\n");
}

void K4TerminalTestTask(void) {
        int terminal_tid;
        int can_tid;
        char line[64];
        int len = 0;

        terminal_tid = WhoIs(TERMINAL_SERVER_NAME);
        can_tid = WhoIs(CAN_SERVER_NAME);

        term_puts(terminal_tid, "\r\nK4 Interrupt-Mediated I/O Train Controller\r\n");
        term_puts(terminal_tid, "Terminal server provides Getc/Putc. CAN server provides CanSend/CanReceive.\r\n");
        print_help(terminal_tid);

        term_puts(terminal_tid, "\r\nAutomatic K4 command test starting...\r\n");

        term_puts(terminal_tid, "\r\n[TEST] help\r\n");
        run_command(terminal_tid, can_tid, "help");

        term_puts(terminal_tid, "\r\n[TEST] unknown command\r\n");
        run_command(terminal_tid, can_tid, "hello");

        term_puts(terminal_tid, "\r\n[TEST] invalid tr command\r\n");
        run_command(terminal_tid, can_tid, "tr");

        term_puts(terminal_tid, "\r\n[TEST] tr 14 0\r\n");
        run_command(terminal_tid, can_tid, "tr 14 0");

        term_puts(terminal_tid, "\r\n[TEST] tr 14 5\r\n");
        run_command(terminal_tid, can_tid, "tr 14 5");

        term_puts(terminal_tid, "\r\n[TEST] tr 14 14\r\n");
        run_command(terminal_tid, can_tid, "tr 14 14");

        term_puts(terminal_tid, "\r\n[TEST] tr 14 15 should fail\r\n");
        run_command(terminal_tid, can_tid, "tr 14 15");

        term_puts(terminal_tid, "\r\n[TEST] tr 300 5 should fail\r\n");
        run_command(terminal_tid, can_tid, "tr 300 5");

        term_puts(terminal_tid, "\r\n[TEST] invalid rv command\r\n");
        run_command(terminal_tid, can_tid, "rv");

        term_puts(terminal_tid, "\r\n[TEST] rv 14\r\n");
        run_command(terminal_tid, can_tid, "rv 14");

        term_puts(terminal_tid, "\r\n[TEST] rv 300 should fail\r\n");
        run_command(terminal_tid, can_tid, "rv 300");

        term_puts(terminal_tid, "\r\n[TEST] invalid sw command\r\n");
        run_command(terminal_tid, can_tid, "sw");

        term_puts(terminal_tid, "\r\n[TEST] sw 1 S\r\n");
        run_command(terminal_tid, can_tid, "sw 1 S");

        term_puts(terminal_tid, "\r\n[TEST] sw 1 C\r\n");
        run_command(terminal_tid, can_tid, "sw 1 C");

        term_puts(terminal_tid, "\r\n[TEST] sw 1 X should be invalid\r\n");
        run_command(terminal_tid, can_tid, "sw 1 X");

        term_puts(terminal_tid, "\r\n[TEST] sw 300 S should fail\r\n");
        run_command(terminal_tid, can_tid, "sw 300 S");

        term_puts(terminal_tid, "\r\nAutomatic K4 command test complete.\r\n");

        term_puts(terminal_tid, "\r\nManual prompt is still available if keyboard input works.\r\n> ");

        for (;;) {
                int ch = Getc(terminal_tid);

                if (ch < 0) {
                        continue;
                }

                if (ch == '\r' || ch == '\n') {
                        Putc(terminal_tid, '\r');
                        Putc(terminal_tid, '\n');
                        line[len] = 0;
                        run_command(terminal_tid, can_tid, line);
                        len = 0;
                        term_puts(terminal_tid, "> ");
                        continue;
                }

                if (ch == 8 || ch == 127) {
                        if (len > 0) {
                                len--;
                                term_puts(terminal_tid, "\b \b");
                        }
                        continue;
                }

                if (len < (int)sizeof(line) - 1) {
                        line[len++] = (char)ch;
                        Putc(terminal_tid, (unsigned char)ch);
                } else {
                        term_puts(terminal_tid, "\r\ncommand too long\r\n> ");
                        len = 0;
                }
        }
}

