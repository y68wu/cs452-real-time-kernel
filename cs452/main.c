#include <stddef.h>
#include <stdint.h>

#include "rpi.h"
#include "mcp2515.h"
#include "train.h"
#include "uart.h"

extern void setup_mmu(); // in mmu.S

#define OUTBUF_SIZE 4096
#define CMDBUF_SIZE 80

static char outbuf[OUTBUF_SIZE];
static unsigned int out_head = 0;
static unsigned int out_tail = 0;

static char cmd_buf[CMDBUF_SIZE];
static unsigned int cmd_len = 0;

static uint32_t start_time_us = 0;
static uint32_t last_clock_update_us = 0;
static uint32_t max_loop_us = 0;
static char switch_pos[256];

static int outbuf_empty(void) {
        return out_head == out_tail;
}

static int outbuf_full(void) {
        return ((out_tail + 1) % OUTBUF_SIZE) == out_head;
}

static void tty_enqueue_char(char c) {
        if (outbuf_full()) return;
        outbuf[out_tail] = c;
        out_tail = (out_tail + 1) % OUTBUF_SIZE;
}

static void tty_enqueue_string(const char *s) {
        while (*s) {
                tty_enqueue_char(*s);
                s++;
        }
}

static void tty_enqueue_uint(unsigned int x) {
        char buf[12];
        unsigned int i = 0;

        if (x == 0) {
                tty_enqueue_char('0');
                return;
        }

        while (x > 0 && i < sizeof(buf)) {
                buf[i++] = (char)('0' + (x % 10));
                x /= 10;
        }

        while (i > 0) {
                tty_enqueue_char(buf[--i]);
        }
}

static void tty_enqueue_two_digits(unsigned int x) {
        tty_enqueue_char((char)('0' + ((x / 10) % 10)));
        tty_enqueue_char((char)('0' + (x % 10)));
}

static void tty_flush_one(void) {
        if (outbuf_empty()) return;
        if (!uart_try_putc(CONSOLE, outbuf[out_head])) return;
        out_head = (out_head + 1) % OUTBUF_SIZE;
}

static void draw_prompt(void) {
        tty_enqueue_string("\r\n> ");
}

static void set_status(const char *s) {
        tty_enqueue_string("\r\nStatus: ");
        tty_enqueue_string(s);
        draw_prompt();
}

static void set_status_uint2(const char *prefix, unsigned int a, const char *mid, unsigned int b, const char *suffix) {
        tty_enqueue_string("\r\nStatus: ");
        tty_enqueue_string(prefix);
        tty_enqueue_uint(a);
        tty_enqueue_string(mid);
        if (mid[0] != '\0') {
                tty_enqueue_uint(b);
        }
        tty_enqueue_string(suffix);
        draw_prompt();
}

static void draw_static_screen(void) {
        tty_enqueue_string("\r\nCS452 A0 Polling Train Controller\r\n");
        tty_enqueue_string("Time: 00:00.0\r\n\r\n");
        tty_enqueue_string("Switch positions:\r\n");
        tty_enqueue_string("(none set yet)\r\n\r\n");
        tty_enqueue_string("Recent sensors:\r\n");
        tty_enqueue_string("(sensor display disabled in clean terminal build)\r\n");
        tty_enqueue_string("Status: started\r\n");
        tty_enqueue_string("Max loop time: 0 us\r\n");
        draw_prompt();
}

static void print_switch_positions(void) {
        int any = 0;

        tty_enqueue_string("\r\nSwitch positions:\r\n");

        for (unsigned int i = 1; i < 256; i++) {
                if (switch_pos[i] == 'S' || switch_pos[i] == 'C') {
                        tty_enqueue_string("  sw ");
                        tty_enqueue_uint(i);
                        tty_enqueue_string(": ");
                        tty_enqueue_char(switch_pos[i]);
                        tty_enqueue_string("\r\n");
                        any = 1;
                }
        }

        if (!any) {
                tty_enqueue_string("  (none set yet)\r\n");
        }

        draw_prompt();
}

static void print_clock(uint32_t now_us) {
        uint32_t elapsed_ds = (uint32_t)((now_us - start_time_us) / 100000u);
        unsigned int tenths = elapsed_ds % 10u;
        unsigned int total_seconds = elapsed_ds / 10u;
        unsigned int seconds = total_seconds % 60u;
        unsigned int minutes = (total_seconds / 60u) % 100u;

        tty_enqueue_string("\r\nTime: ");
        tty_enqueue_two_digits(minutes);
        tty_enqueue_char(':');
        tty_enqueue_two_digits(seconds);
        tty_enqueue_char('.');
        tty_enqueue_char((char)('0' + tenths));
}

static void update_loop_measurement(uint32_t loop_time_us) {
        if (loop_time_us <= max_loop_us) return;
        max_loop_us = loop_time_us;

        tty_enqueue_string("\r\nMax loop time: ");
        tty_enqueue_uint(max_loop_us);
        tty_enqueue_string(" us");
}

static char *skip_spaces(char *p) {
        while (*p == ' ') p++;
        return p;
}

static int parse_uint(char **p, unsigned int *out) {
        unsigned int value = 0;
        int seen_digit = 0;

        *p = skip_spaces(*p);

        while (**p >= '0' && **p <= '9') {
                seen_digit = 1;
                value = value * 10u + (unsigned int)(**p - '0');
                (*p)++;
        }

        if (!seen_digit) return 0;
        *out = value;
        return 1;
}

static int at_end(char *p) {
        p = skip_spaces(p);
        return *p == '\0';
}

static int parse_command(void) {
        char *p = cmd_buf;

        p = skip_spaces(p);

        if (p[0] == 'q' && (p[1] == '\0' || p[1] == ' ')) {
                set_status("quitting");
                return 1;
        }

        if (p[0] == 't' && p[1] == 'r' && p[2] == ' ') {
                unsigned int train = 0;
                unsigned int speed = 0;
                p += 2;
                if (!parse_uint(&p, &train) || !parse_uint(&p, &speed) || !at_end(p) || speed > 14u) {
                        set_status("invalid tr command");
                        return 0;
                }

                if (train_set_speed(train, speed)) {
                        set_status_uint2("queued tr train ", train, " speed ", speed, "");
                } else {
                        set_status("failed to queue tr command");
                }
                return 0;
        }

        if (p[0] == 'r' && p[1] == 'v' && p[2] == ' ') {
                unsigned int train = 0;
                p += 2;
                if (!parse_uint(&p, &train) || !at_end(p)) {
                        set_status("invalid rv command");
                        return 0;
                }

                if (train_reverse(train, timer_get_usec_low())) {
                        tty_enqueue_string("\r\nStatus: queued rv train ");
                        tty_enqueue_uint(train);
                        draw_prompt();
                } else {
                        set_status("failed to queue rv command");
                }
                return 0;
        }

        if (p[0] == 's' && p[1] == 'w' && p[2] == ' ') {
                unsigned int sw = 0;
                char direction;

                p += 2;
                if (!parse_uint(&p, &sw)) {
                        set_status("invalid sw command");
                        return 0;
                }

                p = skip_spaces(p);
                direction = *p;
                if (direction >= 'a' && direction <= 'z') {
                        direction = (char)(direction - 'a' + 'A');
                }
                if (!(direction == 'S' || direction == 'C')) {
                        set_status("invalid sw command");
                        return 0;
                }
                p++;
                if (!at_end(p)) {
                        set_status("invalid sw command");
                        return 0;
                }

                if (train_switch(sw, direction)) {
                        if (sw < 256) {
                                switch_pos[sw] = direction;
                        }

                        tty_enqueue_string("\r\nStatus: queued sw ");
                        tty_enqueue_uint(sw);
                        tty_enqueue_string(" ");
                        tty_enqueue_char(direction);

                        print_switch_positions();
                } else {
                        set_status("failed to queue sw command");
                }
                return 0;
        }

        set_status("unknown command");
        return 0;
}

static int handle_console_input(void) {
        char c;

        if (!uart_try_getc(CONSOLE, &c)) return 0;

        if (c == '\r' || c == '\n') {
                tty_enqueue_string("\r\n");
                cmd_buf[cmd_len] = '\0';
                int should_quit = parse_command();
                cmd_len = 0;
                if (!should_quit) draw_prompt();
                return should_quit;
        }

        if (c == 8 || c == 127) {
                if (cmd_len > 0) {
                        cmd_len--;
                        tty_enqueue_string("\b \b");
                }
                return 0;
        }

        if (c >= 32 && c <= 126) {
                if (cmd_len + 1 < CMDBUF_SIZE) {
                        cmd_buf[cmd_len++] = c;
                        tty_enqueue_char(c);
                } else {
                        cmd_len = 0;
                        set_status("command too long");
                }
        }

        return 0;
}

static void poll_can_debug(uint32_t now_us) {
        (void)now_us;
        /* Disabled in clean terminal build. */
}

int kmain() {
#if defined(MMU)
        setup_mmu();
#endif

        gpio_init();
        uart_config_and_enable(CONSOLE);

        uart_puts(CONSOLE, "\r\nA0 booting...\r\n");
        uart_puts(CONSOLE, "Initializing CAN...\r\n");

        mcp2515_init();

        uart_puts(CONSOLE, "CAN init returned.\r\n");

        train_init();

        start_time_us = timer_get_usec_low();
        last_clock_update_us = start_time_us;

        draw_static_screen();

        for (;;) {
                uint32_t loop_start = timer_get_usec_low();
                uint32_t now = loop_start;

                if ((uint32_t)(now - last_clock_update_us) >= 100000u) {
                        last_clock_update_us += 100000u;

                        /*
                         * In clean terminal mode, do not print the clock while
                         * the user is typing a command. Otherwise the clock line
                         * interleaves with command input in gtkterm.
                         *
                         * Print only every 10 seconds, and only when the output
                         * buffer is empty.
                         */
                        if (cmd_len == 0 && outbuf_empty()
                            && (((uint32_t)((now - start_time_us) / 100000u) % 100u) == 0u)) {
                                print_clock(now);
                                draw_prompt();
                        }
                }

                if (handle_console_input()) {
                        return 0;
                }

                train_poll(now);
                poll_can_debug(now);
                tty_flush_one();

                uint32_t loop_end = timer_get_usec_low();
                update_loop_measurement((uint32_t)(loop_end - loop_start));
        }
}

#if !defined(MMU)
void* memset(void *s, int c, size_t n) {
        for (char* it = (char*)s; n > 0; --n) *it++ = c;
        return s;
}

void* memcpy(void* dest, const void* src, size_t n) {
        char* sit = (char*)src;
        char* cdest = (char*)dest;
        for (size_t i = 0; i < n; ++i) *cdest++ = *sit++;
        return dest;
}
#endif
