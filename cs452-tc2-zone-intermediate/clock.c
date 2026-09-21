#include "clock.h"
#include "events.h"
#include "nameserver.h"
#include "syscall.h"
#include "timer.h"
#include "uart.h"

typedef struct {
        int used;
        int tid;
        int target_tick;
} clock_waiter_t;

static clock_waiter_t waiters[MAX_CLOCK_WAITERS];

static void waiters_init(void) {
        for (int i = 0; i < MAX_CLOCK_WAITERS; i++) {
                waiters[i].used = 0;
                waiters[i].tid = -1;
                waiters[i].target_tick = 0;
        }
}

static int waiters_add(int tid, int target_tick) {
        for (int i = 0; i < MAX_CLOCK_WAITERS; i++) {
                if (!waiters[i].used) {
                        waiters[i].used = 1;
                        waiters[i].tid = tid;
                        waiters[i].target_tick = target_tick;
                        return 0;
                }
        }

        return -1;
}

static void waiters_release_ready(int current_tick) {
        clock_reply_t rep;

        rep.status = 0;
        rep.ticks = current_tick;

        for (int i = 0; i < MAX_CLOCK_WAITERS; i++) {
                if (waiters[i].used && waiters[i].target_tick <= current_tick) {
                        int tid = waiters[i].tid;

                        waiters[i].used = 0;
                        waiters[i].tid = -1;
                        waiters[i].target_tick = 0;

                        Reply(tid, (const char *)&rep, sizeof(rep));
                }
        }
}

static int clock_send_request(clock_request_t *req) {
        int clock_tid;
        clock_reply_t rep;
        int ret;

        clock_tid = WhoIs(CLOCK_SERVER_NAME);
        if (clock_tid < 0) {
                return -1;
        }

        ret = Send(clock_tid, (const char *)req, sizeof(*req), (char *)&rep, sizeof(rep));
        if (ret < 0) {
                return ret;
        }

        if (rep.status < 0) {
                return rep.status;
        }

        return rep.ticks;
}

int Time(void) {
        clock_request_t req;

        req.type = CLOCK_MSG_TIME;
        req.ticks = 0;

        return clock_send_request(&req);
}

int Delay(int ticks) {
        clock_request_t req;

        req.type = CLOCK_MSG_DELAY;
        req.ticks = ticks;

        return clock_send_request(&req);
}

int DelayUntil(int ticks) {
        clock_request_t req;

        req.type = CLOCK_MSG_DELAY_UNTIL;
        req.ticks = ticks;

        return clock_send_request(&req);
}

void ClockNotifierTask(void) {
        int clock_tid = MyParentTid();
        clock_request_t req;
        clock_reply_t rep;

        req.type = CLOCK_MSG_TICK;
        req.ticks = 0;

        for (;;) {
                int await_ret = AwaitEvent(EVENT_TIMER);
                if (await_ret < 0) {
                        uart_printf(CONSOLE, "ClockNotifier: AwaitEvent failed %d\r\n", await_ret);
                        Exit();
                }

                int send_ret = Send(clock_tid, (const char *)&req, sizeof(req), (char *)&rep, sizeof(rep));
                if (send_ret < 0) {
                        uart_printf(CONSOLE, "ClockNotifier: Send failed %d\r\n", send_ret);
                        Exit();
                }
        }
}

void ClockServerTask(void) {
        int current_tick = 0;
        int notifier_tid;

        waiters_init();

        if (RegisterAs(CLOCK_SERVER_NAME) < 0) {
                uart_puts(CONSOLE,
                          "ClockServer: registration failed\r\n");
                Exit();
        }
        uart_printf(CONSOLE, "ClockServer: registered as %s\r\n", CLOCK_SERVER_NAME);

        notifier_tid = Create(1, ClockNotifierTask);
        if (notifier_tid < 0) {
                uart_puts(CONSOLE,
                          "ClockServer: notifier creation failed\r\n");
                Exit();
        }
        uart_printf(CONSOLE, "ClockServer: created notifier %d\r\n", notifier_tid);

        for (;;) {
                int sender_tid = -1;
                clock_request_t req;
                clock_reply_t rep;

                Receive(&sender_tid, (char *)&req, sizeof(req));

                rep.status = 0;
                rep.ticks = current_tick;

                if (req.type == CLOCK_MSG_TICK) {
                        current_tick++;
                        rep.ticks = current_tick;
                        Reply(sender_tid, (const char *)&rep, sizeof(rep));
                        waiters_release_ready(current_tick);
                } else if (req.type == CLOCK_MSG_TIME) {
                        rep.ticks = current_tick;
                        Reply(sender_tid, (const char *)&rep, sizeof(rep));
                } else if (req.type == CLOCK_MSG_DELAY) {
                        if (req.ticks < 0) {
                                rep.status = -2;
                                Reply(sender_tid, (const char *)&rep, sizeof(rep));
                        } else if (req.ticks == 0) {
                                rep.ticks = current_tick;
                                Reply(sender_tid, (const char *)&rep, sizeof(rep));
                        } else {
                                int target_tick = current_tick + req.ticks;
                                if (waiters_add(sender_tid, target_tick) < 0) {
                                        rep.status = -3;
                                        Reply(sender_tid, (const char *)&rep, sizeof(rep));
                                }
                        }
                } else if (req.type == CLOCK_MSG_DELAY_UNTIL) {
                        if (req.ticks <= current_tick) {
                                rep.ticks = current_tick;
                                Reply(sender_tid, (const char *)&rep, sizeof(rep));
                        } else {
                                if (waiters_add(sender_tid, req.ticks) < 0) {
                                        rep.status = -3;
                                        Reply(sender_tid, (const char *)&rep, sizeof(rep));
                                }
                        }
                } else {
                        rep.status = -1;
                        Reply(sender_tid, (const char *)&rep, sizeof(rep));
                }
        }
}

void ClockClientTask(void) {
        int parent_tid = MyParentTid();
        int my_tid = MyTid();
        int clock_tid;
        int dummy = 0;
        clock_client_config_t cfg;

        Send(parent_tid, (const char *)&dummy, sizeof(dummy), (char *)&cfg, sizeof(cfg));

        clock_tid = WhoIs(CLOCK_SERVER_NAME);
        uart_printf(CONSOLE,
                    "ClockClient tid=%d clock_tid=%d delay=%d count=%d\r\n",
                    my_tid, clock_tid, cfg.delay_ticks, cfg.num_delays);

        for (int i = 1; i <= cfg.num_delays; i++) {
                int now = Delay(cfg.delay_ticks);
                uart_printf(CONSOLE,
                            "ClockClient tid=%d delay=%d completed=%d/%d time=%d\r\n",
                            my_tid, cfg.delay_ticks, i, cfg.num_delays, now);
        }

        uart_printf(CONSOLE, "ClockClient tid=%d done\r\n", my_tid);
        Exit();
}

void IdleTask(void) {
#if defined(MODE_TC1) || defined(MODE_TC2)
        /*
         * Train-control builds need a stable interactive terminal and clean
         * timestamped calibration output.  Direct periodic UART output here
         * can fill or interleave with the command stream, so both the TC1
         * calibration harness and TC2 steady-state idle paths remain silent.
         */
        for (;;) {
                Yield();
        }
#else
        unsigned long long start = timer_get_usec();
        unsigned long long last = start;
        unsigned long long loops = 0;

        for (;;) {
                unsigned long long now;

                loops++;
                now = timer_get_usec();

                if (now - last >= 1000000u) {
                        unsigned long long elapsed = now - start;
                        unsigned long long idle_percent = 100u;

                        uart_printf(CONSOLE,
                                    "IdleTask: elapsed_us=%u loops=%u approx_idle=%u%%\r\n",
                                    (unsigned int)elapsed,
                                    (unsigned int)loops,
                                    (unsigned int)idle_percent);

                        last = now;
                        loops = 0;
                }

                Yield();
        }
#endif
}
