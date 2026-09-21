#include "syscall.h"
#include "task.h"
#include "uart.h"
#include "nameserver.h"
#include "rps.h"
#include "perf.h"
#include "clock.h"
#include "terminal.h"
#include "can.h"
#include "train_control.h"

#ifdef MODE_PERF
static void run_perf_case(const char *label, int order_is_receive_first, int msg_size) {
        int receiver_tid;
        int sender_tid;

        if (order_is_receive_first) {
                receiver_tid = Create(3, PerfReceiverTask);
                perf_configure_case(receiver_tid, msg_size, PERF_ITERATIONS, label);
                sender_tid = Create(4, PerfSenderTask);
                uart_printf(CONSOLE, "PERF configured order=%s receiver=%d sender=%d size=%d iter=%d\r\n",
                            label, receiver_tid, sender_tid, msg_size, PERF_ITERATIONS);
        } else {
                receiver_tid = Create(4, PerfReceiverTask);
                perf_configure_case(receiver_tid, msg_size, PERF_ITERATIONS, label);
                sender_tid = Create(3, PerfSenderTask);
                uart_printf(CONSOLE, "PERF configured order=%s receiver=%d sender=%d size=%d iter=%d\r\n",
                            label, receiver_tid, sender_tid, msg_size, PERF_ITERATIONS);
        }

        Yield();
}
#endif

void FirstUserTask(void) {
#ifdef MODE_PERF
        uart_puts(CONSOLE, "FirstUserTask: starting PERF batch test\r\n");

        run_perf_case("S", 0, 4);
        run_perf_case("S", 0, 64);
        run_perf_case("S", 0, 256);

        run_perf_case("R", 1, 4);
        run_perf_case("R", 1, 64);
        run_perf_case("R", 1, 256);

        uart_puts(CONSOLE, "FirstUserTask: PERF batch complete\r\n");
        Exit();
#elif defined(MODE_TC1)
        int tid;

        uart_puts(CONSOLE, "FirstUserTask: starting TC1 train control\r\n");

        uart_puts(CONSOLE, "FirstUserTask: creating name server\r\n");
        tid = Create(1, NameServerTask);
        uart_printf(CONSOLE, "Created NameServer: %d\r\n", tid);

        uart_puts(CONSOLE, "FirstUserTask: creating clock server\r\n");
        tid = Create(2, ClockServerTask);
        uart_printf(CONSOLE, "Created ClockServer: %d\r\n", tid);

        uart_puts(CONSOLE, "FirstUserTask: creating CAN server\r\n");
        tid = Create(3, CanServerTask);
        uart_printf(CONSOLE, "Created CanServer: %d\r\n", tid);

        uart_puts(CONSOLE, "FirstUserTask: creating terminal server\r\n");
        tid = Create(4, TerminalServerTask);
        uart_printf(CONSOLE, "Created TerminalServer: %d\r\n", tid);

        uart_puts(CONSOLE, "FirstUserTask: creating idle task\r\n");
        tid = Create(15, IdleTask);
        uart_printf(CONSOLE, "Created IdleTask: %d\r\n", tid);

        uart_puts(CONSOLE, "FirstUserTask: creating train control task\r\n");
        tid = Create(5, TrainControlTask);
        uart_printf(CONSOLE, "Created TrainControlTask: %d\r\n", tid);

        uart_puts(CONSOLE, "FirstUserTask: exiting\r\n");
        Exit();
#elif defined(MODE_K4)
        int tid;

        uart_puts(CONSOLE, "FirstUserTask: starting K4 terminal I/O test\r\n");

        uart_puts(CONSOLE, "FirstUserTask: creating name server\r\n");
        tid = Create(1, NameServerTask);
        uart_printf(CONSOLE, "Created NameServer: %d\r\n", tid);

        uart_puts(CONSOLE, "FirstUserTask: creating CAN server\r\n");
        tid = Create(2, CanServerTask);
        uart_printf(CONSOLE, "Created CanServer: %d\r\n", tid);

        uart_puts(CONSOLE, "FirstUserTask: creating terminal server\r\n");
        tid = Create(3, TerminalServerTask);
        uart_printf(CONSOLE, "Created TerminalServer: %d\r\n", tid);

        uart_puts(CONSOLE, "FirstUserTask: creating idle task\r\n");
        tid = Create(15, IdleTask);
        uart_printf(CONSOLE, "Created IdleTask: %d\r\n", tid);

        uart_puts(CONSOLE, "FirstUserTask: creating terminal test client\r\n");
        tid = Create(4, K4TerminalTestTask);
        uart_printf(CONSOLE, "Created K4TerminalTestTask: %d\r\n", tid);

        uart_puts(CONSOLE, "FirstUserTask: exiting\r\n");
        Exit();
#elif defined(MODE_CLOCK)
        int tid;
        int sender_tid;
        int dummy;
        clock_client_config_t configs[4] = {
                {10, 20},
                {23, 9},
                {33, 6},
                {71, 3}
        };

        uart_puts(CONSOLE, "FirstUserTask: creating name server\r\n");
        tid = Create(1, NameServerTask);
        uart_printf(CONSOLE, "Created NameServer: %d\r\n", tid);

        uart_puts(CONSOLE, "FirstUserTask: creating clock server\r\n");
        tid = Create(2, ClockServerTask);
        uart_printf(CONSOLE, "Created ClockServer: %d\r\n", tid);

        uart_puts(CONSOLE, "FirstUserTask: creating idle task\r\n");
        tid = Create(15, IdleTask);
        uart_printf(CONSOLE, "Created IdleTask: %d\r\n", tid);

        uart_puts(CONSOLE, "FirstUserTask: creating clock clients\r\n");
        tid = Create(3, ClockClientTask);
        uart_printf(CONSOLE, "Created ClockClient priority=3 tid=%d\r\n", tid);

        tid = Create(4, ClockClientTask);
        uart_printf(CONSOLE, "Created ClockClient priority=4 tid=%d\r\n", tid);

        tid = Create(5, ClockClientTask);
        uart_printf(CONSOLE, "Created ClockClient priority=5 tid=%d\r\n", tid);

        tid = Create(6, ClockClientTask);
        uart_printf(CONSOLE, "Created ClockClient priority=6 tid=%d\r\n", tid);

        for (int i = 0; i < 4; i++) {
                Receive(&sender_tid, (char *)&dummy, sizeof(dummy));
                Reply(sender_tid, (const char *)&configs[i], sizeof(configs[i]));
                uart_printf(CONSOLE,
                            "FirstUserTask: sent config to tid=%d delay=%d count=%d\r\n",
                            sender_tid, configs[i].delay_ticks, configs[i].num_delays);
        }

        uart_puts(CONSOLE, "FirstUserTask: exiting\r\n");
        Exit();
#else
        int tid;

        uart_puts(CONSOLE, "FirstUserTask: creating name server\r\n");
        tid = Create(1, NameServerTask);
        uart_printf(CONSOLE, "Created NameServer: %d\r\n", tid);

        uart_puts(CONSOLE, "FirstUserTask: creating RPS server\r\n");
        tid = Create(2, RPSServerTask);
        uart_printf(CONSOLE, "Created RPSServer: %d\r\n", tid);

        uart_puts(CONSOLE, "FirstUserTask: creating RPS clients\r\n");

        tid = Create(10, RPSClientOneTask);
        uart_printf(CONSOLE, "Created RPSClient1: %d\r\n", tid);

        tid = Create(10, RPSClientTwoTask);
        uart_printf(CONSOLE, "Created RPSClient2: %d\r\n", tid);

        tid = Create(10, RPSClientThreeTask);
        uart_printf(CONSOLE, "Created RPSClient3: %d\r\n", tid);

        tid = Create(10, RPSClientFourTask);
        uart_printf(CONSOLE, "Created RPSClient4: %d\r\n", tid);

        uart_puts(CONSOLE, "FirstUserTask: exiting\r\n");
        Exit();
#endif
}

void TestTask(void) {
        int tid = MyTid();
        int parent = MyParentTid();

        uart_printf(CONSOLE, "TestTask: tid=%d parent=%d\r\n", tid, parent);
        Yield();
        uart_printf(CONSOLE, "TestTask: tid=%d parent=%d\r\n", tid, parent);
        Exit();
}

void task_exit_trampoline(void) {
        Exit();
        for (;;) {
        }
}
