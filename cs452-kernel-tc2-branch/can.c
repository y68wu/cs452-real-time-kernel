#include "can.h"
#include "mcp2515.h"
#include "spi.h"
#include "syscall.h"
#include "events.h"
#include "nameserver.h"
#include "uart.h"

static unsigned long can_irq_save(void) {
        unsigned long flags;
        __asm__ volatile(
                "mrs %0, daif\n"
                "msr daifset, #0xf\n"
                "isb\n"
                : "=r"(flags)
                :
                : "memory");
        return flags;
}

static void can_irq_restore(unsigned long flags) {
        __asm__ volatile(
                "msr daif, %0\n"
                "isb\n"
                :
                : "r"(flags)
                : "memory");
}

static int can_hw_initialized = 0;

static int can_hw_init_once(void) {
        if (can_hw_initialized) {
                return 0;
        }

        spi_init();

        if (mcp2515_init() != 0) {
                return -1;
        }

        /*
         * Important: RAW14 test worked after init had time to settle before
         * the user pressed g.  In kernel mode, do the same explicitly.
         */
        for (volatile int d = 0; d < 100000000; d++) {
        }

        can_hw_initialized = 1;
        uart_puts(CONSOLE, "CAN: hw init ok and settled\r\n");
        return 0;
}


#define CAN_MSG_SEND     1
#define CAN_MSG_RECV     2
#define CAN_MSG_RX_FRAME 3
#define CAN_MSG_TX_POLL  4

#define CAN_TX_Q_SIZE 32
#define CAN_RX_Q_SIZE 32
#define CAN_WAIT_RECV_SIZE 16

typedef struct {
        int type;
        can_frame_t frame;
} can_request_t;

typedef struct {
        int result;
        can_frame_t frame;
} can_reply_t;

typedef struct {
        int sender_tid;
        can_frame_t frame;
} tx_item_t;

static tx_item_t tx_q[CAN_TX_Q_SIZE];
static int tx_head = 0;
static int tx_tail = 0;
static int tx_count = 0;

static can_frame_t rx_q[CAN_RX_Q_SIZE];
static int rx_head = 0;
static int rx_tail = 0;
static int rx_count = 0;

static int waiting_recv[CAN_WAIT_RECV_SIZE];
static int wait_head = 0;
static int wait_tail = 0;
static int wait_count = 0;

static can_frame_t to_can_frame(const mcp2515_frame_t *m) {
        can_frame_t c;
        c.id = m->id;
        c.extended = m->extended;
        c.dlc = m->dlc;
        for (int i = 0; i < 8; i++) {
                c.data[i] = m->data[i];
        }
        return c;
}

static mcp2515_frame_t to_mcp_frame(const can_frame_t *c) {
        mcp2515_frame_t m;
        m.id = c->id;
        m.extended = c->extended;
        m.dlc = c->dlc;
        for (int i = 0; i < 8; i++) {
                m.data[i] = c->data[i];
        }
        return m;
}

static int tx_push(int tid, const can_frame_t *frame) {
        if (tx_count >= CAN_TX_Q_SIZE) {
                return -1;
        }

        tx_q[tx_tail].sender_tid = tid;
        tx_q[tx_tail].frame = *frame;
        tx_tail = (tx_tail + 1) % CAN_TX_Q_SIZE;
        tx_count++;
        return 0;
}

static tx_item_t *tx_peek(void) {
        if (tx_count <= 0) {
                return 0;
        }

        return &tx_q[tx_head];
}

static void tx_pop(void) {
        if (tx_count > 0) {
                tx_head = (tx_head + 1) % CAN_TX_Q_SIZE;
                tx_count--;
        }
}

static int rx_push(const can_frame_t *frame) {
        if (rx_count >= CAN_RX_Q_SIZE) {
                return -1;
        }

        rx_q[rx_tail] = *frame;
        rx_tail = (rx_tail + 1) % CAN_RX_Q_SIZE;
        rx_count++;
        return 0;
}

static int rx_pop(can_frame_t *frame) {
        if (rx_count <= 0) {
                return -1;
        }

        *frame = rx_q[rx_head];
        rx_head = (rx_head + 1) % CAN_RX_Q_SIZE;
        rx_count--;
        return 0;
}

static int wait_push(int tid) {
        if (wait_count >= CAN_WAIT_RECV_SIZE) {
                return -1;
        }

        waiting_recv[wait_tail] = tid;
        wait_tail = (wait_tail + 1) % CAN_WAIT_RECV_SIZE;
        wait_count++;
        return 0;
}

static int wait_pop(int *tid) {
        if (wait_count <= 0) {
                return -1;
        }

        *tid = waiting_recv[wait_head];
        wait_head = (wait_head + 1) % CAN_WAIT_RECV_SIZE;
        wait_count--;
        return 0;
}

static void reply_send_result(int tid, int result) {
        can_reply_t reply;
        reply.result = result;
        reply.frame.id = 0;
        reply.frame.extended = 1;
        reply.frame.dlc = 0;
        Reply(tid, (const char *)&reply, sizeof(reply));
}

static void reply_recv_frame(int tid, const can_frame_t *frame) {
        can_reply_t reply;
        reply.result = 0;
        reply.frame = *frame;
        Reply(tid, (const char *)&reply, sizeof(reply));
}

static void try_send_queued(void) {
        while (tx_count > 0) {
                tx_item_t *item = tx_peek();
                mcp2515_frame_t m = to_mcp_frame(&item->frame);

                if (mcp2515_send(&m) != 0) {
                        return;
                }

                reply_send_result(item->sender_tid, 0);
                tx_pop();
        }
}

static uint32_t cs2_id(uint8_t can_id_value) {
        return (((uint32_t)can_id_value) << 17) | 0x4711u;
}

static void fill_dcc_loc_id(can_frame_t *f, int train) {
        f->data[0] = 0x00;
        f->data[1] = 0x00;
        f->data[2] = 0xC0;
        f->data[3] = (uint8_t)train;
}

int CanSend(int tid, const can_frame_t *frame) {
        mcp2515_frame_t m;
        int ret;

        (void)tid;

        if (!frame || frame->dlc > 8) {
                return -1;
        }

        m.id = frame->id;
        m.extended = frame->extended;
        m.dlc = frame->dlc;
        for (int i = 0; i < 8; i++) {
                m.data[i] = frame->data[i];
        }

        spi_init();
        if (mcp2515_init() != 0) {
                uart_puts(CONSOLE, "CAN: MCP2515 init failed\r\n");
                return -1;
        }

        for (volatile int d = 0; d < 50000000; d++) {
        }

        ret = mcp2515_send(&m);

        for (volatile int d = 0; d < 20000000; d++) {
        }

        return ret == 0 ? 0 : -1;
}

int CanReceive(int tid, can_frame_t *frame) {
        can_request_t req;
        can_reply_t reply;
        int ret;

        req.type = CAN_MSG_RECV;
        req.frame.id = 0;
        req.frame.extended = 1;
        req.frame.dlc = 0;

        ret = Send(tid, (const char *)&req, sizeof(req), (char *)&reply, sizeof(reply));
        if (ret < 0 || reply.result < 0) {
                return -1;
        }

        *frame = reply.frame;
        return 0;
}

int CanTrainSetSpeed(int tid, int train, int speed) {
        mcp2515_frame_t f;
        uint16_t value;
        int ret;

        (void)tid;

        if (train < 0 || train > 255 || speed < 0 || speed > 120) {
                return -1;
        }

        if (speed == 0) {
                value = 0x0000;
        } else {
                value = (uint16_t)((speed * 1000) / 120);
        }

        f.id = 0x00085772;
        f.extended = 1;
        f.dlc = 6;
        f.data[0] = 0x00;
        f.data[1] = 0x00;
        f.data[2] = 0x00;
        f.data[3] = (uint8_t)train;
        f.data[4] = (uint8_t)(value >> 8);
        f.data[5] = (uint8_t)value;
        f.data[6] = 0x00;
        f.data[7] = 0x00;

        spi_init();
        if (mcp2515_init() != 0) {
                uart_puts(CONSOLE, "CAN: train mcp2515 init failed\r\n");
                return -1;
        }

        for (volatile int d = 0; d < 50000000; d++) {
        }

        ret = mcp2515_send(&f);

        for (volatile int d = 0; d < 50000000; d++) {
        }

        return ret == 0 ? 0 : -1;
}

int CanTrainReverse(int tid, int train) {
        can_frame_t f;

        if (train < 0 || train > 255) {
                return -1;
        }

        f.id = cs2_id(0x0A);
        f.extended = 1;
        f.dlc = 5;
        fill_dcc_loc_id(&f, train);
        f.data[4] = 3;
        f.data[5] = 0;
        f.data[6] = 0;
        f.data[7] = 0;

        return CanSend(tid, &f);
}

int CanSwitch(int tid, int switch_no, char direction) {
        can_frame_t f;
        int can_switch_no;

        /*
         * The physical diagram labels are one higher than the CS3/CAN
         * accessory address observed on Track D:
         *
         *   sw 11  moves physical turnout 12
         *   sw 155 moves physical turnout 156
         *
         * Therefore user-facing switch numbers and route tables use the
         * physical diagram labels, while the CAN frame sends switch_no - 1.
         */
        if (switch_no <= 0 || switch_no > 256) {
                return -1;
        }

        can_switch_no = switch_no - 1;

        f.id = cs2_id(0x0B);
        f.extended = 1;
        f.dlc = 6;
        f.data[0] = 0x00;
        f.data[1] = 0x00;
        f.data[2] = 0x30;
        f.data[3] = (uint8_t)can_switch_no;
        f.data[4] = (direction == 'C' || direction == 'c') ? 1 : 0;
        f.data[5] = 1;
        f.data[6] = 0;
        f.data[7] = 0;

        return CanSend(tid, &f);
}

void CanRxNotifierTask(void) {
        int server_tid = MyParentTid();
        can_request_t req;
        can_reply_t reply;
        mcp2515_frame_t m;

        for (;;) {
                AwaitEvent(EVENT_TIMER);

                while (mcp2515_recv(&m) == 0) {
                        req.type = CAN_MSG_RX_FRAME;
                        req.frame = to_can_frame(&m);
                        Send(server_tid, (const char *)&req, sizeof(req), (char *)&reply, sizeof(reply));
                }
        }
}

void CanTxNotifierTask(void) {
        int server_tid = MyParentTid();
        can_request_t req;
        can_reply_t reply;

        for (;;) {
                AwaitEvent(EVENT_TIMER);

                req.type = CAN_MSG_TX_POLL;
                req.frame.id = 0;
                req.frame.extended = 1;
                req.frame.dlc = 0;

                Send(server_tid, (const char *)&req, sizeof(req), (char *)&reply, sizeof(reply));
        }
}

void CanServerTask(void) {
        int sender_tid;
        char msg[64];

        RegisterAs(CAN_SERVER_NAME);
        uart_puts(CONSOLE, "CanServer: registered as can0; initializing hardware\r\n");

        if (can_hw_init_once() != 0) {
                uart_puts(CONSOLE, "CanServer: MCP2515 init failed\r\n");
        }

        uart_puts(CONSOLE, "CanServer: ready\r\n");

        for (;;) {
                Receive(&sender_tid, msg, sizeof(msg));
                Reply(sender_tid, msg, 0);
        }
}
