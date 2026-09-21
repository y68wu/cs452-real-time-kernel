#ifndef _mcp2515_h_
#define _mcp2515_h_ 1

#include <stdint.h>

/*
 * These are aggregate controller-mode bounds measured in complete CANSTAT
 * register transactions, not inner SPI spins.  Runtime recovery is kept much
 * smaller because it executes after the cooperative kernel is live.
 */
#ifndef MCP2515_INIT_MODE_POLL_LIMIT
#define MCP2515_INIT_MODE_POLL_LIMIT 64u
#endif

#ifndef MCP2515_RUNTIME_MODE_POLL_LIMIT
#define MCP2515_RUNTIME_MODE_POLL_LIMIT 8u
#endif

#ifndef MCP2515_RESET_SETTLE_ITERATIONS
#define MCP2515_RESET_SETTLE_ITERATIONS 200000u
#endif

typedef struct {
        uint32_t id;
        uint8_t extended;
        uint8_t dlc;
        uint8_t data[8];
} mcp2515_frame_t;

#define MCP2515_TX_FAILED   -1
#define MCP2515_TX_PENDING   0
#define MCP2515_TX_COMPLETE  1

#define MCP2515_SEND_BUSY   -2
#define MCP2515_SEND_NOT_READY -3
#define MCP2515_RECV_NONE   -2
#define MCP2515_RECV_INVALID -3
#define MCP2515_IO_ERROR    -4

#define MCP2515_RX_OVERFLOW_0 0x01
#define MCP2515_RX_OVERFLOW_1 0x02

int mcp2515_init(void);
int mcp2515_tx_ready(void);
int mcp2515_send(const mcp2515_frame_t *frame);
int mcp2515_tx_status(void);
int mcp2515_tx_abort(void);
int mcp2515_recv(mcp2515_frame_t *frame);
int mcp2515_take_rx_overflow(void);

/*
 * Pure register decoder used by the driver and host tests. It preserves the
 * controller's real standard/extended format and rejects DLC values above 8.
 */
int mcp2515_decode_rx_registers(const uint8_t regs[13],
                                mcp2515_frame_t *frame);

#endif
