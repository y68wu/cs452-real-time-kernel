#ifndef _mcp2515_h_
#define _mcp2515_h_ 1

#include <stdint.h>

#define CAN_MAX_DATA_LEN 8

typedef struct {
        uint32_t id;
        uint8_t extended;
        uint8_t dlc;
        uint8_t data[CAN_MAX_DATA_LEN];
} can_frame_t;

/** Initialize the MCP2515 CAN controller. Should be called after initializing GPIO. */
void mcp2515_init(void);

/** Try to send a CAN frame using TX buffer 0. Returns 1 if queued, 0 if TX buffer is busy. */
int mcp2515_try_send(const can_frame_t *frame);

/** Try to receive a CAN frame from RX buffer 0 or RX buffer 1. Returns 1 if a frame was read, 0 otherwise. */
int mcp2515_try_receive(can_frame_t *frame);

/** Report and drop a frame if RX0/RX1 has data. Kept for debug compatibility. */
int mcp2515_fakerecv(void);

#endif /* _mcp2515_h_ */
