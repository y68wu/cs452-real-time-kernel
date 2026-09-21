#ifndef _mcp2515_h_
#define _mcp2515_h_ 1

#include <stdint.h>

typedef struct {
        uint32_t id;
        uint8_t extended;
        uint8_t dlc;
        uint8_t data[8];
} mcp2515_frame_t;

int mcp2515_init(void);
int mcp2515_tx_ready(void);
int mcp2515_send(const mcp2515_frame_t *frame);
int mcp2515_recv(mcp2515_frame_t *frame);

#endif
