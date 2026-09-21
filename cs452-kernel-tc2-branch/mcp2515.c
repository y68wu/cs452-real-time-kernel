#include "mcp2515.h"
#include "spi.h"

#define CNF3 0x28
#define CNF2 0x29
#define CNF1 0x2A

#define MCP_16MHz_250kbPS_CFG1 0x41
#define MCP_16MHz_250kbPS_CFG2 0xF1
#define MCP_16MHz_250kbPS_CFG3 0x85

#define INSTRUCTION_RESET       0xC0
#define INSTRUCTION_WRITE       0x02
#define INSTRUCTION_READ        0x03
#define INSTRUCTION_BIT_MODIFY  0x05
#define INSTRUCTION_READ_STATUS 0xA0
#define INSTRUCTION_RTS_TXB0    0x81

#define CANSTAT 0x0E
#define CANCTRL 0x0F
#define CANINTF 0x2C

#define CANSTAT_OPMOD 0xE0
#define CANCTRL_REQOP 0xE0
#define OPMODE_CONFIG 0x80
#define OPMODE_NORMAL 0x00

#define RXB0CTRL 0x60
#define RXB1CTRL 0x70

#define TXB0CTRL 0x30
#define TXB0SIDH 0x31
#define TXB0SIDL 0x32
#define TXB0EID8 0x33
#define TXB0EID0 0x34
#define TXB0DLC  0x35
#define TXB0D0   0x36

#define RXB0SIDH 0x61
#define RXB0SIDL 0x62
#define RXB0EID8 0x63
#define RXB0EID0 0x64
#define RXB0DLC  0x65
#define RXB0D0   0x66

#define CANINTF_RX0IF 0x01
#define TXBCTRL_TXREQ 0x08
#define STATUS_RX0    0x01


static void short_delay(void) {
        for (volatile int i = 0; i < 100000; i++) {
        }
}

static void mcp2515_read_regs(uint8_t reg, uint8_t values[], uint8_t n) {
        spi_begin_transaction();
        spi_transfer_one(INSTRUCTION_READ);
        spi_transfer_one(reg);
        for (uint8_t i = 0; i < n; i++) {
                values[i] = spi_transfer_one(0x00);
        }
        spi_end_transaction();
}

static uint8_t mcp2515_read_reg(uint8_t reg) {
        uint8_t ret = 0;
        mcp2515_read_regs(reg, &ret, 1);
        return ret;
}

static void mcp2515_write_regs(uint8_t reg, const uint8_t values[], uint8_t n) {
        spi_begin_transaction();
        spi_transfer_one(INSTRUCTION_WRITE);
        spi_transfer_one(reg);
        for (uint8_t i = 0; i < n; i++) {
                spi_transfer_one(values[i]);
        }
        spi_end_transaction();
}

static void mcp2515_write_reg(uint8_t reg, uint8_t value) {
        mcp2515_write_regs(reg, &value, 1);
}

static void mcp2515_modify_reg(uint8_t reg, uint8_t mask, uint8_t data) {
        spi_begin_transaction();
        spi_transfer_one(INSTRUCTION_BIT_MODIFY);
        spi_transfer_one(reg);
        spi_transfer_one(mask);
        spi_transfer_one(data);
        spi_end_transaction();
}

static uint8_t mcp2515_read_status(void) {
        uint8_t ret;

        spi_begin_transaction();
        spi_transfer_one(INSTRUCTION_READ_STATUS);
        ret = spi_transfer_one(0x00);
        spi_end_transaction();

        return ret;
}

static void encode_extended_id(uint32_t id, uint8_t *sidh, uint8_t *sidl, uint8_t *eid8, uint8_t *eid0) {
        id &= 0x1FFFFFFFu;

        *sidh = (uint8_t)(id >> 21);
        *sidl = (uint8_t)(((id >> 13) & 0xE0u) | 0x08u | ((id >> 16) & 0x03u));
        *eid8 = (uint8_t)(id >> 8);
        *eid0 = (uint8_t)id;
}

static uint32_t decode_extended_id(uint8_t sidh, uint8_t sidl, uint8_t eid8, uint8_t eid0) {
        uint32_t id = 0;

        id |= ((uint32_t)sidh) << 21;
        id |= ((uint32_t)(sidl & 0xE0u)) << 13;
        id |= ((uint32_t)(sidl & 0x03u)) << 16;
        id |= ((uint32_t)eid8) << 8;
        id |= eid0;

        return id & 0x1FFFFFFFu;
}

int mcp2515_init(void) {
        spi_begin_transaction();
        spi_transfer_one(INSTRUCTION_RESET);
        spi_end_transaction();

        for (volatile int d = 0; d < 200000; d++) {
        }

        mcp2515_modify_reg(CANCTRL, CANCTRL_REQOP, OPMODE_CONFIG);

        for (int i = 0; i < 200000; i++) {
                if ((mcp2515_read_reg(CANSTAT) & CANSTAT_OPMOD) == OPMODE_CONFIG) {
                        break;
                }
        }

        mcp2515_write_reg(CNF1, MCP_16MHz_250kbPS_CFG1);
        mcp2515_write_reg(CNF2, MCP_16MHz_250kbPS_CFG2);
        mcp2515_write_reg(CNF3, MCP_16MHz_250kbPS_CFG3);

        mcp2515_write_reg(RXB0CTRL, 0x64);
        mcp2515_write_reg(RXB1CTRL, 0x60);
        mcp2515_write_reg(CANINTF, 0x00);
        mcp2515_write_reg(TXB0CTRL, 0x00);

        mcp2515_modify_reg(CANCTRL, CANCTRL_REQOP, OPMODE_NORMAL);

        for (int i = 0; i < 500000; i++) {
                if ((mcp2515_read_reg(CANSTAT) & CANSTAT_OPMOD) == OPMODE_NORMAL) {
                        return 0;
                }
        }

        return -1;
}

int mcp2515_tx_ready(void) {
        return (mcp2515_read_reg(TXB0CTRL) & TXBCTRL_TXREQ) == 0;
}

int mcp2515_send(const mcp2515_frame_t *frame) {
        uint8_t regs[13];
        uint8_t dlc;
        uint8_t canstat;

        if (!frame || frame->dlc > 8) {
                return -1;
        }

        dlc = frame->dlc;

        canstat = mcp2515_read_reg(CANSTAT);
        if ((canstat & CANSTAT_OPMOD) != OPMODE_NORMAL) {
                mcp2515_modify_reg(CANCTRL, CANCTRL_REQOP, OPMODE_NORMAL);

                for (int i = 0; i < 500000; i++) {
                        canstat = mcp2515_read_reg(CANSTAT);
                        if ((canstat & CANSTAT_OPMOD) == OPMODE_NORMAL) {
                                break;
                        }
                }

                if ((canstat & CANSTAT_OPMOD) != OPMODE_NORMAL) {
                        return -3;
                }
        }

        if (!mcp2515_tx_ready()) {
                mcp2515_modify_reg(TXB0CTRL, TXBCTRL_TXREQ, 0x00);
                short_delay();
        }

        encode_extended_id(frame->id, &regs[0], &regs[1], &regs[2], &regs[3]);
        regs[4] = dlc & 0x0F;

        for (uint8_t i = 0; i < 8; i++) {
                regs[5 + i] = i < dlc ? frame->data[i] : 0;
        }

        mcp2515_write_regs(TXB0SIDH, regs, 13);

        spi_begin_transaction();
        spi_transfer_one(INSTRUCTION_RTS_TXB0);
        spi_end_transaction();

        return 0;
}

int mcp2515_recv(mcp2515_frame_t *frame) {
        uint8_t regs[13];

        if (!frame) {
                return -1;
        }

        if (!(mcp2515_read_status() & STATUS_RX0)) {
                return -2;
        }

        mcp2515_read_regs(RXB0SIDH, regs, 13);

        frame->id = decode_extended_id(regs[0], regs[1], regs[2], regs[3]);
        frame->extended = 1;
        frame->dlc = regs[4] & 0x0F;
        if (frame->dlc > 8) {
                frame->dlc = 8;
        }

        for (uint8_t i = 0; i < 8; i++) {
                frame->data[i] = regs[5 + i];
        }

        mcp2515_modify_reg(CANINTF, CANINTF_RX0IF, 0x00);

        return 0;
}
