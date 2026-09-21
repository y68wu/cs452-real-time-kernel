#include "mcp2515.h"
#include "spi.h"

// configuration registers
static const uint8_t CNF3 = 0x28;
static const uint8_t CNF2 = 0x29;
static const uint8_t CNF1 = 0x2A;

// MCP2515 configuration for 16 MHz clock and 250 kbit/s bitrate
static const uint8_t MCP_16MHz_250kbPS_CFG1 = 0x41;
static const uint8_t MCP_16MHz_250kbPS_CFG2 = 0xF1;
static const uint8_t MCP_16MHz_250kbPS_CFG3 = 0x85;

// MCP2515 normal operation mode
static const uint8_t OPMODE_NORMAL = 0x00;

// MCP2515 instruction set
static const uint8_t INSTRUCTION_WRITE       = 0x02;
static const uint8_t INSTRUCTION_READ        = 0x03;
static const uint8_t INSTRUCTION_BIT_MODIFY  = 0x05;
static const uint8_t INSTRUCTION_READ_STATUS = 0xA0;
static const uint8_t INSTRUCTION_RTS_TX0     = 0x81;

// MCP2515 status mask
static const uint8_t STATUS_RX0 = 0x01;
static const uint8_t STATUS_RX1 = 0x02;

// control and status registers
static const uint8_t CANSTAT = 0x0E;
static const uint8_t CANCTRL = 0x0F;

// OPMOD mask for CANSTAT register
static const uint8_t CANSTAT_OPMOD = 0xE0;

// REQOP mask for CANCTRL register
static const uint8_t CANCTRL_REQOP = 0xE0;

// flags register
static const uint8_t CANINTF = 0x2C;
static const uint8_t CANINTF_RX0IF = 0x01;
static const uint8_t CANINTF_RX1IF = 0x02;

// TX buffer 0 registers
static const uint8_t TXB0CTRL = 0x30;
static const uint8_t TXB0SIDH = 0x31;
static const uint8_t TXB0CTRL_TXREQ = 0x08;

// RX buffer register starts
static const uint8_t RXB0SIDH = 0x61;
static const uint8_t RXB1SIDH = 0x71;

// RX buffer control registers
static const uint8_t RXB0CTRL = 0x60;
static const uint8_t RXB1CTRL = 0x70;

/** Read n consecutive registers starting from the specified one. */
static void mcp2515_read_regs(uint8_t reg, uint8_t values[], const uint8_t n) {
        spi_begin_transaction();
        spi_transfer_one(INSTRUCTION_READ);
        spi_transfer_one(reg);
        for (uint8_t i = 0; i < n; i++) {
                values[i] = spi_transfer_one(0x00);
        }
        spi_end_transaction();
}

/** Read the value of a single register. */
static uint8_t mcp2515_read_reg(uint8_t reg) {
        uint8_t ret = 0;
        mcp2515_read_regs(reg, &ret, 1);
        return ret;
}

/** Write values to n consecutive registers starting from the specified one. */
static void mcp2515_write_regs(uint8_t reg, const uint8_t values[], const uint8_t n) {
        spi_begin_transaction();
        spi_transfer_one(INSTRUCTION_WRITE);
        spi_transfer_one(reg);
        for (uint8_t i = 0; i < n; i++) {
                spi_transfer_one(values[i]);
        }
        spi_end_transaction();
}

/** Write a value to a single register. */
static void mcp2515_write_reg(uint8_t reg, const uint8_t value) {
        mcp2515_write_regs(reg, &value, 1);
}

/** Modify individual bits of a register according to mask. */
static void mcp2515_modify_reg(uint8_t reg, const uint8_t mask, const uint8_t data) {
        spi_begin_transaction();
        spi_transfer_one(INSTRUCTION_BIT_MODIFY);
        spi_transfer_one(reg);
        spi_transfer_one(mask);
        spi_transfer_one(data);
        spi_end_transaction();
}

/** Read the status of the MCP2515, including RX and TX buffers. */
static uint8_t mcp2515_read_status(void) {
        uint8_t ret = 0;
        spi_begin_transaction();
        spi_transfer_one(INSTRUCTION_READ_STATUS);
        ret = spi_transfer_one(0x00);
        spi_end_transaction();
        return ret;
}

static void encode_id(const can_frame_t *frame, uint8_t regs[4]) {
        uint32_t id = frame->id;

        if (frame->extended) {
                id &= 0x1FFFFFFFu;
                regs[0] = (uint8_t)(id >> 21);
                regs[1] = (uint8_t)(((id >> 13) & 0xE0u) | 0x08u | ((id >> 16) & 0x03u));
                regs[2] = (uint8_t)(id >> 8);
                regs[3] = (uint8_t)id;
        } else {
                id &= 0x7FFu;
                regs[0] = (uint8_t)(id >> 3);
                regs[1] = (uint8_t)((id & 0x07u) << 5);
                regs[2] = 0;
                regs[3] = 0;
        }
}

static void decode_frame(const uint8_t regs[13], can_frame_t *frame) {
        uint8_t sidh = regs[0];
        uint8_t sidl = regs[1];
        uint8_t eid8 = regs[2];
        uint8_t eid0 = regs[3];

        if (sidl & 0x08u) {
                frame->extended = 1;
                frame->id = ((uint32_t)sidh << 21)
                          | ((uint32_t)(sidl & 0xE0u) << 13)
                          | ((uint32_t)(sidl & 0x03u) << 16)
                          | ((uint32_t)eid8 << 8)
                          | (uint32_t)eid0;
        } else {
                frame->extended = 0;
                frame->id = ((uint32_t)sidh << 3) | ((uint32_t)sidl >> 5);
        }

        frame->dlc = regs[4] & 0x0Fu;
        if (frame->dlc > CAN_MAX_DATA_LEN) {
                frame->dlc = CAN_MAX_DATA_LEN;
        }

        for (uint8_t i = 0; i < frame->dlc; i++) {
                frame->data[i] = regs[5 + i];
        }
}

void mcp2515_init(void) {
        spi_init();

        // No need to reset MCP2515 here as a hardware reset is done during boot.
        // MCP2515 automatically enters config mode after hardware reset.

        // Set the bitrate configuration registers.
        mcp2515_write_reg(CNF1, MCP_16MHz_250kbPS_CFG1);
        mcp2515_write_reg(CNF2, MCP_16MHz_250kbPS_CFG2);
        mcp2515_write_reg(CNF3, MCP_16MHz_250kbPS_CFG3);

        // Do not filter messages. Allow rollover of RXB0 to RXB1.
        mcp2515_write_reg(RXB0CTRL, 0x64);
        mcp2515_write_reg(RXB1CTRL, 0x60);

        // Clear interrupt flags.
        mcp2515_write_reg(CANINTF, 0x00);

        // Start MCP2515 by setting operation mode to normal.
        mcp2515_modify_reg(CANCTRL, CANCTRL_REQOP, OPMODE_NORMAL);

        // Do not hang forever if the CAN controller is not responding.
        for (unsigned int i = 0; i < 1000000u; i++) {
                if ((mcp2515_read_reg(CANSTAT) & CANSTAT_OPMOD) == OPMODE_NORMAL) {
                        return;
                }
        }
}

int mcp2515_try_send(const can_frame_t *frame) {
        uint8_t regs[13];

        if (mcp2515_read_reg(TXB0CTRL) & TXB0CTRL_TXREQ) {
                return 0;
        }

        encode_id(frame, regs);
        regs[4] = frame->dlc > CAN_MAX_DATA_LEN ? CAN_MAX_DATA_LEN : frame->dlc;

        for (uint8_t i = 0; i < CAN_MAX_DATA_LEN; i++) {
                regs[5 + i] = i < regs[4] ? frame->data[i] : 0;
        }

        mcp2515_write_regs(TXB0SIDH, regs, 13);

        spi_begin_transaction();
        spi_transfer_one(INSTRUCTION_RTS_TX0);
        spi_end_transaction();

        return 1;
}

int mcp2515_try_receive(can_frame_t *frame) {
        uint8_t status = mcp2515_read_status();
        uint8_t regs[13];

        if (status & STATUS_RX0) {
                mcp2515_read_regs(RXB0SIDH, regs, 13);
                decode_frame(regs, frame);
                mcp2515_modify_reg(CANINTF, CANINTF_RX0IF, 0x00);
                return 1;
        }

        if (status & STATUS_RX1) {
                mcp2515_read_regs(RXB1SIDH, regs, 13);
                decode_frame(regs, frame);
                mcp2515_modify_reg(CANINTF, CANINTF_RX1IF, 0x00);
                return 1;
        }

        return 0;
}

int mcp2515_fakerecv(void) {
        can_frame_t frame;
        return mcp2515_try_receive(&frame);
}
