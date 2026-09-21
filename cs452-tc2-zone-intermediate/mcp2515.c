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
#define EFLG    0x2D

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

#define RXB1SIDH 0x71
#define RXB1SIDL 0x72
#define RXB1EID8 0x73
#define RXB1EID0 0x74
#define RXB1DLC  0x75
#define RXB1D0   0x76

#define CANINTF_RX0IF 0x01
#define CANINTF_RX1IF 0x02
#define CANINTF_TX0IF 0x04
#define TXBCTRL_TXREQ 0x08
#define TXBCTRL_TXERR 0x10
#define TXBCTRL_MLOA  0x20
#define TXBCTRL_ABTF  0x40
#define TXBCTRL_ERROR_MASK \
        (TXBCTRL_TXERR | TXBCTRL_MLOA | TXBCTRL_ABTF)
#define EFLG_RX0OVR 0x40
#define EFLG_RX1OVR 0x80
#define STATUS_RX0    0x01
#define STATUS_RX1    0x02

static int mcp2515_transfer_byte(uint8_t tx, uint8_t *rx) {
        uint8_t discarded;

        if (!rx) {
                rx = &discarded;
        }
        if (spi_transfer_one(tx, rx) != SPI_SUCCESS) {
                /*
                 * spi_transfer_one() already drops TA on timeout.  Calling
                 * abort again is intentional: it also makes this helper safe
                 * with alternate SPI backends and clears both FIFOs.
                 */
                spi_abort_transaction();
                return MCP2515_IO_ERROR;
        }
        return 0;
}

static int mcp2515_finish_transaction(void) {
        if (spi_end_transaction() != SPI_SUCCESS) {
                spi_abort_transaction();
                return MCP2515_IO_ERROR;
        }
        return 0;
}

static int mcp2515_read_regs(uint8_t reg, uint8_t values[], uint8_t n) {
        if (!values && n > 0) {
                return MCP2515_IO_ERROR;
        }

        spi_begin_transaction();
        if (mcp2515_transfer_byte(INSTRUCTION_READ, 0) < 0 ||
            mcp2515_transfer_byte(reg, 0) < 0) {
                return MCP2515_IO_ERROR;
        }
        for (uint8_t i = 0; i < n; i++) {
                if (mcp2515_transfer_byte(0x00, &values[i]) < 0) {
                        return MCP2515_IO_ERROR;
                }
        }
        return mcp2515_finish_transaction();
}

static int mcp2515_read_reg(uint8_t reg, uint8_t *value) {
        if (!value) {
                return MCP2515_IO_ERROR;
        }
        return mcp2515_read_regs(reg, value, 1);
}

static int mcp2515_write_regs(uint8_t reg, const uint8_t values[],
                              uint8_t n) {
        if (!values && n > 0) {
                return MCP2515_IO_ERROR;
        }

        spi_begin_transaction();
        if (mcp2515_transfer_byte(INSTRUCTION_WRITE, 0) < 0 ||
            mcp2515_transfer_byte(reg, 0) < 0) {
                return MCP2515_IO_ERROR;
        }
        for (uint8_t i = 0; i < n; i++) {
                if (mcp2515_transfer_byte(values[i], 0) < 0) {
                        return MCP2515_IO_ERROR;
                }
        }
        return mcp2515_finish_transaction();
}

static int mcp2515_write_reg(uint8_t reg, uint8_t value) {
        return mcp2515_write_regs(reg, &value, 1);
}

static int mcp2515_modify_reg(uint8_t reg, uint8_t mask, uint8_t data) {
        spi_begin_transaction();
        if (mcp2515_transfer_byte(INSTRUCTION_BIT_MODIFY, 0) < 0 ||
            mcp2515_transfer_byte(reg, 0) < 0 ||
            mcp2515_transfer_byte(mask, 0) < 0 ||
            mcp2515_transfer_byte(data, 0) < 0) {
                return MCP2515_IO_ERROR;
        }
        return mcp2515_finish_transaction();
}

static int mcp2515_read_status(uint8_t *status) {
        if (!status) {
                return MCP2515_IO_ERROR;
        }

        spi_begin_transaction();
        if (mcp2515_transfer_byte(INSTRUCTION_READ_STATUS, 0) < 0 ||
            mcp2515_transfer_byte(0x00, status) < 0) {
                return MCP2515_IO_ERROR;
        }
        return mcp2515_finish_transaction();
}

static void encode_extended_id(uint32_t id, uint8_t *sidh, uint8_t *sidl, uint8_t *eid8, uint8_t *eid0) {
        id &= 0x1FFFFFFFu;

        *sidh = (uint8_t)(id >> 21);
        *sidl = (uint8_t)(((id >> 13) & 0xE0u) | 0x08u | ((id >> 16) & 0x03u));
        *eid8 = (uint8_t)(id >> 8);
        *eid0 = (uint8_t)id;
}

static void encode_standard_id(uint32_t id, uint8_t *sidh, uint8_t *sidl,
                               uint8_t *eid8, uint8_t *eid0) {
        id &= 0x7ffu;
        *sidh = (uint8_t)(id >> 3);
        *sidl = (uint8_t)((id & 0x07u) << 5);
        *eid8 = 0;
        *eid0 = 0;
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

static uint32_t decode_standard_id(uint8_t sidh, uint8_t sidl) {
        return ((((uint32_t)sidh) << 3) |
                (((uint32_t)sidl) >> 5)) & 0x7ffu;
}

int mcp2515_decode_rx_registers(const uint8_t regs[13],
                                mcp2515_frame_t *frame) {
        if (!regs || !frame) {
                return -1;
        }

        frame->extended = (regs[1] & 0x08u) != 0;
        frame->id = frame->extended ?
                decode_extended_id(regs[0], regs[1], regs[2], regs[3]) :
                decode_standard_id(regs[0], regs[1]);
        frame->dlc = regs[4] & 0x0fu;
        for (uint8_t i = 0; i < 8; ++i) {
                frame->data[i] = regs[5 + i];
        }

        return frame->dlc <= 8 ? 0 : MCP2515_RECV_INVALID;
}

int mcp2515_init(void) {
        uint8_t canstat;

        spi_begin_transaction();
        if (mcp2515_transfer_byte(INSTRUCTION_RESET, 0) < 0 ||
            mcp2515_finish_transaction() < 0) {
                return MCP2515_IO_ERROR;
        }

        /*
         * One-time, explicitly bounded reset-settle delay.  Runtime mode
         * recovery below never executes this CPU delay.
         */
        for (volatile unsigned int d = 0;
             d < MCP2515_RESET_SETTLE_ITERATIONS; ++d) {
        }

        if (mcp2515_modify_reg(CANCTRL, CANCTRL_REQOP,
                               OPMODE_CONFIG) < 0) {
                return MCP2515_IO_ERROR;
        }

        int configuration_mode_ready = 0;
        for (unsigned int i = 0;
             i < MCP2515_INIT_MODE_POLL_LIMIT; ++i) {
                if (mcp2515_read_reg(CANSTAT, &canstat) < 0) {
                        return MCP2515_IO_ERROR;
                }
                if ((canstat & CANSTAT_OPMOD) == OPMODE_CONFIG) {
                        configuration_mode_ready = 1;
                        break;
                }
        }
        if (!configuration_mode_ready) {
                return MCP2515_SEND_NOT_READY;
        }

        if (mcp2515_write_reg(CNF1, MCP_16MHz_250kbPS_CFG1) < 0 ||
            mcp2515_write_reg(CNF2, MCP_16MHz_250kbPS_CFG2) < 0 ||
            mcp2515_write_reg(CNF3, MCP_16MHz_250kbPS_CFG3) < 0 ||
            mcp2515_write_reg(RXB0CTRL, 0x64) < 0 ||
            mcp2515_write_reg(RXB1CTRL, 0x60) < 0 ||
            mcp2515_write_reg(CANINTF, 0x00) < 0 ||
            mcp2515_write_reg(EFLG, 0x00) < 0 ||
            mcp2515_write_reg(TXB0CTRL, 0x00) < 0 ||
            mcp2515_modify_reg(CANCTRL, CANCTRL_REQOP,
                               OPMODE_NORMAL) < 0) {
                return MCP2515_IO_ERROR;
        }

        for (unsigned int i = 0;
             i < MCP2515_INIT_MODE_POLL_LIMIT; ++i) {
                if (mcp2515_read_reg(CANSTAT, &canstat) < 0) {
                        return MCP2515_IO_ERROR;
                }
                if ((canstat & CANSTAT_OPMOD) == OPMODE_NORMAL) {
                        return 0;
                }
        }

        return MCP2515_SEND_NOT_READY;
}

int mcp2515_tx_ready(void) {
        uint8_t control;

        if (mcp2515_read_reg(TXB0CTRL, &control) < 0) {
                return MCP2515_IO_ERROR;
        }
        return (control & TXBCTRL_TXREQ) == 0;
}

int mcp2515_send(const mcp2515_frame_t *frame) {
        uint8_t regs[13];
        uint8_t dlc;
        uint8_t canstat;

        if (!frame || frame->dlc > 8 || frame->extended > 1 ||
            (frame->extended && frame->id > 0x1fffffffu) ||
            (!frame->extended && frame->id > 0x7ffu)) {
                return -1;
        }

        dlc = frame->dlc;

        if (mcp2515_read_reg(CANSTAT, &canstat) < 0) {
                return MCP2515_IO_ERROR;
        }
        if ((canstat & CANSTAT_OPMOD) != OPMODE_NORMAL) {
                if (mcp2515_modify_reg(CANCTRL, CANCTRL_REQOP,
                                       OPMODE_NORMAL) < 0) {
                        return MCP2515_IO_ERROR;
                }

                /*
                 * This is a runtime path in a cooperative kernel.  Keep the
                 * aggregate recovery attempt to a fixed handful of complete
                 * register transactions; each transaction has its own SPI
                 * TXD/RXD/DONE bound.  A controller that cannot transition
                 * promptly is failed closed by CanServer.
                 */
                for (unsigned int i = 0;
                     i < MCP2515_RUNTIME_MODE_POLL_LIMIT; ++i) {
                        if (mcp2515_read_reg(CANSTAT, &canstat) < 0) {
                                return MCP2515_IO_ERROR;
                        }
                        if ((canstat & CANSTAT_OPMOD) == OPMODE_NORMAL) {
                                break;
                        }
                }

                if ((canstat & CANSTAT_OPMOD) != OPMODE_NORMAL) {
                        return MCP2515_SEND_NOT_READY;
                }
        }

        int tx_ready = mcp2515_tx_ready();
        if (tx_ready < 0) {
                return MCP2515_IO_ERROR;
        }
        if (!tx_ready) {
                return MCP2515_SEND_BUSY;
        }

        /*
         * Clear only stale completion/error flags after TXREQ is known clear.
         * A busy controller is never aborted or overwritten by a new frame.
         */
        if (mcp2515_modify_reg(TXB0CTRL, TXBCTRL_ERROR_MASK, 0x00) < 0 ||
            mcp2515_modify_reg(CANINTF, CANINTF_TX0IF, 0x00) < 0) {
                return MCP2515_IO_ERROR;
        }

        if (frame->extended) {
                encode_extended_id(frame->id, &regs[0], &regs[1],
                                   &regs[2], &regs[3]);
        } else {
                encode_standard_id(frame->id, &regs[0], &regs[1],
                                   &regs[2], &regs[3]);
        }
        regs[4] = dlc & 0x0F;

        for (uint8_t i = 0; i < 8; i++) {
                regs[5 + i] = i < dlc ? frame->data[i] : 0;
        }

        if (mcp2515_write_regs(TXB0SIDH, regs, 13) < 0) {
                return MCP2515_IO_ERROR;
        }

        spi_begin_transaction();
        if (mcp2515_transfer_byte(INSTRUCTION_RTS_TXB0, 0) < 0 ||
            mcp2515_finish_transaction() < 0) {
                return MCP2515_IO_ERROR;
        }

        return 0;
}

int mcp2515_tx_status(void) {
        uint8_t control;

        if (mcp2515_read_reg(TXB0CTRL, &control) < 0) {
                return MCP2515_IO_ERROR;
        }

        if (control & TXBCTRL_ERROR_MASK) {
                return MCP2515_TX_FAILED;
        }
        if (control & TXBCTRL_TXREQ) {
                return MCP2515_TX_PENDING;
        }
        return MCP2515_TX_COMPLETE;
}

int mcp2515_tx_abort(void) {
        uint8_t control;

        if (mcp2515_modify_reg(TXB0CTRL, TXBCTRL_TXREQ, 0x00) < 0 ||
            mcp2515_read_reg(TXB0CTRL, &control) < 0) {
                return MCP2515_IO_ERROR;
        }
        if (control & TXBCTRL_TXREQ) {
                return -1;
        }
        if (mcp2515_modify_reg(TXB0CTRL, TXBCTRL_ERROR_MASK, 0x00) < 0 ||
            mcp2515_modify_reg(CANINTF, CANINTF_TX0IF, 0x00) < 0) {
                return MCP2515_IO_ERROR;
        }
        return 0;
}

int mcp2515_recv(mcp2515_frame_t *frame) {
        uint8_t regs[13];
        uint8_t status;
        uint8_t register_base;
        uint8_t interrupt_flag;

        if (!frame) {
                return -1;
        }

        if (mcp2515_read_status(&status) < 0) {
                return MCP2515_IO_ERROR;
        }
        if (status & STATUS_RX0) {
                register_base = RXB0SIDH;
                interrupt_flag = CANINTF_RX0IF;
        } else if (status & STATUS_RX1) {
                register_base = RXB1SIDH;
                interrupt_flag = CANINTF_RX1IF;
        } else {
                return MCP2515_RECV_NONE;
        }

        if (mcp2515_read_regs(register_base, regs, 13) < 0 ||
            mcp2515_modify_reg(CANINTF, interrupt_flag, 0x00) < 0) {
                return MCP2515_IO_ERROR;
        }

        return mcp2515_decode_rx_registers(regs, frame);
}

int mcp2515_take_rx_overflow(void) {
        uint8_t hardware_flags;
        int overflow = 0;

        if (mcp2515_read_reg(EFLG, &hardware_flags) < 0) {
                return MCP2515_IO_ERROR;
        }
        hardware_flags &= EFLG_RX0OVR | EFLG_RX1OVR;

        if (hardware_flags & EFLG_RX0OVR) {
                overflow |= MCP2515_RX_OVERFLOW_0;
        }
        if (hardware_flags & EFLG_RX1OVR) {
                overflow |= MCP2515_RX_OVERFLOW_1;
        }
        if (hardware_flags &&
            mcp2515_modify_reg(EFLG, hardware_flags, 0x00) < 0) {
                return MCP2515_IO_ERROR;
        }
        return overflow;
}
