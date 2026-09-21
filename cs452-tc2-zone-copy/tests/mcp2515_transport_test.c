#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "mcp2515.h"
#include "spi.h"

#define INSTRUCTION_RESET       0xC0
#define INSTRUCTION_WRITE       0x02
#define INSTRUCTION_READ        0x03
#define INSTRUCTION_BIT_MODIFY  0x05
#define INSTRUCTION_READ_STATUS 0xA0
#define INSTRUCTION_RTS_TXB0    0x81

#define CANSTAT  0x0E
#define CANCTRL  0x0F
#define CANINTF  0x2C
#define EFLG     0x2D
#define TXB0CTRL 0x30
#define TXB0SIDH 0x31
#define RXB0SIDH 0x61
#define RXB1SIDH 0x71

#define CANINTF_RX0IF 0x01
#define CANINTF_RX1IF 0x02
#define CANINTF_TX0IF 0x04
#define TXREQ 0x08
#define TXERR 0x10
#define MLOA  0x20
#define ABTF  0x40
#define MODE_MASK   0xE0
#define MODE_CONFIG 0x80
#define MODE_NORMAL 0x00

static uint8_t mock_registers[256];
static uint8_t instruction;
static uint8_t address;
static uint8_t modify_mask;
static int phase;
static int rts_count;
static int spi_abort_count;
static int spi_fail_after_transfers = -1;
static int spi_fail_next_end;
static int mode_transition_after_reads;
static int mode_request_active;
static uint8_t requested_mode;
static unsigned int mode_poll_reads;
static unsigned int mode_poll_reads_total;

void spi_init(void) {}

void spi_begin_transaction(void) {
        instruction = 0;
        address = 0;
        modify_mask = 0;
        phase = 0;
}

void spi_abort_transaction(void) {
        spi_abort_count++;
}

int spi_end_transaction(void) {
        if (spi_fail_next_end) {
                spi_fail_next_end = 0;
                return SPI_ERROR_DONE_TIMEOUT;
        }
        return SPI_SUCCESS;
}

static uint8_t read_status(void) {
        uint8_t status = 0;
        if (mock_registers[CANINTF] & CANINTF_RX0IF) status |= 0x01;
        if (mock_registers[CANINTF] & CANINTF_RX1IF) status |= 0x02;
        if (mock_registers[TXB0CTRL] & TXREQ) status |= 0x04;
        return status;
}

static uint8_t mock_transfer_one(uint8_t value) {
        if (phase == 0) {
                instruction = value;
                phase = 1;
                if (instruction == INSTRUCTION_RESET) {
                        memset(mock_registers, 0, sizeof(mock_registers));
                } else if (instruction == INSTRUCTION_RTS_TXB0) {
                        mock_registers[TXB0CTRL] |= TXREQ;
                        rts_count++;
                }
                return 0;
        }

        if (instruction == INSTRUCTION_READ_STATUS) {
                return read_status();
        }

        if (instruction == INSTRUCTION_READ) {
                if (phase == 1) {
                        address = value;
                        phase = 2;
                        return 0;
                }
                if (address == CANSTAT && mode_request_active) {
                        mode_poll_reads++;
                        mode_poll_reads_total++;
                        if (mode_transition_after_reads >= 0 &&
                            mode_poll_reads >=
                                    (unsigned int)
                                    mode_transition_after_reads) {
                                mock_registers[CANSTAT] =
                                        requested_mode;
                                mode_request_active = 0;
                        }
                }
                return mock_registers[address++];
        }

        if (instruction == INSTRUCTION_WRITE) {
                if (phase == 1) {
                        address = value;
                        phase = 2;
                } else {
                        mock_registers[address++] = value;
                }
                return 0;
        }

        if (instruction == INSTRUCTION_BIT_MODIFY) {
                if (phase == 1) {
                        address = value;
                        phase = 2;
                } else if (phase == 2) {
                        modify_mask = value;
                        phase = 3;
                } else {
                        mock_registers[address] =
                                (uint8_t)((mock_registers[address] &
                                           (uint8_t)~modify_mask) |
                                          (value & modify_mask));
                        if (address == CANCTRL &&
                            (modify_mask & MODE_MASK) != 0) {
                                requested_mode =
                                        value & MODE_MASK;
                                mode_poll_reads = 0;
                                if (mode_transition_after_reads == 0) {
                                        mock_registers[CANSTAT] =
                                                requested_mode;
                                        mode_request_active = 0;
                                } else {
                                        mode_request_active = 1;
                                }
                        }
                }
                return 0;
        }

        return 0;
}

int spi_transfer_one(uint8_t value, uint8_t *rx_byte) {
        if (!rx_byte) {
                return SPI_ERROR_ARGUMENT;
        }
        if (spi_fail_after_transfers == 0) {
                spi_fail_after_transfers = -1;
                return SPI_ERROR_TX_TIMEOUT;
        }
        if (spi_fail_after_transfers > 0) {
                spi_fail_after_transfers--;
        }

        *rx_byte = mock_transfer_one(value);
        return SPI_SUCCESS;
}

static void reset_mode_behavior(int transition_after_reads) {
        mode_transition_after_reads = transition_after_reads;
        mode_request_active = 0;
        requested_mode = MODE_NORMAL;
        mode_poll_reads = 0;
        mode_poll_reads_total = 0;
}

static void encode_standard(uint8_t regs[13], uint32_t id, uint8_t dlc,
                            uint8_t seed) {
        memset(regs, 0, 13);
        regs[0] = (uint8_t)(id >> 3);
        regs[1] = (uint8_t)((id & 0x07u) << 5);
        regs[4] = dlc;
        for (int i = 0; i < 8; ++i) regs[5 + i] = (uint8_t)(seed + i);
}

static void encode_extended(uint8_t regs[13], uint32_t id, uint8_t dlc,
                            uint8_t seed) {
        memset(regs, 0, 13);
        regs[0] = (uint8_t)(id >> 21);
        regs[1] = (uint8_t)(((id >> 13) & 0xe0u) |
                            0x08u | ((id >> 16) & 0x03u));
        regs[2] = (uint8_t)(id >> 8);
        regs[3] = (uint8_t)id;
        regs[4] = dlc;
        for (int i = 0; i < 8; ++i) regs[5 + i] = (uint8_t)(seed + i);
}

static int test_decode_formats(void) {
        uint8_t regs[13];
        mcp2515_frame_t frame;

        encode_standard(regs, 0x5abu, 8, 0x10);
        if (mcp2515_decode_rx_registers(regs, &frame) != 0 ||
            frame.extended != 0 || frame.id != 0x5abu ||
            frame.dlc != 8 || frame.data[7] != 0x17) {
                fprintf(stderr, "standard frame decode failed\n");
                return -1;
        }

        encode_extended(regs, 0x01abcdeu, 6, 0x20);
        if (mcp2515_decode_rx_registers(regs, &frame) != 0 ||
            frame.extended != 1 || frame.id != 0x01abcdeu ||
            frame.dlc != 6 || frame.data[0] != 0x20) {
                fprintf(stderr, "extended frame decode failed\n");
                return -1;
        }

        regs[4] = 9;
        if (mcp2515_decode_rx_registers(regs, &frame) !=
                    MCP2515_RECV_INVALID ||
            frame.dlc != 9 || frame.extended != 1) {
                fprintf(stderr, "invalid DLC was clamped or accepted\n");
                return -1;
        }
        return 0;
}

static int test_both_receive_buffers(void) {
        uint8_t regs[13];
        mcp2515_frame_t frame;

        memset(mock_registers, 0, sizeof(mock_registers));
        encode_standard(regs, 0x321u, 2, 0x30);
        memcpy(&mock_registers[RXB0SIDH], regs, sizeof(regs));
        encode_extended(regs, 0x00123456u, 8, 0x40);
        memcpy(&mock_registers[RXB1SIDH], regs, sizeof(regs));
        mock_registers[CANINTF] = CANINTF_RX0IF | CANINTF_RX1IF;

        if (mcp2515_recv(&frame) != 0 || frame.extended != 0 ||
            frame.id != 0x321u ||
            (mock_registers[CANINTF] & CANINTF_RX0IF) != 0 ||
            (mock_registers[CANINTF] & CANINTF_RX1IF) == 0) {
                fprintf(stderr, "RXB0 was not drained first\n");
                return -1;
        }
        if (mcp2515_recv(&frame) != 0 || frame.extended != 1 ||
            frame.id != 0x00123456u ||
            (mock_registers[CANINTF] &
             (CANINTF_RX0IF | CANINTF_RX1IF)) != 0) {
                fprintf(stderr, "RXB1 was not drained\n");
                return -1;
        }
        if (mcp2515_recv(&frame) != MCP2515_RECV_NONE) {
                fprintf(stderr, "empty receive buffers reported a frame\n");
                return -1;
        }

        mock_registers[EFLG] = 0xc0;
        if (mcp2515_take_rx_overflow() !=
                    (MCP2515_RX_OVERFLOW_0 |
                     MCP2515_RX_OVERFLOW_1) ||
            (mock_registers[EFLG] & 0xc0) != 0) {
                fprintf(stderr, "RX overflow flags were not reported/cleared\n");
                return -1;
        }
        return 0;
}

static int test_transmit_lifecycle(void) {
        mcp2515_frame_t frame;
        uint8_t sentinel;

        memset(&frame, 0, sizeof(frame));
        frame.id = 0x00085772u;
        frame.extended = 1;
        frame.dlc = 6;
        for (int i = 0; i < 6; ++i) frame.data[i] = (uint8_t)(0x50 + i);

        memset(mock_registers, 0, sizeof(mock_registers));
        reset_mode_behavior(0);
        mock_registers[CANSTAT] = 0;
        mock_registers[TXB0CTRL] = TXREQ;
        mock_registers[TXB0SIDH] = 0xa5;
        sentinel = mock_registers[TXB0SIDH];
        rts_count = 0;
        if (mcp2515_send(&frame) != MCP2515_SEND_BUSY ||
            mock_registers[TXB0SIDH] != sentinel ||
            (mock_registers[TXB0CTRL] & TXREQ) == 0 ||
            rts_count != 0) {
                fprintf(stderr, "busy TX buffer was aborted/overwritten\n");
                return -1;
        }

        mock_registers[TXB0CTRL] = ABTF | MLOA | TXERR;
        mock_registers[CANINTF] = CANINTF_TX0IF;
        if (mcp2515_send(&frame) != 0 || rts_count != 1 ||
            (mock_registers[TXB0CTRL] & TXREQ) == 0 ||
            (mock_registers[TXB0CTRL] & (ABTF | MLOA | TXERR)) != 0 ||
            (mock_registers[CANINTF] & CANINTF_TX0IF) != 0 ||
            mock_registers[TXB0SIDH + 4] != 6 ||
            mock_registers[TXB0SIDH + 5] != 0x50) {
                fprintf(stderr, "TX start did not load a clean frame\n");
                return -1;
        }
        if (mcp2515_tx_status() != MCP2515_TX_PENDING) {
                fprintf(stderr, "TXREQ was not reported pending\n");
                return -1;
        }

        mock_registers[TXB0CTRL] = 0;
        if (mcp2515_tx_status() != MCP2515_TX_COMPLETE) {
                fprintf(stderr, "completed TX was not reported\n");
                return -1;
        }
        for (uint8_t error = TXERR; error <= ABTF; error <<= 1) {
                mock_registers[TXB0CTRL] = error;
                if (mcp2515_tx_status() != MCP2515_TX_FAILED) {
                        fprintf(stderr, "TX error bit 0x%x was ignored\n",
                                error);
                        return -1;
                }
        }

        mock_registers[TXB0CTRL] = TXREQ | TXERR | MLOA | ABTF;
        mock_registers[CANINTF] = CANINTF_TX0IF;
        if (mcp2515_tx_abort() != 0 ||
            (mock_registers[TXB0CTRL] &
             (TXREQ | TXERR | MLOA | ABTF)) != 0 ||
            (mock_registers[CANINTF] & CANINTF_TX0IF) != 0) {
                fprintf(stderr, "bounded abort did not clear TX state\n");
                return -1;
        }

        frame.id = 0x456u;
        frame.extended = 0;
        frame.dlc = 1;
        mock_registers[TXB0CTRL] = 0;
        if (mcp2515_send(&frame) != 0 ||
            (mock_registers[TXB0SIDH + 1] & 0x08u) != 0) {
                fprintf(stderr, "standard TX was forced to extended format\n");
                return -1;
        }
        return 0;
}

static int test_mode_recovery_bounds(void) {
        mcp2515_frame_t frame;
        int result;

        memset(&frame, 0, sizeof(frame));
        frame.id = 0x456u;
        frame.extended = 0;
        frame.dlc = 1;
        frame.data[0] = 0x5a;

        memset(mock_registers, 0, sizeof(mock_registers));
        mock_registers[CANSTAT] = MODE_CONFIG;
        mock_registers[TXB0SIDH] = 0xa5;
        rts_count = 0;
        reset_mode_behavior(-1);
        result = mcp2515_send(&frame);
        if (result != MCP2515_SEND_NOT_READY ||
            mode_poll_reads != MCP2515_RUNTIME_MODE_POLL_LIMIT ||
            mode_poll_reads_total !=
                    MCP2515_RUNTIME_MODE_POLL_LIMIT ||
            mock_registers[TXB0SIDH] != 0xa5 ||
            rts_count != 0) {
                fprintf(stderr,
                        "runtime mode recovery exceeded aggregate bound\n");
                return -1;
        }

        memset(mock_registers, 0, sizeof(mock_registers));
        mock_registers[CANSTAT] = MODE_CONFIG;
        rts_count = 0;
        reset_mode_behavior(-1);
        /*
         * Initial CANSTAT read is three bytes and BIT MODIFY is four.
         * Fail the first byte of the first recovery poll.
         */
        spi_fail_after_transfers = 7;
        result = mcp2515_send(&frame);
        spi_fail_after_transfers = -1;
        if (result != MCP2515_IO_ERROR || rts_count != 0) {
                fprintf(stderr,
                        "runtime recovery SPI fault was not fail-closed\n");
                return -1;
        }

        memset(mock_registers, 0, sizeof(mock_registers));
        mock_registers[CANSTAT] = MODE_CONFIG;
        rts_count = 0;
        reset_mode_behavior(
                (int)MCP2515_RUNTIME_MODE_POLL_LIMIT);
        result = mcp2515_send(&frame);
        if (result != 0 ||
            mode_poll_reads != MCP2515_RUNTIME_MODE_POLL_LIMIT ||
            mode_poll_reads_total !=
                    MCP2515_RUNTIME_MODE_POLL_LIMIT ||
            rts_count != 1) {
                fprintf(stderr,
                        "runtime final allowed mode poll was rejected\n");
                return -1;
        }

        reset_mode_behavior(-1);
        result = mcp2515_init();
        if (result != MCP2515_SEND_NOT_READY ||
            mode_poll_reads != MCP2515_INIT_MODE_POLL_LIMIT ||
            mode_poll_reads_total != MCP2515_INIT_MODE_POLL_LIMIT) {
                fprintf(stderr,
                        "initialization mode poll was not bounded\n");
                return -1;
        }

        reset_mode_behavior((int)MCP2515_INIT_MODE_POLL_LIMIT);
        result = mcp2515_init();
        if (result != 0 ||
            mode_poll_reads != MCP2515_INIT_MODE_POLL_LIMIT ||
            mode_poll_reads_total !=
                    2u * MCP2515_INIT_MODE_POLL_LIMIT) {
                fprintf(stderr,
                        "initialization final allowed polls were rejected\n");
                return -1;
        }

        reset_mode_behavior(0);
        return 0;
}

static int expect_io_error(const char *operation, int result) {
        if (result != MCP2515_IO_ERROR) {
                fprintf(stderr, "%s swallowed SPI failure: %d\n",
                        operation, result);
                return -1;
        }
        return 0;
}

static int test_spi_failure_propagation(void) {
        mcp2515_frame_t frame;

        memset(&frame, 0, sizeof(frame));
        frame.id = 0x123u;
        frame.extended = 0;
        frame.dlc = 1;
        frame.data[0] = 0x5a;
        memset(mock_registers, 0, sizeof(mock_registers));

        spi_abort_count = 0;
        spi_fail_after_transfers = 0;
        if (expect_io_error("init reset transfer", mcp2515_init()) < 0 ||
            spi_abort_count == 0) {
                return -1;
        }

        spi_fail_after_transfers = -1;
        spi_fail_next_end = 1;
        if (expect_io_error("init reset completion", mcp2515_init()) < 0 ||
            spi_abort_count < 2) {
                return -1;
        }

        spi_fail_after_transfers = 1;
        if (expect_io_error("TX-ready register address",
                            mcp2515_tx_ready()) < 0) {
                return -1;
        }

        spi_fail_after_transfers = -1;
        spi_fail_next_end = 1;
        if (expect_io_error("TX-ready transaction completion",
                            mcp2515_tx_ready()) < 0) {
                return -1;
        }

        spi_fail_after_transfers = 0;
        if (expect_io_error("send", mcp2515_send(&frame)) < 0) {
                return -1;
        }

        spi_fail_after_transfers = 0;
        if (expect_io_error("TX status", mcp2515_tx_status()) < 0) {
                return -1;
        }

        spi_fail_after_transfers = 0;
        if (expect_io_error("TX abort", mcp2515_tx_abort()) < 0) {
                return -1;
        }

        spi_fail_after_transfers = 0;
        if (expect_io_error("receive", mcp2515_recv(&frame)) < 0) {
                return -1;
        }

        spi_fail_after_transfers = 0;
        if (expect_io_error("overflow read",
                            mcp2515_take_rx_overflow()) < 0) {
                return -1;
        }

        /*
         * A fault is one transaction's result, not a sticky software lock.
         * The next healthy register transaction must still work.
         */
        spi_fail_after_transfers = -1;
        spi_fail_next_end = 0;
        mock_registers[TXB0CTRL] = 0;
        if (mcp2515_tx_ready() != 1) {
                fprintf(stderr, "MCP2515 did not recover after SPI error\n");
                return -1;
        }
        return 0;
}

int main(void) {
        if (test_decode_formats() < 0 ||
            test_both_receive_buffers() < 0 ||
            test_transmit_lifecycle() < 0 ||
            test_mode_recovery_bounds() < 0 ||
            test_spi_failure_propagation() < 0) {
                return 1;
        }
        printf("validated MCP2515 TX/RX lifecycle, bounded mode recovery, frame format, and SPI errors\n");
        return 0;
}
