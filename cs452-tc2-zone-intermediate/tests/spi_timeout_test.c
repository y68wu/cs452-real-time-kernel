#include <stdint.h>
#include <stdio.h>

#define SPI_TEST_BACKEND 1
#define SPI_STATUS_POLL_LIMIT 4u

static uint32_t mock_read_cs(void);
static void mock_write_cs(uint32_t value);
static uint32_t mock_read_fifo(void);
static void mock_write_fifo(uint32_t value);
static void mock_write_clk(uint32_t value);

#define SPI_READ_CS() mock_read_cs()
#define SPI_WRITE_CS(value) mock_write_cs(value)
#define SPI_READ_FIFO() mock_read_fifo()
#define SPI_WRITE_FIFO(value) mock_write_fifo(value)
#define SPI_WRITE_CLK(value) mock_write_clk(value)

#include "../spi.c"

#define TEST_TXD  0x00040000u
#define TEST_RXD  0x00020000u
#define TEST_DONE 0x00010000u
#define TEST_TA   0x00000080u

enum mock_phase {
        MOCK_IDLE = 0,
        MOCK_WAIT_TXD,
        MOCK_WAIT_RXD,
        MOCK_WAIT_DONE
};

static uint32_t mock_cs;
static uint32_t mock_fifo;
static uint32_t mock_clk;
static enum mock_phase phase;
static unsigned int ready_after[4];
static unsigned int status_reads[4];
static unsigned int fifo_writes;
static unsigned int fifo_reads;

static void reset_mock(unsigned int txd_after, unsigned int rxd_after,
                       unsigned int done_after) {
        mock_cs = 0;
        mock_fifo = 0xa5u;
        mock_clk = 0;
        phase = MOCK_IDLE;
        for (unsigned int i = 0; i < 4; ++i) {
                ready_after[i] = 0;
                status_reads[i] = 0;
        }
        ready_after[MOCK_WAIT_TXD] = txd_after;
        ready_after[MOCK_WAIT_RXD] = rxd_after;
        ready_after[MOCK_WAIT_DONE] = done_after;
        fifo_writes = 0;
        fifo_reads = 0;
}

static uint32_t mock_read_cs(void) {
        uint32_t status = mock_cs & ~(TEST_TXD | TEST_RXD | TEST_DONE);
        uint32_t ready_bit = 0;

        if (phase != MOCK_IDLE) {
                status_reads[phase]++;
        }
        if (phase == MOCK_WAIT_TXD) {
                ready_bit = TEST_TXD;
        } else if (phase == MOCK_WAIT_RXD) {
                ready_bit = TEST_RXD;
        } else if (phase == MOCK_WAIT_DONE) {
                ready_bit = TEST_DONE;
        }

        if (phase != MOCK_IDLE && ready_after[phase] > 0 &&
            status_reads[phase] >= ready_after[phase]) {
                status |= ready_bit;
        }
        return status;
}

static void mock_write_cs(uint32_t value) {
        uint32_t previous = mock_cs;
        mock_cs = value;

        if ((value & TEST_TA) && !(previous & TEST_TA)) {
                phase = MOCK_WAIT_TXD;
        } else if (!(value & TEST_TA)) {
                phase = MOCK_IDLE;
        }
}

static uint32_t mock_read_fifo(void) {
        fifo_reads++;
        phase = MOCK_WAIT_DONE;
        return mock_fifo;
}

static void mock_write_fifo(uint32_t value) {
        fifo_writes++;
        mock_fifo = (value ^ 0xffu) & 0xffu;
        phase = MOCK_WAIT_RXD;
}

static void mock_write_clk(uint32_t value) {
        mock_clk = value;
}

static int run_transaction(uint8_t tx, uint8_t *rx) {
        int status;

        spi_begin_transaction();
        status = spi_transfer_one(tx, rx);
        if (status != SPI_SUCCESS) {
                return status;
        }
        return spi_end_transaction();
}

static int test_normal_and_boundary(void) {
        uint8_t rx = 0;

        reset_mock(1, 1, 1);
        spi_init();
        if (mock_clk != 50u) {
                fprintf(stderr, "SPI clock divider was not initialized\n");
                return -1;
        }
        if (run_transaction(0x3cu, &rx) != SPI_SUCCESS ||
            rx != (uint8_t)0xc3u || fifo_writes != 1 ||
            fifo_reads != 1 || (mock_cs & TEST_TA) != 0) {
                fprintf(stderr, "normal SPI transaction failed\n");
                return -1;
        }

        reset_mock(SPI_STATUS_POLL_LIMIT, SPI_STATUS_POLL_LIMIT,
                   SPI_STATUS_POLL_LIMIT);
        if (run_transaction(0x55u, &rx) != SPI_SUCCESS ||
            status_reads[MOCK_WAIT_TXD] != SPI_STATUS_POLL_LIMIT ||
            status_reads[MOCK_WAIT_RXD] != SPI_STATUS_POLL_LIMIT ||
            status_reads[MOCK_WAIT_DONE] <
                    SPI_STATUS_POLL_LIMIT ||
            (mock_cs & TEST_TA) != 0) {
                fprintf(stderr, "ready-on-final-poll boundary was rejected\n");
                return -1;
        }
        return 0;
}

static int test_each_timeout_stage(void) {
        uint8_t rx = 0;

        reset_mock(0, 1, 1);
        spi_begin_transaction();
        if (spi_transfer_one(0x11u, &rx) != SPI_ERROR_TX_TIMEOUT ||
            fifo_writes != 0 || (mock_cs & TEST_TA) != 0 ||
            status_reads[MOCK_WAIT_TXD] >
                    SPI_STATUS_POLL_LIMIT + 1u) {
                fprintf(stderr, "TXD timeout was not bounded/aborted\n");
                return -1;
        }

        reset_mock(1, 0, 1);
        spi_begin_transaction();
        if (spi_transfer_one(0x22u, &rx) != SPI_ERROR_RX_TIMEOUT ||
            fifo_writes != 1 || fifo_reads != 0 ||
            (mock_cs & TEST_TA) != 0 ||
            status_reads[MOCK_WAIT_RXD] >
                    SPI_STATUS_POLL_LIMIT + 1u) {
                fprintf(stderr, "RXD timeout was not bounded/aborted\n");
                return -1;
        }

        reset_mock(1, 1, 0);
        spi_begin_transaction();
        if (spi_transfer_one(0x33u, &rx) != SPI_SUCCESS ||
            spi_end_transaction() != SPI_ERROR_DONE_TIMEOUT ||
            (mock_cs & TEST_TA) != 0 ||
            status_reads[MOCK_WAIT_DONE] >
                    SPI_STATUS_POLL_LIMIT + 1u) {
                fprintf(stderr, "DONE timeout was not bounded/aborted\n");
                return -1;
        }

        reset_mock(1, 1, 1);
        spi_begin_transaction();
        if (spi_transfer_one(0x44u, 0) != SPI_ERROR_ARGUMENT ||
            (mock_cs & TEST_TA) != 0) {
                fprintf(stderr, "null RX destination did not abort\n");
                return -1;
        }
        return 0;
}

int main(void) {
        if (test_normal_and_boundary() < 0 ||
            test_each_timeout_stage() < 0) {
                return 1;
        }
        printf("validated bounded SPI TXD/RXD/DONE polling, boundary, and recovery\n");
        return 0;
}
