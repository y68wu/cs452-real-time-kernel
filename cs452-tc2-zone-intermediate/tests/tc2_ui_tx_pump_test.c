#include "tc2_ui_tx_pump.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define TEST_CAPTURE_CAPACITY \
        (TC2_UI_TX_FRAME_CAPACITY * 3u)

typedef struct {
        char bytes[TEST_CAPTURE_CAPACITY];
        size_t length;
        size_t maximum_chunk;
        unsigned int calls;
        unsigned int block_on_call;
        unsigned int fail_on_call;
        unsigned int invalid_on_call;
} capture_sink;

static tc2_ui_tx_pump test_pump;
static capture_sink test_sink;
static char test_payload_a[TC2_UI_TX_FRAME_CAPACITY];
static char test_payload_b[TC2_UI_TX_FRAME_CAPACITY];
static char test_payload_c[TC2_UI_TX_FRAME_CAPACITY];
static char test_expected[TEST_CAPTURE_CAPACITY];
static char test_restore[TC2_UI_TX_RESTORE_CAPACITY + 1u];

static int report_failure(
        const char *test, int line, const char *condition) {
        (void)fprintf(stderr, "FAIL %s:%d: %s\n",
                      test, line, condition);
        return 0;
}

#define CHECK(condition) \
        do { \
                if (!(condition)) { \
                        return report_failure( \
                                __func__, __LINE__, #condition); \
                } \
        } while (0)

static void reset_sink(void) {
        (void)memset(&test_sink, 0, sizeof(test_sink));
}

static void fill_pattern(
        char *bytes, size_t length, unsigned int seed) {
        for (size_t index = 0; index < length; ++index) {
                bytes[index] = (char)('!' +
                        ((index * 29u + seed * 17u) % 90u));
        }
}

static void append_expected(
        const char *bytes, size_t length, size_t *offset) {
        (void)memcpy(test_expected + *offset, bytes, length);
        *offset += length;
}

static int capture_write(
        void *context, const char *bytes, size_t length) {
        capture_sink *sink = context;
        ++sink->calls;
        if (length > sink->maximum_chunk) {
                sink->maximum_chunk = length;
        }
        if (sink->block_on_call != 0 &&
            sink->calls == sink->block_on_call) {
                return TC2_UI_TX_SINK_WOULD_BLOCK;
        }
        if (sink->fail_on_call != 0 &&
            sink->calls == sink->fail_on_call) {
                return TC2_UI_TX_SINK_FAILED;
        }
        if (sink->invalid_on_call != 0 &&
            sink->calls == sink->invalid_on_call) {
                return 77;
        }
        if (!bytes || length == 0 ||
            length > TC2_UI_TX_CHUNK_CAPACITY ||
            sink->length > TEST_CAPTURE_CAPACITY ||
            length > TEST_CAPTURE_CAPACITY - sink->length) {
                return TC2_UI_TX_SINK_FAILED;
        }
        (void)memcpy(sink->bytes + sink->length, bytes, length);
        sink->length += length;
        return TC2_UI_TX_SINK_ACCEPTED;
}

static int publish_frame(
        tc2_ui_tx_pump *pump, uint32_t generation,
        tc2_ui_tx_frame_kind kind,
        const char *payload, size_t payload_length,
        const char *restore, size_t restore_length,
        size_t staging_chunk) {
        int status = Tc2UiTxPumpBeginFrame(pump, generation);
        size_t offset = 0;
        if (status != TC2_UI_TX_OK) return status;
        if (staging_chunk == 0) staging_chunk = payload_length;
        while (offset < payload_length) {
                size_t remaining = payload_length - offset;
                size_t chunk = remaining < staging_chunk ?
                        remaining : staging_chunk;
                if (Tc2UiTxPumpStageWrite(
                            pump, payload + offset, chunk) != 0) {
                        return TC2_UI_TX_OVERSIZE;
                }
                offset += chunk;
        }
        return Tc2UiTxPumpCommitFrame(
                pump, kind, restore, restore_length);
}

static int drain_until_idle(
        tc2_ui_tx_pump *pump, capture_sink *sink,
        unsigned int chunks_per_call,
        size_t *emitted_bytes,
        unsigned int *completed_frames) {
        unsigned int guard = 0;
        if (emitted_bytes) *emitted_bytes = 0;
        if (completed_frames) *completed_frames = 0;
        while (Tc2UiTxPumpHasWork(pump)) {
                tc2_ui_tx_drain_result result;
                int status = Tc2UiTxPumpDrain(
                        pump, capture_write, sink,
                        chunks_per_call, &result);
                if (status != TC2_UI_TX_OK &&
                    status != TC2_UI_TX_BACKPRESSURE) {
                        return status;
                }
                if (result.emitted_chunks > chunks_per_call) {
                        return TC2_UI_TX_INVALID_STATE;
                }
                if (emitted_bytes) {
                        *emitted_bytes += result.emitted_bytes;
                }
                if (completed_frames) {
                        *completed_frames +=
                                result.completed_frames;
                }
                if (++guard > 100000u) {
                        return TC2_UI_TX_INVALID_STATE;
                }
        }
        return TC2_UI_TX_OK;
}

static int test_exact_stream_and_chunk_bound(void) {
        static const char restore[] =
                "\033[0m\033[?25h\033[44;1H> "
                "dispatch 14 A 80 d7";
        const size_t payload_length = 8193u;
        const size_t restore_length = sizeof(restore) - 1u;
        size_t expected_length = 0;
        size_t emitted_bytes = 0;
        unsigned int completed_frames = 0;
        tc2_ui_tx_drain_result result;

        Tc2UiTxPumpInit(&test_pump);
        reset_sink();
        fill_pattern(test_payload_a, payload_length, 1u);
        append_expected(test_payload_a, payload_length,
                        &expected_length);
        append_expected(restore, restore_length, &expected_length);

        CHECK(Tc2UiTxPumpNeedsFullRedraw(&test_pump));
        CHECK(publish_frame(
                      &test_pump, 1u, TC2_UI_TX_FRAME_FULL,
                      test_payload_a, payload_length,
                      restore, restore_length, 137u) ==
              TC2_UI_TX_OK);
        CHECK(!Tc2UiTxPumpNeedsFullRedraw(&test_pump));
        CHECK(Tc2UiTxPumpLastAcceptedGeneration(&test_pump) == 1u);
        CHECK(drain_until_idle(
                      &test_pump, &test_sink, 2u,
                      &emitted_bytes, &completed_frames) ==
              TC2_UI_TX_OK);
        CHECK(emitted_bytes == expected_length);
        CHECK(completed_frames == 1u);
        CHECK(test_sink.maximum_chunk <=
              TC2_UI_TX_CHUNK_CAPACITY);
        CHECK(test_sink.maximum_chunk > 0);
        CHECK(test_sink.length == expected_length);
        CHECK(memcmp(test_sink.bytes, test_expected,
                     expected_length) == 0);
        CHECK(Tc2UiTxPumpLastCompletedGeneration(
                      &test_pump) == 1u);
        CHECK(!Tc2UiTxPumpHasWork(&test_pump));

        (void)memset(&result, 0x5a, sizeof(result));
        CHECK(Tc2UiTxPumpDrain(
                      &test_pump, capture_write, &test_sink,
                      1u, &result) == TC2_UI_TX_IDLE);
        CHECK(result.emitted_bytes == 0);
        CHECK(result.emitted_chunks == 0);
        CHECK(result.completed_frames == 0);
        CHECK(result.idle_after == 1u);
        return 1;
}

static int test_no_frame_interleaving(void) {
        static const char restore_a[] =
                "\033[0m\033[40;1H> frame-ten";
        static const char restore_b[] =
                "\033[0m\033[40;1H> frame-eleven";
        const size_t length_a = 509u;
        const size_t length_b = 421u;
        size_t expected_length = 0;
        unsigned int completed_frames = 0;

        Tc2UiTxPumpInit(&test_pump);
        reset_sink();
        fill_pattern(test_payload_a, length_a, 2u);
        fill_pattern(test_payload_b, length_b, 3u);
        append_expected(test_payload_a, length_a,
                        &expected_length);
        append_expected(restore_a, sizeof(restore_a) - 1u,
                        &expected_length);
        append_expected(test_payload_b, length_b,
                        &expected_length);
        append_expected(restore_b, sizeof(restore_b) - 1u,
                        &expected_length);

        CHECK(publish_frame(
                      &test_pump, 10u, TC2_UI_TX_FRAME_FULL,
                      test_payload_a, length_a,
                      restore_a, sizeof(restore_a) - 1u, 191u) ==
              TC2_UI_TX_OK);
        CHECK(publish_frame(
                      &test_pump, 11u, TC2_UI_TX_FRAME_DELTA,
                      test_payload_b, length_b,
                      restore_b, sizeof(restore_b) - 1u, 73u) ==
              TC2_UI_TX_OK);
        CHECK(drain_until_idle(
                      &test_pump, &test_sink, 5u,
                      NULL, &completed_frames) == TC2_UI_TX_OK);
        CHECK(completed_frames == 2u);
        CHECK(test_sink.length == expected_length);
        CHECK(memcmp(test_sink.bytes, test_expected,
                     expected_length) == 0);
        CHECK(Tc2UiTxPumpLastCompletedGeneration(
                      &test_pump) == 11u);
        return 1;
}

static int test_backpressure_and_coalescing(void) {
        static const char restore_1[] = "\033[0m>P1";
        static const char restore_2[] = "\033[0m>P2";
        static const char restore_3[] = "\033[0m>P3";
        const size_t length_1 = 500u;
        const size_t length_2 = 180u;
        const size_t length_3 = 260u;
        size_t expected_length = 0;
        tc2_ui_tx_drain_result result;

        Tc2UiTxPumpInit(&test_pump);
        reset_sink();
        fill_pattern(test_payload_a, length_1, 4u);
        fill_pattern(test_payload_b, length_2, 5u);
        fill_pattern(test_payload_c, length_3, 6u);
        CHECK(publish_frame(
                      &test_pump, 1u, TC2_UI_TX_FRAME_FULL,
                      test_payload_a, length_1,
                      restore_1, sizeof(restore_1) - 1u, 0) ==
              TC2_UI_TX_OK);

        test_sink.block_on_call = 1u;
        CHECK(Tc2UiTxPumpDrain(
                      &test_pump, capture_write, &test_sink,
                      4u, &result) == TC2_UI_TX_BACKPRESSURE);
        CHECK(result.emitted_bytes == 0);
        CHECK(result.emitted_chunks == 0);
        CHECK(test_sink.length == 0);

        CHECK(publish_frame(
                      &test_pump, 2u, TC2_UI_TX_FRAME_DELTA,
                      test_payload_b, length_2,
                      restore_2, sizeof(restore_2) - 1u, 0) ==
              TC2_UI_TX_OK);
        CHECK(publish_frame(
                      &test_pump, 3u, TC2_UI_TX_FRAME_DELTA,
                      test_payload_c, length_3,
                      restore_3, sizeof(restore_3) - 1u, 0) ==
              TC2_UI_TX_NEEDS_FULL_REDRAW);
        CHECK(Tc2UiTxPumpNeedsFullRedraw(&test_pump));
        CHECK(Tc2UiTxPumpLastAcceptedGeneration(
                      &test_pump) == 2u);

        CHECK(publish_frame(
                      &test_pump, 3u, TC2_UI_TX_FRAME_FULL,
                      test_payload_c, length_3,
                      restore_3, sizeof(restore_3) - 1u, 0) ==
              TC2_UI_TX_OK);
        CHECK(!Tc2UiTxPumpNeedsFullRedraw(&test_pump));
        append_expected(test_payload_a, length_1,
                        &expected_length);
        append_expected(restore_1, sizeof(restore_1) - 1u,
                        &expected_length);
        append_expected(test_payload_c, length_3,
                        &expected_length);
        append_expected(restore_3, sizeof(restore_3) - 1u,
                        &expected_length);
        CHECK(drain_until_idle(
                      &test_pump, &test_sink, 3u,
                      NULL, NULL) == TC2_UI_TX_OK);
        CHECK(test_sink.length == expected_length);
        CHECK(memcmp(test_sink.bytes, test_expected,
                     expected_length) == 0);
        CHECK(Tc2UiTxPumpLastCompletedGeneration(
                      &test_pump) == 3u);

        /*
         * Full redraws are independently renderable.  When all three slots
         * are occupied, the newest full replaces only the queued full.
         */
        Tc2UiTxPumpInit(&test_pump);
        reset_sink();
        expected_length = 0;
        CHECK(publish_frame(
                      &test_pump, 10u, TC2_UI_TX_FRAME_FULL,
                      test_payload_a, length_1,
                      restore_1, sizeof(restore_1) - 1u, 0) ==
              TC2_UI_TX_OK);
        CHECK(publish_frame(
                      &test_pump, 11u, TC2_UI_TX_FRAME_FULL,
                      test_payload_b, length_2,
                      restore_2, sizeof(restore_2) - 1u, 0) ==
              TC2_UI_TX_OK);
        CHECK(publish_frame(
                      &test_pump, 12u, TC2_UI_TX_FRAME_FULL,
                      test_payload_c, length_3,
                      restore_3, sizeof(restore_3) - 1u, 0) ==
              TC2_UI_TX_OK);
        append_expected(test_payload_a, length_1,
                        &expected_length);
        append_expected(restore_1, sizeof(restore_1) - 1u,
                        &expected_length);
        append_expected(test_payload_c, length_3,
                        &expected_length);
        append_expected(restore_3, sizeof(restore_3) - 1u,
                        &expected_length);
        CHECK(drain_until_idle(
                      &test_pump, &test_sink, 8u,
                      NULL, NULL) == TC2_UI_TX_OK);
        CHECK(test_sink.length == expected_length);
        CHECK(memcmp(test_sink.bytes, test_expected,
                     expected_length) == 0);
        CHECK(Tc2UiTxPumpLastAcceptedGeneration(
                      &test_pump) == 12u);
        CHECK(Tc2UiTxPumpLastCompletedGeneration(
                      &test_pump) == 12u);
        return 1;
}

static int test_output_failure_and_recovery(void) {
        static const char restore_1[] = "\033[0m>old";
        static const char restore_2[] = "\033[0m>recovered";
        const size_t length_1 = 300u;
        const size_t length_2 = 333u;
        size_t expected_length = 0;
        tc2_ui_tx_drain_result result;

        Tc2UiTxPumpInit(&test_pump);
        reset_sink();
        fill_pattern(test_payload_a, length_1, 7u);
        fill_pattern(test_payload_b, length_2, 8u);
        CHECK(publish_frame(
                      &test_pump, 1u, TC2_UI_TX_FRAME_FULL,
                      test_payload_a, length_1,
                      restore_1, sizeof(restore_1) - 1u, 0) ==
              TC2_UI_TX_OK);
        test_sink.fail_on_call = 1u;
        CHECK(Tc2UiTxPumpDrain(
                      &test_pump, capture_write, &test_sink,
                      2u, &result) == TC2_UI_TX_OUTPUT_FAILURE);
        CHECK(result.output_failed == 1u);
        CHECK(result.full_redraw_required == 1u);
        CHECK(result.idle_after == 1u);
        CHECK(test_sink.length == 0);
        CHECK(!Tc2UiTxPumpHasWork(&test_pump));
        CHECK(Tc2UiTxPumpOutputFailed(&test_pump));
        CHECK(Tc2UiTxPumpNeedsFullRedraw(&test_pump));
        CHECK(Tc2UiTxPumpLastCompletedGeneration(
                      &test_pump) == 0u);

        CHECK(publish_frame(
                      &test_pump, 2u, TC2_UI_TX_FRAME_DELTA,
                      test_payload_b, length_2,
                      restore_2, sizeof(restore_2) - 1u, 0) ==
              TC2_UI_TX_NEEDS_FULL_REDRAW);
        CHECK(publish_frame(
                      &test_pump, 2u, TC2_UI_TX_FRAME_FULL,
                      test_payload_b, length_2,
                      restore_2, sizeof(restore_2) - 1u, 0) ==
              TC2_UI_TX_OK);
        CHECK(Tc2UiTxPumpOutputFailed(&test_pump));
        test_sink.fail_on_call = 0;
        append_expected(test_payload_b, length_2,
                        &expected_length);
        append_expected(restore_2, sizeof(restore_2) - 1u,
                        &expected_length);
        CHECK(drain_until_idle(
                      &test_pump, &test_sink, 2u,
                      NULL, NULL) == TC2_UI_TX_OK);
        CHECK(!Tc2UiTxPumpOutputFailed(&test_pump));
        CHECK(!Tc2UiTxPumpNeedsFullRedraw(&test_pump));
        CHECK(test_sink.length == expected_length);
        CHECK(memcmp(test_sink.bytes, test_expected,
                     expected_length) == 0);
        CHECK(Tc2UiTxPumpLastCompletedGeneration(
                      &test_pump) == 2u);

        Tc2UiTxPumpInit(&test_pump);
        reset_sink();
        CHECK(publish_frame(
                      &test_pump, 10u, TC2_UI_TX_FRAME_FULL,
                      test_payload_a, length_1,
                      restore_1, sizeof(restore_1) - 1u, 0) ==
              TC2_UI_TX_OK);
        test_sink.invalid_on_call = 1u;
        CHECK(Tc2UiTxPumpDrain(
                      &test_pump, capture_write, &test_sink,
                      1u, NULL) == TC2_UI_TX_OUTPUT_FAILURE);
        CHECK(Tc2UiTxPumpOutputFailed(&test_pump));
        CHECK(Tc2UiTxPumpNeedsFullRedraw(&test_pump));
        return 1;
}

static int test_generation_wrap_and_abort(void) {
        static const char restore[] = "\033[0m>wrap";

        Tc2UiTxPumpInit(&test_pump);
        reset_sink();
        fill_pattern(test_payload_a, 17u, 9u);
        CHECK(publish_frame(
                      &test_pump, UINT32_MAX,
                      TC2_UI_TX_FRAME_FULL,
                      test_payload_a, 17u,
                      restore, sizeof(restore) - 1u, 0) ==
              TC2_UI_TX_OK);
        CHECK(drain_until_idle(
                      &test_pump, &test_sink, 1u,
                      NULL, NULL) == TC2_UI_TX_OK);
        CHECK(publish_frame(
                      &test_pump, 1u, TC2_UI_TX_FRAME_DELTA,
                      test_payload_a, 17u,
                      restore, sizeof(restore) - 1u, 0) ==
              TC2_UI_TX_OK);
        CHECK(drain_until_idle(
                      &test_pump, &test_sink, 1u,
                      NULL, NULL) == TC2_UI_TX_OK);
        CHECK(Tc2UiTxPumpLastAcceptedGeneration(
                      &test_pump) == 1u);
        CHECK(Tc2UiTxPumpLastCompletedGeneration(
                      &test_pump) == 1u);
        CHECK(Tc2UiTxPumpBeginFrame(
                      &test_pump, UINT32_MAX) ==
              TC2_UI_TX_STALE_GENERATION);
        CHECK(Tc2UiTxPumpBeginFrame(&test_pump, 1u) ==
              TC2_UI_TX_STALE_GENERATION);
        CHECK(Tc2UiTxPumpBeginFrame(&test_pump, 0u) ==
              TC2_UI_TX_INVALID_ARGUMENT);
        CHECK(Tc2UiTxPumpBeginFrame(
                      &test_pump, UINT32_C(0x80000001)) ==
              TC2_UI_TX_STALE_GENERATION);

        CHECK(Tc2UiTxPumpBeginFrame(&test_pump, 2u) ==
              TC2_UI_TX_OK);
        Tc2UiTxPumpAbortFrame(&test_pump);
        CHECK(!Tc2UiTxPumpNeedsFullRedraw(&test_pump));
        CHECK(Tc2UiTxPumpLastAcceptedGeneration(
                      &test_pump) == 1u);
        CHECK(Tc2UiTxPumpBeginFrame(&test_pump, 2u) ==
              TC2_UI_TX_OK);
        CHECK(Tc2UiTxPumpStageWrite(
                      &test_pump, "x", 1u) == 0);
        Tc2UiTxPumpAbortFrame(&test_pump);
        CHECK(Tc2UiTxPumpNeedsFullRedraw(&test_pump));
        CHECK(Tc2UiTxPumpLastAcceptedGeneration(
                      &test_pump) == 1u);
        return 1;
}

static int test_invalid_and_oversize_inputs(void) {
        static const char restore[] = "\033[0m>";
        const size_t restore_length = sizeof(restore) - 1u;
        const size_t boundary_payload =
                TC2_UI_TX_FRAME_CAPACITY -
                TC2_UI_TX_RESTORE_CAPACITY;
        size_t expected_length = 0;

        Tc2UiTxPumpInit(NULL);
        CHECK(!Tc2UiTxPumpHasWork(NULL));
        CHECK(!Tc2UiTxPumpNeedsFullRedraw(NULL));
        CHECK(!Tc2UiTxPumpOutputFailed(NULL));
        CHECK(Tc2UiTxPumpLastAcceptedGeneration(NULL) == 0u);
        CHECK(Tc2UiTxPumpLastCompletedGeneration(NULL) == 0u);

        Tc2UiTxPumpInit(&test_pump);
        reset_sink();
        fill_pattern(test_payload_a,
                     TC2_UI_TX_FRAME_CAPACITY, 10u);
        (void)memset(test_restore, 'R', sizeof(test_restore));
        CHECK(Tc2UiTxPumpStageWrite(
                      &test_pump, "x", 1u) == -1);
        CHECK(Tc2UiTxPumpCommitFrame(
                      &test_pump, TC2_UI_TX_FRAME_FULL,
                      restore, restore_length) ==
              TC2_UI_TX_INVALID_STATE);
        CHECK(Tc2UiTxPumpBeginFrame(&test_pump, 0u) ==
              TC2_UI_TX_INVALID_ARGUMENT);

        CHECK(Tc2UiTxPumpBeginFrame(&test_pump, 1u) ==
              TC2_UI_TX_OK);
        CHECK(Tc2UiTxPumpBeginFrame(&test_pump, 2u) ==
              TC2_UI_TX_BUSY);
        CHECK(Tc2UiTxPumpStageWrite(
                      &test_pump, NULL, 1u) == -1);
        CHECK(Tc2UiTxPumpStageWrite(
                      &test_pump, "x", 0u) == -1);
        CHECK(Tc2UiTxPumpStageWrite(
                      &test_pump, "x", 1u) == 0);
        CHECK(Tc2UiTxPumpCommitFrame(
                      &test_pump,
                      (tc2_ui_tx_frame_kind)99,
                      restore, restore_length) ==
              TC2_UI_TX_INVALID_ARGUMENT);

        CHECK(Tc2UiTxPumpBeginFrame(&test_pump, 1u) ==
              TC2_UI_TX_OK);
        CHECK(Tc2UiTxPumpStageWrite(
                      &test_pump, test_payload_a,
                      TC2_UI_TX_FRAME_CAPACITY + 1u) == -1);
        CHECK(Tc2UiTxPumpCommitFrame(
                      &test_pump, TC2_UI_TX_FRAME_FULL,
                      restore, restore_length) ==
              TC2_UI_TX_OVERSIZE);

        CHECK(Tc2UiTxPumpBeginFrame(&test_pump, 1u) ==
              TC2_UI_TX_OK);
        CHECK(Tc2UiTxPumpStageWrite(
                      &test_pump, "x", 1u) == 0);
        CHECK(Tc2UiTxPumpCommitFrame(
                      &test_pump, TC2_UI_TX_FRAME_FULL,
                      test_restore,
                      TC2_UI_TX_RESTORE_CAPACITY + 1u) ==
              TC2_UI_TX_OVERSIZE);

        CHECK(Tc2UiTxPumpBeginFrame(&test_pump, 1u) ==
              TC2_UI_TX_OK);
        CHECK(Tc2UiTxPumpStageWrite(
                      &test_pump, test_payload_a,
                      TC2_UI_TX_FRAME_CAPACITY) == 0);
        CHECK(Tc2UiTxPumpCommitFrame(
                      &test_pump, TC2_UI_TX_FRAME_FULL,
                      restore, restore_length) ==
              TC2_UI_TX_OVERSIZE);

        CHECK(Tc2UiTxPumpBeginFrame(&test_pump, 1u) ==
              TC2_UI_TX_OK);
        CHECK(Tc2UiTxPumpStageWrite(
                      &test_pump, "x", 1u) == 0);
        CHECK(Tc2UiTxPumpCommitFrame(
                      &test_pump, TC2_UI_TX_FRAME_FULL,
                      NULL, restore_length) ==
              TC2_UI_TX_INVALID_ARGUMENT);

        CHECK(Tc2UiTxPumpBeginFrame(&test_pump, 1u) ==
              TC2_UI_TX_OK);
        CHECK(Tc2UiTxPumpStageWrite(
                      &test_pump, "x", 1u) == 0);
        CHECK(Tc2UiTxPumpCommitFrame(
                      &test_pump, TC2_UI_TX_FRAME_FULL,
                      restore, 0u) ==
              TC2_UI_TX_INVALID_ARGUMENT);

        /*
         * The largest legal aggregate frame must remain exact, including
         * the maximum-size caller-owned prompt/cursor restoration trailer.
         */
        CHECK(Tc2UiTxPumpBeginFrame(&test_pump, 1u) ==
              TC2_UI_TX_OK);
        CHECK(Tc2UiTxPumpStageWrite(
                      &test_pump, test_payload_a,
                      boundary_payload) == 0);
        CHECK(Tc2UiTxPumpCommitFrame(
                      &test_pump, TC2_UI_TX_FRAME_FULL,
                      test_restore,
                      TC2_UI_TX_RESTORE_CAPACITY) ==
              TC2_UI_TX_OK);
        append_expected(test_payload_a, boundary_payload,
                        &expected_length);
        append_expected(test_restore,
                        TC2_UI_TX_RESTORE_CAPACITY,
                        &expected_length);
        CHECK(expected_length == TC2_UI_TX_FRAME_CAPACITY);
        CHECK(Tc2UiTxPumpDrain(
                      NULL, capture_write, &test_sink,
                      1u, NULL) ==
              TC2_UI_TX_INVALID_ARGUMENT);
        CHECK(Tc2UiTxPumpDrain(
                      &test_pump, NULL, &test_sink,
                      1u, NULL) ==
              TC2_UI_TX_INVALID_ARGUMENT);
        CHECK(Tc2UiTxPumpDrain(
                      &test_pump, capture_write, &test_sink,
                      0u, NULL) ==
              TC2_UI_TX_INVALID_ARGUMENT);
        CHECK(drain_until_idle(
                      &test_pump, &test_sink, 7u,
                      NULL, NULL) == TC2_UI_TX_OK);
        CHECK(test_sink.length == expected_length);
        CHECK(memcmp(test_sink.bytes, test_expected,
                     expected_length) == 0);
        CHECK(test_sink.maximum_chunk <=
              TC2_UI_TX_CHUNK_CAPACITY);
        return 1;
}

typedef int (*test_function)(void);

typedef struct {
        const char *name;
        test_function function;
} named_test;

int main(void) {
        static const named_test tests[] = {
                {"exact stream and chunk bound",
                 test_exact_stream_and_chunk_bound},
                {"no frame interleaving",
                 test_no_frame_interleaving},
                {"backpressure and coalescing",
                 test_backpressure_and_coalescing},
                {"output failure and recovery",
                 test_output_failure_and_recovery},
                {"generation wrap and abort",
                 test_generation_wrap_and_abort},
                {"invalid and oversize inputs",
                 test_invalid_and_oversize_inputs},
        };
        unsigned int passed = 0;

        for (size_t index = 0;
             index < sizeof(tests) / sizeof(tests[0]);
             ++index) {
                if (!tests[index].function()) {
                        (void)fprintf(stderr, "FAILED: %s\n",
                                      tests[index].name);
                        return 1;
                }
                ++passed;
                (void)printf("PASS: %s\n", tests[index].name);
        }
        (void)printf("tc2_ui_tx_pump_test: %u/%zu passed\n",
                     passed, sizeof(tests) / sizeof(tests[0]));
        return 0;
}
