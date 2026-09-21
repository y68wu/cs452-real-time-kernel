#ifndef TC2_UI_TX_PUMP_H
#define TC2_UI_TX_PUMP_H

#include <stddef.h>
#include <stdint.h>

/*
 * A complete TC2 full redraw is currently about 7 KiB.  The extra headroom
 * keeps publication atomic if the renderer grows while still making the
 * storage cost explicit.  Keep pump objects in static/BSS storage, not on a
 * task stack.
 */
#define TC2_UI_TX_FRAME_CAPACITY 16384
#define TC2_UI_TX_RESTORE_CAPACITY 384
#define TC2_UI_TX_CHUNK_CAPACITY 192
#define TC2_UI_TX_SLOT_COUNT 3

typedef enum {
        TC2_UI_TX_FRAME_FULL = 1,
        TC2_UI_TX_FRAME_DELTA = 2
} tc2_ui_tx_frame_kind;

typedef enum {
        TC2_UI_TX_OK = 0,
        TC2_UI_TX_IDLE = 1,
        TC2_UI_TX_BACKPRESSURE = 2,
        TC2_UI_TX_NEEDS_FULL_REDRAW = 3,
        TC2_UI_TX_INVALID_ARGUMENT = -1,
        TC2_UI_TX_BUSY = -2,
        TC2_UI_TX_STALE_GENERATION = -3,
        TC2_UI_TX_OVERSIZE = -4,
        TC2_UI_TX_INVALID_STATE = -5,
        TC2_UI_TX_OUTPUT_FAILURE = -6
} tc2_ui_tx_status;

/*
 * The drain sink must be nonblocking and must obey an all-or-none chunk
 * contract:
 *
 *   ACCEPTED    every byte in the supplied chunk was accepted
 *   WOULD_BLOCK no byte in the supplied chunk was accepted
 *   FAILED      acceptance is unknown; the terminal stream may be partial
 *
 * Any result other than ACCEPTED or WOULD_BLOCK is treated as FAILED.
 */
typedef enum {
        TC2_UI_TX_SINK_ACCEPTED = 0,
        TC2_UI_TX_SINK_WOULD_BLOCK = 1,
        TC2_UI_TX_SINK_FAILED = -1
} tc2_ui_tx_sink_result;

typedef int (*tc2_ui_tx_write_fn)(
        void *context, const char *bytes, size_t length);

typedef struct {
        char bytes[TC2_UI_TX_FRAME_CAPACITY];
        size_t length;
        uint32_t generation;
        unsigned char kind;
} tc2_ui_tx_slot;

typedef struct {
        tc2_ui_tx_slot slots[TC2_UI_TX_SLOT_COUNT];
        int active_slot;
        int pending_slot;
        int staging_slot;
        size_t active_offset;
        uint32_t last_accepted_generation;
        uint32_t last_completed_generation;
        int staging_error;
        unsigned char full_redraw_required;
        unsigned char output_failed;
} tc2_ui_tx_pump;

typedef struct {
        size_t emitted_bytes;
        unsigned int emitted_chunks;
        unsigned int completed_frames;
        uint32_t last_completed_generation;
        unsigned int idle_after;
        unsigned int full_redraw_required;
        unsigned int output_failed;
} tc2_ui_tx_drain_result;

void Tc2UiTxPumpInit(tc2_ui_tx_pump *pump);

/*
 * Begin -> StageWrite -> Commit is one publication transaction.  StageWrite
 * has the renderer callback signature and only copies into an unpublished
 * slot.  Commit makes the immutable frame visible to Drain in one state
 * transition.  Abort publishes nothing.
 *
 * Generations are nonzero RFC-1982-style uint32 serials.  Equal, older, and
 * exactly-half-range candidates are rejected.  UINT32_MAX -> 1 is newer.
 *
 * Calls are intended for one cooperative owner; they are not reentrant and do
 * not disable interrupts.  Transactional publication is atomic with respect
 * to that owner.
 */
int Tc2UiTxPumpBeginFrame(
        tc2_ui_tx_pump *pump, uint32_t generation);
int Tc2UiTxPumpStageWrite(
        void *context, const char *bytes, size_t length);
void Tc2UiTxPumpAbortFrame(tc2_ui_tx_pump *pump);

/*
 * prompt_restore is appended verbatim and atomically after the renderer
 * payload.  It must contain the caller's complete style reset, cursor
 * placement, prompt, and any partially typed command that must be restored.
 * The pump never invents ANSI bytes, and no next-generation bytes can precede
 * this trailer.
 */
int Tc2UiTxPumpCommitFrame(
        tc2_ui_tx_pump *pump, tc2_ui_tx_frame_kind kind,
        const char *prompt_restore, size_t prompt_restore_length);

/*
 * Attempt at most max_chunks sink calls.  Every call is 1..192 bytes and
 * contains bytes from exactly one immutable frame.  WOULD_BLOCK retains the
 * exact offset.  A hard/unknown failure discards active and pending bytes,
 * invalidates deltas, and requires a newly generated full redraw.
 */
int Tc2UiTxPumpDrain(
        tc2_ui_tx_pump *pump, tc2_ui_tx_write_fn write,
        void *write_context, unsigned int max_chunks,
        tc2_ui_tx_drain_result *result);

int Tc2UiTxPumpHasWork(const tc2_ui_tx_pump *pump);
int Tc2UiTxPumpNeedsFullRedraw(const tc2_ui_tx_pump *pump);
int Tc2UiTxPumpOutputFailed(const tc2_ui_tx_pump *pump);
uint32_t Tc2UiTxPumpLastAcceptedGeneration(
        const tc2_ui_tx_pump *pump);
uint32_t Tc2UiTxPumpLastCompletedGeneration(
        const tc2_ui_tx_pump *pump);

#endif
