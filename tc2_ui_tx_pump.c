#ifndef MODE_TC2

typedef int tc2_ui_tx_pump_disabled_translation_unit;

#else

#include "tc2_ui_tx_pump.h"

#define TC2_UI_TX_NO_SLOT (-1)
#define TC2_UI_TX_SERIAL_HALF_RANGE UINT32_C(0x80000000)

static int valid_slot_index(int slot) {
        return slot >= 0 && slot < TC2_UI_TX_SLOT_COUNT;
}

static int valid_frame_kind(tc2_ui_tx_frame_kind kind) {
        return kind == TC2_UI_TX_FRAME_FULL ||
               kind == TC2_UI_TX_FRAME_DELTA;
}

static int generation_is_newer(
        uint32_t candidate, uint32_t reference) {
        uint32_t difference;
        if (candidate == 0) return 0;
        if (reference == 0) return 1;
        difference = candidate - reference;
        return difference != 0 &&
               difference < TC2_UI_TX_SERIAL_HALF_RANGE;
}

static void clear_slot(
        tc2_ui_tx_pump *pump, int slot) {
        if (!pump || !valid_slot_index(slot)) return;
        pump->slots[slot].length = 0;
        pump->slots[slot].generation = 0;
        pump->slots[slot].kind = 0;
}

static int find_free_slot(const tc2_ui_tx_pump *pump) {
        if (!pump) return TC2_UI_TX_NO_SLOT;
        for (int slot = 0; slot < TC2_UI_TX_SLOT_COUNT; ++slot) {
                if (slot != pump->active_slot &&
                    slot != pump->pending_slot &&
                    slot != pump->staging_slot) {
                        return slot;
                }
        }
        return TC2_UI_TX_NO_SLOT;
}

static void copy_bytes(
        char *destination, const char *source, size_t length) {
        for (size_t index = 0; index < length; ++index) {
                destination[index] = source[index];
        }
}

static void discard_staging(
        tc2_ui_tx_pump *pump) {
        int staging;
        if (!pump) return;
        staging = pump->staging_slot;
        pump->staging_slot = TC2_UI_TX_NO_SLOT;
        pump->staging_error = TC2_UI_TX_OK;
        clear_slot(pump, staging);
}

static void require_full_after_abandoned_staging(
        tc2_ui_tx_pump *pump) {
        if (!pump) return;
        if (valid_slot_index(pump->staging_slot) &&
            (pump->slots[pump->staging_slot].length > 0 ||
             pump->staging_error != TC2_UI_TX_OK)) {
                pump->full_redraw_required = 1;
        }
        discard_staging(pump);
}

static void discard_transmit_queue_after_failure(
        tc2_ui_tx_pump *pump) {
        int active;
        int pending;
        if (!pump) return;
        active = pump->active_slot;
        pending = pump->pending_slot;
        pump->active_slot = TC2_UI_TX_NO_SLOT;
        pump->pending_slot = TC2_UI_TX_NO_SLOT;
        pump->active_offset = 0;
        clear_slot(pump, active);
        clear_slot(pump, pending);
        require_full_after_abandoned_staging(pump);
        pump->full_redraw_required = 1;
        pump->output_failed = 1;
}

static void fill_drain_result(
        const tc2_ui_tx_pump *pump,
        tc2_ui_tx_drain_result *result) {
        if (!pump || !result) return;
        result->last_completed_generation =
                pump->last_completed_generation;
        result->idle_after =
                !valid_slot_index(pump->active_slot);
        result->full_redraw_required =
                pump->full_redraw_required != 0;
        result->output_failed = pump->output_failed != 0;
}

void Tc2UiTxPumpInit(tc2_ui_tx_pump *pump) {
        if (!pump) return;
        pump->active_slot = TC2_UI_TX_NO_SLOT;
        pump->pending_slot = TC2_UI_TX_NO_SLOT;
        pump->staging_slot = TC2_UI_TX_NO_SLOT;
        pump->active_offset = 0;
        pump->last_accepted_generation = 0;
        pump->last_completed_generation = 0;
        pump->staging_error = TC2_UI_TX_OK;
        pump->full_redraw_required = 1;
        pump->output_failed = 0;
        for (int slot = 0; slot < TC2_UI_TX_SLOT_COUNT; ++slot) {
                clear_slot(pump, slot);
        }
}

int Tc2UiTxPumpBeginFrame(
        tc2_ui_tx_pump *pump, uint32_t generation) {
        int slot;
        if (!pump || generation == 0) {
                return TC2_UI_TX_INVALID_ARGUMENT;
        }
        if (valid_slot_index(pump->staging_slot)) {
                return TC2_UI_TX_BUSY;
        }
        if (!generation_is_newer(
                    generation,
                    pump->last_accepted_generation)) {
                return TC2_UI_TX_STALE_GENERATION;
        }
        slot = find_free_slot(pump);
        if (!valid_slot_index(slot)) {
                return TC2_UI_TX_BUSY;
        }
        clear_slot(pump, slot);
        pump->slots[slot].generation = generation;
        pump->staging_slot = slot;
        pump->staging_error = TC2_UI_TX_OK;
        return TC2_UI_TX_OK;
}

int Tc2UiTxPumpStageWrite(
        void *context, const char *bytes, size_t length) {
        tc2_ui_tx_pump *pump = context;
        tc2_ui_tx_slot *slot;
        if (!pump || !bytes || length == 0 ||
            !valid_slot_index(pump->staging_slot)) {
                return -1;
        }
        if (pump->staging_error != TC2_UI_TX_OK) {
                return -1;
        }
        slot = &pump->slots[pump->staging_slot];
        if (slot->length > TC2_UI_TX_FRAME_CAPACITY ||
            length > TC2_UI_TX_FRAME_CAPACITY - slot->length) {
                pump->staging_error = TC2_UI_TX_OVERSIZE;
                return -1;
        }
        copy_bytes(slot->bytes + slot->length, bytes, length);
        slot->length += length;
        return 0;
}

void Tc2UiTxPumpAbortFrame(tc2_ui_tx_pump *pump) {
        require_full_after_abandoned_staging(pump);
}

int Tc2UiTxPumpCommitFrame(
        tc2_ui_tx_pump *pump, tc2_ui_tx_frame_kind kind,
        const char *prompt_restore, size_t prompt_restore_length) {
        int staging;
        tc2_ui_tx_slot *slot;
        int old_pending;
        if (!pump || !valid_slot_index(pump->staging_slot)) {
                return TC2_UI_TX_INVALID_STATE;
        }
        staging = pump->staging_slot;
        slot = &pump->slots[staging];
        if (pump->staging_error != TC2_UI_TX_OK) {
                int error = pump->staging_error;
                require_full_after_abandoned_staging(pump);
                return error;
        }
        if (!valid_frame_kind(kind) || !prompt_restore ||
            prompt_restore_length == 0 || slot->length == 0) {
                require_full_after_abandoned_staging(pump);
                return TC2_UI_TX_INVALID_ARGUMENT;
        }
        if (prompt_restore_length > TC2_UI_TX_RESTORE_CAPACITY) {
                require_full_after_abandoned_staging(pump);
                return TC2_UI_TX_OVERSIZE;
        }
        if (slot->length > TC2_UI_TX_FRAME_CAPACITY ||
            prompt_restore_length >
                    TC2_UI_TX_FRAME_CAPACITY - slot->length) {
                pump->staging_error = TC2_UI_TX_OVERSIZE;
                require_full_after_abandoned_staging(pump);
                return TC2_UI_TX_OVERSIZE;
        }
        if (!generation_is_newer(
                    slot->generation,
                    pump->last_accepted_generation)) {
                require_full_after_abandoned_staging(pump);
                return TC2_UI_TX_STALE_GENERATION;
        }
        if (kind == TC2_UI_TX_FRAME_DELTA &&
            pump->full_redraw_required) {
                discard_staging(pump);
                return TC2_UI_TX_NEEDS_FULL_REDRAW;
        }

        copy_bytes(slot->bytes + slot->length,
                   prompt_restore, prompt_restore_length);
        slot->length += prompt_restore_length;
        slot->kind = (unsigned char)kind;

        if (!valid_slot_index(pump->active_slot)) {
                pump->active_slot = staging;
                pump->active_offset = 0;
        } else if (!valid_slot_index(pump->pending_slot)) {
                pump->pending_slot = staging;
        } else if (kind == TC2_UI_TX_FRAME_FULL) {
                old_pending = pump->pending_slot;
                pump->pending_slot = staging;
                clear_slot(pump, old_pending);
        } else {
                /*
                 * The renderer's newest delta is relative to the delta that
                 * is already pending.  Dropping either one would make the
                 * other unsafe, so emit neither and request one newest-state
                 * full frame after the immutable active frame.
                 */
                old_pending = pump->pending_slot;
                pump->pending_slot = TC2_UI_TX_NO_SLOT;
                clear_slot(pump, old_pending);
                discard_staging(pump);
                pump->full_redraw_required = 1;
                return TC2_UI_TX_NEEDS_FULL_REDRAW;
        }

        pump->staging_slot = TC2_UI_TX_NO_SLOT;
        pump->staging_error = TC2_UI_TX_OK;
        pump->last_accepted_generation = slot->generation;
        if (kind == TC2_UI_TX_FRAME_FULL) {
                pump->full_redraw_required = 0;
        }
        return TC2_UI_TX_OK;
}

int Tc2UiTxPumpDrain(
        tc2_ui_tx_pump *pump, tc2_ui_tx_write_fn write,
        void *write_context, unsigned int max_chunks,
        tc2_ui_tx_drain_result *result) {
        tc2_ui_tx_drain_result local_result = {0};
        if (!pump || !write || max_chunks == 0) {
                return TC2_UI_TX_INVALID_ARGUMENT;
        }
        if (!valid_slot_index(pump->active_slot)) {
                fill_drain_result(pump, &local_result);
                if (result) *result = local_result;
                return TC2_UI_TX_IDLE;
        }

        while (local_result.emitted_chunks < max_chunks &&
               valid_slot_index(pump->active_slot)) {
                int active = pump->active_slot;
                tc2_ui_tx_slot *slot = &pump->slots[active];
                size_t remaining;
                size_t chunk;
                int sink_status;
                if (pump->active_offset >= slot->length) {
                        discard_transmit_queue_after_failure(pump);
                        fill_drain_result(pump, &local_result);
                        if (result) *result = local_result;
                        return TC2_UI_TX_OUTPUT_FAILURE;
                }
                remaining = slot->length - pump->active_offset;
                chunk = remaining < TC2_UI_TX_CHUNK_CAPACITY ?
                        remaining : TC2_UI_TX_CHUNK_CAPACITY;
                sink_status = write(
                        write_context,
                        slot->bytes + pump->active_offset,
                        chunk);
                if (sink_status == TC2_UI_TX_SINK_WOULD_BLOCK) {
                        fill_drain_result(pump, &local_result);
                        if (result) *result = local_result;
                        return TC2_UI_TX_BACKPRESSURE;
                }
                if (sink_status != TC2_UI_TX_SINK_ACCEPTED) {
                        discard_transmit_queue_after_failure(pump);
                        fill_drain_result(pump, &local_result);
                        if (result) *result = local_result;
                        return TC2_UI_TX_OUTPUT_FAILURE;
                }
                pump->active_offset += chunk;
                local_result.emitted_bytes += chunk;
                ++local_result.emitted_chunks;

                if (pump->active_offset == slot->length) {
                        uint32_t completed_generation =
                                slot->generation;
                        int completed_kind = slot->kind;
                        int next = pump->pending_slot;
                        pump->last_completed_generation =
                                completed_generation;
                        ++local_result.completed_frames;
                        pump->active_slot = next;
                        pump->pending_slot = TC2_UI_TX_NO_SLOT;
                        pump->active_offset = 0;
                        clear_slot(pump, active);
                        if (completed_kind ==
                                    TC2_UI_TX_FRAME_FULL &&
                            pump->output_failed) {
                                pump->output_failed = 0;
                        }
                }
        }

        fill_drain_result(pump, &local_result);
        if (result) *result = local_result;
        return TC2_UI_TX_OK;
}

int Tc2UiTxPumpHasWork(const tc2_ui_tx_pump *pump) {
        return pump && valid_slot_index(pump->active_slot);
}

int Tc2UiTxPumpNeedsFullRedraw(const tc2_ui_tx_pump *pump) {
        return pump && pump->full_redraw_required != 0;
}

int Tc2UiTxPumpOutputFailed(const tc2_ui_tx_pump *pump) {
        return pump && pump->output_failed != 0;
}

uint32_t Tc2UiTxPumpLastAcceptedGeneration(
        const tc2_ui_tx_pump *pump) {
        return pump ? pump->last_accepted_generation : 0;
}

uint32_t Tc2UiTxPumpLastCompletedGeneration(
        const tc2_ui_tx_pump *pump) {
        return pump ? pump->last_completed_generation : 0;
}

#endif
