#ifndef TC2_OFFLINE_CONTROLLER_H
#define TC2_OFFLINE_CONTROLLER_H

#include <stddef.h>
#include <stdint.h>

#include "tc2_offline_planner.h"
#include "tc2_offline_runtime.h"
#include "tc2_ui_overlay.h"
#include "tc2_ui_renderer.h"
#include "tc2_ui_tx_pump.h"

#ifdef MODE_TC2

/*
 * One bounded, prediction-only owner for the complete offline UI pipeline:
 *
 *   planner -> deterministic runtime -> provenance overlay
 *           -> delta renderer -> atomic nonblocking transmit pump
 *
 * The controller deliberately exposes no CAN, turnout, train-control, or
 * reservation-service dependency.  Every position and sensor pulse it
 * publishes is explicitly predicted/estimated and is never motion authority.
 */
#define TC2_OFFLINE_CONTROLLER_MAX_TRAINS \
        TC2_OFFLINE_RUNTIME_MAX_TRAINS

#define TC2_OFFLINE_CONTROLLER_EVIDENCE_LABEL \
        "OFFLINE SIMULATION / PREDICTED / NO CAN / NO PHYSICAL MOTION"

typedef enum {
        TC2_OFFLINE_CONTROLLER_OK = 0,
        TC2_OFFLINE_CONTROLLER_NO_CHANGE = 1,
        TC2_OFFLINE_CONTROLLER_BACKPRESSURE = 2,
        TC2_OFFLINE_CONTROLLER_NEEDS_FULL_REDRAW = 3,
        TC2_OFFLINE_CONTROLLER_INVALID_ARGUMENT = -1,
        TC2_OFFLINE_CONTROLLER_NOT_INITIALIZED = -2,
        TC2_OFFLINE_CONTROLLER_FAILED_CLOSED = -3,
        TC2_OFFLINE_CONTROLLER_PLANNER_ERROR = -4,
        TC2_OFFLINE_CONTROLLER_RUNTIME_ERROR = -5,
        TC2_OFFLINE_CONTROLLER_OVERLAY_ERROR = -6,
        TC2_OFFLINE_CONTROLLER_RENDER_ERROR = -7,
        TC2_OFFLINE_CONTROLLER_TX_ERROR = -8,
        TC2_OFFLINE_CONTROLLER_CAPACITY = -9,
        TC2_OFFLINE_CONTROLLER_NOT_FOUND = -10,
        TC2_OFFLINE_CONTROLLER_INVALID_STATE = -11
} tc2_offline_controller_status;

typedef struct {
        int active;
        int train;
        int start_index;
        int speed;
        int destination_index;
        uint32_t plan_generation;
        uint64_t publication_serial;
        uint32_t launch_epoch;
        uint32_t position_revision;
        uint32_t last_event_serial;
        uint32_t last_position_event_tick;
        int last_row;
        int last_column;
} tc2_offline_controller_train;

typedef struct {
        int initialized;
        int failed_closed;
        int last_error;
        int last_subsystem_error;
        int predicted_only;
        int is_live;
        int source;
        uint32_t tick;
        int overlay_train_count;
        int output_pending;
        int full_redraw_required;
        int output_failed;
        uint32_t last_accepted_generation;
        uint32_t last_completed_generation;
        tc2_offline_snapshot runtime;
} tc2_offline_controller_snapshot;

typedef struct {
        uint32_t generation;
        int frame_kind;
        int queued;
        tc2_ui_render_result renderer;
} tc2_offline_controller_render_result;

/*
 * This object is intentionally large and must live in static/BSS storage.
 * It owns all staging buffers so no task-stack allocation or heap is needed.
 */
typedef struct {
        int initialized;
        int failed_closed;
        int last_error;
        int last_subsystem_error;
        int event_error;
        uint32_t tick;
        uint32_t next_plan_generation;
        uint64_t next_publication_serial;
        uint32_t next_launch_epoch;
        uint32_t next_render_generation;
        tc2_offline_runtime runtime;
        tc2_ui_overlay overlay;
        tc2_ui_renderer renderer;
        tc2_ui_tx_pump pump;
        tc2_offline_plan planner_scratch;
        tc2_offline_controller_train
                trains[TC2_OFFLINE_CONTROLLER_MAX_TRAINS];
} tc2_offline_controller;

int Tc2OfflineControllerInitialize(
        tc2_offline_controller *controller, uint32_t initial_tick);
void Tc2OfflineControllerReset(
        tc2_offline_controller *controller, uint32_t initial_tick);

/*
 * StageTrip validates a complete A-F -> d1-d8 plan before publishing it.
 * All speeds 1..120 are supported.  Re-staging a removed identical trip gets
 * a newer controller publication generation, so stale UI updates cannot
 * resurrect the old trip.
 */
int Tc2OfflineControllerStageTrip(
        tc2_offline_controller *controller,
        track_node track[TRACK_MAX],
        int train, int start_index, int speed,
        int destination_index);
int Tc2OfflineControllerStartTrip(
        tc2_offline_controller *controller, int train);
int Tc2OfflineControllerStartAll(
        tc2_offline_controller *controller);
int Tc2OfflineControllerRemoveTrip(
        tc2_offline_controller *controller, int train);

/*
 * Advance the prediction runtime and atomically apply every predicted event
 * to the retained overlay.  Any provenance mismatch or impossible state
 * fails the controller closed until Reset.
 */
int Tc2OfflineControllerStep(
        tc2_offline_controller *controller, uint32_t now_tick);

/*
 * Render publishes one immutable frame to the transmit pump and appends
 * prompt_restore verbatim.  Drain never blocks: WOULD_BLOCK preserves the
 * exact byte offset; hard output failure requires a later full redraw.
 */
int Tc2OfflineControllerRender(
        tc2_offline_controller *controller,
        const char *prompt_restore, size_t prompt_restore_length,
        tc2_offline_controller_render_result *result);
int Tc2OfflineControllerDrain(
        tc2_offline_controller *controller,
        tc2_ui_tx_write_fn write, void *write_context,
        unsigned int max_chunks,
        tc2_ui_tx_drain_result *result);

int Tc2OfflineControllerGetTrainSnapshot(
        const tc2_offline_controller *controller, int train,
        tc2_offline_train_snapshot *snapshot);
int Tc2OfflineControllerGetSnapshot(
        const tc2_offline_controller *controller,
        tc2_offline_controller_snapshot *snapshot);
const tc2_ui_overlay *Tc2OfflineControllerOverlay(
        const tc2_offline_controller *controller);

int Tc2OfflineControllerHasPendingOutput(
        const tc2_offline_controller *controller);
int Tc2OfflineControllerNeedsFullRedraw(
        const tc2_offline_controller *controller);
const char *Tc2OfflineControllerEvidenceLabel(void);

#endif

#endif
