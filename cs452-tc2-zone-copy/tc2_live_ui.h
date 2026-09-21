#ifndef TC2_LIVE_UI_H
#define TC2_LIVE_UI_H

#include <stddef.h>
#include <stdint.h>

#include "tc2_dispatch.h"
#include "tc2_route_projection.h"
#include "tc2_ui_overlay.h"
#include "tc2_ui_renderer.h"
#include "tc2_ui_tx_pump.h"
#include "train_sensor.h"

#ifdef MODE_TC2

#define TC2_LIVE_UI_MAX_WAYPOINTS (TRACK_MAX + 2)
#define TC2_LIVE_UI_EVIDENCE_LABEL \
        "LIVE CAN + PREDICTED TRAJECTORY / SENSOR EVENTS ARE PHYSICAL"

typedef struct {
        int active;
        int train;
        int start_index;
        int destination_index;
        int speed;
        int current_speed;
        int last_job_state;
        uint32_t plan_generation;
        uint32_t launch_epoch;
        uint32_t position_revision;
        uint32_t last_sensor_sequence;
        uint32_t anchor_tick;
        int64_t anchor_distance_um;
        int64_t displayed_distance_um;
        /*
         * Physical sensor evidence is a one-way route gate. Prediction may
         * approach the next detector, but it cannot render beyond that
         * detector until the matching attributed rising edge advances this
         * distance.
         */
        int64_t confirmed_sensor_distance_um;
        /*
         * A one-cell CURRENT handoff is deliberately incomplete: the UI
         * keeps fetching the exact projection for the same generation.
         * When the retained cell is mid-edge, hold it until a detector on
         * the replacement route confirms which branch the train entered.
         */
        int localization_placeholder;
        int awaiting_current_sensor;
        int waypoint_count;
        tc2_dispatch_projection_waypoint
                waypoints[TC2_LIVE_UI_MAX_WAYPOINTS];
} tc2_live_ui_train;

typedef struct {
        int initialized;
        uint32_t tick;
        uint32_t next_render_generation;
        tc2_ui_overlay overlay;
        tc2_ui_renderer renderer;
        tc2_ui_tx_pump pump;
        tc2_live_ui_train trains[TC2_UI_MAX_TRAINS];
} tc2_live_ui;

typedef struct {
        uint32_t generation;
        int frame_kind;
        int queued;
        tc2_ui_render_result renderer;
} tc2_live_ui_render_result;

void Tc2LiveUiInitialize(tc2_live_ui *ui, uint32_t initial_tick);
int Tc2LiveUiHasPlan(
        const tc2_live_ui *ui, int train, uint32_t plan_generation);
int Tc2LiveUiAcceptProjection(
        tc2_live_ui *ui,
        const tc2_dispatch_projection_header *header,
        const tc2_dispatch_projection_waypoint *waypoints,
        int waypoint_count, uint32_t now_tick);
int Tc2LiveUiSync(
        tc2_live_ui *ui, const tc2_dispatch_snapshot *dispatch,
        const train_sensor_snapshot_t *sensors, uint32_t now_tick);
int Tc2LiveUiRender(
        tc2_live_ui *ui, const char *prompt_restore,
        size_t prompt_restore_length,
        tc2_live_ui_render_result *result);
int Tc2LiveUiDrain(
        tc2_live_ui *ui, tc2_ui_tx_write_fn write,
        void *write_context, unsigned int max_chunks,
        tc2_ui_tx_drain_result *result);
int Tc2LiveUiHasPendingOutput(const tc2_live_ui *ui);
int Tc2LiveUiNeedsFullRedraw(const tc2_live_ui *ui);

/*
 * Map scalar progress between two published visual waypoints to one ordered
 * rail cell. Ordinary dispatch projections are required to return one for
 * every A-F/d1-d8 segment; zero is a fail-closed unknown geometry result.
 */
int Tc2LiveUiInterpolateTrackCell(
        const tc2_dispatch_projection_waypoint *first,
        const tc2_dispatch_projection_waypoint *second,
        int64_t offset_um, int64_t span_um,
        int *row, int *column);

#endif
#endif
