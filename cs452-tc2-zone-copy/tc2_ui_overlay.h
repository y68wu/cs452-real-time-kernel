#ifndef TC2_UI_OVERLAY_H
#define TC2_UI_OVERLAY_H

#include <stddef.h>

#include "tc2_track_d_layout.h"

#define TC2_UI_MAX_TRAINS 16
#define TC2_UI_COLOR_COUNT 16
#define TC2_UI_SENSOR_COUNT TC2_TRACK_D_DIRECTED_SENSOR_COUNT

/*
 * Clock ticks are 10 ms in this project.  A sensor therefore remains in the
 * overlay for 300 ms and alternates visible/hidden every 100 ms.
 */
#define TC2_UI_SENSOR_FLASH_TICKS 30
#define TC2_UI_SENSOR_FLASH_PHASE_TICKS 10

typedef enum {
        TC2_UI_SOURCE_OFFLINE_SIMULATION = 0,
        TC2_UI_SOURCE_LIVE_CAN = 1
} tc2_ui_source;

typedef enum {
        TC2_UI_POSITION_NONE = 0,
        TC2_UI_POSITION_START,
        TC2_UI_POSITION_SENSOR,
        TC2_UI_POSITION_DESTINATION,
        TC2_UI_POSITION_PREDICTED
} tc2_ui_position_kind;

typedef enum {
        TC2_UI_QUALITY_UNKNOWN = 0,
        TC2_UI_QUALITY_PREDICTED,
        TC2_UI_QUALITY_ESTIMATED,
        TC2_UI_QUALITY_SENSOR_CONFIRMED
} tc2_ui_position_quality;

typedef struct {
        int active;
        int train;
        int color_slot;
        int start_index;
        int destination_index;
        int speed;
        int sensor_index;
        int row;
        int column;
        int position_kind;
        int position_quality;
        unsigned int plan_generation;
        unsigned int position_revision;
        unsigned int updated_at_tick;
} tc2_ui_train_overlay;

typedef struct {
        int active;
        int sensor_index;
        int train;
        int color_slot;
        /*
         * A flash is evidence, not decoration.  Keep the source, quality, and
         * trip generation captured with the event so a renderer cannot
         * accidentally present a simulated crossing as live CAN telemetry.
         */
        int evidence_source;
        int evidence_quality;
        unsigned int plan_generation;
        int row;
        int column;
        int width;
        unsigned int sequence;
        unsigned int started_at_tick;
        unsigned int expires_at_tick;
} tc2_ui_sensor_overlay;

typedef struct {
        int source;
        unsigned int refresh_generation;
        /*
         * Per physical train id generation high-water marks survive removal.
         * Without this tombstone an equal/older trip could recreate a marker
         * after RemoveTrain and make stale state look current.
         */
        unsigned int plan_generation_high_water[256];
        tc2_ui_train_overlay trains[TC2_UI_MAX_TRAINS];
        tc2_ui_sensor_overlay sensors[TC2_UI_SENSOR_COUNT];
} tc2_ui_overlay;

void Tc2UiOverlayInit(tc2_ui_overlay *overlay, int source);
int Tc2UiOverlaySetSource(tc2_ui_overlay *overlay, int source);
const char *Tc2UiOverlaySourceName(int source);
const char *Tc2UiOverlayQualityName(int quality);

/*
 * UpsertAtStart and UpsertAtPredictedCell are the only operations that may
 * create an active train.  The latter is reserved for an atomic CURRENT
 * continuation whose physical marker already occupies a drawn rail cell.
 * Every later update must carry the same non-zero plan_generation and a
 * monotonically advancing, non-zero position_revision assigned by the one UI
 * owner. Generation high-water marks survive RemoveTrain, so only a newer
 * generation may recreate the same physical train. A sensor update
 * additionally carries its raw sensor journal
 * sequence.  These are deliberately separate sequence domains: a model
 * revision must never suppress a later physical sensor event.
 *
 * Return values for start and position updates:
 *   1: the train position changed;
 *   0: an equal/older plan generation, position revision, or raw sensor
 *      sequence was ignored;
 *  -1: invalid input, unknown train, or stale plan generation.
 * A stale raw sensor event never moves the train marker, even if its caller's
 * UI position revision is newer.  Operator destinations d1-d8 are virtual
 * points and therefore can only be predicted or estimated; physical sensor
 * confirmation is represented only by UpdateAtSensor.
 */
int Tc2UiOverlayUpsertAtStart(
        tc2_ui_overlay *overlay, int train, int start_index,
        int destination_index, unsigned int plan_generation,
        int quality, unsigned int now_tick);
int Tc2UiOverlayUpsertAtPredictedCell(
        tc2_ui_overlay *overlay, int train, int row, int column,
        int destination_index, unsigned int plan_generation,
        int quality, unsigned int now_tick);
int Tc2UiOverlayUpdateAtSensor(
        tc2_ui_overlay *overlay, int train, int sensor_index,
        unsigned int plan_generation, unsigned int position_revision,
        unsigned int sensor_sequence,
        int quality, unsigned int now_tick);
/*
 * Record detector evidence without changing the train marker.  The live
 * dashboard uses this path because physical sensors rebase route distance
 * and flash their labels, while the red train marker remains owned by the
 * continuous shortest-route projection.
 */
int Tc2UiOverlayFlashSensor(
        tc2_ui_overlay *overlay, int train, int sensor_index,
        unsigned int plan_generation, unsigned int sensor_sequence,
        int quality, unsigned int now_tick);
int Tc2UiOverlayUpdateAtDestination(
        tc2_ui_overlay *overlay, int train, int destination_index,
        unsigned int plan_generation, unsigned int position_revision,
        int quality, unsigned int now_tick);
int Tc2UiOverlayUpdatePredictedCell(
        tc2_ui_overlay *overlay, int train, int row, int column,
        unsigned int plan_generation, unsigned int position_revision,
        unsigned int now_tick);
int Tc2UiOverlaySetTrainSpeed(
        tc2_ui_overlay *overlay, int train, int speed,
        unsigned int plan_generation, unsigned int now_tick);
int Tc2UiOverlayRemoveTrain(tc2_ui_overlay *overlay, int train);
int Tc2UiOverlayExpireSensorFlashes(
        tc2_ui_overlay *overlay, unsigned int now_tick);

const tc2_ui_train_overlay *Tc2UiOverlayFindTrain(
        const tc2_ui_overlay *overlay, int train);
const tc2_ui_sensor_overlay *Tc2UiOverlaySensor(
        const tc2_ui_overlay *overlay, int sensor_index);
int Tc2UiOverlayActiveTrainCount(const tc2_ui_overlay *overlay);

/*
 * Each refresh flips the train-marker phase.  A renderer can alternate normal
 * and reverse-video colors without inventing continuous physical motion.
 */
unsigned int Tc2UiOverlayAdvanceRefresh(tc2_ui_overlay *overlay);
int Tc2UiOverlayMarkerPhase(const tc2_ui_overlay *overlay);
int Tc2UiOverlaySensorVisible(
        const tc2_ui_sensor_overlay *sensor, unsigned int now_tick);

/* Stable 256-color terminal palette entry for a validated color slot. */
int Tc2UiOverlayXtermColor(int color_slot);
const char *Tc2UiOverlayColorName(int color_slot);

#endif
