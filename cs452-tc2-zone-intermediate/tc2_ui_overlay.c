#ifndef MODE_TC2

typedef int tc2_ui_overlay_disabled_translation_unit;

#else

#include "tc2_ui_overlay.h"

static const unsigned char xterm_colors[TC2_UI_COLOR_COUNT] = {
        196, 46, 21, 226, 201, 51, 208, 93,
        118, 207, 39, 214, 129, 45, 160, 82
};

static int valid_source(int source) {
        return source == TC2_UI_SOURCE_OFFLINE_SIMULATION ||
               source == TC2_UI_SOURCE_LIVE_CAN;
}

static int valid_quality(int quality) {
        return quality >= TC2_UI_QUALITY_UNKNOWN &&
               quality <= TC2_UI_QUALITY_SENSOR_CONFIRMED;
}

static int quality_allowed_for_source(int source, int quality) {
        if (!valid_quality(quality)) return 0;
        return source == TC2_UI_SOURCE_LIVE_CAN ||
               quality != TC2_UI_QUALITY_SENSOR_CONFIRMED;
}

static int sensor_quality_allowed_for_source(
        int source, int quality) {
        if (source == TC2_UI_SOURCE_LIVE_CAN) {
                return quality == TC2_UI_QUALITY_SENSOR_CONFIRMED;
        }
        if (source == TC2_UI_SOURCE_OFFLINE_SIMULATION) {
                return quality == TC2_UI_QUALITY_PREDICTED ||
                        quality == TC2_UI_QUALITY_ESTIMATED;
        }
        return 0;
}

static int sequence_is_newer(
        unsigned int candidate, unsigned int previous) {
        if (candidate == 0 || candidate == previous) return 0;
        if (previous == 0) return 1;
        return candidate - previous < (1u << 31);
}

static int tick_reached(
        unsigned int now_tick, unsigned int deadline_tick) {
        return now_tick == deadline_tick ||
               now_tick - deadline_tick < (1u << 31);
}

static tc2_ui_train_overlay *find_train_mutable(
        tc2_ui_overlay *overlay, int train) {
        if (!overlay || train < 1 || train > 255) return 0;
        for (int slot = 0; slot < TC2_UI_MAX_TRAINS; ++slot) {
                if (overlay->trains[slot].active &&
                    overlay->trains[slot].train == train) {
                        return &overlay->trains[slot];
                }
        }
        return 0;
}

static int color_in_use(
        const tc2_ui_overlay *overlay, int color_slot) {
        for (int slot = 0; slot < TC2_UI_MAX_TRAINS; ++slot) {
                if (overlay->trains[slot].active &&
                    overlay->trains[slot].color_slot == color_slot) {
                        return 1;
                }
        }
        return 0;
}

static int fixed_color_slot_for_train(int train) {
        switch (train) {
        case 14:
                return 0; /* RED */
        case 18:
                return 1; /* GREEN */
        case 15:
                return 2; /* BLUE */
        case 17:
                return 3; /* YELLOW */
        default:
                return -1;
        }
}

static tc2_ui_train_overlay *train_using_color(
        tc2_ui_overlay *overlay, int color_slot) {
        if (!overlay || color_slot < 0 ||
            color_slot >= TC2_UI_COLOR_COUNT) {
                return 0;
        }
        for (int slot = 0; slot < TC2_UI_MAX_TRAINS; ++slot) {
                if (overlay->trains[slot].active &&
                    overlay->trains[slot].color_slot == color_slot) {
                        return &overlay->trains[slot];
                }
        }
        return 0;
}

static void set_train_color_slot(
        tc2_ui_overlay *overlay,
        tc2_ui_train_overlay *train, int color_slot) {
        if (!overlay || !train) return;
        train->color_slot = color_slot;
        /*
         * A visible detector flash belongs to the same train overlay.  If a
         * generic train borrowed a reserved color before T14/T15/T17/T18 was
         * staged, relocate both objects atomically so one train never appears
         * in two colors during the flash lifetime.
         */
        for (int sensor_index = 0;
             sensor_index < TC2_UI_SENSOR_COUNT;
             ++sensor_index) {
                tc2_ui_sensor_overlay *sensor =
                        &overlay->sensors[sensor_index];
                if (sensor->active &&
                    sensor->train == train->train) {
                        sensor->color_slot = color_slot;
                }
        }
}

static tc2_ui_train_overlay *allocate_train(
        tc2_ui_overlay *overlay, int train) {
        int free_train_slot = -1;
        int free_color_slot = -1;
        int fixed_color;
        if (!overlay || train < 1 || train > 255) return 0;

        for (int slot = 0; slot < TC2_UI_MAX_TRAINS; ++slot) {
                if (!overlay->trains[slot].active &&
                    free_train_slot < 0) {
                        free_train_slot = slot;
                }
        }
        if (free_train_slot < 0) return 0;

        fixed_color = fixed_color_slot_for_train(train);
        if (fixed_color >= 0) {
                tc2_ui_train_overlay *borrower =
                        train_using_color(overlay, fixed_color);
                if (borrower) {
                        int relocation = -1;
                        /*
                         * Generic trains normally use slots four through
                         * fifteen. They may borrow an unused named-train
                         * color only after those twelve are full. If that
                         * named train is later staged, move the borrower to
                         * the remaining free color and retain capacity 16.
                         */
                        for (int color = 4;
                             color < TC2_UI_COLOR_COUNT; ++color) {
                                if (!color_in_use(overlay, color)) {
                                        relocation = color;
                                        break;
                                }
                        }
                        for (int color = 0;
                             relocation < 0 &&
                             color < TC2_UI_COLOR_COUNT; ++color) {
                                if (color != fixed_color &&
                                    !color_in_use(overlay, color)) {
                                        relocation = color;
                                }
                        }
                        if (relocation < 0) return 0;
                        set_train_color_slot(
                                overlay, borrower, relocation);
                }
                free_color_slot = fixed_color;
        } else {
                for (int color = 4;
                     color < TC2_UI_COLOR_COUNT; ++color) {
                        if (!color_in_use(overlay, color)) {
                                free_color_slot = color;
                                break;
                        }
                }
                for (int color = 0;
                     free_color_slot < 0 && color < 4; ++color) {
                        if (!color_in_use(overlay, color)) {
                                free_color_slot = color;
                        }
                }
        }
        if (free_color_slot < 0) return 0;

        overlay->trains[free_train_slot].active = 1;
        overlay->trains[free_train_slot].train = train;
        overlay->trains[free_train_slot].color_slot = free_color_slot;
        overlay->trains[free_train_slot].start_index = -1;
        overlay->trains[free_train_slot].destination_index = -1;
        overlay->trains[free_train_slot].speed = 0;
        overlay->trains[free_train_slot].sensor_index = -1;
        overlay->trains[free_train_slot].row = -1;
        overlay->trains[free_train_slot].column = -1;
        overlay->trains[free_train_slot].position_kind =
                TC2_UI_POSITION_NONE;
        overlay->trains[free_train_slot].position_quality =
                TC2_UI_QUALITY_UNKNOWN;
        overlay->trains[free_train_slot].plan_generation = 0;
        overlay->trains[free_train_slot].position_revision = 0;
        overlay->trains[free_train_slot].updated_at_tick = 0;
        return &overlay->trains[free_train_slot];
}

static void clear_sensor_overlay(
        tc2_ui_sensor_overlay *sensor, int sensor_index,
        int clear_sequence_history) {
        sensor->active = 0;
        sensor->sensor_index = sensor_index;
        sensor->train = -1;
        sensor->color_slot = -1;
        sensor->evidence_source = -1;
        sensor->evidence_quality = TC2_UI_QUALITY_UNKNOWN;
        sensor->plan_generation = 0;
        sensor->row = -1;
        sensor->column = -1;
        sensor->width = 0;
        if (clear_sequence_history) sensor->sequence = 0;
        sensor->started_at_tick = 0;
        sensor->expires_at_tick = 0;
}

static void clear_train_sensor_flashes(
        tc2_ui_overlay *overlay, int train) {
        for (int sensor_index = 0;
             sensor_index < TC2_UI_SENSOR_COUNT;
             ++sensor_index) {
                tc2_ui_sensor_overlay *sensor =
                        &overlay->sensors[sensor_index];
                if (sensor->active && sensor->train == train) {
                        clear_sensor_overlay(
                                sensor, sensor_index, 0);
                }
        }
}

void Tc2UiOverlayInit(tc2_ui_overlay *overlay, int source) {
        if (!overlay) return;
        overlay->source = valid_source(source) ?
                source : TC2_UI_SOURCE_OFFLINE_SIMULATION;
        overlay->refresh_generation = 0;
        for (int train = 0; train < 256; ++train) {
                overlay->plan_generation_high_water[train] = 0;
        }
        for (int slot = 0; slot < TC2_UI_MAX_TRAINS; ++slot) {
                overlay->trains[slot].active = 0;
                overlay->trains[slot].train = -1;
                overlay->trains[slot].color_slot = -1;
                overlay->trains[slot].start_index = -1;
                overlay->trains[slot].destination_index = -1;
                overlay->trains[slot].speed = 0;
                overlay->trains[slot].sensor_index = -1;
                overlay->trains[slot].row = -1;
                overlay->trains[slot].column = -1;
                overlay->trains[slot].position_kind =
                        TC2_UI_POSITION_NONE;
                overlay->trains[slot].position_quality =
                        TC2_UI_QUALITY_UNKNOWN;
                overlay->trains[slot].plan_generation = 0;
                overlay->trains[slot].position_revision = 0;
                overlay->trains[slot].updated_at_tick = 0;
        }
        for (int sensor = 0; sensor < TC2_UI_SENSOR_COUNT; ++sensor) {
                clear_sensor_overlay(
                        &overlay->sensors[sensor], sensor, 1);
        }
}

int Tc2UiOverlaySetSource(tc2_ui_overlay *overlay, int source) {
        if (!overlay || !valid_source(source)) return -1;
        if (overlay->source == source) return 0;
        if (overlay->source == TC2_UI_SOURCE_LIVE_CAN &&
            source == TC2_UI_SOURCE_OFFLINE_SIMULATION) {
                for (int slot = 0; slot < TC2_UI_MAX_TRAINS; ++slot) {
                        tc2_ui_train_overlay *train =
                                &overlay->trains[slot];
                        if (train->active &&
                            train->position_quality ==
                                    TC2_UI_QUALITY_SENSOR_CONFIRMED) {
                                train->position_quality =
                                        TC2_UI_QUALITY_PREDICTED;
                        }
                }
        }
        /*
         * Flash provenance belongs to the source mode that produced it.
         * Clear flashes in both transition directions rather than allowing
         * simulated evidence to survive into LIVE or vice versa.
         */
        for (int sensor = 0;
             sensor < TC2_UI_SENSOR_COUNT; ++sensor) {
                clear_sensor_overlay(
                        &overlay->sensors[sensor], sensor, 0);
        }
        overlay->source = source;
        return 0;
}

const char *Tc2UiOverlaySourceName(int source) {
        if (source == TC2_UI_SOURCE_LIVE_CAN) return "LIVE CAN";
        if (source == TC2_UI_SOURCE_OFFLINE_SIMULATION) {
                return "OFFLINE SIMULATION / NO PHYSICAL MOTION";
        }
        return "INVALID SOURCE";
}

const char *Tc2UiOverlayQualityName(int quality) {
        switch (quality) {
        case TC2_UI_QUALITY_PREDICTED:
                return "predicted";
        case TC2_UI_QUALITY_ESTIMATED:
                return "estimated";
        case TC2_UI_QUALITY_SENSOR_CONFIRMED:
                return "sensor-confirmed";
        case TC2_UI_QUALITY_UNKNOWN:
                return "unknown";
        default:
                return "invalid";
        }
}

int Tc2UiOverlayUpsertAtStart(
        tc2_ui_overlay *overlay, int train, int start_index,
        int destination_index, unsigned int plan_generation,
        int quality, unsigned int now_tick) {
        const tc2_track_d_start_layout *start;
        tc2_ui_train_overlay *entry;
        if (!overlay ||
            train < 1 || train > 255 ||
            !quality_allowed_for_source(overlay->source, quality) ||
            plan_generation == 0 ||
            destination_index < 0 ||
            destination_index >= TC2_TRACK_D_DESTINATION_COUNT) {
                return -1;
        }
        start = Tc2TrackDStartLayout((size_t)start_index);
        if (!start) return -1;
        entry = find_train_mutable(overlay, train);
        if (entry) {
                if (entry->plan_generation == plan_generation) {
                        return 0;
                }
                if (!sequence_is_newer(
                            plan_generation,
                            entry->plan_generation)) {
                        return 0;
                }
                clear_train_sensor_flashes(overlay, train);
        } else {
                unsigned int high_water =
                        overlay->plan_generation_high_water[train];
                if (high_water != 0 &&
                    !sequence_is_newer(
                            plan_generation, high_water)) {
                        return 0;
                }
                entry = allocate_train(overlay, train);
                if (!entry) return -1;
        }

        entry->start_index = start_index;
        entry->destination_index = destination_index;
        entry->speed = 0;
        entry->sensor_index = -1;
        entry->row = start->row;
        entry->column = start->column;
        entry->position_kind = TC2_UI_POSITION_START;
        entry->position_quality = quality;
        entry->plan_generation = plan_generation;
        entry->position_revision = 0;
        entry->updated_at_tick = now_tick;
        overlay->plan_generation_high_water[train] =
                plan_generation;
        return 1;
}

int Tc2UiOverlayUpsertAtPredictedCell(
        tc2_ui_overlay *overlay, int train, int row, int column,
        int destination_index, unsigned int plan_generation,
        int quality, unsigned int now_tick) {
        tc2_ui_train_overlay *entry;
        if (!overlay ||
            train < 1 || train > 255 ||
            !quality_allowed_for_source(overlay->source, quality) ||
            quality == TC2_UI_QUALITY_SENSOR_CONFIRMED ||
            plan_generation == 0 ||
            destination_index < 0 ||
            destination_index >= TC2_TRACK_D_DESTINATION_COUNT ||
            !Tc2TrackDLayoutCellIsOccupied(row, column)) {
                return -1;
        }
        entry = find_train_mutable(overlay, train);
        if (entry) {
                if (entry->plan_generation == plan_generation) {
                        return 0;
                }
                if (!sequence_is_newer(
                            plan_generation,
                            entry->plan_generation)) {
                        return 0;
                }
                clear_train_sensor_flashes(overlay, train);
        } else {
                unsigned int high_water =
                        overlay->plan_generation_high_water[train];
                if (high_water != 0 &&
                    !sequence_is_newer(
                            plan_generation, high_water)) {
                        return 0;
                }
                entry = allocate_train(overlay, train);
                if (!entry) return -1;
        }

        entry->start_index = -1;
        entry->destination_index = destination_index;
        entry->speed = 0;
        entry->sensor_index = -1;
        entry->row = row;
        entry->column = column;
        entry->position_kind = TC2_UI_POSITION_PREDICTED;
        entry->position_quality = quality;
        entry->plan_generation = plan_generation;
        /* A newly published CURRENT position is renderable immediately. */
        entry->position_revision = plan_generation;
        entry->updated_at_tick = now_tick;
        overlay->plan_generation_high_water[train] =
                plan_generation;
        return 1;
}

int Tc2UiOverlayUpdateAtSensor(
        tc2_ui_overlay *overlay, int train, int sensor_index,
        unsigned int plan_generation, unsigned int position_revision,
        unsigned int sensor_sequence,
        int quality, unsigned int now_tick) {
        tc2_track_d_directed_sensor_cell cell;
        tc2_ui_train_overlay *entry;
        tc2_ui_sensor_overlay *sensor;
        if (!overlay || plan_generation == 0 ||
            position_revision == 0 || sensor_sequence == 0 ||
            !sensor_quality_allowed_for_source(
                    overlay->source, quality) ||
            Tc2TrackDDirectedSensorCell(sensor_index, &cell) < 0) {
                return -1;
        }
        entry = find_train_mutable(overlay, train);
        if (!entry || entry->plan_generation != plan_generation) {
                return -1;
        }
        sensor = &overlay->sensors[sensor_index];
        if (entry->position_revision != 0 &&
            !sequence_is_newer(position_revision,
                               entry->position_revision)) {
                return 0;
        }
        if (!sequence_is_newer(sensor_sequence, sensor->sequence)) {
                return 0;
        }

        entry->sensor_index = sensor_index;
        entry->row = cell.row;
        entry->column = cell.column;
        entry->position_kind = TC2_UI_POSITION_SENSOR;
        entry->position_quality = quality;
        entry->position_revision = position_revision;
        entry->updated_at_tick = now_tick;

        sensor->active = 1;
        sensor->sensor_index = sensor_index;
        sensor->train = train;
        sensor->color_slot = entry->color_slot;
        sensor->evidence_source = overlay->source;
        sensor->evidence_quality = quality;
        sensor->plan_generation = plan_generation;
        sensor->row = cell.row;
        sensor->column = cell.column;
        sensor->width = cell.width;
        sensor->sequence = sensor_sequence;
        sensor->started_at_tick = now_tick;
        sensor->expires_at_tick =
                now_tick + TC2_UI_SENSOR_FLASH_TICKS;
        return 1;
}

int Tc2UiOverlayFlashSensor(
        tc2_ui_overlay *overlay, int train, int sensor_index,
        unsigned int plan_generation, unsigned int sensor_sequence,
        int quality, unsigned int now_tick) {
        tc2_track_d_directed_sensor_cell cell;
        tc2_ui_train_overlay *entry;
        tc2_ui_sensor_overlay *sensor;
        if (!overlay || plan_generation == 0 ||
            sensor_sequence == 0 ||
            !sensor_quality_allowed_for_source(
                    overlay->source, quality) ||
            Tc2TrackDDirectedSensorCell(sensor_index, &cell) < 0) {
                return -1;
        }
        entry = find_train_mutable(overlay, train);
        if (!entry || entry->plan_generation != plan_generation) {
                return -1;
        }
        sensor = &overlay->sensors[sensor_index];
        if (!sequence_is_newer(sensor_sequence, sensor->sequence)) {
                return 0;
        }

        sensor->active = 1;
        sensor->sensor_index = sensor_index;
        sensor->train = train;
        sensor->color_slot = entry->color_slot;
        sensor->evidence_source = overlay->source;
        sensor->evidence_quality = quality;
        sensor->plan_generation = plan_generation;
        sensor->row = cell.row;
        sensor->column = cell.column;
        sensor->width = cell.width;
        sensor->sequence = sensor_sequence;
        sensor->started_at_tick = now_tick;
        sensor->expires_at_tick =
                now_tick + TC2_UI_SENSOR_FLASH_TICKS;
        return 1;
}

int Tc2UiOverlayUpdateAtDestination(
        tc2_ui_overlay *overlay, int train, int destination_index,
        unsigned int plan_generation, unsigned int position_revision,
        int quality, unsigned int now_tick) {
        const tc2_track_d_destination_layout *destination;
        tc2_ui_train_overlay *entry;
        if (!overlay || plan_generation == 0 ||
            position_revision == 0 ||
            !quality_allowed_for_source(overlay->source, quality) ||
            quality == TC2_UI_QUALITY_SENSOR_CONFIRMED) {
                return -1;
        }
        destination = Tc2TrackDDestinationLayout(
                (size_t)destination_index);
        if (!destination) return -1;
        entry = find_train_mutable(overlay, train);
        if (!entry || entry->plan_generation != plan_generation ||
            entry->destination_index != destination_index) {
                return -1;
        }
        if (entry->position_revision != 0 &&
            !sequence_is_newer(position_revision,
                               entry->position_revision)) {
                return 0;
        }

        entry->destination_index = destination_index;
        entry->sensor_index = -1;
        entry->row = destination->row;
        entry->column = destination->column;
        entry->position_kind = TC2_UI_POSITION_DESTINATION;
        entry->position_quality = quality;
        entry->position_revision = position_revision;
        entry->updated_at_tick = now_tick;
        return 1;
}

int Tc2UiOverlayUpdatePredictedCell(
        tc2_ui_overlay *overlay, int train, int row, int column,
        unsigned int plan_generation, unsigned int position_revision,
        unsigned int now_tick) {
        tc2_ui_train_overlay *entry;
        if (!overlay || plan_generation == 0 ||
            position_revision == 0 ||
            row < 0 ||
            row >= TC2_TRACK_D_LAYOUT_ROWS ||
            column < 0 ||
            column >= TC2_TRACK_D_LAYOUT_MAP_COLUMNS) {
                return -1;
        }
        entry = find_train_mutable(overlay, train);
        if (!entry || entry->plan_generation != plan_generation) {
                return -1;
        }
        if (entry->position_revision != 0 &&
            !sequence_is_newer(position_revision,
                               entry->position_revision)) {
                return 0;
        }
        entry->sensor_index = -1;
        entry->row = row;
        entry->column = column;
        entry->position_kind = TC2_UI_POSITION_PREDICTED;
        entry->position_quality = TC2_UI_QUALITY_PREDICTED;
        entry->position_revision = position_revision;
        entry->updated_at_tick = now_tick;
        return 1;
}

int Tc2UiOverlaySetTrainSpeed(
        tc2_ui_overlay *overlay, int train, int speed,
        unsigned int plan_generation, unsigned int now_tick) {
        tc2_ui_train_overlay *entry;
        if (!overlay || train < 1 || train > 255 ||
            speed < 0 || speed > 120 || plan_generation == 0) {
                return -1;
        }
        entry = find_train_mutable(overlay, train);
        if (!entry || entry->plan_generation != plan_generation) {
                return -1;
        }
        if (entry->speed == speed) return 0;
        entry->speed = speed;
        entry->updated_at_tick = now_tick;
        return 1;
}

int Tc2UiOverlayRemoveTrain(tc2_ui_overlay *overlay, int train) {
        tc2_ui_train_overlay *entry =
                find_train_mutable(overlay, train);
        if (!entry) return -1;
        clear_train_sensor_flashes(overlay, train);
        entry->active = 0;
        entry->train = -1;
        entry->color_slot = -1;
        entry->start_index = -1;
        entry->destination_index = -1;
        entry->speed = 0;
        entry->sensor_index = -1;
        entry->row = -1;
        entry->column = -1;
        entry->position_kind = TC2_UI_POSITION_NONE;
        entry->position_quality = TC2_UI_QUALITY_UNKNOWN;
        entry->plan_generation = 0;
        entry->position_revision = 0;
        entry->updated_at_tick = 0;
        return 0;
}

int Tc2UiOverlayExpireSensorFlashes(
        tc2_ui_overlay *overlay, unsigned int now_tick) {
        int expired = 0;
        if (!overlay) return -1;
        for (int index = 0; index < TC2_UI_SENSOR_COUNT; ++index) {
                tc2_ui_sensor_overlay *sensor =
                        &overlay->sensors[index];
                if (sensor->active &&
                    tick_reached(now_tick,
                                 sensor->expires_at_tick)) {
                        clear_sensor_overlay(sensor, index, 0);
                        ++expired;
                }
        }
        return expired;
}

const tc2_ui_train_overlay *Tc2UiOverlayFindTrain(
        const tc2_ui_overlay *overlay, int train) {
        if (!overlay || train < 1 || train > 255) return 0;
        for (int slot = 0; slot < TC2_UI_MAX_TRAINS; ++slot) {
                if (overlay->trains[slot].active &&
                    overlay->trains[slot].train == train) {
                        return &overlay->trains[slot];
                }
        }
        return 0;
}

const tc2_ui_sensor_overlay *Tc2UiOverlaySensor(
        const tc2_ui_overlay *overlay, int sensor_index) {
        if (!overlay || sensor_index < 0 ||
            sensor_index >= TC2_UI_SENSOR_COUNT) {
                return 0;
        }
        return &overlay->sensors[sensor_index];
}

int Tc2UiOverlayActiveTrainCount(const tc2_ui_overlay *overlay) {
        int count = 0;
        if (!overlay) return -1;
        for (int slot = 0; slot < TC2_UI_MAX_TRAINS; ++slot) {
                if (overlay->trains[slot].active) ++count;
        }
        return count;
}

unsigned int Tc2UiOverlayAdvanceRefresh(tc2_ui_overlay *overlay) {
        if (!overlay) return 0;
        return ++overlay->refresh_generation;
}

int Tc2UiOverlayMarkerPhase(const tc2_ui_overlay *overlay) {
        if (!overlay) return 0;
        return (int)(overlay->refresh_generation & 1U);
}

int Tc2UiOverlaySensorVisible(
        const tc2_ui_sensor_overlay *sensor, unsigned int now_tick) {
        unsigned int elapsed;
        if (!sensor || !sensor->active ||
            !tick_reached(now_tick, sensor->started_at_tick) ||
            tick_reached(now_tick, sensor->expires_at_tick)) {
                return 0;
        }
        elapsed = now_tick - sensor->started_at_tick;
        return ((elapsed / TC2_UI_SENSOR_FLASH_PHASE_TICKS) & 1) == 0;
}

int Tc2UiOverlayXtermColor(int color_slot) {
        if (color_slot < 0 ||
            color_slot >= TC2_UI_COLOR_COUNT) {
                return -1;
        }
        return xterm_colors[color_slot];
}

const char *Tc2UiOverlayColorName(int color_slot) {
        static const char *const names[TC2_UI_COLOR_COUNT] = {
                "RED", "GREEN", "BLUE", "YELLOW",
                "MAGENTA", "CYAN", "ORANGE", "PURPLE",
                "LIME", "PINK", "SKY", "GOLD",
                "VIOLET", "AQUA", "CRIMSON", "MINT"
        };
        return color_slot >= 0 && color_slot < TC2_UI_COLOR_COUNT ?
                names[color_slot] : "UNKNOWN";
}

#endif
