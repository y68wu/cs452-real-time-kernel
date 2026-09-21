#ifndef MODE_TC2

typedef int tc2_ui_renderer_disabled_translation_unit;

#else

#include "tc2_dispatch.h"
#include "tc2_ui_renderer.h"

enum {
        STYLE_COLOR_MASK = 0x1f,
        STYLE_REVERSE = 0x20,
        STYLE_BOLD = 0x40,
        STYLE_UNDERLINE = 0x80,
        STYLE_WARNING = 1 | STYLE_REVERSE | STYLE_BOLD,
        STYLE_LIVE = 2 | STYLE_BOLD
};

typedef struct {
        tc2_ui_render_write_fn write;
        void *context;
        char bytes[TC2_UI_RENDER_CHUNK_CAPACITY];
        size_t length;
        unsigned int total;
        int failed;
} render_writer;

typedef struct {
        int valid;
        int row;
        int column;
        int color_slot;
        int invalid_evidence;
} marker_cell;

static int valid_color_slot(int color_slot) {
        return color_slot >= 0 && color_slot < TC2_UI_COLOR_COUNT;
}

static unsigned char color_style(
        int color_slot, unsigned char flags) {
        if (!valid_color_slot(color_slot)) return STYLE_WARNING;
        return (unsigned char)(color_slot + 1) | flags;
}

static int flush_writer(render_writer *writer) {
        if (!writer || writer->failed) return -1;
        if (writer->length == 0) return 0;
        if (!writer->write ||
            writer->write(writer->context, writer->bytes,
                          writer->length) != 0) {
                writer->failed = 1;
                return -1;
        }
        writer->total += (unsigned int)writer->length;
        writer->length = 0;
        return 0;
}

static int append_character(render_writer *writer, char character) {
        if (!writer || writer->failed) return -1;
        if (writer->length == sizeof(writer->bytes) &&
            flush_writer(writer) < 0) {
                return -1;
        }
        writer->bytes[writer->length++] = character;
        return 0;
}

static int append_text(render_writer *writer, const char *text) {
        if (!text) return -1;
        while (*text) {
                if (append_character(writer, *text++) < 0) return -1;
        }
        return 0;
}

static int append_unsigned(
        render_writer *writer, unsigned int value) {
        char reverse[12];
        size_t count = 0;
        do {
                reverse[count++] = (char)('0' + value % 10U);
                value /= 10U;
        } while (value > 0U && count < sizeof(reverse));
        while (count > 0) {
                if (append_character(writer, reverse[--count]) < 0) {
                        return -1;
                }
        }
        return 0;
}

static int emit_cursor(
        render_writer *writer, int row, int column) {
        if (row < 0 || row >= TC2_UI_RENDER_ROWS ||
            column < 0 || column >= TC2_UI_RENDER_COLUMNS) {
                return -1;
        }
        if (append_text(writer, "\033[") < 0 ||
            append_unsigned(writer, (unsigned int)row + 1U) < 0 ||
            append_character(writer, ';') < 0 ||
            append_unsigned(writer, (unsigned int)column + 1U) < 0 ||
            append_character(writer, 'H') < 0) {
                return -1;
        }
        return 0;
}

static int emit_style(
        render_writer *writer, unsigned char style) {
        int color_identifier = style & STYLE_COLOR_MASK;
        if (append_text(writer, "\033[0") < 0) return -1;
        if (style & STYLE_BOLD) {
                if (append_text(writer, ";1") < 0) return -1;
        }
        if (style & STYLE_UNDERLINE) {
                if (append_text(writer, ";4") < 0) return -1;
        }
        if (style & STYLE_REVERSE) {
                if (append_text(writer, ";7") < 0) return -1;
        }
        if (color_identifier > 0 &&
            color_identifier <= TC2_UI_COLOR_COUNT) {
                int xterm = Tc2UiOverlayXtermColor(
                        color_identifier - 1);
                if (xterm < 0 ||
                    append_text(writer, ";38;5;") < 0 ||
                    append_unsigned(writer,
                                    (unsigned int)xterm) < 0) {
                        return -1;
                }
        }
        return append_character(writer, 'm');
}

static void clear_row(
        char glyph[TC2_UI_RENDER_COLUMNS],
        unsigned char style[TC2_UI_RENDER_COLUMNS]) {
        for (int column = 0;
             column < TC2_UI_RENDER_COLUMNS; ++column) {
                glyph[column] = ' ';
                style[column] = 0;
        }
}

static void put_character(
        char glyph[TC2_UI_RENDER_COLUMNS],
        unsigned char style[TC2_UI_RENDER_COLUMNS],
        int column, char character, unsigned char cell_style) {
        if (column < 0 || column >= TC2_UI_RENDER_COLUMNS) return;
        glyph[column] = character;
        style[column] = cell_style;
}

static int put_text(
        char glyph[TC2_UI_RENDER_COLUMNS],
        unsigned char style[TC2_UI_RENDER_COLUMNS],
        int column, const char *text, unsigned char cell_style) {
        int written = 0;
        if (!text) return 0;
        while (text[written] &&
               column + written < TC2_UI_RENDER_COLUMNS) {
                if (column + written >= 0) {
                        glyph[column + written] = text[written];
                        style[column + written] = cell_style;
                }
                ++written;
        }
        return written;
}

static int put_unsigned_width(
        char glyph[TC2_UI_RENDER_COLUMNS],
        unsigned char style[TC2_UI_RENDER_COLUMNS],
        int column, unsigned int value, int minimum_width,
        unsigned char cell_style) {
        char reverse[12];
        char forward[12];
        int count = 0;
        int total;
        do {
                reverse[count++] = (char)('0' + value % 10U);
                value /= 10U;
        } while (value > 0U && count < (int)sizeof(reverse));
        if (minimum_width < 0) minimum_width = 0;
        if (minimum_width > (int)sizeof(forward)) {
                minimum_width = (int)sizeof(forward);
        }
        total = count > minimum_width ? count : minimum_width;
        if (total > (int)sizeof(forward)) {
                total = (int)sizeof(forward);
        }
        for (int index = 0; index < total; ++index) {
                int source = total - index - 1;
                forward[index] = source < count ?
                        reverse[source] : '0';
        }
        for (int index = 0; index < total; ++index) {
                put_character(glyph, style, column + index,
                              forward[index], cell_style);
        }
        return total;
}

static int train_is_structurally_valid(
        const tc2_ui_train_overlay *train) {
        if (!train || !train->active ||
            train->train < 1 || train->train > 255 ||
            !valid_color_slot(train->color_slot) ||
            (train->start_index != TC2_DISPATCH_START_CURRENT &&
             (train->start_index < 0 ||
              train->start_index >= TC2_TRACK_D_START_COUNT)) ||
            train->destination_index < 0 ||
            train->destination_index >=
                    TC2_TRACK_D_DESTINATION_COUNT ||
            train->speed < 0 || train->speed > 120 ||
            train->position_kind < TC2_UI_POSITION_NONE ||
            train->position_kind > TC2_UI_POSITION_PREDICTED ||
            train->position_quality < TC2_UI_QUALITY_UNKNOWN ||
            train->position_quality >
                    TC2_UI_QUALITY_SENSOR_CONFIRMED) {
                return 0;
        }
        return 1;
}

static int source_is_valid(const tc2_ui_overlay *overlay) {
        return overlay &&
                (overlay->source ==
                         TC2_UI_SOURCE_OFFLINE_SIMULATION ||
                 overlay->source == TC2_UI_SOURCE_LIVE_CAN);
}

static int train_position_is_valid(
        const tc2_ui_train_overlay *train) {
        if (!train_is_structurally_valid(train) ||
            train->plan_generation == 0) {
                return 0;
        }
        switch (train->position_kind) {
        case TC2_UI_POSITION_START: {
                const tc2_track_d_start_layout *start =
                        Tc2TrackDStartLayout(
                                (size_t)train->start_index);
                return start && train->sensor_index == -1 &&
                        train->row == start->row &&
                        train->column == start->column &&
                        train->position_revision == 0;
        }
        case TC2_UI_POSITION_SENSOR: {
                tc2_track_d_directed_sensor_cell sensor;
                return train->position_revision != 0 &&
                        Tc2TrackDDirectedSensorCell(
                                train->sensor_index,
                                &sensor) == 0 &&
                        train->row == sensor.row &&
                        train->column == sensor.column;
        }
        case TC2_UI_POSITION_DESTINATION: {
                const tc2_track_d_destination_layout *destination =
                        Tc2TrackDDestinationLayout(
                                (size_t)train->destination_index);
                return destination &&
                        train->sensor_index == -1 &&
                        train->row == destination->row &&
                        train->column ==
                                destination->column &&
                        train->position_revision != 0 &&
                        train->position_quality !=
                                TC2_UI_QUALITY_SENSOR_CONFIRMED;
        }
        case TC2_UI_POSITION_PREDICTED:
                return train->sensor_index == -1 &&
                        train->row >= 0 &&
                        train->row < TC2_TRACK_D_LAYOUT_ROWS &&
                        train->column >= 0 &&
                        train->column <
                                TC2_TRACK_D_LAYOUT_MAP_COLUMNS &&
                        train->position_revision != 0 &&
                        train->position_quality ==
                                TC2_UI_QUALITY_PREDICTED;
        default:
                return 0;
        }
}

static int train_evidence_is_valid(
        const tc2_ui_overlay *overlay,
        const tc2_ui_train_overlay *train) {
        if (!source_is_valid(overlay) ||
            !train_position_is_valid(train)) {
                return 0;
        }
        if (train->position_quality ==
                    TC2_UI_QUALITY_SENSOR_CONFIRMED) {
                return overlay->source == TC2_UI_SOURCE_LIVE_CAN &&
                        train->position_kind ==
                                TC2_UI_POSITION_SENSOR;
        }
        return 1;
}

static int train_is_unique(
        const tc2_ui_overlay *overlay,
        const tc2_ui_train_overlay *train) {
        if (!overlay || !train || !train->active) return 0;
        for (int slot = 0; slot < TC2_UI_MAX_TRAINS; ++slot) {
                const tc2_ui_train_overlay *other =
                        &overlay->trains[slot];
                if (!other->active || other == train) continue;
                if (other->train == train->train ||
                    other->color_slot == train->color_slot) {
                        return 0;
                }
        }
        return 1;
}

static int train_is_renderable(
        const tc2_ui_overlay *overlay,
        const tc2_ui_train_overlay *train) {
        return train_evidence_is_valid(overlay, train) &&
                train_is_unique(overlay, train);
}

static int sensor_flash_is_valid(
        const tc2_ui_overlay *overlay, int sensor_index) {
        const tc2_ui_sensor_overlay *sensor;
        tc2_track_d_directed_sensor_cell catalog;
        if (!overlay || sensor_index < 0 ||
            sensor_index >= TC2_UI_SENSOR_COUNT) {
                return 0;
        }
        sensor = &overlay->sensors[sensor_index];
        if (!sensor->active ||
            sensor->sensor_index != sensor_index ||
            sensor->train < 1 || sensor->train > 255 ||
            !valid_color_slot(sensor->color_slot) ||
            sensor->plan_generation == 0 ||
            sensor->evidence_source != overlay->source ||
            (overlay->source == TC2_UI_SOURCE_LIVE_CAN &&
             sensor->evidence_quality !=
                     TC2_UI_QUALITY_SENSOR_CONFIRMED) ||
            (overlay->source ==
                     TC2_UI_SOURCE_OFFLINE_SIMULATION &&
             sensor->evidence_quality !=
                     TC2_UI_QUALITY_PREDICTED &&
             sensor->evidence_quality !=
                     TC2_UI_QUALITY_ESTIMATED) ||
            sensor->sequence == 0 ||
            sensor->expires_at_tick !=
                    sensor->started_at_tick +
                            TC2_UI_SENSOR_FLASH_TICKS ||
            Tc2TrackDDirectedSensorCell(
                    sensor_index, &catalog) < 0 ||
            sensor->row != catalog.row ||
            sensor->column != catalog.column ||
            sensor->width != catalog.width ||
            sensor->column < 0 || sensor->width <= 0 ||
            sensor->column + sensor->width >
                    TC2_TRACK_D_LAYOUT_MAP_COLUMNS) {
                return 0;
        }
        for (int slot = 0; slot < TC2_UI_MAX_TRAINS; ++slot) {
                const tc2_ui_train_overlay *train =
                        &overlay->trains[slot];
                if (train_is_renderable(overlay, train) &&
                    train->train == sensor->train &&
                    train->color_slot == sensor->color_slot &&
                    train->plan_generation ==
                            sensor->plan_generation) {
                        return 1;
                }
        }
        return 0;
}

static marker_cell marker_for_train(
        const tc2_ui_overlay *overlay,
        const tc2_ui_train_overlay *train) {
        marker_cell marker = {0, -1, -1, -1, 0};
        if (!train_is_renderable(overlay, train)) return marker;
        marker.color_slot = train->color_slot;

        switch (train->position_kind) {
        case TC2_UI_POSITION_START: {
                const tc2_track_d_start_layout *start =
                        Tc2TrackDStartLayout(
                                (size_t)train->start_index);
                if (!start) return marker;
                marker.row = start->row;
                marker.column = start->column;
                break;
        }
        case TC2_UI_POSITION_SENSOR: {
                tc2_track_d_directed_sensor_cell cell;
                if (Tc2TrackDDirectedSensorCell(
                            train->sensor_index, &cell) < 0) {
                        return marker;
                }
                marker.row = cell.row;
                marker.column = cell.column + cell.width;
                break;
        }
        case TC2_UI_POSITION_DESTINATION: {
                const tc2_track_d_destination_layout *destination =
                        Tc2TrackDDestinationLayout(
                                (size_t)train->destination_index);
                if (!destination) return marker;
                marker.row = destination->row;
                marker.column = destination->column;
                break;
        }
        case TC2_UI_POSITION_PREDICTED: {
                marker.row = train->row;
                marker.column = train->column;
                break;
        }
        default:
                return marker;
        }

        if (marker.row < 0 ||
            marker.row >= TC2_TRACK_D_LAYOUT_ROWS ||
            marker.column < 0 ||
            marker.column >= TC2_TRACK_D_LAYOUT_MAP_COLUMNS) {
                return marker;
        }
        marker.valid = 1;
        return marker;
}

static int marker_multiplicity(
        const tc2_ui_overlay *overlay, int row, int column,
        int *one_color) {
        int count = 0;
        if (one_color) *one_color = -1;
        if (!overlay) return 0;
        for (int slot = 0; slot < TC2_UI_MAX_TRAINS; ++slot) {
                marker_cell marker =
                        marker_for_train(overlay,
                                         &overlay->trains[slot]);
                if (!marker.valid ||
                    marker.row != row ||
                    marker.column != column) {
                        continue;
                }
                ++count;
                if (one_color) *one_color = marker.color_slot;
        }
        return count;
}

static void paint_sensor_flashes(
        char glyph[TC2_UI_RENDER_COLUMNS],
        unsigned char style[TC2_UI_RENDER_COLUMNS],
        const tc2_ui_overlay *overlay, int row,
        unsigned int now_tick) {
        (void)glyph;
        if (!overlay) return;
        for (int sensor_index = 0;
             sensor_index < TC2_UI_SENSOR_COUNT; ++sensor_index) {
                const tc2_ui_sensor_overlay *sensor =
                        &overlay->sensors[sensor_index];
                if (!sensor_flash_is_valid(
                            overlay, sensor_index) ||
                    sensor->row != row ||
                    !Tc2UiOverlaySensorVisible(
                            sensor, now_tick)) {
                        continue;
                }
                unsigned char flash_style = color_style(
                        sensor->color_slot,
                        STYLE_BOLD | STYLE_REVERSE);
                for (int column = sensor->column;
                     column < sensor->column + sensor->width &&
                     column < TC2_UI_RENDER_COLUMNS; ++column) {
                        style[column] = flash_style;
                }
        }
}

static void paint_train_header(
        char glyph[TC2_UI_RENDER_COLUMNS],
        unsigned char style[TC2_UI_RENDER_COLUMNS],
        const tc2_ui_overlay *overlay) {
        int column = put_text(
                glyph, style, 0, "TRACK D", STYLE_BOLD);
        if (!overlay) return;
        for (int slot = 0; slot < TC2_UI_MAX_TRAINS; ++slot) {
                const tc2_ui_train_overlay *train =
                        &overlay->trains[slot];
                unsigned char train_style;
                if (!train->active) continue;
                if (column + 18 >= TC2_UI_RENDER_COLUMNS) break;
                /*
                 * Identity color is immutable.  Evidence problems may hide
                 * or mark the position glyph, but must never repaint T15
                 * BLUE as the red warning identity during CURRENT reroutes.
                 */
                train_style = color_style(
                        train->color_slot, STYLE_BOLD);
                column += put_text(
                        glyph, style, column, " | ", STYLE_BOLD);
                column += put_text(
                        glyph, style, column, "T", train_style);
                column += put_unsigned_width(
                        glyph, style, column,
                        (unsigned int)train->train, 1,
                        train_style);
                put_character(glyph, style, column++, ' ', train_style);
                column += put_text(
                        glyph, style, column,
                        Tc2UiOverlayColorName(train->color_slot),
                        train_style);
                column += put_text(
                        glyph, style, column, " speed=",
                        train_style);
                column += put_unsigned_width(
                        glyph, style, column,
                        (unsigned int)train->speed, 1,
                        train_style);
        }
}

static void paint_train_markers(
        char glyph[TC2_UI_RENDER_COLUMNS],
        unsigned char style[TC2_UI_RENDER_COLUMNS],
        const tc2_ui_overlay *overlay, int row,
        unsigned int *conflicts) {
        if (!overlay) return;
        for (int slot = 0; slot < TC2_UI_MAX_TRAINS; ++slot) {
                marker_cell marker =
                        marker_for_train(overlay,
                                         &overlay->trains[slot]);
                int count;
                int color_slot;
                unsigned char marker_style;
                if (!marker.valid || marker.row != row) continue;

                /*
                 * Only the first train that owns this cell paints it.  A
                 * duplicate becomes one red conflict marker rather than
                 * allowing array order to hide a collision.
                 */
                count = 1;
                for (int previous = 0; previous < slot; ++previous) {
                        marker_cell other = marker_for_train(
                                overlay, &overlay->trains[previous]);
                        if (other.valid &&
                            other.row == marker.row &&
                            other.column == marker.column) {
                                count = 0;
                                break;
                        }
                        count = 1;
                }
                if (slot > 0 && count == 0) continue;

                count = marker_multiplicity(
                        overlay, marker.row, marker.column,
                        &color_slot);
                if (count > 1) {
                        put_character(glyph, style, marker.column,
                                      '!', STYLE_WARNING);
                        if (conflicts) ++*conflicts;
                } else {
                        marker_style = marker.invalid_evidence ?
                                STYLE_WARNING :
                                color_style(
                                        color_slot,
                                        STYLE_BOLD | STYLE_REVERSE);
                        put_character(glyph, style, marker.column,
                                      '@', marker_style);
                }
        }
}

static unsigned int count_invalid_evidence(
        const tc2_ui_overlay *overlay) {
        unsigned int invalid = 0;
        if (!overlay) return 1;
        if (!source_is_valid(overlay)) ++invalid;
        for (int slot = 0; slot < TC2_UI_MAX_TRAINS; ++slot) {
                const tc2_ui_train_overlay *train =
                        &overlay->trains[slot];
                if (!train->active) continue;
                if (!train_position_is_valid(train) ||
                    (source_is_valid(overlay) &&
                     !train_evidence_is_valid(
                             overlay, train))) {
                        ++invalid;
                }
        }
        for (int first = 0;
             first < TC2_UI_MAX_TRAINS; ++first) {
                if (!overlay->trains[first].active) continue;
                for (int second = first + 1;
                     second < TC2_UI_MAX_TRAINS; ++second) {
                        if (!overlay->trains[second].active) continue;
                        if (overlay->trains[first].train ==
                                    overlay->trains[second].train ||
                            overlay->trains[first].color_slot ==
                                    overlay->trains[second].color_slot) {
                                ++invalid;
                        }
                }
        }
        for (int sensor = 0;
             sensor < TC2_UI_SENSOR_COUNT; ++sensor) {
                if (overlay->sensors[sensor].active &&
                    !sensor_flash_is_valid(overlay, sensor)) {
                        ++invalid;
                }
        }
        return invalid;
}

static const char *quality_abbreviation(
        const tc2_ui_overlay *overlay,
        const tc2_ui_train_overlay *train) {
        if (!train_is_renderable(overlay, train)) {
                return "INVALID";
        }
        switch (train->position_quality) {
        case TC2_UI_QUALITY_PREDICTED:
                return "pred";
        case TC2_UI_QUALITY_ESTIMATED:
                return "est";
        case TC2_UI_QUALITY_SENSOR_CONFIRMED:
                return "live";
        default:
                return "unk";
        }
}

static int put_position_abbreviation(
        char glyph[TC2_UI_RENDER_COLUMNS],
        unsigned char style[TC2_UI_RENDER_COLUMNS],
        int column, const tc2_ui_train_overlay *train,
        unsigned char cell_style) {
        int written = 0;
        if (!train) return 0;
        switch (train->position_kind) {
        case TC2_UI_POSITION_START:
                written += put_text(
                        glyph, style, column + written,
                        "@start", cell_style);
                if (train->start_index >= 0 &&
                    train->start_index <
                            TC2_TRACK_D_START_COUNT) {
                        put_character(
                                glyph, style, column + written++,
                                (char)('A' + train->start_index),
                                cell_style);
                }
                break;
        case TC2_UI_POSITION_SENSOR: {
                tc2_track_d_directed_sensor_cell sensor;
                written += put_text(
                        glyph, style, column + written,
                        "@", cell_style);
                if (Tc2TrackDDirectedSensorCell(
                            train->sensor_index, &sensor) == 0) {
                        written += put_text(
                                glyph, style, column + written,
                                sensor.name, cell_style);
                } else {
                        written += put_text(
                                glyph, style, column + written,
                                "bad-sensor", cell_style);
                }
                break;
        }
        case TC2_UI_POSITION_DESTINATION:
                written += put_text(
                        glyph, style, column + written,
                        "@D", cell_style);
                written += put_unsigned_width(
                        glyph, style, column + written,
                        (unsigned int)train->destination_index + 1U,
                        1, cell_style);
                written += put_text(
                        glyph, style, column + written,
                        "OP", cell_style);
                break;
        case TC2_UI_POSITION_PREDICTED:
                written += put_text(
                        glyph, style, column + written,
                        "@r", cell_style);
                written += put_unsigned_width(
                        glyph, style, column + written,
                        (unsigned int)(train->row < 0 ? 0 : train->row),
                        2, cell_style);
                written += put_text(
                        glyph, style, column + written,
                        "c", cell_style);
                written += put_unsigned_width(
                        glyph, style, column + written,
                        (unsigned int)(train->column < 0 ?
                                       0 : train->column),
                        3, cell_style);
                break;
        default:
                written += put_text(
                        glyph, style, column + written,
                        "@none", cell_style);
                break;
        }
        return written;
}

static void paint_legend_entry(
        char glyph[TC2_UI_RENDER_COLUMNS],
        unsigned char style[TC2_UI_RENDER_COLUMNS],
        int entry_column, const tc2_ui_overlay *overlay,
        const tc2_ui_train_overlay *train) {
        unsigned char entry_style = 0;
        int column = entry_column;
        if (!train || !train->active) {
                put_text(glyph, style, column, "-- inactive --", 0);
                return;
        }
        if (!train_is_renderable(overlay, train)) {
                entry_style = STYLE_WARNING;
        } else {
                entry_style = color_style(
                        train->color_slot, STYLE_BOLD);
        }

        put_character(glyph, style, column++, 'T', entry_style);
        column += put_unsigned_width(
                glyph, style, column,
                (unsigned int)(train->train < 0 ?
                               0 : train->train),
                3, entry_style);
        put_character(glyph, style, column++, ' ', entry_style);
        put_character(
                glyph, style, column++,
                train->start_index >= 0 &&
                train->start_index < TC2_TRACK_D_START_COUNT ?
                        (char)('A' + train->start_index) : '?',
                entry_style);
        put_character(glyph, style, column++, '>', entry_style);
        put_character(glyph, style, column++, 'D', entry_style);
        column += put_unsigned_width(
                glyph, style, column,
                train->destination_index >= 0 &&
                train->destination_index <
                        TC2_TRACK_D_DESTINATION_COUNT ?
                        (unsigned int)train->destination_index + 1U : 0U,
                1, entry_style);
        put_character(glyph, style, column++, ' ', entry_style);
        column += put_text(
                glyph, style, column,
                quality_abbreviation(overlay, train),
                entry_style);
        put_character(glyph, style, column++, ' ', entry_style);
        put_position_abbreviation(
                glyph, style, column, train, entry_style);
}

static void paint_supplemental_row(
        char glyph[TC2_UI_RENDER_COLUMNS],
        unsigned char style[TC2_UI_RENDER_COLUMNS],
        const tc2_ui_overlay *overlay, int row,
        unsigned int now_tick, unsigned int conflicts,
        unsigned int invalid_evidence) {
        int valid_source = source_is_valid(overlay);
        int offline = valid_source &&
                overlay->source ==
                        TC2_UI_SOURCE_OFFLINE_SIMULATION;
        int live = valid_source &&
                overlay->source == TC2_UI_SOURCE_LIVE_CAN;
        int active_count = 0;
        if (row == 33) {
                if (!valid_source) {
                        put_text(
                                glyph, style, 0,
                                "INVALID UI SOURCE - FRAME EVIDENCE REJECTED; CHECK MODE INITIALIZATION",
                                STYLE_WARNING);
                } else if (offline) {
                        put_text(
                                glyph, style, 0,
                                "OFFLINE PREDICTION / SIMULATION - NO CAN, NO PHYSICAL MOTION, NO LIVE SENSOR EVIDENCE",
                                STYLE_WARNING);
                } else {
                        put_text(
                                glyph, style, 0,
                                "LIVE CAN TELEMETRY - PHYSICAL MOTION POSSIBLE; SENSOR CONFIRMATION REQUIRES VALID CAN EVIDENCE",
                                STYLE_LIVE);
                }
                return;
        }
        if (row == 34) {
                put_text(
                        glyph, style, 0,
                        "MAP KEY: @=one train  !=marker conflict  colored label=direction-specific sensor flash  SW1..SW18,SW153..SW156",
                        STYLE_BOLD);
                return;
        }
        if (row >= 35 && row <= 38) {
                int first_slot = (row - 35) * 4;
                for (int entry = 0; entry < 4; ++entry) {
                        paint_legend_entry(
                                glyph, style, entry * 33,
                                overlay,
                                overlay ?
                                &overlay->trains[
                                        first_slot + entry] : 0);
                }
                return;
        }
        if (overlay) {
                for (int slot = 0;
                     slot < TC2_UI_MAX_TRAINS; ++slot) {
                        if (overlay->trains[slot].active) ++active_count;
                }
        }
        if (row == 39) {
                int column = 0;
                column += put_text(
                        glyph, style, column,
                        "STATUS: source=", STYLE_BOLD);
                column += put_text(
                        glyph, style, column,
                        offline ? "OFFLINE" :
                        live ? "LIVE" : "INVALID",
                        offline || !valid_source ?
                                STYLE_WARNING : STYLE_LIVE);
                column += put_text(
                        glyph, style, column,
                        " active_trains=", STYLE_BOLD);
                column += put_unsigned_width(
                        glyph, style, column,
                        (unsigned int)active_count, 1, STYLE_BOLD);
                column += put_text(
                        glyph, style, column,
                        " refresh_generation=", STYLE_BOLD);
                column += put_unsigned_width(
                        glyph, style, column,
                        overlay ? overlay->refresh_generation : 0U,
                        1, STYLE_BOLD);
                column += put_text(
                        glyph, style, column,
                        " tick=", STYLE_BOLD);
                put_unsigned_width(
                        glyph, style, column,
                        now_tick, 1, STYLE_BOLD);
                return;
        }
        if (row == 40) {
                put_text(
                        glyph, style, 0,
                        !valid_source ?
                        "EVIDENCE: rejected because the UI source is invalid; no marker or sensor flash is trusted." :
                        offline ?
                        "EVIDENCE: predicted/estimated only; no live sensor evidence; colored sensor flashes are simulated events." :
                        "EVIDENCE: live confirmations are shown only after validated CAN sensor events; OP endpoints remain virtual.",
                        offline || !valid_source ?
                                STYLE_WARNING : STYLE_LIVE);
                return;
        }
        if (row == 41) {
                int column = put_text(
                        glyph, style, 0, "STARTS: ", STYLE_BOLD);
                for (int index = 0;
                     index < TC2_TRACK_D_START_COUNT; ++index) {
                        const tc2_track_d_start_layout *start =
                                Tc2TrackDStartLayout(
                                        (size_t)index);
                        if (!start) continue;
                        put_character(
                                glyph, style, column++,
                                start->label, STYLE_BOLD);
                        put_character(
                                glyph, style, column++,
                                '=', STYLE_BOLD);
                        column += put_text(
                                glyph, style, column,
                                start->entry_node, STYLE_BOLD);
                        if (index + 1 <
                            TC2_TRACK_D_START_COUNT) {
                                column += put_text(
                                        glyph, style, column,
                                        "  ", STYLE_BOLD);
                        }
                }
                return;
        }
        if (row == 42) {
                put_text(
                        glyph, style, 0,
                        "DESTINATIONS: d1..d8 are operator endpoints; uppercase D-bank labels are directed sensors.",
                        STYLE_BOLD);
                return;
        }
        if (row == 43) {
                int column = put_text(
                        glyph, style, 0,
                        "ALERTS: marker_conflicts=", STYLE_BOLD);
                column += put_unsigned_width(
                        glyph, style, column,
                        conflicts, 1,
                        conflicts ? STYLE_WARNING : STYLE_BOLD);
                column += put_text(
                        glyph, style, column,
                        " invalid_evidence=", STYLE_BOLD);
                column += put_unsigned_width(
                        glyph, style, column,
                        invalid_evidence, 1,
                        invalid_evidence ?
                                STYLE_WARNING : STYLE_BOLD);
                put_text(
                        glyph, style, column,
                        conflicts || invalid_evidence ?
                        "  CHECK BEFORE DEMO" :
                        "  deterministic frame OK",
                        conflicts || invalid_evidence ?
                                STYLE_WARNING : STYLE_LIVE);
        }
}

static int build_row(
        char glyph[TC2_UI_RENDER_COLUMNS],
        unsigned char style[TC2_UI_RENDER_COLUMNS],
        const tc2_ui_overlay *overlay, int row,
        unsigned int now_tick, unsigned int *conflicts,
        unsigned int *invalid_evidence) {
        clear_row(glyph, style);
        if (row < TC2_TRACK_D_LAYOUT_ROWS) {
                char base[TC2_UI_RENDER_COLUMNS + 1];
                int length = Tc2TrackDLayoutRenderLine(
                        (size_t)row, base, sizeof(base));
                if (length < 0 ||
                    length > TC2_UI_RENDER_COLUMNS) {
                        return -1;
                }
                for (int column = 0; column < length; ++column) {
                        glyph[column] = base[column];
                }
                if (row == 0) {
                        paint_train_header(glyph, style, overlay);
                }
                paint_sensor_flashes(
                        glyph, style, overlay, row, now_tick);
                paint_train_markers(
                        glyph, style, overlay, row,
                        conflicts);
        } else {
                paint_supplemental_row(
                        glyph, style, overlay, row, now_tick,
                        conflicts ? *conflicts : 0U,
                        invalid_evidence ? *invalid_evidence : 0U);
        }
        return 0;
}

void Tc2UiRendererInit(tc2_ui_renderer *renderer) {
        if (!renderer) return;
        renderer->initialized = 0;
        for (int row = 0; row < TC2_UI_RENDER_ROWS; ++row) {
                for (int column = 0;
                     column < TC2_UI_RENDER_COLUMNS; ++column) {
                        renderer->previous_glyph[row][column] = ' ';
                        renderer->previous_style[row][column] = 0;
                }
        }
}

void Tc2UiRendererInvalidate(tc2_ui_renderer *renderer) {
        if (renderer) renderer->initialized = 0;
}

int Tc2UiRendererRender(
        tc2_ui_renderer *renderer, const tc2_ui_overlay *overlay,
        unsigned int now_tick, tc2_ui_render_write_fn write,
        void *write_context, tc2_ui_render_result *result) {
        render_writer writer;
        unsigned int conflicts = 0;
        unsigned int invalid_evidence;
        int full_redraw;

        if (result) {
                result->full_redraw = 0;
                result->changed_runs = 0;
                result->emitted_bytes = 0;
                result->marker_conflicts = 0;
                result->invalid_evidence = 0;
        }
        if (!renderer || !overlay || !write) {
                if (renderer) Tc2UiRendererInvalidate(renderer);
                return -1;
        }

        writer.write = write;
        writer.context = write_context;
        writer.length = 0;
        writer.total = 0;
        writer.failed = 0;
        full_redraw = !renderer->initialized;
        invalid_evidence = count_invalid_evidence(overlay);

        if (full_redraw &&
            append_text(
                    &writer,
                    "\033[0m\033[?25l\033[2J\033[H") < 0) {
                Tc2UiRendererInvalidate(renderer);
                return -1;
        }

        for (int row = 0; row < TC2_UI_RENDER_ROWS; ++row) {
                char glyph[TC2_UI_RENDER_COLUMNS];
                unsigned char style[TC2_UI_RENDER_COLUMNS];
                int column = 0;
                if (build_row(
                            glyph, style, overlay, row, now_tick,
                            &conflicts, &invalid_evidence) < 0) {
                        Tc2UiRendererInvalidate(renderer);
                        return -1;
                }
                while (column < TC2_UI_RENDER_COLUMNS) {
                        int run_start;
                        int run_end;
                        unsigned char current_style = 0;
                        while (column < TC2_UI_RENDER_COLUMNS &&
                               !full_redraw &&
                               glyph[column] ==
                                       renderer->previous_glyph[
                                               row][column] &&
                               style[column] ==
                                       renderer->previous_style[
                                               row][column]) {
                                ++column;
                        }
                        if (column >= TC2_UI_RENDER_COLUMNS) break;
                        run_start = column;
                        while (column < TC2_UI_RENDER_COLUMNS &&
                               (full_redraw ||
                                glyph[column] !=
                                        renderer->previous_glyph[
                                                row][column] ||
                                style[column] !=
                                        renderer->previous_style[
                                                row][column])) {
                                ++column;
                        }
                        run_end = column;

                        if (emit_cursor(
                                    &writer, row, run_start) < 0) {
                                Tc2UiRendererInvalidate(renderer);
                                return -1;
                        }
                        current_style = style[run_start];
                        if (emit_style(
                                    &writer,
                                    current_style) < 0) {
                                Tc2UiRendererInvalidate(renderer);
                                return -1;
                        }
                        for (int cell = run_start;
                             cell < run_end; ++cell) {
                                if (cell > run_start &&
                                    style[cell] != current_style) {
                                        if (emit_style(
                                                    &writer,
                                                    style[cell]) < 0) {
                                                Tc2UiRendererInvalidate(
                                                        renderer);
                                                return -1;
                                        }
                                        current_style = style[cell];
                                }
                                if (append_character(
                                            &writer,
                                            glyph[cell]) < 0) {
                                        Tc2UiRendererInvalidate(
                                                renderer);
                                        return -1;
                                }
                        }
                        if (current_style != 0 &&
                            emit_style(&writer, 0) < 0) {
                                Tc2UiRendererInvalidate(renderer);
                                return -1;
                        }
                        if (result) ++result->changed_runs;
                }
                for (int copy = 0;
                     copy < TC2_UI_RENDER_COLUMNS; ++copy) {
                        renderer->previous_glyph[row][copy] =
                                glyph[copy];
                        renderer->previous_style[row][copy] =
                                style[copy];
                }
        }

        if (flush_writer(&writer) < 0) {
                Tc2UiRendererInvalidate(renderer);
                return -1;
        }
        renderer->initialized = 1;
        if (result) {
                result->full_redraw =
                        full_redraw ? 1U : 0U;
                result->emitted_bytes = writer.total;
                result->marker_conflicts = conflicts;
                result->invalid_evidence = invalid_evidence;
        }
        return 0;
}

int Tc2UiRendererRestoreTerminal(
        tc2_ui_render_write_fn write, void *write_context) {
        static const char restore[] =
                "\033[0m\033[?25h\033[38;1H";
        if (!write ||
            write(write_context, restore,
                  sizeof(restore) - 1U) != 0) {
                return -1;
        }
        return 0;
}

#endif
