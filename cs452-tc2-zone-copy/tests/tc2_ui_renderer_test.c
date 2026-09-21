#include <stdio.h>
#include <string.h>

#include "../tc2_ui_renderer.h"

enum {
        CAPTURE_CAPACITY = 131072,
        TEST_TERMINAL_ROWS = 40,
        TERM_BOLD = 1,
        TERM_UNDERLINE = 2,
        TERM_REVERSE = 4
};

typedef struct {
        char bytes[CAPTURE_CAPACITY];
        size_t length;
        int calls;
        int fail_on_call;
} capture_sink;

typedef struct {
        char glyph[TEST_TERMINAL_ROWS][TC2_UI_RENDER_COLUMNS];
        int color[TEST_TERMINAL_ROWS][TC2_UI_RENDER_COLUMNS];
        unsigned char flags[TEST_TERMINAL_ROWS][TC2_UI_RENDER_COLUMNS];
        int row;
        int column;
        int current_color;
        unsigned char current_flags;
        int cursor_hidden;
        int errors;
} virtual_terminal;

static capture_sink capture;
static virtual_terminal terminal;

static int fail(const char *message) {
        fprintf(stderr, "tc2_ui_renderer_test: %s\n", message);
        return 1;
}

static void reset_capture(void) {
        capture.length = 0;
        capture.calls = 0;
        capture.fail_on_call = 0;
}

static int capture_write(
        void *context, const char *bytes, size_t length) {
        capture_sink *sink = context;
        ++sink->calls;
        if (sink->fail_on_call > 0 &&
            sink->calls == sink->fail_on_call) {
                return -1;
        }
        if (!bytes || sink->length + length > sizeof(sink->bytes)) {
                return -1;
        }
        memcpy(sink->bytes + sink->length, bytes, length);
        sink->length += length;
        return 0;
}

static int capture_contains(const char *needle) {
        size_t needle_length = strlen(needle);
        if (needle_length == 0) return 1;
        if (needle_length > capture.length) return 0;
        for (size_t offset = 0;
             offset + needle_length <= capture.length; ++offset) {
                if (memcmp(capture.bytes + offset,
                           needle, needle_length) == 0) {
                        return 1;
                }
        }
        return 0;
}

static void initialize_terminal(virtual_terminal *term) {
        memset(term, 0, sizeof(*term));
        for (int row = 0; row < TEST_TERMINAL_ROWS; ++row) {
                for (int column = 0;
                     column < TC2_UI_RENDER_COLUMNS; ++column) {
                        term->glyph[row][column] = ' ';
                        term->color[row][column] = -1;
                }
        }
        term->current_color = -1;
}

static void clear_terminal_cells(virtual_terminal *term) {
        for (int row = 0; row < TEST_TERMINAL_ROWS; ++row) {
                for (int column = 0;
                     column < TC2_UI_RENDER_COLUMNS; ++column) {
                        term->glyph[row][column] = ' ';
                        term->color[row][column] = -1;
                        term->flags[row][column] = 0;
                }
        }
}

static int is_csi_final(unsigned char character) {
        return character >= 0x40 && character <= 0x7e;
}

static void apply_sgr(
        virtual_terminal *term, const int *parameters, int count) {
        if (count == 0) {
                term->current_color = -1;
                term->current_flags = 0;
                return;
        }
        for (int index = 0; index < count; ++index) {
                int value = parameters[index];
                if (value == 0) {
                        term->current_color = -1;
                        term->current_flags = 0;
                } else if (value == 1) {
                        term->current_flags |= TERM_BOLD;
                } else if (value == 4) {
                        term->current_flags |= TERM_UNDERLINE;
                } else if (value == 7) {
                        term->current_flags |= TERM_REVERSE;
                } else if (value == 38 && index + 2 < count &&
                           parameters[index + 1] == 5) {
                        term->current_color =
                                parameters[index + 2];
                        index += 2;
                }
        }
}

static int apply_capture_to_terminal(virtual_terminal *term) {
        size_t offset = 0;
        while (offset < capture.length) {
                unsigned char character =
                        (unsigned char)capture.bytes[offset++];
                if (character == 0x1b) {
                        int parameters[12] = {0};
                        int parameter_count = 1;
                        int private_sequence = 0;
                        unsigned char final;
                        if (offset >= capture.length ||
                            capture.bytes[offset++] != '[') {
                                ++term->errors;
                                return -1;
                        }
                        if (offset < capture.length &&
                            capture.bytes[offset] == '?') {
                                private_sequence = 1;
                                ++offset;
                        }
                        while (offset < capture.length &&
                               !is_csi_final(
                                       (unsigned char)
                                       capture.bytes[offset])) {
                                unsigned char part =
                                        (unsigned char)
                                        capture.bytes[offset++];
                                if (part >= '0' && part <= '9') {
                                        parameters[
                                                parameter_count - 1] =
                                                parameters[
                                                        parameter_count - 1] *
                                                        10 +
                                                part - '0';
                                } else if (part == ';' &&
                                           parameter_count <
                                                   (int)(sizeof(
                                                           parameters) /
                                                         sizeof(
                                                           parameters[0]))) {
                                        ++parameter_count;
                                } else {
                                        ++term->errors;
                                        return -1;
                                }
                        }
                        if (offset >= capture.length) {
                                ++term->errors;
                                return -1;
                        }
                        final = (unsigned char)capture.bytes[offset++];
                        if (private_sequence) {
                                if (final == 'l') term->cursor_hidden = 1;
                                if (final == 'h') term->cursor_hidden = 0;
                                continue;
                        }
                        if (final == 'H' || final == 'f') {
                                int row = parameters[0] ?
                                        parameters[0] : 1;
                                int column =
                                        parameter_count > 1 &&
                                        parameters[1] ?
                                                parameters[1] : 1;
                                --row;
                                --column;
                                if (row < 0 ||
                                    row >= TEST_TERMINAL_ROWS ||
                                    column < 0 ||
                                    column >= TC2_UI_RENDER_COLUMNS) {
                                        ++term->errors;
                                        return -1;
                                }
                                term->row = row;
                                term->column = column;
                        } else if (final == 'J') {
                                if (parameters[0] == 2) {
                                        clear_terminal_cells(term);
                                        term->row = 0;
                                        term->column = 0;
                                }
                        } else if (final == 'K') {
                                if (parameters[0] == 2) {
                                        for (int column = 0;
                                             column <
                                                     TC2_UI_RENDER_COLUMNS;
                                             ++column) {
                                                term->glyph[
                                                        term->row][column] =
                                                        ' ';
                                                term->color[
                                                        term->row][column] =
                                                        -1;
                                                term->flags[
                                                        term->row][column] =
                                                        0;
                                        }
                                }
                        } else if (final == 'm') {
                                apply_sgr(
                                        term, parameters,
                                        parameter_count);
                        } else {
                                ++term->errors;
                                return -1;
                        }
                        continue;
                }
                if (term->row < 0 ||
                    term->row >= TEST_TERMINAL_ROWS ||
                    term->column < 0 ||
                    term->column >= TC2_UI_RENDER_COLUMNS) {
                        ++term->errors;
                        return -1;
                }
                term->glyph[term->row][term->column] =
                        (char)character;
                term->color[term->row][term->column] =
                        term->current_color;
                term->flags[term->row][term->column] =
                        term->current_flags;
                ++term->column;
        }
        return 0;
}

static int find_on_terminal(
        const virtual_terminal *term, const char *text,
        int *found_row, int *found_column) {
        int width = (int)strlen(text);
        for (int row = 0; row < TEST_TERMINAL_ROWS; ++row) {
                for (int column = 0;
                     column + width <= TC2_UI_RENDER_COLUMNS;
                     ++column) {
                        if (memcmp(&term->glyph[row][column],
                                   text, (size_t)width) == 0) {
                                if (found_row) *found_row = row;
                                if (found_column) {
                                        *found_column = column;
                                }
                                return 1;
                        }
                }
        }
        return 0;
}

static int count_map_glyph(
        const virtual_terminal *term, char glyph) {
        int count = 0;
        for (int row = 0;
             row < TC2_TRACK_D_LAYOUT_ROWS; ++row) {
                for (int column = 0;
                     column < TC2_TRACK_D_LAYOUT_MAP_COLUMNS;
                     ++column) {
                        if (term->glyph[row][column] == glyph) {
                                ++count;
                        }
                }
        }
        return count;
}

static int first_frame_and_palette_test(void) {
        tc2_ui_renderer renderer;
        tc2_ui_overlay overlay;
        tc2_ui_overlay before;
        tc2_ui_render_result result;

        Tc2UiRendererInit(&renderer);
        Tc2UiOverlayInit(
                &overlay, TC2_UI_SOURCE_OFFLINE_SIMULATION);
        for (int slot = 0; slot < TC2_UI_MAX_TRAINS; ++slot) {
                int train = 14 + slot;
                if (Tc2UiOverlayUpsertAtStart(
                            &overlay, train, slot % 6,
                            slot % 8, 1U,
                            TC2_UI_QUALITY_PREDICTED, 90U) < 0 ||
                    Tc2UiOverlayUpdatePredictedCell(
                            &overlay, train, 30,
                            slot * 5, 1U, 1U, 100U) < 0) {
                        return fail("could not prepare sixteen trains");
                }
        }
        memcpy(&before, &overlay, sizeof(before));
        reset_capture();
        initialize_terminal(&terminal);
        terminal.current_color = 201;
        terminal.current_flags =
                TERM_BOLD | TERM_REVERSE | TERM_UNDERLINE;
        if (Tc2UiRendererRender(
                    &renderer, &overlay, 100U,
                    capture_write, &capture, &result) < 0 ||
            !result.full_redraw || result.changed_runs == 0 ||
            result.emitted_bytes != capture.length ||
            result.marker_conflicts != 0 ||
            result.invalid_evidence != 0 ||
            memcmp(&before, &overlay, sizeof(before)) != 0 ||
            !capture_contains(
                    "\033[0m\033[?25l\033[2J\033[H") ||
            apply_capture_to_terminal(&terminal) < 0 ||
            !terminal.cursor_hidden || terminal.errors != 0 ||
            terminal.current_color != -1 ||
            terminal.current_flags != 0 ||
            terminal.color[32][130] != -1 ||
            terminal.flags[32][130] != 0) {
                return fail("first frame or retained model is invalid");
        }
        if (!find_on_terminal(&terminal, "TRACK D", 0, 0) ||
            !find_on_terminal(&terminal, "T14 RED", 0, 0) ||
            !find_on_terminal(&terminal, "A1/A2", 0, 0) ||
            !find_on_terminal(&terminal, "C4/C3", 0, 0) ||
            !find_on_terminal(&terminal, "d1", 0, 0) ||
            !find_on_terminal(&terminal, "d8", 0, 0) ||
            find_on_terminal(&terminal, "TRAIN COLOR LEGEND", 0, 0) ||
            find_on_terminal(&terminal, "<-o->", 0, 0) ||
            capture_contains("SENSOR_CONFIRMED")) {
                return fail("required map truth is missing");
        }
        for (int slot = 0; slot < TC2_UI_MAX_TRAINS; ++slot) {
                int color = Tc2UiOverlayXtermColor(slot);
                for (int previous = 0;
                     previous < slot; ++previous) {
                        if (color ==
                            Tc2UiOverlayXtermColor(previous)) {
                                return fail(
                                        "marker palette is not stable/unique");
                        }
                }
        }

        reset_capture();
        if (Tc2UiRendererRender(
                    &renderer, &overlay, 100U,
                    capture_write, &capture, &result) < 0 ||
            result.full_redraw || result.changed_runs != 0 ||
            result.emitted_bytes != 0 || capture.length != 0) {
                return fail("unchanged frame emitted terminal traffic");
        }
        return 0;
}

static int physical_sensor_marker_and_delta_test(void) {
        tc2_ui_renderer renderer;
        tc2_ui_overlay overlay;
        tc2_ui_render_result result;
        tc2_track_d_directed_sensor_cell a11;
        tc2_track_d_directed_sensor_cell a13;
        const tc2_ui_train_overlay *train14;
        const tc2_ui_train_overlay *train13;

        Tc2UiRendererInit(&renderer);
        Tc2UiOverlayInit(&overlay, TC2_UI_SOURCE_LIVE_CAN);
        if (Tc2UiOverlayUpsertAtStart(
                    &overlay, 14, 0, 6, 1U,
                    TC2_UI_QUALITY_PREDICTED, 0U) < 0 ||
            Tc2UiOverlayUpsertAtStart(
                    &overlay, 13, 1, 0, 1U,
                    TC2_UI_QUALITY_PREDICTED, 0U) < 0 ||
            Tc2UiOverlayUpdateAtSensor(
                    &overlay, 14, 10, 1U, 1U, 1U,
                    TC2_UI_QUALITY_SENSOR_CONFIRMED, 100U) < 0 ||
            Tc2UiOverlayUpdateAtSensor(
                    &overlay, 13, 12, 1U, 1U, 1U,
                    TC2_UI_QUALITY_SENSOR_CONFIRMED, 100U) < 0 ||
            Tc2TrackDDirectedSensorCell(10, &a11) < 0 ||
            Tc2TrackDDirectedSensorCell(12, &a13) < 0) {
                return fail("could not prepare physical CAN positions");
        }
        train14 = Tc2UiOverlayFindTrain(&overlay, 14);
        train13 = Tc2UiOverlayFindTrain(&overlay, 13);
        reset_capture();
        initialize_terminal(&terminal);
        if (!train14 || !train13 ||
            Tc2UiRendererRender(
                    &renderer, &overlay, 100U,
                    capture_write, &capture, &result) < 0 ||
            apply_capture_to_terminal(&terminal) < 0 ||
            train14->color_slot != 0 ||
            Tc2UiOverlayXtermColor(train14->color_slot) != 196 ||
            terminal.glyph[a11.row][a11.column + a11.width] != '@' ||
            terminal.color[a11.row][a11.column + a11.width] !=
                    Tc2UiOverlayXtermColor(train14->color_slot) ||
            terminal.color[a11.row][a11.column] !=
                    Tc2UiOverlayXtermColor(train14->color_slot) ||
            terminal.glyph[a13.row][a13.column + a13.width] != '@' ||
            terminal.color[a13.row][a13.column + a13.width] !=
                    Tc2UiOverlayXtermColor(train13->color_slot)) {
                return fail("physical CAN markers/colors are incorrect");
        }

        reset_capture();
        if (Tc2UiRendererRender(
                    &renderer, &overlay, 110U,
                    capture_write, &capture, &result) < 0 ||
            result.full_redraw ||
            capture_contains("\033[2J") ||
            apply_capture_to_terminal(&terminal) < 0 ||
            result.changed_runs == 0 ||
            capture.length == 0 ||
            terminal.glyph[a11.row][a11.column + a11.width] != '@' ||
            terminal.color[a11.row][a11.column] != -1) {
                return fail("physical sensor flash did not toggle cleanly");
        }

        reset_capture();
        Tc2UiOverlayAdvanceRefresh(&overlay);
        if (Tc2UiRendererRender(
                    &renderer, &overlay, 110U,
                    capture_write, &capture, &result) < 0 ||
            result.full_redraw || result.changed_runs != 0 ||
            capture.length != 0 || capture_contains("\033[2J") ||
            apply_capture_to_terminal(&terminal) < 0 ||
            terminal.glyph[a11.row][a11.column + a11.width] != '@') {
                return fail("refresh phase moved the physical marker");
        }

        /*
         * A continuously projected train may legitimately occupy a sensor
         * label cell.  Rendering must preserve that exact route cell instead
         * of shifting every predicted marker to the label's right edge,
         * which made right-to-left trips visibly jump backward.
         */
        if (Tc2UiOverlayUpdatePredictedCell(
                    &overlay, 14, a11.row, a11.column,
                    1U, 2U, 120U) < 0) {
                return fail(
                        "could not place prediction on a sensor cell");
        }
        reset_capture();
        if (Tc2UiRendererRender(
                    &renderer, &overlay, 120U,
                    capture_write, &capture, &result) < 0 ||
            apply_capture_to_terminal(&terminal) < 0 ||
            terminal.glyph[a11.row][a11.column] != '@' ||
            terminal.color[a11.row][a11.column] !=
                    Tc2UiOverlayXtermColor(train14->color_slot) ||
            terminal.glyph[a11.row][a11.column + a11.width] == '@') {
                return fail(
                        "predicted sensor-cell marker was shifted");
        }
        return 0;
}

static int conflict_and_recovery_test(void) {
        tc2_ui_renderer renderer;
        tc2_ui_overlay overlay;
        tc2_ui_render_result result;

        Tc2UiRendererInit(&renderer);
        Tc2UiOverlayInit(
                &overlay, TC2_UI_SOURCE_OFFLINE_SIMULATION);
        if (Tc2UiOverlayUpsertAtStart(
                    &overlay, 14, 0, 0, 1U,
                    TC2_UI_QUALITY_PREDICTED, 0U) < 0 ||
            Tc2UiOverlayUpsertAtStart(
                    &overlay, 13, 1, 1, 1U,
                    TC2_UI_QUALITY_PREDICTED, 0U) < 0 ||
            Tc2UiOverlayUpdatePredictedCell(
                    &overlay, 14, 24, 80, 1U, 1U, 10U) < 0 ||
            Tc2UiOverlayUpdatePredictedCell(
                    &overlay, 13, 24, 80, 1U, 1U, 10U) < 0) {
                return fail("could not prepare marker conflict");
        }
        reset_capture();
        initialize_terminal(&terminal);
        if (Tc2UiRendererRender(
                    &renderer, &overlay, 10U,
                    capture_write, &capture, &result) < 0 ||
            apply_capture_to_terminal(&terminal) < 0 ||
            result.marker_conflicts != 1 ||
            terminal.glyph[24][80] != '!') {
                return fail("co-located trains were silently overwritten");
        }

        if (Tc2UiOverlayUpdatePredictedCell(
                    &overlay, 13, 24, 90, 1U, 2U, 20U) < 0) {
                return fail("could not move train out of conflict");
        }
        reset_capture();
        if (Tc2UiRendererRender(
                    &renderer, &overlay, 20U,
                    capture_write, &capture, &result) < 0 ||
            result.full_redraw || result.marker_conflicts != 0 ||
            capture_contains("\033[2J") ||
            apply_capture_to_terminal(&terminal) < 0 ||
            terminal.glyph[24][80] != '@' ||
            terminal.glyph[24][90] != '@') {
                return fail("conflict delta did not recover both markers");
        }
        return 0;
}

static int render_invalid_case(
        tc2_ui_overlay *overlay, unsigned int expected_invalid,
        int expect_no_marker, const char *message) {
        tc2_ui_renderer renderer;
        tc2_ui_render_result result;
        Tc2UiRendererInit(&renderer);
        reset_capture();
        initialize_terminal(&terminal);
        if (Tc2UiRendererRender(
                    &renderer, overlay, 50U,
                    capture_write, &capture, &result) < 0 ||
            apply_capture_to_terminal(&terminal) < 0 ||
            result.invalid_evidence != expected_invalid ||
            (expect_no_marker &&
             count_map_glyph(&terminal, '@') != 0)) {
                return fail(message);
        }
        return 0;
}

static int fail_closed_validation_matrix_test(void) {
        tc2_ui_overlay overlay;

        Tc2UiOverlayInit(
                &overlay, TC2_UI_SOURCE_OFFLINE_SIMULATION);
        if (Tc2UiOverlayUpsertAtStart(
                    &overlay, 14, 0, 6, 1U,
                    TC2_UI_QUALITY_PREDICTED, 0U) < 0) {
                return fail("could not prepare invalid-source case");
        }
        overlay.source = 99;
        if (render_invalid_case(
                    &overlay, 1U, 1,
                    "invalid source was rendered or miscounted")) {
                return 1;
        }
        if (find_on_terminal(
                    &terminal,
                    "OFFLINE PREDICTION / SIMULATION", 0, 0) ||
            find_on_terminal(
                    &terminal, "source=OFFLINE", 0, 0)) {
                return fail(
                        "invalid source was mislabeled as offline");
        }

        Tc2UiOverlayInit(
                &overlay, TC2_UI_SOURCE_OFFLINE_SIMULATION);
        if (Tc2UiOverlayUpsertAtStart(
                    &overlay, 14, 0, 6, 1U,
                    TC2_UI_QUALITY_PREDICTED, 0U) < 0) {
                return fail("could not prepare NONE-position case");
        }
        overlay.trains[0].position_kind = TC2_UI_POSITION_NONE;
        if (render_invalid_case(
                    &overlay, 1U, 1,
                    "active NONE position was rendered")) {
                return 1;
        }

        Tc2UiOverlayInit(
                &overlay, TC2_UI_SOURCE_OFFLINE_SIMULATION);
        if (Tc2UiOverlayUpsertAtStart(
                    &overlay, 14, 0, 6, 1U,
                    TC2_UI_QUALITY_PREDICTED, 0U) < 0) {
                return fail("could not prepare bad-start case");
        }
        ++overlay.trains[0].row;
        if (render_invalid_case(
                    &overlay, 1U, 1,
                    "forged start coordinates were rendered")) {
                return 1;
        }

        Tc2UiOverlayInit(
                &overlay, TC2_UI_SOURCE_OFFLINE_SIMULATION);
        if (Tc2UiOverlayUpsertAtStart(
                    &overlay, 14, 0, 6, 1U,
                    TC2_UI_QUALITY_PREDICTED, 0U) < 0 ||
            Tc2UiOverlayUpdatePredictedCell(
                    &overlay, 14, 10, 10,
                    1U, 1U, 10U) < 0) {
                return fail("could not prepare bad-prediction case");
        }
        overlay.trains[0].column =
                TC2_TRACK_D_LAYOUT_MAP_COLUMNS;
        if (render_invalid_case(
                    &overlay, 1U, 1,
                    "out-of-map prediction was rendered")) {
                return 1;
        }

        Tc2UiOverlayInit(&overlay, TC2_UI_SOURCE_LIVE_CAN);
        if (Tc2UiOverlayUpsertAtStart(
                    &overlay, 14, 0, 6, 1U,
                    TC2_UI_QUALITY_PREDICTED, 0U) < 0 ||
            Tc2UiOverlayUpdateAtSensor(
                    &overlay, 14, 10, 1U, 1U, 1U,
                    TC2_UI_QUALITY_SENSOR_CONFIRMED, 10U) < 0) {
                return fail("could not prepare bad-sensor case");
        }
        overlay.sensors[10].active = 0;
        overlay.trains[0].sensor_index =
                TC2_TRACK_D_DIRECTED_SENSOR_COUNT;
        if (render_invalid_case(
                    &overlay, 1U, 1,
                    "out-of-range sensor evidence was rendered")) {
                return 1;
        }

        Tc2UiOverlayInit(&overlay, TC2_UI_SOURCE_LIVE_CAN);
        if (Tc2UiOverlayUpsertAtStart(
                    &overlay, 14, 0, 6, 1U,
                    TC2_UI_QUALITY_PREDICTED, 0U) < 0 ||
            Tc2UiOverlayUpdateAtDestination(
                    &overlay, 14, 6, 1U, 1U,
                    TC2_UI_QUALITY_ESTIMATED, 10U) < 0) {
                return fail("could not prepare virtual-destination case");
        }
        overlay.trains[0].position_quality =
                TC2_UI_QUALITY_SENSOR_CONFIRMED;
        if (render_invalid_case(
                    &overlay, 1U, 1,
                    "virtual destination accepted sensor confirmation")) {
                return 1;
        }

        Tc2UiOverlayInit(
                &overlay, TC2_UI_SOURCE_OFFLINE_SIMULATION);
        if (Tc2UiOverlayUpsertAtStart(
                    &overlay, 14, 0, 6, 1U,
                    TC2_UI_QUALITY_PREDICTED, 0U) < 0 ||
            Tc2UiOverlayUpsertAtStart(
                    &overlay, 13, 1, 5, 1U,
                    TC2_UI_QUALITY_PREDICTED, 0U) < 0) {
                return fail("could not prepare duplicate-identity case");
        }
        overlay.trains[1].train = overlay.trains[0].train;
        overlay.trains[1].color_slot =
                overlay.trains[0].color_slot;
        if (render_invalid_case(
                    &overlay, 1U, 1,
                    "duplicate train identity/color was rendered")) {
                return 1;
        }

        Tc2UiOverlayInit(&overlay, TC2_UI_SOURCE_LIVE_CAN);
        if (Tc2UiOverlayUpsertAtStart(
                    &overlay, 14, 0, 6, 1U,
                    TC2_UI_QUALITY_PREDICTED, 0U) < 0 ||
            Tc2UiOverlayUpdateAtSensor(
                    &overlay, 14, 10, 1U, 1U, 1U,
                    TC2_UI_QUALITY_SENSOR_CONFIRMED, 10U) < 0) {
                return fail("could not prepare orphan-flash case");
        }
        overlay.sensors[10].train = 99;
        if (render_invalid_case(
                    &overlay, 1U, 0,
                    "orphan sensor flash was not rejected")) {
                return 1;
        }
        {
                tc2_track_d_directed_sensor_cell sensor;
                if (Tc2TrackDDirectedSensorCell(
                            10, &sensor) < 0 ||
                    terminal.glyph[sensor.row][
                            sensor.column + sensor.width] != '@') {
                        return fail(
                                "invalid flash suppressed the physical marker");
                }
        }

        overlay.sensors[10].train = 14;
        ++overlay.sensors[10].row;
        if (render_invalid_case(
                    &overlay, 1U, 0,
                    "forged sensor-flash coordinates were accepted")) {
                return 1;
        }

        Tc2UiOverlayInit(&overlay, TC2_UI_SOURCE_LIVE_CAN);
        if (Tc2UiOverlayUpsertAtStart(
                    &overlay, 14, 0, 6, 1U,
                    TC2_UI_QUALITY_PREDICTED, 0U) < 0 ||
            Tc2UiOverlayUpdateAtSensor(
                    &overlay, 14, 10, 1U, 1U, 1U,
                    TC2_UI_QUALITY_SENSOR_CONFIRMED, 10U) < 0) {
                return fail(
                        "could not prepare flash-provenance cases");
        }
        overlay.sensors[10].evidence_quality =
                TC2_UI_QUALITY_PREDICTED;
        if (render_invalid_case(
                    &overlay, 1U, 0,
                    "predicted crossing colored a live sensor")) {
                return 1;
        }

        overlay.sensors[10].evidence_quality =
                TC2_UI_QUALITY_SENSOR_CONFIRMED;
        overlay.sensors[10].plan_generation = 2U;
        if (render_invalid_case(
                    &overlay, 1U, 0,
                    "stale-plan sensor flash was accepted")) {
                return 1;
        }

        overlay.sensors[10].plan_generation = 1U;
        overlay.sensors[10].evidence_source = -1;
        if (render_invalid_case(
                    &overlay, 1U, 0,
                    "source-less live sensor flash was accepted")) {
                return 1;
        }
        return 0;
}

static int evidence_and_sink_failure_test(void) {
        tc2_ui_renderer renderer;
        tc2_ui_overlay overlay;
        tc2_ui_render_result result;

        Tc2UiRendererInit(&renderer);
        Tc2UiOverlayInit(
                &overlay, TC2_UI_SOURCE_OFFLINE_SIMULATION);
        if (Tc2UiOverlayUpsertAtStart(
                    &overlay, 14, 0, 6, 1U,
                    TC2_UI_QUALITY_PREDICTED, 0U) < 0) {
                return fail("could not prepare evidence test");
        }
        overlay.trains[0].position_quality =
                TC2_UI_QUALITY_SENSOR_CONFIRMED;
        reset_capture();
        initialize_terminal(&terminal);
        if (Tc2UiRendererRender(
                    &renderer, &overlay, 1U,
                    capture_write, &capture, &result) < 0 ||
            result.invalid_evidence != 1 ||
            apply_capture_to_terminal(&terminal) < 0 ||
            count_map_glyph(&terminal, '@') != 0 ||
            capture_contains("sensor-confirmed")) {
                return fail("offline confirmation was not rejected visibly");
        }

        overlay.trains[0].start_index = 99;
        reset_capture();
        if (Tc2UiRendererRender(
                    &renderer, &overlay, 2U,
                    capture_write, &capture, &result) < 0 ||
            result.invalid_evidence != 1) {
                return fail("malformed active train was not fail-closed");
        }

        Tc2UiRendererInvalidate(&renderer);
        reset_capture();
        capture.fail_on_call = 1;
        if (Tc2UiRendererRender(
                    &renderer, &overlay, 3U,
                    capture_write, &capture, &result) >= 0 ||
            renderer.initialized) {
                return fail("sink failure retained a partial frame");
        }
        reset_capture();
        if (Tc2UiRendererRender(
                    &renderer, &overlay, 4U,
                    capture_write, &capture, &result) < 0 ||
            !result.full_redraw ||
            !capture_contains("\033[2J")) {
                return fail("sink recovery did not force a full redraw");
        }
        reset_capture();
        if (Tc2UiRendererRestoreTerminal(
                    capture_write, &capture) < 0 ||
            !capture_contains("\033[0m\033[?25h\033[38;1H")) {
                return fail("terminal restore sequence is incomplete");
        }
        return 0;
}

static int fixed_identity_header_test(void) {
        tc2_ui_renderer renderer;
        tc2_ui_overlay overlay;
        tc2_ui_render_result result;
        const tc2_track_d_destination_layout *d1 =
                Tc2TrackDDestinationLayout(0);
        int row;
        int column;
        const char *identity = "T15 BLUE speed=60";

        if (!d1) return fail("T15 identity layout unavailable");
        Tc2UiRendererInit(&renderer);
        Tc2UiOverlayInit(&overlay, TC2_UI_SOURCE_LIVE_CAN);
        if (Tc2UiOverlayUpsertAtPredictedCell(
                    &overlay, 15, d1->row, d1->column,
                    0, 9U, TC2_UI_QUALITY_PREDICTED, 1U) < 0 ||
            Tc2UiOverlaySetTrainSpeed(
                    &overlay, 15, 60, 9U, 1U) < 0) {
                return fail("could not prepare T15 identity header");
        }
        /* Deliberately poison marker evidence; identity must stay BLUE. */
        for (int slot = 0; slot < TC2_UI_MAX_TRAINS; ++slot) {
                if (overlay.trains[slot].active &&
                    overlay.trains[slot].train == 15) {
                        overlay.trains[slot].position_quality =
                                TC2_UI_QUALITY_SENSOR_CONFIRMED;
                }
        }
        reset_capture();
        initialize_terminal(&terminal);
        if (Tc2UiRendererRender(
                    &renderer, &overlay, 2U,
                    capture_write, &capture, &result) < 0 ||
            result.invalid_evidence != 1 ||
            apply_capture_to_terminal(&terminal) < 0 ||
            !find_on_terminal(
                    &terminal, identity, &row, &column)) {
                return fail("invalid marker repainted or hid T15 identity");
        }
        int expected_color = Tc2UiOverlayXtermColor(2);
        for (int offset = 0;
             identity[offset] != '\0'; ++offset) {
                if (terminal.color[row][column + offset] !=
                            expected_color ||
                    !(terminal.flags[row][column + offset] & TERM_BOLD)) {
                        return fail("T15 identity header was not always BLUE");
                }
        }
        return 0;
}

static int compact_terminal_prompt_test(void) {
        static const char prompt_restore[] =
                "\033[0m\033[37;1H\033[2K"
                "T14 RED | speed=80 | position=A11"
                "\033[38;1H\033[2K> \033[?25h";
        tc2_ui_renderer renderer;
        tc2_ui_overlay overlay;
        tc2_ui_render_result result;

        Tc2UiRendererInit(&renderer);
        Tc2UiOverlayInit(&overlay, TC2_UI_SOURCE_LIVE_CAN);
        reset_capture();
        initialize_terminal(&terminal);
        if (Tc2UiRendererRender(
                    &renderer, &overlay, 1U,
                    capture_write, &capture, &result) < 0) {
                return fail("compact terminal fixture did not render");
        }
        for (int refresh = 0; refresh < 100; ++refresh) {
                if (capture_write(
                            &capture, prompt_restore,
                            sizeof(prompt_restore) - 1U) < 0) {
                        return fail("compact prompt capture overflowed");
                }
        }
        if (apply_capture_to_terminal(&terminal) < 0 ||
            terminal.errors != 0 ||
            terminal.row != 37 ||
            !find_on_terminal(
                    &terminal, "TRACK D", 0, 0) ||
            !find_on_terminal(
                    &terminal, "T14 RED | speed=80", 0, 0)) {
                return fail(
                        "repeated live prompt scrolled/corrupted 40-row terminal");
        }
        return 0;
}

int main(void) {
        if (first_frame_and_palette_test() ||
            physical_sensor_marker_and_delta_test() ||
            conflict_and_recovery_test() ||
            fail_closed_validation_matrix_test() ||
            evidence_and_sink_failure_test() ||
            fixed_identity_header_test() ||
            compact_terminal_prompt_test()) {
                return 1;
        }
        printf("tc2_ui_renderer_test: PASS\n");
        return 0;
}
