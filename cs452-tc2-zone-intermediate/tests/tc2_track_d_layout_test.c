#include <stdio.h>
#include <string.h>

#include "../tc2_track_d_layout.h"

static int fail(const char *message) {
        fprintf(stderr, "tc2_track_d_layout_test: %s\n", message);
        return 1;
}

static void append_text(
        char *output, size_t capacity, size_t *length,
        const char *text) {
        while (*text && *length + 1 < capacity) {
                output[(*length)++] = *text++;
        }
        output[*length] = '\0';
}

static int catalog_test(void) {
        char error[96];
        int sensor_seen[5][16] = {{0}};
        int switch_seen[157] = {0};

        if (Tc2TrackDLayoutValidate(error, sizeof(error)) < 0) {
                fprintf(stderr, "catalog validation failed: %s\n", error);
                return 1;
        }
        if (Tc2TrackDLayoutLineCount() !=
            TC2_TRACK_D_LAYOUT_ROWS) {
                return fail("unexpected line count");
        }

        for (size_t index = 0; index < TC2_TRACK_D_START_COUNT; ++index) {
                const tc2_track_d_start_layout *start =
                        Tc2TrackDStartLayout(index);
                if (!start || start->label != (char)('A' + index) ||
                    start->row >= TC2_TRACK_D_LAYOUT_ROWS ||
                    start->column >= TC2_TRACK_D_LAYOUT_MAP_COLUMNS ||
                    !Tc2TrackDLayoutCellIsOccupied(
                            start->row, start->column)) {
                        return fail("start catalog/bounds mismatch");
                }
        }
        if (Tc2TrackDStartLayout(TC2_TRACK_D_START_COUNT) != 0) {
                return fail("out-of-range start lookup succeeded");
        }

        for (size_t index = 0;
             index < TC2_TRACK_D_DESTINATION_COUNT; ++index) {
                const tc2_track_d_destination_layout *destination =
                        Tc2TrackDDestinationLayout(index);
                if (!destination ||
                    destination->label[0] != 'd' ||
                    destination->label[1] != (char)('1' + index) ||
                    destination->label[2] != '\0' ||
                    !destination->forward_anchor ||
                    !destination->reverse_anchor ||
                    destination->row >= TC2_TRACK_D_LAYOUT_ROWS ||
                    destination->column >=
                            TC2_TRACK_D_LAYOUT_MAP_COLUMNS ||
                    !Tc2TrackDLayoutCellIsOccupied(
                            destination->row,
                            destination->column)) {
                        return fail("destination catalog/bounds mismatch");
                }
        }
        if (strcmp(Tc2TrackDDestinationLayout(6)->label, "d7") != 0 ||
            strcmp(Tc2TrackDDestinationLayout(6)->forward_anchor, "E9") != 0 ||
            strcmp(Tc2TrackDDestinationLayout(6)->reverse_anchor, "D6") != 0 ||
            Tc2TrackDDestinationLayout(6)->forward_offset_mm != 305) {
                return fail("d7 operator endpoint metadata changed");
        }

        for (size_t index = 0;
             index < TC2_TRACK_D_DETECTOR_COUNT; ++index) {
                const tc2_track_d_detector_layout *detector =
                        Tc2TrackDDetectorLayout(index);
                const char *names[2] = {
                        detector ? detector->first_direction : 0,
                        detector ? detector->second_direction : 0
                };
                if (!detector ||
                    detector->row >= TC2_TRACK_D_LAYOUT_ROWS ||
                    detector->column >=
                            TC2_TRACK_D_LAYOUT_MAP_COLUMNS) {
                        return fail("detector catalog/bounds mismatch");
                }
                for (int side = 0; side < 2; ++side) {
                        const char *name = names[side];
                        tc2_track_d_directed_sensor_cell cell;
                        int sensor_index = (int)(index * 2) + side;
                        int bank;
                        int number = 0;
                        if (!name || name[0] < 'A' || name[0] > 'E') {
                                return fail("invalid sensor bank");
                        }
                        if (Tc2TrackDDirectedSensorCell(
                                    sensor_index, &cell) < 0 ||
                            strcmp(cell.name, name) != 0 ||
                            cell.row != detector->row ||
                            cell.width !=
                                    strlen(detector->display_label) ||
                            cell.column >=
                                    TC2_TRACK_D_LAYOUT_MAP_COLUMNS ||
                            !Tc2TrackDLayoutCellIsOccupied(
                                    cell.row, cell.column)) {
                                return fail("directed sensor cell mismatch");
                        }
                        bank = name[0] - 'A';
                        for (int character = 1; name[character]; ++character) {
                                if (name[character] < '0' ||
                                    name[character] > '9') {
                                        return fail("invalid sensor number");
                                }
                                number = number * 10 +
                                         name[character] - '0';
                        }
                        if (number < 1 || number > 16 ||
                            sensor_seen[bank][number - 1]++) {
                                return fail("duplicate/missing directed sensor");
                        }
                }
        }
        for (int bank = 0; bank < 5; ++bank) {
                for (int number = 0; number < 16; ++number) {
                        if (sensor_seen[bank][number] != 1) {
                                return fail("not all 80 directed sensors present");
                        }
                }
        }
        {
                tc2_track_d_directed_sensor_cell a11;
                tc2_track_d_directed_sensor_cell a12;
                if (Tc2TrackDDirectedSensorCell(10, &a11) < 0 ||
                    Tc2TrackDDirectedSensorCell(11, &a12) < 0 ||
                    strcmp(a11.name, "A11") != 0 ||
                    strcmp(a12.name, "A12") != 0 ||
                    a11.row != a12.row ||
                    a11.column != a12.column) {
                        return fail(
                                "A11/A12 identities must share one detector cell");
                }
                if (Tc2TrackDDirectedSensorCell(-1, &a11) >= 0 ||
                    Tc2TrackDDirectedSensorCell(80, &a11) >= 0 ||
                    Tc2TrackDDirectedSensorCell(0, 0) >= 0) {
                        return fail("invalid directed sensor lookup accepted");
                }
        }
        {
                /* Captured T14 A->D1 physical order from data/7. */
                const int path[] = {0, 44, 70, 54, 56, 75, 39};
                tc2_track_d_directed_sensor_cell cell[7];
                for (size_t index = 0;
                     index < sizeof(path) / sizeof(path[0]); ++index) {
                        if (Tc2TrackDDirectedSensorCell(
                                    path[index], &cell[index]) < 0) {
                                return fail(
                                        "captured physical path cell missing");
                        }
                }
                if (cell[0].row != cell[1].row ||
                    cell[1].row != cell[2].row ||
                    !(cell[0].column > cell[1].column &&
                      cell[1].column > cell[2].column) ||
                    !(cell[3].row > cell[4].row &&
                      cell[4].row > cell[5].row &&
                      cell[5].row > cell[6].row) ||
                    !(cell[3].column <= cell[4].column &&
                      cell[4].column < cell[5].column &&
                      cell[5].column < cell[6].column)) {
                        return fail(
                                "captured A1/C13/E7/D7/D9/E12/C8 "
                                "path does not follow the displayed track");
                }
        }

        for (size_t index = 0;
             index < TC2_TRACK_D_SWITCH_COUNT; ++index) {
                const tc2_track_d_switch_layout *turnout =
                        Tc2TrackDSwitchLayout(index);
                if (!turnout || turnout->number < 1 ||
                    turnout->number > 156 ||
                    turnout->row >= TC2_TRACK_D_LAYOUT_ROWS ||
                    turnout->column >= TC2_TRACK_D_LAYOUT_COLUMNS ||
                    switch_seen[turnout->number]++) {
                        return fail("invalid/duplicate switch catalog entry");
                }
        }
        for (int number = 1; number <= 18; ++number) {
                if (!switch_seen[number]) {
                        return fail("missing ordinary switch");
                }
        }
        for (int number = 153; number <= 156; ++number) {
                if (!switch_seen[number]) {
                        return fail("missing three-way switch");
                }
        }
        if (Tc2TrackDLayoutCellIsOccupied(0, 0) ||
            Tc2TrackDLayoutCellIsOccupied(2, 0) ||
            Tc2TrackDLayoutCellIsOccupied(-1, 0) ||
            Tc2TrackDLayoutCellIsOccupied(
                    TC2_TRACK_D_LAYOUT_ROWS, 0) ||
            Tc2TrackDLayoutCellIsOccupied(
                    1, TC2_TRACK_D_LAYOUT_MAP_COLUMNS)) {
                return fail("blank/out-of-range cell accepted as track");
        }
        return 0;
}

static int render_test(void) {
        char screen[TC2_TRACK_D_LAYOUT_ROWS *
                    (TC2_TRACK_D_LAYOUT_COLUMNS + 2) + 1];
        char line[TC2_TRACK_D_LAYOUT_COLUMNS + 1];
        size_t screen_length = 0;

        screen[0] = '\0';
        for (size_t row = 0; row < Tc2TrackDLayoutLineCount(); ++row) {
                int length = Tc2TrackDLayoutRenderLine(
                        row, line, sizeof(line));
                if (length < 0 ||
                    length > TC2_TRACK_D_LAYOUT_COLUMNS ||
                    strlen(line) != (size_t)length) {
                        return fail("line renderer length/bounds failure");
                }
                append_text(screen, sizeof(screen), &screen_length, line);
                append_text(screen, sizeof(screen), &screen_length, "\n");
        }

        if (Tc2TrackDLayoutRenderLine(
                    1, line, sizeof(line)) < 0 ||
            strcmp(
                    line,
                    "F------------C4/C3----/-------------------------\\------------C8/C7----------------\\----A5/A6-------------B10/B9----E") != 0 ||
            Tc2TrackDLayoutRenderLine(
                    5, line, sizeof(line)) < 0 ||
            strcmp(
                    line,
                    "    /--E11/E12----/--D11/D12------d8---C15/C16------\\-------C5/C6-------\\             \\---A7/A8--------B11/B12-----D") != 0 ||
            Tc2TrackDLayoutRenderLine(
                    7, line, sizeof(line)) < 0 ||
            strcmp(
                    line,
                    "  /                                                                       \\             \\") != 0 ||
            Tc2TrackDLayoutRenderLine(
                    9, line, sizeof(line)) < 0 ||
            strcmp(
                    line,
                    "/          /---E13/E14(d6)----\\-D13/D14------B1/B2--/---------d4--C9/C10----\\             \\---A9/A10------B7/B8----C") != 0) {
                return fail(
                        "operator-supplied upper geometry drifted");
        }

        if (!strstr(screen, "TRACK D") ||
            !strstr(screen, "x") ||
            !strstr(screen, "A1/A2") ||
            !strstr(screen, "C4/C3") ||
            !strstr(screen, "d1") ||
            !strstr(screen, "d8")) {
                return fail("representative dashboard visual is missing");
        }

        if (strstr(screen, "<-o->") ||
            strstr(screen, "TRAIN COLOR LEGEND") ||
            strstr(screen, "TURNOUT CELLS")) {
                return fail("compact operator UI contains diagnostic clutter");
        }
        /*
         * These are two physically separate approach rails. The old drawing
         * incorrectly joined them with '-' cells and allowed the train
         * marker to imply impossible A7/A8->C5/C6 and A9/A10->C9/C10 moves.
         */
        if (Tc2TrackDLayoutCellIsOccupied(5, 80) ||
            Tc2TrackDLayoutCellIsOccupied(9, 82)) {
                return fail("disconnected right approaches were rejoined");
        }
        if (Tc2TrackDStartLayout(0)->row <=
                    Tc2TrackDStartLayout(2)->row ||
            Tc2TrackDDestinationLayout(0)->column <=
                    Tc2TrackDDestinationLayout(1)->column ||
            Tc2TrackDDestinationLayout(6)->column >=
                    Tc2TrackDDestinationLayout(1)->column ||
            Tc2TrackDDestinationLayout(4)->column >=
                    Tc2TrackDDestinationLayout(2)->column) {
                return fail(
                        "operator orientation is not A/B below, D1 right, D7 left");
        }

        if (Tc2TrackDLayoutRenderLine(
                    TC2_TRACK_D_LAYOUT_ROWS, line, sizeof(line)) >= 0 ||
            Tc2TrackDLayoutRenderLine(0, line, 4) >= 0) {
                return fail("invalid render request accepted");
        }
        return 0;
}

int main(void) {
        if (catalog_test() != 0) return 1;
        if (render_test() != 0) return 1;
        printf("tc2_track_d_layout_test: PASS "
               "(6 starts, 8 destinations, 80 sensors, 40 pairs, "
               "22 switches, %d-column bounded renderer)\n",
               TC2_TRACK_D_LAYOUT_COLUMNS);
        return 0;
}
