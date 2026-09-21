#include "tc2_track_d_layout.h"

#ifndef MODE_TC2

/*
 * Keep the source in the shared wildcard build without adding TC2 dashboard
 * symbols to TC1/PERF/K4 images.
 */
typedef int tc2_track_d_layout_disabled_translation_unit;

#else

static const tc2_track_d_start_layout starts[
        TC2_TRACK_D_START_COUNT] = {
        {'A', "EN5", 35, 115},
        {'B', "EN4", 33, 115},
        {'C', "EN7", 9, 115},
        {'D', "EN10", 5, 115},
        {'E', "EN9", 1, 115},
        {'F', "EN3", 1, 0}
};

/*
 * Lowercase d1-d8 are operator endpoints, never sensor names.  Uppercase
 * D1-D16 remain directed sensor identities.  Coordinates follow the
 * operator's physical Track D view: F/E/D/C above, B/A below, d7 at the left
 * outside loop and d1 at the CAN-side right outside loop.
 */
static const tc2_track_d_destination_layout destinations[
        TC2_TRACK_D_DESTINATION_COUNT] = {
        {"d1", "A12", "A15", 381, 381, 21, 97},
        {"d2", "A4", "B15", 226, 226, 21, 82},
        {"d3", "C11", "B6", 89, 254, 33, 63},
        {"d4", "C10", "B2", 76, 279, 9, 62},
        {"d5", "E5", "E6", 0, 0, 33, 26},
        {"d6", "E13", "E14", 0, 0, 9, 23},
        {"d7", "E9", "D6", 305, 305, 21, 0},
        {"d8", "C15", "D11", 203, 203, 5, 34}
};

/*
 * Each row is one physical detector.  The odd/even directed nodes are
 * co-located, but the renderer preserves both names and arrows:
 *
 *                 A11 <- o -> A12
 *
 * The compact rendered form omits spaces to keep the schematic readable.
 */
static const tc2_track_d_detector_layout detectors[
        TC2_TRACK_D_DETECTOR_COUNT] = {
        {"A1", "A2", "A1/A2", 35, 93},
        {"A3", "A4", "A3/A4", 29, 78},
        {"A5", "A6", "A5/A6", 1, 87},
        {"A7", "A8", "A7/A8", 5, 90},
        {"A9", "A10", "A9/A10", 9, 94},
        {"A11", "A12", "A11/A12", 13, 94},
        {"A13", "A14", "A13/A14", 33, 95},
        {"A15", "A16", "A15/A16", 29, 93},

        {"B1", "B2", "B1/B2", 9, 45},
        {"B3", "B4", "B3/B4", 11, 48},
        {"B5", "B6", "B5/B6", 33, 48},
        {"B7", "B8", "B7/B8", 9, 106},
        {"B9", "B10", "B10/B9", 1, 105},
        {"B11", "B12", "B11/B12", 5, 103},
        {"B13", "B14", "B13/B14", 15, 31},
        {"B15", "B16", "B15/B16", 13, 76},

        {"C1", "C2", "C1/C2", 15, 46},
        {"C3", "C4", "C4/C3", 1, 13},
        {"C5", "C6", "C5/C6", 5, 60},
        {"C7", "C8", "C8/C7", 1, 61},
        {"C9", "C10", "C9/C10", 9, 66},
        {"C11", "C12", "C11/C12", 33, 67},
        {"C13", "C14", "C13/C14", 35, 64},
        {"C15", "C16", "C15/C16", 5, 39},

        {"D1", "D2", "D1/D2", 27, 32},
        {"D3", "D4", "D3/D4", 33, 36},
        {"D5", "D6", "D5/D6", 29, 4},
        {"D7", "D8", "D8/D7", 33, 0},
        {"D9", "D10", "D10/D9", 11, 0},
        {"D11", "D12", "D11/D12", 5, 21},
        {"D13", "D14", "D13/D14", 9, 32},
        {"D15", "D16", "D15/D16", 11, 28},

        {"E1", "E2", "E1/E2", 27, 47},
        {"E3", "E4", "E3/E4", 31, 29},
        {"E5", "E6", "E5/E6", 33, 20},
        {"E7", "E8", "E7/E8", 35, 18},
        {"E9", "E10", "E9/E10", 13, 4},
        {"E11", "E12", "E11/E12", 5, 7},
        {"E13", "E14", "E13/E14", 9, 15},
        {"E15", "E16", "E16/E15", 31, 52}
};

/*
 * Turnouts use a stable key area at the right of the physical schematic.
 * Keeping every number in the catalog also gives the live layer fixed cells
 * to color when a route selects a straight or curved branch.
 */
static const tc2_track_d_switch_layout switches[
        TC2_TRACK_D_SWITCH_COUNT] = {
        {1, 0, 0}, {2, 0, 0}, {3, 0, 0}, {4, 0, 0},
        {5, 0, 0}, {6, 0, 0}, {7, 0, 0}, {8, 0, 0},
        {9, 0, 0}, {10, 0, 0}, {11, 0, 0}, {12, 0, 0},
        {13, 0, 0}, {14, 0, 0}, {15, 0, 0}, {16, 0, 0},
        {17, 0, 0}, {18, 0, 0}, {153, 0, 0}, {154, 0, 0},
        {155, 0, 0}, {156, 0, 0}
};

/*
 * Only the first TC2_TRACK_D_LAYOUT_MAP_COLUMNS columns are used here.
 * Direction-specific detector tokens are painted over these track strokes.
 */
static const char *const map_background[TC2_TRACK_D_LAYOUT_ROWS] = {
        "TRACK D",
        "F------------C4/C3----/-------------------------\\------------C8/C7----------------\\----A5/A6-------------B10/B9----E",
        "                     /                           \\                                 \\",
        "                    /                             \\                                 \\",
        "                   /                               \\                                 \\",
        "    /--E11/E12----/--D11/D12------d8---C15/C16------\\-------C5/C6-------\\             \\---A7/A8--------B11/B12-----D",
        "   /                                                                     \\             \\",
        "  /                                                                       \\             \\",
        " /                                                                         \\             \\",
        "/          /---E13/E14(d6)----\\-D13/D14------B1/B2--/---------d4--C9/C10----\\             \\---A9/A10------B7/B8----C",
        "|         /                    \\                   /                         \\             \\",
        "D10/D9   /                  D15/D16             B3/B4                         \\             \\",
        "|       /                        \\               /                             \\             \\",
        "|   E9/E10                        \\      x      /                           B15/B16           A11/A12",
        "|     /                            \\     |     /                                 \\             \\",
        "|    /                         B13/B14   |    C1/C2                               \\             \\",
        "|   /                                \\   |   /                                    |             |",
        "|  /                                  \\  |  /                                     |             |",
        "| /                                    \\ | /                                      |             |",
        "|/                                      \\|/                                       |             |",
        "|                                        |                                        |             |",
        "d7                                      /|\\                                       d2             d1",
        "|                                      / | \\                                      |             |",
        "|                                     /  |  \\                                     |             |",
        "|\\                                   /   |   \\                                    |             |",
        "| \\                                 /    |    \\                                   |             |",
        "|  \\                               /     |     \\                                  |             | ",
        "|   \\                           D1/D2    |     E1/E2                              /             /",
        "|    \\                           /       |        \\                              /             /",
        "|   D5/D6                       /        |         \\                          A3/A4          A15/A16",
        "|      \\                       /         |          \\                          /             /",
        "|       \\                    E3/E4       x          E16/E15                   /            /",
        "|        \\                    |                        |                      |           |",
        "D8/D7-----\\---------E5/E6(d5)-|-----D3/D4-------B5/B6--|-------d3--C11/C12----|           |----A13/A14-------------B",
        " \\                                                                            /           /",
        "  \\---------------E7/E8-----------------------------------------C13/C14------/-----------/---A1/A2-----------------A"
};

static size_t text_length(const char *text) {
        size_t length = 0;
        if (!text) return 0;
        while (text[length]) ++length;
        return length;
}

static int texts_equal(const char *left, const char *right) {
        if (!left || !right) return 0;
        while (*left && *right && *left == *right) {
                ++left;
                ++right;
        }
        return *left == '\0' && *right == '\0';
}

static void copy_error(
        char *output, size_t capacity, const char *message) {
        size_t index = 0;
        if (!output || capacity == 0) return;
        while (message[index] && index + 1 < capacity) {
                output[index] = message[index];
                ++index;
        }
        output[index] = '\0';
}

static int decimal_length(int value) {
        int length = 1;
        while (value >= 10) {
                value /= 10;
                ++length;
        }
        return length;
}

static int parse_sensor_name(
        const char *name, char *bank, int *number) {
        int value = 0;
        int digits = 0;
        if (!name || name[0] < 'A' || name[0] > 'E') return -1;
        for (size_t index = 1; name[index]; ++index) {
                if (name[index] < '0' || name[index] > '9') return -1;
                value = value * 10 + name[index] - '0';
                ++digits;
        }
        if (digits == 0 || value < 1 || value > 16) return -1;
        if (bank) *bank = name[0];
        if (number) *number = value;
        return 0;
}

static void put_character(
        char line[TC2_TRACK_D_LAYOUT_COLUMNS],
        size_t column, char character) {
        if (column < TC2_TRACK_D_LAYOUT_COLUMNS) line[column] = character;
}

static void put_text(
        char line[TC2_TRACK_D_LAYOUT_COLUMNS],
        size_t column, const char *text) {
        if (!text) return;
        for (size_t index = 0;
             text[index] && column + index < TC2_TRACK_D_LAYOUT_COLUMNS;
             ++index) {
                line[column + index] = text[index];
        }
}

static void put_start(
        char line[TC2_TRACK_D_LAYOUT_COLUMNS],
        const tc2_track_d_start_layout *start) {
        put_character(line, start->column, start->label);
}

static void put_destination_marker(
        char line[TC2_TRACK_D_LAYOUT_COLUMNS],
        const tc2_track_d_destination_layout *destination) {
        put_text(line, destination->column, destination->label);
}

static void put_detector(
        char line[TC2_TRACK_D_LAYOUT_COLUMNS],
        const tc2_track_d_detector_layout *detector) {
        put_text(line, detector->column, detector->display_label);
}

size_t Tc2TrackDLayoutLineCount(void) {
        return TC2_TRACK_D_LAYOUT_ROWS;
}

int Tc2TrackDLayoutRenderLine(
        size_t row, char *output, size_t capacity) {
        char line[TC2_TRACK_D_LAYOUT_COLUMNS];
        size_t length;
        if (row >= TC2_TRACK_D_LAYOUT_ROWS || !output) return -1;

        for (size_t column = 0;
             column < TC2_TRACK_D_LAYOUT_COLUMNS; ++column) {
                line[column] = ' ';
        }
        put_text(line, 0, map_background[row]);
        for (size_t index = 0;
             index < TC2_TRACK_D_START_COUNT; ++index) {
                if (starts[index].row == row) put_start(line, &starts[index]);
        }
        for (size_t index = 0;
             index < TC2_TRACK_D_DESTINATION_COUNT; ++index) {
                if (destinations[index].row == row) {
                        put_destination_marker(line, &destinations[index]);
                }
        }
        for (size_t index = 0;
             index < TC2_TRACK_D_DETECTOR_COUNT; ++index) {
                if (detectors[index].row == row) {
                        put_detector(line, &detectors[index]);
                }
        }

        length = TC2_TRACK_D_LAYOUT_COLUMNS;
        while (length > 0 && line[length - 1] == ' ') --length;
        if (capacity <= length) return -1;
        for (size_t index = 0; index < length; ++index) {
                output[index] = line[index];
        }
        output[length] = '\0';
        return (int)length;
}

int Tc2TrackDLayoutCellIsOccupied(int row, int column) {
        char line[TC2_TRACK_D_LAYOUT_COLUMNS + 1];
        int length;

        if (row <= 0 || row >= TC2_TRACK_D_LAYOUT_ROWS ||
            column < 0 ||
            column >= TC2_TRACK_D_LAYOUT_MAP_COLUMNS) {
                return 0;
        }
        length = Tc2TrackDLayoutRenderLine(
                (size_t)row, line, sizeof(line));
        return length > column && line[column] != ' ';
}

const tc2_track_d_start_layout *Tc2TrackDStartLayout(size_t index) {
        if (index >= TC2_TRACK_D_START_COUNT) return 0;
        return &starts[index];
}

const tc2_track_d_destination_layout *
Tc2TrackDDestinationLayout(size_t index) {
        if (index >= TC2_TRACK_D_DESTINATION_COUNT) return 0;
        return &destinations[index];
}

const tc2_track_d_detector_layout *
Tc2TrackDDetectorLayout(size_t index) {
        if (index >= TC2_TRACK_D_DETECTOR_COUNT) return 0;
        return &detectors[index];
}

int Tc2TrackDDirectedSensorCell(
        int sensor_index, tc2_track_d_directed_sensor_cell *cell) {
        const tc2_track_d_detector_layout *detector;
        int detector_index;
        size_t column;
        const char *name;

        if (!cell || sensor_index < 0 ||
            sensor_index >= TC2_TRACK_D_DIRECTED_SENSOR_COUNT) {
                return -1;
        }

        detector_index = sensor_index / 2;
        detector = &detectors[detector_index];
        name = (sensor_index & 1) ?
                detector->second_direction : detector->first_direction;
        /*
         * Both directed names belong to one physical detector.  The compact
         * UI deliberately maps both directions to the same track cell so a
         * train cannot appear to jump sideways when the CAN direction flips.
         */
        column = detector->column;

        if (column >= TC2_TRACK_D_LAYOUT_MAP_COLUMNS ||
            text_length(name) == 0 ||
            !detector->display_label ||
            column + text_length(detector->display_label) >
                    TC2_TRACK_D_LAYOUT_MAP_COLUMNS) {
                return -1;
        }

        cell->name = name;
        cell->row = detector->row;
        cell->column = (unsigned char)column;
        cell->width =
                (unsigned char)text_length(detector->display_label);
        return 0;
}

const tc2_track_d_switch_layout *Tc2TrackDSwitchLayout(size_t index) {
        if (index >= TC2_TRACK_D_SWITCH_COUNT) return 0;
        return &switches[index];
}

int Tc2TrackDLayoutValidate(char *error, size_t error_capacity) {
        int directed_sensor_count = 0;
        int physical_pair_count = 0;

        if (TC2_TRACK_D_START_COUNT != 6) {
                copy_error(error, error_capacity, "expected six starts");
                return -1;
        }
        if (TC2_TRACK_D_DESTINATION_COUNT != 8) {
                copy_error(error, error_capacity, "expected eight destinations");
                return -1;
        }

        for (size_t index = 0;
             index < TC2_TRACK_D_START_COUNT; ++index) {
                if (starts[index].label != (char)('A' + index) ||
                    !starts[index].entry_node ||
                    starts[index].row >= TC2_TRACK_D_LAYOUT_ROWS ||
                    starts[index].column >= TC2_TRACK_D_LAYOUT_MAP_COLUMNS) {
                        copy_error(error, error_capacity,
                                   "invalid start catalog");
                        return -1;
                }
        }

        for (size_t index = 0;
             index < TC2_TRACK_D_DESTINATION_COUNT; ++index) {
                const tc2_track_d_destination_layout *destination =
                        &destinations[index];
                if (!destination->label ||
                    destination->label[0] != 'd' ||
                    destination->label[1] != (char)('1' + index) ||
                    destination->label[2] != '\0' ||
                    !destination->forward_anchor ||
                    !destination->reverse_anchor ||
                    destination->forward_offset_mm < 0 ||
                    destination->reverse_offset_mm < 0 ||
                    destination->row >= TC2_TRACK_D_LAYOUT_ROWS ||
                    destination->column + 2 >
                            TC2_TRACK_D_LAYOUT_MAP_COLUMNS) {
                        copy_error(error, error_capacity,
                                   "invalid destination catalog");
                        return -1;
                }
        }

        for (size_t index = 0;
             index < TC2_TRACK_D_DETECTOR_COUNT; ++index) {
                const tc2_track_d_detector_layout *detector =
                        &detectors[index];
                char first_bank;
                char second_bank;
                int first_number;
                int second_number;
                if (parse_sensor_name(detector->first_direction,
                                      &first_bank, &first_number) < 0 ||
                    parse_sensor_name(detector->second_direction,
                                      &second_bank, &second_number) < 0 ||
                    first_bank != second_bank ||
                    (first_number & 1) == 0 ||
                    second_number != first_number + 1) {
                        copy_error(error, error_capacity,
                                   "invalid directed reverse pair");
                        return -1;
                }
                if (!detector->display_label ||
                    text_length(detector->display_label) <
                            text_length(detector->first_direction) + 1 ||
                    detector->display_label[
                            text_length(detector->display_label)] != '\0') {
                        copy_error(error, error_capacity,
                                   "invalid detector display label");
                        return -1;
                }
                if (detector->row >= TC2_TRACK_D_LAYOUT_ROWS ||
                    detector->column >=
                            TC2_TRACK_D_LAYOUT_MAP_COLUMNS ||
                    detector->column +
                                    text_length(
                                            detector->display_label) >
                            TC2_TRACK_D_LAYOUT_MAP_COLUMNS) {
                        copy_error(error, error_capacity,
                                   "detector coordinate out of bounds");
                        return -1;
                }
                for (size_t other = 0; other < index; ++other) {
                        const tc2_track_d_detector_layout *previous =
                                &detectors[other];
                        if (texts_equal(detector->first_direction,
                                        previous->first_direction) ||
                            texts_equal(detector->first_direction,
                                        previous->second_direction) ||
                            texts_equal(detector->second_direction,
                                        previous->first_direction) ||
                            texts_equal(detector->second_direction,
                                        previous->second_direction)) {
                                copy_error(error, error_capacity,
                                           "duplicate directed sensor");
                                return -1;
                        }
                }
                directed_sensor_count += 2;
                ++physical_pair_count;
        }
        if (directed_sensor_count !=
                    TC2_TRACK_D_DIRECTED_SENSOR_COUNT ||
            physical_pair_count != TC2_TRACK_D_DETECTOR_COUNT) {
                copy_error(error, error_capacity,
                           "incomplete directed sensor catalog");
                return -1;
        }

        for (size_t index = 0;
             index < TC2_TRACK_D_SWITCH_COUNT; ++index) {
                const tc2_track_d_switch_layout *turnout =
                        &switches[index];
                if (turnout->row >= TC2_TRACK_D_LAYOUT_ROWS ||
                    turnout->column + 2 +
                                    (size_t)decimal_length(turnout->number) >
                            TC2_TRACK_D_LAYOUT_COLUMNS) {
                        copy_error(error, error_capacity,
                                   "switch coordinate out of bounds");
                        return -1;
                }
                for (size_t other = 0; other < index; ++other) {
                        if (turnout->number == switches[other].number) {
                                copy_error(error, error_capacity,
                                           "duplicate switch number");
                                return -1;
                        }
                }
        }

        for (size_t row = 0;
             row < TC2_TRACK_D_LAYOUT_ROWS; ++row) {
                char rendered[TC2_TRACK_D_LAYOUT_COLUMNS + 1];
                int length = Tc2TrackDLayoutRenderLine(
                        row, rendered, sizeof(rendered));
                if (length < 0 ||
                    length > TC2_TRACK_D_LAYOUT_COLUMNS) {
                        copy_error(error, error_capacity,
                                   "rendered line out of bounds");
                        return -1;
                }
        }

        copy_error(error, error_capacity, "ok");
        return 0;
}

#endif
