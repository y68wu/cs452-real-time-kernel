#ifndef TC2_TRACK_D_LAYOUT_H
#define TC2_TRACK_D_LAYOUT_H

#include <stddef.h>

/*
 * This module is deliberately independent of the kernel, terminal, CAN, and
 * dispatcher code.  It describes the immutable Track D dashboard background;
 * a later live-UI layer can paint trains and sensor flashes over these cells.
 *
 * Tc2TrackDLayoutRenderLine() never appends a newline.  Its result is at most
 * TC2_TRACK_D_LAYOUT_COLUMNS characters, so a terminal owner can stream one
 * bounded line followed by "\r\n" without constructing a full-screen buffer.
 */
#define TC2_TRACK_D_LAYOUT_COLUMNS 132
#define TC2_TRACK_D_LAYOUT_MAP_COLUMNS 130
#define TC2_TRACK_D_LAYOUT_ROWS 36
#define TC2_TRACK_D_START_COUNT 6
#define TC2_TRACK_D_DESTINATION_COUNT 8
#define TC2_TRACK_D_DETECTOR_COUNT 40
#define TC2_TRACK_D_DIRECTED_SENSOR_COUNT 80
#define TC2_TRACK_D_SWITCH_COUNT 22

typedef struct {
        char label;
        const char *entry_node;
        unsigned char row;
        unsigned char column;
} tc2_track_d_start_layout;

typedef struct {
        const char *label;
        const char *forward_anchor;
        const char *reverse_anchor;
        int forward_offset_mm;
        int reverse_offset_mm;
        unsigned char row;
        unsigned char column;
} tc2_track_d_destination_layout;

typedef struct {
        const char *first_direction;
        const char *second_direction;
        const char *display_label;
        unsigned char row;
        unsigned char column;
} tc2_track_d_detector_layout;

typedef struct {
        const char *name;
        unsigned char row;
        unsigned char column;
        unsigned char width;
} tc2_track_d_directed_sensor_cell;

typedef struct {
        int number;
        unsigned char row;
        unsigned char column;
} tc2_track_d_switch_layout;

size_t Tc2TrackDLayoutLineCount(void);

/*
 * Returns the rendered line length, or -1 for an invalid row or insufficient
 * output capacity.  capacity includes the terminating NUL byte.
 */
int Tc2TrackDLayoutRenderLine(
        size_t row, char *output, size_t capacity);

/*
 * Returns one only for a visible cell on the immutable physical map.  Live
 * and simulated train markers use this guard so interpolation can never
 * paint a train in whitespace between tracks.
 */
int Tc2TrackDLayoutCellIsOccupied(int row, int column);

const tc2_track_d_start_layout *Tc2TrackDStartLayout(size_t index);
const tc2_track_d_destination_layout *
Tc2TrackDDestinationLayout(size_t index);
const tc2_track_d_detector_layout *
Tc2TrackDDetectorLayout(size_t index);
/*
 * Maps the protocol sensor index (A1=0 ... E16=79) to its physical detector
 * label cell.  Both directed names return the same cell because they are one
 * detector; the displayed pair flashes together while status text preserves
 * the exact directed sensor name delivered by CAN.
 */
int Tc2TrackDDirectedSensorCell(
        int sensor_index, tc2_track_d_directed_sensor_cell *cell);
const tc2_track_d_switch_layout *Tc2TrackDSwitchLayout(size_t index);

/*
 * Validates the complete immutable catalog.  On failure, error receives a
 * short diagnostic when it is non-NULL and error_capacity is non-zero.
 */
int Tc2TrackDLayoutValidate(char *error, size_t error_capacity);

#endif
