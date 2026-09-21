#include "tc2_ui_path_geometry.h"

#ifndef MODE_TC2

typedef int tc2_ui_path_geometry_disabled_translation_unit;

#else

#include "tc2_route_projection.h"
#include "tc2_track_d_layout.h"

typedef struct {
        unsigned char row;
        unsigned char column;
} tc2_ui_path_point;

typedef struct {
        signed char first_sensor;
        signed char second_sensor;
        signed char second_destination;
        unsigned char point_count;
        const tc2_ui_path_point *points;
} tc2_ui_path;

#define UI_POINT(r, c) {(unsigned char)(r), (unsigned char)(c)}

static const tc2_ui_path_point path_a13_c13[] = {
        UI_POINT(33, 95), UI_POINT(33, 91),
        UI_POINT(35, 89), UI_POINT(35, 64)
};
static const tc2_ui_path_point path_c4_c6[] = {
        UI_POINT(1, 13), UI_POINT(1, 48),
        UI_POINT(5, 52), UI_POINT(5, 60)
};
static const tc2_ui_path_point path_c6_b15[] = {
        UI_POINT(5, 60), UI_POINT(5, 72), UI_POINT(12, 79),
        UI_POINT(13, 78), UI_POINT(13, 76)
};
static const tc2_ui_path_point path_c12_a4[] = {
        UI_POINT(33, 67), UI_POINT(33, 77),
        UI_POINT(32, 78), UI_POINT(31, 78),
        UI_POINT(30, 79), UI_POINT(29, 78)
};
static const tc2_ui_path_point path_d5_e6[] = {
        UI_POINT(29, 4), UI_POINT(29, 6),
        UI_POINT(33, 10), UI_POINT(33, 20)
};
static const tc2_ui_path_point path_d6_d7[] = {
        UI_POINT(29, 4), UI_POINT(28, 5),
        UI_POINT(23, 0), UI_POINT(21, 0)
};
static const tc2_ui_path_point path_d6_d9[] = {
        UI_POINT(29, 4), UI_POINT(28, 5),
        UI_POINT(23, 0), UI_POINT(11, 0)
};
static const tc2_ui_path_point path_d10_d5[] = {
        UI_POINT(11, 0), UI_POINT(23, 0),
        UI_POINT(28, 5), UI_POINT(29, 4)
};
static const tc2_ui_path_point path_e1_b14[] = {
        UI_POINT(27, 47), UI_POINT(26, 47),
        UI_POINT(15, 36), UI_POINT(15, 31)
};
static const tc2_ui_path_point path_e9_d7[] = {
        UI_POINT(13, 4), UI_POINT(13, 5), UI_POINT(14, 6),
        UI_POINT(20, 0), UI_POINT(21, 0)
};
static const tc2_ui_path_point path_e14_e9[] = {
        UI_POINT(9, 15), UI_POINT(9, 11),
        UI_POINT(13, 7), UI_POINT(13, 4)
};
static const tc2_ui_path_point path_a8_c7[] = {
        UI_POINT(5, 90), UI_POINT(5, 86),
        UI_POINT(1, 82), UI_POINT(1, 61)
};
static const tc2_ui_path_point path_b14_d16[] = {
        UI_POINT(15, 31), UI_POINT(15, 34), UI_POINT(14, 35),
        UI_POINT(11, 32), UI_POINT(11, 28)
};
static const tc2_ui_path_point path_b15_d2[] = {
        UI_POINT(13, 76), UI_POINT(13, 80),
        UI_POINT(15, 82), UI_POINT(21, 82)
};
static const tc2_ui_path_point path_b16_c5[] = {
        UI_POINT(13, 76), UI_POINT(13, 78), UI_POINT(12, 79),
        UI_POINT(5, 72), UI_POINT(5, 60)
};
static const tc2_ui_path_point path_b16_c10[] = {
        UI_POINT(13, 76), UI_POINT(13, 78), UI_POINT(12, 79),
        UI_POINT(9, 76), UI_POINT(9, 66)
};
static const tc2_ui_path_point path_a4_d2[] = {
        UI_POINT(29, 78), UI_POINT(29, 80),
        UI_POINT(27, 82), UI_POINT(21, 82)
};
static const tc2_ui_path_point path_a4_b16[] = {
        UI_POINT(29, 78), UI_POINT(29, 80), UI_POINT(27, 82),
        UI_POINT(15, 82), UI_POINT(13, 80), UI_POINT(13, 76)
};
static const tc2_ui_path_point path_c11_e16[] = {
        UI_POINT(33, 67), UI_POINT(33, 56),
        UI_POINT(31, 54), UI_POINT(31, 52)
};
static const tc2_ui_path_point path_d1_b14[] = {
        UI_POINT(27, 32), UI_POINT(27, 34), UI_POINT(20, 41),
        UI_POINT(15, 36), UI_POINT(15, 31)
};
static const tc2_ui_path_point path_d6_e10[] = {
        UI_POINT(29, 4), UI_POINT(28, 5), UI_POINT(23, 0),
        UI_POINT(20, 0), UI_POINT(14, 6), UI_POINT(13, 5),
        UI_POINT(13, 4)
};
static const tc2_ui_path_point path_d16_e14[] = {
        UI_POINT(11, 28), UI_POINT(11, 30), UI_POINT(10, 31),
        UI_POINT(9, 30), UI_POINT(9, 15)
};
static const tc2_ui_path_point path_e3_d1[] = {
        UI_POINT(31, 29), UI_POINT(31, 30),
        UI_POINT(28, 33), UI_POINT(27, 32)
};
static const tc2_ui_path_point path_e6_e3[] = {
        UI_POINT(33, 20), UI_POINT(33, 29),
        UI_POINT(32, 30), UI_POINT(31, 29)
};
static const tc2_ui_path_point path_e16_e1[] = {
        UI_POINT(31, 52), UI_POINT(30, 52),
        UI_POINT(27, 49), UI_POINT(27, 47)
};
static const tc2_ui_path_point path_b7_a10[] = {
        UI_POINT(9, 106), UI_POINT(9, 94)
};
static const tc2_ui_path_point path_a10_c7[] = {
        UI_POINT(9, 94), UI_POINT(9, 90),
        UI_POINT(1, 82), UI_POINT(1, 61)
};
static const tc2_ui_path_point path_c7_e11[] = {
        UI_POINT(1, 61), UI_POINT(1, 22),
        UI_POINT(5, 18), UI_POINT(5, 7)
};
static const tc2_ui_path_point path_e11_d10[] = {
        UI_POINT(5, 7), UI_POINT(5, 4),
        UI_POINT(9, 0), UI_POINT(11, 0)
};
static const tc2_ui_path_point path_d10_d8[] = {
        UI_POINT(11, 0), UI_POINT(33, 0)
};
static const tc2_ui_path_point path_d8_e8[] = {
        UI_POINT(33, 0), UI_POINT(35, 2), UI_POINT(35, 18)
};
static const tc2_ui_path_point path_e8_c14[] = {
        UI_POINT(35, 18), UI_POINT(35, 64)
};
static const tc2_ui_path_point path_e7_d7[] = {
        UI_POINT(35, 18), UI_POINT(35, 2), UI_POINT(33, 0)
};
static const tc2_ui_path_point path_d7_d9[] = {
        UI_POINT(33, 0), UI_POINT(11, 0)
};
static const tc2_ui_path_point path_d7_e10[] = {
        UI_POINT(33, 0), UI_POINT(20, 0),
        UI_POINT(14, 6), UI_POINT(13, 5), UI_POINT(13, 4)
};
static const tc2_ui_path_point path_d9_e12[] = {
        UI_POINT(11, 0), UI_POINT(9, 0),
        UI_POINT(5, 4), UI_POINT(5, 7)
};
static const tc2_ui_path_point path_e12_c8[] = {
        UI_POINT(5, 7), UI_POINT(5, 18),
        UI_POINT(1, 22), UI_POINT(1, 61)
};
static const tc2_ui_path_point path_c8_a12[] = {
        UI_POINT(1, 61), UI_POINT(1, 82), UI_POINT(13, 94)
};
static const tc2_ui_path_point path_a12_d1[] = {
        UI_POINT(13, 94), UI_POINT(15, 96),
        UI_POINT(20, 96), UI_POINT(21, 97)
};
static const tc2_ui_path_point path_e10_e13[] = {
        UI_POINT(13, 4), UI_POINT(13, 7),
        UI_POINT(9, 11), UI_POINT(9, 15)
};
static const tc2_ui_path_point path_e13_d15[] = {
        UI_POINT(9, 15), UI_POINT(9, 30), UI_POINT(10, 31),
        UI_POINT(11, 30), UI_POINT(11, 28)
};
static const tc2_ui_path_point path_d15_b13[] = {
        UI_POINT(11, 28), UI_POINT(11, 32), UI_POINT(14, 35),
        UI_POINT(15, 34), UI_POINT(15, 31)
};
static const tc2_ui_path_point path_b13_d2[] = {
        UI_POINT(15, 31), UI_POINT(15, 36), UI_POINT(20, 41),
        UI_POINT(27, 34), UI_POINT(27, 32)
};
static const tc2_ui_path_point path_d2_e4[] = {
        UI_POINT(27, 32), UI_POINT(28, 33),
        UI_POINT(31, 30), UI_POINT(31, 29)
};
static const tc2_ui_path_point path_e4_e5[] = {
        UI_POINT(31, 29), UI_POINT(32, 30),
        UI_POINT(33, 29), UI_POINT(33, 20)
};
static const tc2_ui_path_point path_e5_d6[] = {
        UI_POINT(33, 20), UI_POINT(33, 10),
        UI_POINT(29, 6), UI_POINT(29, 4)
};
static const tc2_ui_path_point path_c9_b15[] = {
        UI_POINT(9, 66), UI_POINT(9, 76), UI_POINT(12, 79),
        UI_POINT(13, 78), UI_POINT(13, 76)
};
static const tc2_ui_path_point path_b15_a3[] = {
        UI_POINT(13, 76), UI_POINT(13, 80), UI_POINT(15, 82),
        UI_POINT(27, 82), UI_POINT(29, 80), UI_POINT(29, 78)
};
static const tc2_ui_path_point path_a3_c11[] = {
        UI_POINT(29, 78), UI_POINT(30, 79), UI_POINT(31, 78),
        UI_POINT(32, 78), UI_POINT(33, 77), UI_POINT(33, 67)
};
static const tc2_ui_path_point path_c14_a15[] = {
        UI_POINT(35, 64), UI_POINT(35, 89), UI_POINT(34, 90),
        UI_POINT(32, 90), UI_POINT(31, 91), UI_POINT(30, 93),
        UI_POINT(29, 94), UI_POINT(29, 96), UI_POINT(29, 93)
};
static const tc2_ui_path_point path_a15_d1[] = {
        UI_POINT(29, 93), UI_POINT(29, 94), UI_POINT(27, 96),
        UI_POINT(22, 96), UI_POINT(21, 97)
};

#define UI_PATH(first, second, destination, points_value) \
        {(first), (second), (destination), \
         (unsigned char)(sizeof(points_value) / \
                         sizeof((points_value)[0])), \
         (points_value)}

/*
 * These are the matrix pairs that otherwise cross whitespace and trigger
 * endpoint snapping. Sensor numbers are protocol indexes
 * (A1=0 ... E16=79); destination numbers are zero-based d1-d8 indexes.
 */
static const tc2_ui_path paths[] = {
        UI_PATH(12, 44, -1, path_a13_c13),
        UI_PATH(35, 37, -1, path_c4_c6),
        UI_PATH(37, 30, -1, path_c6_b15),
        UI_PATH(43, 3, -1, path_c12_a4),
        UI_PATH(52, 69, -1, path_d5_e6),
        UI_PATH(53, -1, 6, path_d6_d7),
        UI_PATH(53, 56, -1, path_d6_d9),
        UI_PATH(57, 52, -1, path_d10_d5),
        UI_PATH(64, 29, -1, path_e1_b14),
        UI_PATH(72, -1, 6, path_e9_d7),
        UI_PATH(77, 72, -1, path_e14_e9),
        UI_PATH(7, 38, -1, path_a8_c7),
        UI_PATH(29, 63, -1, path_b14_d16),
        UI_PATH(30, -1, 1, path_b15_d2),
        UI_PATH(31, 36, -1, path_b16_c5),
        UI_PATH(31, 41, -1, path_b16_c10),
        UI_PATH(3, -1, 1, path_a4_d2),
        UI_PATH(3, 31, -1, path_a4_b16),
        UI_PATH(42, 79, -1, path_c11_e16),
        UI_PATH(48, 29, -1, path_d1_b14),
        UI_PATH(53, 73, -1, path_d6_e10),
        UI_PATH(63, 77, -1, path_d16_e14),
        UI_PATH(66, 48, -1, path_e3_d1),
        UI_PATH(69, 66, -1, path_e6_e3),
        UI_PATH(79, 64, -1, path_e16_e1),
        UI_PATH(22, 9, -1, path_b7_a10),
        UI_PATH(9, 38, -1, path_a10_c7),
        UI_PATH(38, 74, -1, path_c7_e11),
        UI_PATH(74, 57, -1, path_e11_d10),
        UI_PATH(57, 55, -1, path_d10_d8),
        UI_PATH(55, 71, -1, path_d8_e8),
        UI_PATH(71, 45, -1, path_e8_c14),
        UI_PATH(70, 54, -1, path_e7_d7),
        UI_PATH(54, 56, -1, path_d7_d9),
        UI_PATH(54, 73, -1, path_d7_e10),
        UI_PATH(56, 75, -1, path_d9_e12),
        UI_PATH(75, 39, -1, path_e12_c8),
        UI_PATH(39, 11, -1, path_c8_a12),
        UI_PATH(11, -1, 0, path_a12_d1),
        UI_PATH(73, 76, -1, path_e10_e13),
        UI_PATH(76, 62, -1, path_e13_d15),
        UI_PATH(62, 28, -1, path_d15_b13),
        UI_PATH(28, 49, -1, path_b13_d2),
        UI_PATH(49, 67, -1, path_d2_e4),
        UI_PATH(67, 68, -1, path_e4_e5),
        UI_PATH(68, 53, -1, path_e5_d6),
        UI_PATH(40, 30, -1, path_c9_b15),
        UI_PATH(30, 2, -1, path_b15_a3),
        UI_PATH(2, 42, -1, path_a3_c11),
        UI_PATH(45, 14, -1, path_c14_a15),
        UI_PATH(14, -1, 0, path_a15_d1)
};

enum {
        TC2_UI_PATH_COUNT =
                (int)(sizeof(paths) / sizeof(paths[0]))
};

static int absolute_value(int value) {
        return value < 0 ? -value : value;
}

static int segment_steps(
        const tc2_ui_path_point *first,
        const tc2_ui_path_point *second) {
        int row_delta =
                (int)second->row - (int)first->row;
        int column_delta =
                (int)second->column - (int)first->column;
        int row_steps = absolute_value(row_delta);
        int column_steps = absolute_value(column_delta);
        /*
         * One ASCII row represents one diagonal rail step. Its horizontal
         * displacement varies with label width, so row-changing segments
         * advance once per row; purely horizontal rails advance per column.
         */
        return row_steps != 0 ? row_steps : column_steps;
}

static int path_last_step(const tc2_ui_path *path) {
        int total = 0;
        if (!path || !path->points || path->point_count < 2) return -1;
        for (int index = 0; index + 1 < path->point_count; ++index) {
                int steps = segment_steps(
                        &path->points[index],
                        &path->points[index + 1]);
                if (steps < 1 || total > 32767 - steps) return -1;
                total += steps;
        }
        return total;
}

static int path_cell_at(
        const tc2_ui_path *path, int ordinal,
        int *row, int *column) {
        int remaining = ordinal;
        if (!path || !row || !column || ordinal < 0) return -1;
        for (int index = 0; index + 1 < path->point_count; ++index) {
                const tc2_ui_path_point *first =
                        &path->points[index];
                const tc2_ui_path_point *second =
                        &path->points[index + 1];
                int steps = segment_steps(first, second);
                if (steps < 1) return -1;
                if (remaining <= steps) {
                        *row = (int)first->row +
                                (((int)second->row -
                                  (int)first->row) *
                                 remaining) / steps;
                        *column = (int)first->column +
                                (((int)second->column -
                                  (int)first->column) *
                                 remaining) / steps;
                        return 0;
                }
                remaining -= steps;
        }
        return -1;
}

static int path_matches(
        const tc2_ui_path *path,
        const tc2_dispatch_projection_waypoint *first,
        const tc2_dispatch_projection_waypoint *second) {
        if (!path || !first || !second ||
            first->sensor_index != path->first_sensor) {
                return 0;
        }
        if (path->second_sensor >= 0) {
                return second->sensor_index ==
                        path->second_sensor;
        }
        return second->kind ==
                        TC2_ROUTE_WAYPOINT_DESTINATION &&
               second->destination_index ==
                        path->second_destination;
}

/*
 * CURRENT reroutes are reverse-first.  Their synthetic START waypoint can
 * therefore be the far end of one of the forward catalog paths.  Keep using
 * the same ordered rail cells, but traverse them backwards.  For a
 * destination edge the synthetic START intentionally has no destination
 * identity, so its retained map cell is the fail-closed identity check.
 */
static int path_matches_reverse(
        const tc2_ui_path *path,
        const tc2_dispatch_projection_waypoint *first,
        const tc2_dispatch_projection_waypoint *second) {
        if (!path || !first || !second ||
            second->sensor_index != path->first_sensor) {
                return 0;
        }
        if (path->second_sensor >= 0) {
                return first->sensor_index == path->second_sensor;
        }
        if (first->kind == TC2_ROUTE_WAYPOINT_DESTINATION &&
            first->destination_index == path->second_destination) {
                return 1;
        }
        if (first->kind != TC2_ROUTE_WAYPOINT_START ||
            path->point_count < 1) {
                return 0;
        }
        const tc2_ui_path_point *endpoint =
                &path->points[path->point_count - 1];
        return first->ui_row == endpoint->row &&
                first->ui_column == endpoint->column;
}

int Tc2UiPathGeometryInterpolate(
        const tc2_dispatch_projection_waypoint *first,
        const tc2_dispatch_projection_waypoint *second,
        int64_t offset_um, int64_t span_um,
        int *row, int *column) {
        if (!first || !second || !row || !column ||
            span_um <= 0 || offset_um < 0 ||
            offset_um > span_um) {
                return 0;
        }
        for (int index = 0; index < TC2_UI_PATH_COUNT; ++index) {
                const tc2_ui_path *path = &paths[index];
                int reverse = 0;
                if (!path_matches(path, first, second)) {
                        if (!path_matches_reverse(
                                    path, first, second)) {
                                continue;
                        }
                        reverse = 1;
                }
                int last_step = path_last_step(path);
                if (last_step < 1) return 0;
                int ordinal = offset_um <= 0 ? 0 :
                        offset_um >= span_um ? last_step :
                        (int)((offset_um * last_step) / span_um);
                if (reverse) ordinal = last_step - ordinal;
                if (path_cell_at(
                            path, ordinal, row, column) < 0 ||
                    !Tc2TrackDLayoutCellIsOccupied(*row, *column)) {
                        return 0;
                }
                return 1;
        }
        return 0;
}

int Tc2UiPathGeometryValidate(void) {
        if (TC2_UI_PATH_COUNT != 51) return -1;
        for (int index = 0; index < TC2_UI_PATH_COUNT; ++index) {
                const tc2_ui_path *path = &paths[index];
                int last_step = path_last_step(path);
                if (path->first_sensor < 0 ||
                    (path->second_sensor < 0 &&
                     (path->second_destination < 0 ||
                      path->second_destination >=
                              TC2_TRACK_D_DESTINATION_COUNT)) ||
                    last_step < 1) {
                        return -1;
                }
                for (int prior = 0; prior < index; ++prior) {
                        if (paths[prior].first_sensor ==
                                    path->first_sensor &&
                            paths[prior].second_sensor ==
                                    path->second_sensor &&
                            paths[prior].second_destination ==
                                    path->second_destination) {
                                return -1;
                        }
                }
                for (int ordinal = 0;
                     ordinal <= last_step; ++ordinal) {
                        int row;
                        int column;
                        if (path_cell_at(
                                    path, ordinal,
                                    &row, &column) < 0 ||
                            !Tc2TrackDLayoutCellIsOccupied(
                                    row, column)) {
                                return -1;
                        }
                }
        }
        return 0;
}

#endif
