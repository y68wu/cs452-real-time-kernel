#ifndef TC2_UI_PATH_GEOMETRY_H
#define TC2_UI_PATH_GEOMETRY_H

#include <stdint.h>

#include "tc2_dispatch.h"

#ifdef MODE_TC2

/*
 * Interpolate the non-Euclidean bends shared by the complete A-F/d1-d8
 * shortest-route matrix. The returned cells are ordered physical rail cells
 * from first to second; zero means this pair is an ordinary straight screen
 * segment handled by the caller.
 */
int Tc2UiPathGeometryInterpolate(
        const tc2_dispatch_projection_waypoint *first,
        const tc2_dispatch_projection_waypoint *second,
        int64_t offset_um, int64_t span_um,
        int *row, int *column);

/* Validate every catalog endpoint, segment, and generated rail cell. */
int Tc2UiPathGeometryValidate(void);

#endif

#endif
