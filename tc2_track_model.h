#ifndef _tc2_track_model_h_
#define _tc2_track_model_h_ 1

#include "track_data.h"

#define TC2_TRACK_START_COUNT 6
#define TC2_TRACK_DESTINATION_COUNT 8
#define TC2_TRACK_DESTINATION_SIDE_COUNT 2
#define TC2_TRACK_SENSOR_NODE_COUNT 80
#define TC2_TRACK_SENSOR_PAIR_COUNT 40
#define TC2_TRACK_SWITCH_COUNT 22

/*
 * Geometry evidence and train-motion calibration are deliberately separate.
 * A confirmed physical endpoint does not imply that every train and speed
 * has a calibrated braking curve for that endpoint.
 */
typedef enum {
        TC2_TRACK_GEOMETRY_PROVISIONAL = 0,
        TC2_TRACK_GEOMETRY_TRIAL,
        TC2_TRACK_GEOMETRY_CONFIRMED
} tc2_track_geometry_evidence;

typedef struct {
        const char *label;
        const char *enter_node;
} tc2_track_start_definition;

typedef struct {
        /*
         * Last directed sensor before the physical operator endpoint when
         * approached from this side. Opposite sides may therefore be two
         * different detectors rather than one reverse sensor pair.
         */
        const char *anchor_sensor;
        int base_offset_mm;
        tc2_track_geometry_evidence geometry_evidence;
        const char *straight_guide;
        const char *previous_anchor_sensor;
        int previous_anchor_distance_mm;
} tc2_track_destination_side;

typedef struct {
        const char *label;
        tc2_track_destination_side
                side[TC2_TRACK_DESTINATION_SIDE_COUNT];
} tc2_track_destination_definition;

const tc2_track_start_definition *Tc2TrackStartDefinition(int index);
const tc2_track_destination_definition *
Tc2TrackDestinationDefinition(int index);
const tc2_track_destination_side *
Tc2TrackDestinationSide(int destination_index, int side);

/*
 * Convert the generated Track B graph branch direction to the corresponding
 * physical Track D turnout command.
 */
char Tc2TrackPhysicalTurnoutDirection(int graph_direction);

/*
 * Validate the static operator catalog against the generated Track B graph
 * used for the physical Track D table. Returns zero only when all starts,
 * directed destination anchors, physical interval partitions, reverse
 * sensor pairs, and turnout branch/merge pairs are internally consistent.
 */
int Tc2TrackCatalogValidate(const track_node track[TRACK_MAX]);

#endif
