#include <stdio.h>

#include "../tc2_live_ui.h"
#include "../tc2_offline_planner.h"
#include "../tc2_ui_path_geometry.h"
#include "../track_data.h"

enum {
        SAMPLE_STEPS = 256,
        EXPECTED_ROUTES = 48,
        EXPECTED_VISUAL_PAIRS = 84,
        MAX_UNIQUE_PAIRS = 128
};

typedef struct {
        int first_kind;
        int first_sensor;
        int first_graph;
        int first_destination;
        int second_kind;
        int second_sensor;
        int second_graph;
        int second_destination;
} visual_pair_key;

static int visual_waypoint(
        const tc2_dispatch_projection_waypoint *waypoint) {
        return waypoint &&
               waypoint->kind != TC2_ROUTE_WAYPOINT_TURNOUT &&
               waypoint->ui_width > 0 &&
               waypoint->ui_row < TC2_TRACK_D_LAYOUT_ROWS &&
               waypoint->ui_column <
                       TC2_TRACK_D_LAYOUT_MAP_COLUMNS;
}

static visual_pair_key make_key(
        const tc2_dispatch_projection_waypoint *first,
        const tc2_dispatch_projection_waypoint *second) {
        visual_pair_key key = {
                .first_kind = first->kind,
                .first_sensor = first->sensor_index,
                .first_graph = first->graph_node,
                .first_destination = first->destination_index,
                .second_kind = second->kind,
                .second_sensor = second->sensor_index,
                .second_graph = second->graph_node,
                .second_destination = second->destination_index
        };
        return key;
}

static int same_key(
        const visual_pair_key *left,
        const visual_pair_key *right) {
        return left->first_kind == right->first_kind &&
               left->first_sensor == right->first_sensor &&
               left->first_graph == right->first_graph &&
               left->first_destination ==
                       right->first_destination &&
               left->second_kind == right->second_kind &&
               left->second_sensor == right->second_sensor &&
               left->second_graph == right->second_graph &&
               left->second_destination ==
                       right->second_destination;
}

int main(void) {
        track_node track[TRACK_MAX];
        visual_pair_key unique[MAX_UNIQUE_PAIRS];
        int unique_count = 0;
        int route_count = 0;
        int sampled_segments = 0;

        init_trackb(track);
        if (Tc2TrackDLayoutValidate(0, 0) < 0 ||
            Tc2UiPathGeometryValidate() < 0) {
                fprintf(stderr,
                        "ui geometry matrix: invalid base catalog\n");
                return 1;
        }

        for (int start = 0;
             start < TC2_TRACK_START_COUNT; ++start) {
                for (int destination = 0;
                     destination <
                             TC2_TRACK_DESTINATION_COUNT;
                     ++destination) {
                        tc2_offline_plan plan;
                        if (Tc2OfflinePlannerBuild(
                                    track, 14, start, 100,
                                    destination, &plan) !=
                                            TC2_OFFLINE_PLAN_OK ||
                            !plan.valid) {
                                fprintf(
                                        stderr,
                                        "ui geometry matrix: plan "
                                        "%c->d%d failed\n",
                                        'A' + start,
                                        destination + 1);
                                return 1;
                        }
                        ++route_count;
                        int prior = -1;
                        int prior_end_row = -1;
                        int prior_end_column = -1;
                        int have_prior_end = 0;
                        for (int index = 0;
                             index <
                                     plan.projection_header
                                             .waypoint_count;
                             ++index) {
                                if (!visual_waypoint(
                                            &plan.waypoints[index])) {
                                        continue;
                                }
                                if (prior < 0) {
                                        prior = index;
                                        continue;
                                }
                                const tc2_dispatch_projection_waypoint
                                        *first =
                                                &plan.waypoints[prior];
                                const tc2_dispatch_projection_waypoint
                                        *second =
                                                &plan.waypoints[index];
                                int64_t span =
                                        second->distance_um -
                                        first->distance_um;
                                visual_pair_key key =
                                        make_key(first, second);
                                int known = 0;
                                for (int pair = 0;
                                     pair < unique_count; ++pair) {
                                        if (same_key(
                                                    &unique[pair],
                                                    &key)) {
                                                known = 1;
                                                break;
                                        }
                                }
                                if (!known) {
                                        if (unique_count >=
                                                    MAX_UNIQUE_PAIRS) {
                                                return 1;
                                        }
                                        unique[unique_count++] = key;
                                }
                                if (span <= 0) {
                                        have_prior_end = 0;
                                        prior = index;
                                        continue;
                                }

                                int changes = 0;
                                int previous_row = -1;
                                int previous_column = -1;
                                for (int sample = 0;
                                     sample <= SAMPLE_STEPS;
                                     ++sample) {
                                        int row;
                                        int column;
                                        int64_t offset =
                                                (span * sample) /
                                                SAMPLE_STEPS;
                                        if (!Tc2LiveUiInterpolateTrackCell(
                                                    first, second,
                                                    offset, span,
                                                    &row, &column) ||
                                            !Tc2TrackDLayoutCellIsOccupied(
                                                    row, column)) {
                                                fprintf(
                                                        stderr,
                                                        "ui geometry "
                                                        "blank/snap "
                                                        "%c->d%d "
                                                        "pair=%d/%d "
                                                        "sample=%d\n",
                                                        'A' + start,
                                                        destination + 1,
                                                        prior, index,
                                                        sample);
                                                return 1;
                                        }
                                        if (sample == 0 &&
                                            have_prior_end &&
                                            (row != prior_end_row ||
                                             column !=
                                                     prior_end_column)) {
                                                fprintf(
                                                        stderr,
                                                        "ui geometry "
                                                        "boundary jump "
                                                        "%c->d%d "
                                                        "pair=%d/%d "
                                                        "prior=(%d,%d) "
                                                        "next=(%d,%d)\n",
                                                        'A' + start,
                                                        destination + 1,
                                                        prior, index,
                                                        prior_end_row,
                                                        prior_end_column,
                                                        row, column);
                                                return 1;
                                        }
                                        if (previous_row >= 0 &&
                                            (row != previous_row ||
                                             column !=
                                                     previous_column)) {
                                                ++changes;
                                        }
                                        previous_row = row;
                                        previous_column = column;
                                }
                                prior_end_row = previous_row;
                                prior_end_column = previous_column;
                                have_prior_end = 1;
                                if ((first->ui_row != second->ui_row ||
                                     first->ui_column !=
                                             second->ui_column) &&
                                    changes == 0) {
                                        fprintf(
                                                stderr,
                                                "ui geometry froze "
                                                "%c->d%d pair=%d/%d\n",
                                                'A' + start,
                                                destination + 1,
                                                prior, index);
                                        return 1;
                                }
                                ++sampled_segments;
                                prior = index;
                        }
                }
        }

        if (route_count != EXPECTED_ROUTES ||
            unique_count != EXPECTED_VISUAL_PAIRS ||
            sampled_segments < 400) {
                fprintf(stderr,
                        "ui geometry matrix: routes=%d pairs=%d "
                        "segments=%d\n",
                        route_count, unique_count, sampled_segments);
                return 1;
        }
        printf("tc2_ui_route_geometry_matrix_test: PASS "
               "(48 routes, 84 visual pairs, %d sampled segments, "
               "no blank-cell snap)\n",
               sampled_segments);
        return 0;
}
