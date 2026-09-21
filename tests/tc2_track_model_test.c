#include <stdio.h>

#include "../tc2_track_model.h"
#include "../track_route.h"

static int strings_equal(const char *left, const char *right) {
        if (!left || !right) return 0;
        while (*left && *right && *left == *right) {
                ++left;
                ++right;
        }
        return *left == '\0' && *right == '\0';
}

static int require_string(
        const char *actual, const char *expected,
        const char *field, int index) {
        if (strings_equal(actual, expected)) return 0;
        fprintf(stderr, "%s[%d] expected %s, got %s\n",
                field, index, expected,
                actual ? actual : "(null)");
        return -1;
}

int main(void) {
        static const char *const expected_start_labels[
                TC2_TRACK_START_COUNT] =
                {"A", "B", "C", "D", "E", "F"};
        static const char *const expected_start_nodes[
                TC2_TRACK_START_COUNT] =
                {"EN5", "EN4", "EN7", "EN10", "EN9", "EN3"};
        static const char *const expected_destination_labels[
                TC2_TRACK_DESTINATION_COUNT] =
                {"d1", "d2", "d3", "d4", "d5", "d6", "d7", "d8"};
        static const char *const expected_anchor[
                TC2_TRACK_DESTINATION_COUNT][
                        TC2_TRACK_DESTINATION_SIDE_COUNT] = {
                {"A12", "A15"}, {"A4", "B15"},
                {"C11", "B6"}, {"C10", "B2"},
                {"E5", "E6"}, {"E13", "E14"},
                {"E9", "D6"}, {"C15", "D11"}
        };
        static const int expected_offset[
                TC2_TRACK_DESTINATION_COUNT][
                        TC2_TRACK_DESTINATION_SIDE_COUNT] = {
                {381, 381}, {226, 226}, {89, 254}, {76, 279},
                {0, 0}, {0, 0}, {305, 305}, {203, 203}
        };

        track_node track[TRACK_MAX];
        init_trackb(track);
        if (Tc2TrackCatalogValidate(track) != 0) {
                fprintf(stderr, "valid Track D catalog rejected\n");
                return 1;
        }

        for (int start = 0; start < TC2_TRACK_START_COUNT; ++start) {
                const tc2_track_start_definition *definition =
                        Tc2TrackStartDefinition(start);
                if (!definition ||
                    require_string(
                            definition->label,
                            expected_start_labels[start],
                            "start label", start) < 0 ||
                    require_string(
                            definition->enter_node,
                            expected_start_nodes[start],
                            "start node", start) < 0) {
                        return 1;
                }
        }
        if (Tc2TrackStartDefinition(-1) ||
            Tc2TrackStartDefinition(TC2_TRACK_START_COUNT)) {
                fprintf(stderr, "out-of-range start accepted\n");
                return 1;
        }
        if (Tc2TrackPhysicalTurnoutDirection(DIR_STRAIGHT) != 'S' ||
            Tc2TrackPhysicalTurnoutDirection(DIR_CURVED) != 'C' ||
            Tc2TrackPhysicalTurnoutDirection(-1) != '\0') {
                fprintf(stderr,
                        "Track D graph-to-physical turnout mapping drifted\n");
                return 1;
        }

        int confirmed = 0;
        int trial = 0;
        int provisional = 0;
        for (int destination = 0;
             destination < TC2_TRACK_DESTINATION_COUNT; ++destination) {
                const tc2_track_destination_definition *definition =
                        Tc2TrackDestinationDefinition(destination);
                if (!definition ||
                    require_string(
                            definition->label,
                            expected_destination_labels[destination],
                            "destination label", destination) < 0) {
                        return 1;
                }
                for (int side = 0;
                     side < TC2_TRACK_DESTINATION_SIDE_COUNT; ++side) {
                        const tc2_track_destination_side *entry =
                                Tc2TrackDestinationSide(
                                        destination, side);
                        if (!entry ||
                            require_string(
                                    entry->anchor_sensor,
                                    expected_anchor[destination][side],
                                    "destination anchor",
                                    destination * 2 + side) < 0 ||
                            entry->base_offset_mm !=
                                    expected_offset[destination][side]) {
                                fprintf(stderr,
                                        "destination offset d%d side %d expected %d, got %d\n",
                                        destination + 1, side,
                                        expected_offset[destination][side],
                                        entry ?
                                                entry->base_offset_mm :
                                                -1);
                                return 1;
                        }
                        if (entry->geometry_evidence ==
                            TC2_TRACK_GEOMETRY_CONFIRMED) {
                                ++confirmed;
                        } else if (entry->geometry_evidence ==
                                   TC2_TRACK_GEOMETRY_TRIAL) {
                                ++trial;
                        } else {
                                ++provisional;
                        }

                        int anchor = TrackFindNodeByName(
                                track, entry->anchor_sensor);
                        track_route route;
                        for (int start = 0;
                             start < TC2_TRACK_START_COUNT; ++start) {
                                int from = TrackFindNodeByName(
                                        track,
                                        expected_start_nodes[start]);
                                if (anchor < 0 || from < 0 ||
                                    TrackFindShortestRoute(
                                            track, from, anchor,
                                            &route) < 0 ||
                                    route.node_count < 2) {
                                        fprintf(stderr,
                                                "catalog route missing %s -> %s\n",
                                                expected_start_nodes[start],
                                                entry->anchor_sensor);
                                        return 1;
                                }
                        }
                }
        }

        if (confirmed != 0 || trial != 16 || provisional != 0) {
                fprintf(stderr,
                        "geometry evidence partition drifted\n");
                return 1;
        }

        int a12 = TrackFindNodeByName(track, "A12");
        if (a12 < 0 || !track[a12].reverse) {
                fprintf(stderr, "test graph lacks A12 reverse pair\n");
                return 1;
        }
        track_node *saved_reverse = track[a12].reverse;
        track[a12].reverse = &track[a12];
        if (Tc2TrackCatalogValidate(track) == 0) {
                fprintf(stderr,
                        "catalog accepted a broken direction sensor pair\n");
                return 1;
        }
        track[a12].reverse = saved_reverse;
        if (Tc2TrackCatalogValidate(track) != 0) {
                fprintf(stderr,
                        "catalog did not recover after graph restoration\n");
                return 1;
        }

        printf("validated Track D catalog: 6 starts, 8 endpoints, "
               "16 directed anchors, 80 sensors/40 pairs, "
               "22 turnouts; evidence=0 confirmed/16 trial/0 provisional\n");
        return 0;
}
