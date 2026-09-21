#include <limits.h>
#include <stdint.h>
#include <stdio.h>

#include "../tc2_motion_model.h"
#include "../tc2_prediction_model.h"

enum {
        EXPECTED_PLAN_COUNT =
                TC2_TRACK_START_COUNT *
                TC2_TRACK_DESTINATION_COUNT *
                TC2_TRACK_DESTINATION_SIDE_COUNT * 120
};

static int strings_equal(const char *left, const char *right) {
        if (!left || !right) return 0;
        while (*left && *right && *left == *right) {
                ++left;
                ++right;
        }
        return *left == '\0' && *right == '\0';
}

static int route_for(
        track_node track[TRACK_MAX], int start_index,
        int destination_index, int side_index,
        track_route *route) {
        const tc2_track_start_definition *start =
                Tc2TrackStartDefinition(start_index);
        const tc2_track_destination_side *side =
                Tc2TrackDestinationSide(
                        destination_index, side_index);
        int from = start ?
                TrackFindNodeByName(track, start->enter_node) : -1;
        int anchor = side ?
                TrackFindNodeByName(track, side->anchor_sensor) : -1;
        if (!route || from < 0 || anchor < 0 ||
            TrackFindShortestRoute(
                    track, from, anchor, route) < 0 ||
            route->node_count < 2) {
                return -1;
        }
        return 0;
}

static int route_scalar_at_offset(
        track_node track[TRACK_MAX], const track_route *route,
        int route_offset, int64_t *scalar_um) {
        int distance_mm;
        if (!scalar_um || route_offset < 0 ||
            route_offset >= route->node_count ||
            TrackRouteDistanceBetweenOffsets(
                    track, route, 0, route_offset,
                    &distance_mm) < 0 ||
            distance_mm < 0) {
                return -1;
        }
        *scalar_um = (int64_t)distance_mm * 1000;
        return 0;
}

static int check_scalar_location(
        track_node track[TRACK_MAX], const track_route *route,
        int64_t scalar_um, int anchor_node,
        int anchor_route_offset, int64_t offset_um,
        const char *kind, int start, int destination,
        int side, int speed) {
        int64_t anchor_scalar_um;
        if (anchor_route_offset < 0 ||
            anchor_route_offset >= route->node_count ||
            anchor_node != route->nodes[anchor_route_offset] ||
            offset_um < 0 ||
            route_scalar_at_offset(
                    track, route, anchor_route_offset,
                    &anchor_scalar_um) < 0 ||
            anchor_scalar_um + offset_um != scalar_um) {
                fprintf(stderr,
                        "%s location mismatch A-F=%d D=%d side=%d speed=%d "
                        "scalar=%lld anchor=%d offset=%d residual=%lld\n",
                        kind, start, destination, side, speed,
                        (long long)scalar_um, anchor_node,
                        anchor_route_offset, (long long)offset_um);
                return -1;
        }

        /*
         * The selected node must be the greatest route scalar not exceeding
         * the requested scalar. This catches a location that is algebraically
         * self-consistent but failed to re-anchor as far forward as possible.
         */
        if (anchor_route_offset + 1 < route->node_count) {
                int64_t next_scalar_um;
                if (route_scalar_at_offset(
                            track, route,
                            anchor_route_offset + 1,
                            &next_scalar_um) < 0 ||
                    next_scalar_um <= scalar_um) {
                        fprintf(stderr,
                                "%s did not choose maximal route anchor "
                                "start=%d destination=%d side=%d speed=%d\n",
                                kind, start, destination, side, speed);
                        return -1;
                }
        }
        return 0;
}

static int check_exhaustive_matrix(track_node track[TRACK_MAX]) {
        int plans = 0;
        int exact_profiles = 0;
        int transferred_profiles = 0;
        int geometry_confirmed = 0;
        int geometry_trial = 0;
        int geometry_provisional = 0;
        int endpoint_reanchors = 0;
        int command_reanchors = 0;
        int immediate_stops = 0;

        for (int start_index = 0;
             start_index < TC2_TRACK_START_COUNT; ++start_index) {
                for (int destination_index = 0;
                     destination_index <
                             TC2_TRACK_DESTINATION_COUNT;
                     ++destination_index) {
                        for (int side_index = 0;
                             side_index <
                                     TC2_TRACK_DESTINATION_SIDE_COUNT;
                             ++side_index) {
                                track_route route;
                                if (route_for(
                                            track, start_index,
                                            destination_index,
                                            side_index, &route) < 0) {
                                        fprintf(stderr,
                                                "route unavailable start=%d "
                                                "destination=%d side=%d\n",
                                                start_index,
                                                destination_index,
                                                side_index);
                                        return -1;
                                }
                                const tc2_track_destination_side *side =
                                        Tc2TrackDestinationSide(
                                                destination_index,
                                                side_index);
                                int geometry_anchor =
                                        TrackFindNodeByName(
                                                track,
                                                side->anchor_sensor);
                                int64_t physical_baseline_um =
                                        (int64_t)route.distance_mm * 1000 +
                                        (int64_t)side->base_offset_mm *
                                                1000;

                                for (int speed = 1; speed <= 120; ++speed) {
                                        tc2_prediction_request request = {
                                                .train = 14,
                                                .start_index = start_index,
                                                .destination_index =
                                                        destination_index,
                                                .destination_side =
                                                        side_index,
                                                .speed = speed,
                                                .reversal_count =
                                                        route.reversal_count
                                        };
                                        tc2_prediction_plan plan;
                                        int status = Tc2PredictionBuildPlan(
                                                track, &route, &request,
                                                &plan);
                                        if (status != TC2_PREDICTION_OK) {
                                                fprintf(stderr,
                                                        "plan rejected "
                                                        "start=%d D%d side=%d "
                                                        "speed=%d status=%d\n",
                                                        start_index,
                                                        destination_index + 1,
                                                        side_index, speed,
                                                        status);
                                                return -1;
                                        }

                                        int exact =
                                                start_index == 0 &&
                                                destination_index == 6 &&
                                                side_index == 0 &&
                                                route.reversal_count == 0;
                                        if (plan.geometry_anchor_node !=
                                                    geometry_anchor ||
                                            plan.geometry_anchor_route_offset !=
                                                    route.node_count - 1 ||
                                            plan.geometry_base_offset_mm !=
                                                    side->base_offset_mm ||
                                            plan.geometry_evidence !=
                                                    side->geometry_evidence ||
                                            plan.exact_profile != exact ||
                                            plan.profile_evidence !=
                                                    (exact ?
                                                    TC2_PREDICTION_PROFILE_EXACT_T14_A_D7 :
                                                    TC2_PREDICTION_PROFILE_TRANSFERRED_T14_A_D7)) {
                                                fprintf(stderr,
                                                        "evidence/anchor drift "
                                                        "start=%d D%d side=%d "
                                                        "speed=%d\n",
                                                        start_index,
                                                        destination_index + 1,
                                                        side_index, speed);
                                                return -1;
                                        }

                                        int expected_velocity =
                                                Tc2MotionProvisionalVelocityUmPerTick(
                                                        speed);
                                        int expected_stop =
                                                Tc2MotionProvisionalStopDistanceUm(
                                                        speed);
                                        int expected_correction =
                                                Tc2MotionRouteEndpointCorrectionMm(
                                                        request.train,
                                                        request.start_index,
                                                        request.destination_index,
                                                        speed);
                                        int expected_braking =
                                                Tc2MotionBrakingDistanceMm(
                                                        speed);
                                        int64_t expected_corrected =
                                                physical_baseline_um +
                                                (int64_t)expected_correction *
                                                        1000;
                                        int64_t expected_endpoint =
                                                expected_corrected > 0 ?
                                                expected_corrected : 0;
                                        int64_t expected_raw_command =
                                                expected_corrected -
                                                expected_stop;
                                        int64_t expected_command =
                                                expected_raw_command > 0 ?
                                                expected_raw_command : 0;
                                        int64_t expected_ceiling =
                                                physical_baseline_um +
                                                (int64_t)expected_braking *
                                                        1000;

                                        if (plan.velocity_um_per_tick !=
                                                    expected_velocity ||
                                            plan.point_stop_distance_um !=
                                                    expected_stop ||
                                            plan.profile_correction_mm !=
                                                    expected_correction ||
                                            plan.physical_destination_distance_um !=
                                                    physical_baseline_um ||
                                            plan.corrected_endpoint_distance_um !=
                                                    expected_corrected ||
                                            plan.endpoint_distance_um !=
                                                    expected_endpoint ||
                                            plan.raw_command_distance_um !=
                                                    expected_raw_command ||
                                            plan.command_distance_um !=
                                                    expected_command ||
                                            plan.conservative_braking_distance_mm !=
                                                    expected_braking ||
                                            plan.minimum_reservation_ceiling_distance_um !=
                                                    expected_ceiling ||
                                            plan.minimum_reservation_ceiling_distance_um <
                                                    plan.physical_destination_distance_um ||
                                            plan.action !=
                                                    (expected_raw_command > 0 ?
                                                    TC2_PREDICTION_TIMED :
                                                    TC2_PREDICTION_IMMEDIATE_STOP)) {
                                                fprintf(stderr,
                                                        "scalar prediction drift "
                                                        "start=%d D%d side=%d "
                                                        "speed=%d\n",
                                                        start_index,
                                                        destination_index + 1,
                                                        side_index, speed);
                                                return -1;
                                        }

                                        if (check_scalar_location(
                                                    track, &route,
                                                    plan.endpoint_distance_um,
                                                    plan.endpoint_anchor_node,
                                                    plan.endpoint_anchor_route_offset,
                                                    plan.endpoint_offset_um,
                                                    "endpoint", start_index,
                                                    destination_index,
                                                    side_index, speed) < 0 ||
                                            check_scalar_location(
                                                    track, &route,
                                                    plan.command_distance_um,
                                                    plan.command_anchor_node,
                                                    plan.command_anchor_route_offset,
                                                    plan.command_offset_um,
                                                    "command", start_index,
                                                    destination_index,
                                                    side_index, speed) < 0) {
                                                return -1;
                                        }

                                        if (plan.endpoint_anchor_route_offset <
                                            plan.geometry_anchor_route_offset) {
                                                ++endpoint_reanchors;
                                        }
                                        if (plan.command_anchor_route_offset <
                                            plan.geometry_anchor_route_offset) {
                                                ++command_reanchors;
                                        }
                                        if (plan.action ==
                                            TC2_PREDICTION_IMMEDIATE_STOP) {
                                                ++immediate_stops;
                                        }
                                        if (plan.profile_evidence ==
                                            TC2_PREDICTION_PROFILE_EXACT_T14_A_D7) {
                                                ++exact_profiles;
                                        } else {
                                                ++transferred_profiles;
                                        }
                                        if (plan.geometry_evidence ==
                                            TC2_TRACK_GEOMETRY_CONFIRMED) {
                                                ++geometry_confirmed;
                                        } else if (plan.geometry_evidence ==
                                                   TC2_TRACK_GEOMETRY_TRIAL) {
                                                ++geometry_trial;
                                        } else {
                                                ++geometry_provisional;
                                        }
                                        ++plans;
                                }
                        }
                }
        }

        if (plans != EXPECTED_PLAN_COUNT ||
            exact_profiles != 120 ||
            transferred_profiles != EXPECTED_PLAN_COUNT - 120 ||
            geometry_confirmed != 0 ||
            geometry_trial != EXPECTED_PLAN_COUNT ||
            geometry_provisional != 0 ||
            command_reanchors <= 0) {
                fprintf(stderr,
                        "coverage partition drift plans=%d exact=%d "
                        "transferred=%d geometry=%d/%d/%d "
                        "reanchors=%d/%d immediate=%d\n",
                        plans, exact_profiles, transferred_profiles,
                        geometry_confirmed, geometry_trial,
                        geometry_provisional, endpoint_reanchors,
                        command_reanchors, immediate_stops);
                return -1;
        }
        return 0;
}

static int check_exact_regression(track_node track[TRACK_MAX]) {
        static const int speed[] = {20, 40, 60, 80, 100, 120};
        static const int correction[] =
                {0, 0, 0, 0, 0, 0};
        track_route route;
        if (route_for(track, 0, 6, 0, &route) < 0 ||
            route.reversal_count != 0) {
                fprintf(stderr, "exact regression route unavailable\n");
                return -1;
        }

        for (unsigned int i = 0;
             i < sizeof(speed) / sizeof(speed[0]); ++i) {
                tc2_prediction_request request = {
                        .train = 14,
                        .start_index = 0,
                        .destination_index = 6,
                        .destination_side = 0,
                        .speed = speed[i],
                        .reversal_count = 0
                };
                tc2_prediction_plan plan;
                if (Tc2PredictionBuildPlan(
                            track, &route, &request, &plan) !=
                            TC2_PREDICTION_OK ||
                    !plan.exact_profile ||
                    plan.profile_evidence !=
                            TC2_PREDICTION_PROFILE_EXACT_T14_A_D7 ||
                    plan.profile_correction_mm != correction[i]) {
                        fprintf(stderr,
                                "exact T14/A/D7 speed %d expected "
                                "correction %+d, got %+d\n",
                                speed[i], correction[i],
                                plan.profile_correction_mm);
                        return -1;
                }
        }
        return 0;
}

static int check_profile_geometry_orthogonality(
        track_node track[TRACK_MAX]) {
        struct orthogonal_case {
                int train;
                int destination;
                tc2_track_geometry_evidence geometry;
                tc2_prediction_profile_evidence profile;
        };
        static const struct orthogonal_case cases[] = {
                {14, 6, TC2_TRACK_GEOMETRY_TRIAL,
                 TC2_PREDICTION_PROFILE_EXACT_T14_A_D7},
                {15, 6, TC2_TRACK_GEOMETRY_TRIAL,
                 TC2_PREDICTION_PROFILE_TRANSFERRED_T14_A_D7},
                {14, 0, TC2_TRACK_GEOMETRY_TRIAL,
                 TC2_PREDICTION_PROFILE_TRANSFERRED_T14_A_D7},
                {14, 1, TC2_TRACK_GEOMETRY_TRIAL,
                 TC2_PREDICTION_PROFILE_TRANSFERRED_T14_A_D7}
        };

        for (unsigned int i = 0;
             i < sizeof(cases) / sizeof(cases[0]); ++i) {
                track_route route;
                if (route_for(
                            track, 0, cases[i].destination,
                            0, &route) < 0) {
                        return -1;
                }
                tc2_prediction_request request = {
                        .train = cases[i].train,
                        .start_index = 0,
                        .destination_index =
                                cases[i].destination,
                        .destination_side = 0,
                        .speed = 80,
                        .reversal_count = route.reversal_count
                };
                tc2_prediction_plan plan;
                if (Tc2PredictionBuildPlan(
                            track, &route, &request, &plan) !=
                            TC2_PREDICTION_OK ||
                    plan.geometry_evidence != cases[i].geometry ||
                    plan.profile_evidence != cases[i].profile) {
                        fprintf(stderr,
                                "geometry/profile evidence collapsed "
                                "case=%u\n", i);
                        return -1;
                }
        }

        if (!strings_equal(
                    Tc2PredictionProfileEvidenceName(
                            TC2_PREDICTION_PROFILE_EXACT_T14_A_D7),
                    "EXACT_T14_A_D7") ||
            !strings_equal(
                    Tc2PredictionProfileEvidenceName(
                            TC2_PREDICTION_PROFILE_TRANSFERRED_T14_A_D7),
                    "TRANSFERRED_T14_A_D7") ||
            !strings_equal(
                    Tc2PredictionProfileEvidenceName(
                            TC2_PREDICTION_PROFILE_NONE),
                    "NONE") ||
            !strings_equal(
                    Tc2PredictionActionName(TC2_PREDICTION_TIMED),
                    "TIMED") ||
            !strings_equal(
                    Tc2PredictionActionName(
                            TC2_PREDICTION_IMMEDIATE_STOP),
                    "IMMEDIATE_STOP")) {
                fprintf(stderr, "prediction evidence/action names drifted\n");
                return -1;
        }
        return 0;
}

static int expect_status(
        int actual, int expected, const char *case_name) {
        if (actual == expected) return 0;
        fprintf(stderr, "%s expected status %d, got %d\n",
                case_name, expected, actual);
        return -1;
}

static int expect_safe_failure(int actual, const char *case_name) {
        if (actual == TC2_PREDICTION_INVALID ||
            actual == TC2_PREDICTION_ROUTE_MISMATCH ||
            actual == TC2_PREDICTION_RANGE) {
                return 0;
        }
        fprintf(stderr, "%s did not fail safely, status=%d\n",
                case_name, actual);
        return -1;
}

static int check_invalid_and_mismatch(track_node track[TRACK_MAX]) {
        track_route route;
        if (route_for(track, 0, 6, 0, &route) < 0) return -1;
        tc2_prediction_request valid = {
                .train = 14,
                .start_index = 0,
                .destination_index = 6,
                .destination_side = 0,
                .speed = 80,
                .reversal_count = route.reversal_count
        };
        tc2_prediction_plan plan;

        if (expect_status(
                    Tc2PredictionBuildPlan(
                            track, &route, &valid, 0),
                    TC2_PREDICTION_INVALID, "null plan") < 0 ||
            expect_status(
                    Tc2PredictionBuildPlan(
                            0, &route, &valid, &plan),
                    TC2_PREDICTION_INVALID, "null track") < 0 ||
            expect_status(
                    Tc2PredictionBuildPlan(
                            track, 0, &valid, &plan),
                    TC2_PREDICTION_INVALID, "null route") < 0 ||
            expect_status(
                    Tc2PredictionBuildPlan(
                            track, &route, 0, &plan),
                    TC2_PREDICTION_INVALID, "null request") < 0) {
                return -1;
        }

#define EXPECT_INVALID_FIELD(field, value, label)                         \
        do {                                                              \
                tc2_prediction_request request = valid;                   \
                request.field = (value);                                  \
                if (expect_status(                                        \
                            Tc2PredictionBuildPlan(                        \
                                    track, &route, &request, &plan),       \
                            TC2_PREDICTION_INVALID, (label)) < 0) {        \
                        return -1;                                        \
                }                                                         \
        } while (0)

        EXPECT_INVALID_FIELD(train, 0, "train zero");
        EXPECT_INVALID_FIELD(train, 256, "train above protocol range");
        EXPECT_INVALID_FIELD(train, INT_MAX, "train integer boundary");
        EXPECT_INVALID_FIELD(start_index, -1, "negative start");
        EXPECT_INVALID_FIELD(
                start_index, TC2_TRACK_START_COUNT,
                "start above catalog");
        EXPECT_INVALID_FIELD(destination_index, -1, "negative destination");
        EXPECT_INVALID_FIELD(
                destination_index, TC2_TRACK_DESTINATION_COUNT,
                "destination above catalog");
        EXPECT_INVALID_FIELD(destination_side, -1, "negative side");
        EXPECT_INVALID_FIELD(
                destination_side, TC2_TRACK_DESTINATION_SIDE_COUNT,
                "side above catalog");
        EXPECT_INVALID_FIELD(speed, 0, "speed zero");
        EXPECT_INVALID_FIELD(speed, 121, "speed above command range");
        EXPECT_INVALID_FIELD(reversal_count, -1, "negative reversal count");
        EXPECT_INVALID_FIELD(
                reversal_count, INT_MAX,
                "reversal integer boundary");
#undef EXPECT_INVALID_FIELD

        track_route bad = route;
        bad.node_count = 0;
        if (expect_status(
                    Tc2PredictionBuildPlan(
                            track, &bad, &valid, &plan),
                    TC2_PREDICTION_INVALID, "empty route") < 0) {
                return -1;
        }
        bad = route;
        bad.node_count = TRACK_MAX + 1;
        if (expect_status(
                    Tc2PredictionBuildPlan(
                            track, &bad, &valid, &plan),
                    TC2_PREDICTION_INVALID, "oversize route") < 0) {
                return -1;
        }
        bad = route;
        bad.distance_mm = -1;
        if (expect_status(
                    Tc2PredictionBuildPlan(
                            track, &bad, &valid, &plan),
                    TC2_PREDICTION_INVALID, "negative route distance") < 0) {
                return -1;
        }

        tc2_prediction_request mismatch = valid;
        mismatch.start_index = 1;
        if (expect_status(
                    Tc2PredictionBuildPlan(
                            track, &route, &mismatch, &plan),
                    TC2_PREDICTION_ROUTE_MISMATCH,
                    "route start mismatch") < 0) {
                return -1;
        }
        mismatch = valid;
        mismatch.destination_side = 1;
        if (expect_status(
                    Tc2PredictionBuildPlan(
                            track, &route, &mismatch, &plan),
                    TC2_PREDICTION_ROUTE_MISMATCH,
                    "route destination side mismatch") < 0) {
                return -1;
        }
        bad = route;
        ++bad.distance_mm;
        if (expect_status(
                    Tc2PredictionBuildPlan(
                            track, &bad, &valid, &plan),
                    TC2_PREDICTION_ROUTE_MISMATCH,
                    "route distance mismatch") < 0) {
                return -1;
        }
        bad = route;
        bad.nodes[bad.node_count - 1] = bad.nodes[0];
        if (expect_status(
                    Tc2PredictionBuildPlan(
                            track, &bad, &valid, &plan),
                    TC2_PREDICTION_ROUTE_MISMATCH,
                    "route final node mismatch") < 0) {
                return -1;
        }

        /*
         * Public valid Track D routes are far below int64 limits. Forged
         * integer-boundary metadata must be rejected before any scalar
         * multiplication/addition is accepted as a plan.
         */
        bad = route;
        bad.distance_mm = INT_MAX;
        if (expect_safe_failure(
                    Tc2PredictionBuildPlan(
                            track, &bad, &valid, &plan),
                    "INT_MAX route distance") < 0) {
                return -1;
        }
        bad = route;
        bad.distance_mm = INT_MIN;
        if (expect_safe_failure(
                    Tc2PredictionBuildPlan(
                            track, &bad, &valid, &plan),
                    "INT_MIN route distance") < 0) {
                return -1;
        }
        return 0;
}

int main(void) {
        track_node track[TRACK_MAX];
        init_trackb(track);
        if (Tc2TrackCatalogValidate(track) != 0) {
                fprintf(stderr, "Track D catalog validation failed\n");
                return 1;
        }
        if (check_exhaustive_matrix(track) < 0 ||
            check_exact_regression(track) < 0 ||
            check_profile_geometry_orthogonality(track) < 0 ||
            check_invalid_and_mismatch(track) < 0) {
                return 1;
        }
        printf("tc2_prediction_model_test: PASS (%d plans)\n",
                EXPECTED_PLAN_COUNT);
        return 0;
}
