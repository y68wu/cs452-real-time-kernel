#include <stdio.h>
#include <string.h>

#include "../tc2_offline_planner.h"
#include "../tc2_track_model.h"
#include "../track_data.h"

#ifdef MODE_TC2

enum {
        ORACLE_SEQUENCE_CAPACITY = 96
};

static const int oracle_trains[] = {18, 14, 15, 17};

/*
 * Independent Track-D physical oracle.
 *
 * These strings are deliberately not generated from track_data.c.  They
 * describe the operator-confirmed physical turnout settings for each
 * shortest A-F -> d1-d8 route.  A graph, direction mapping, route selector,
 * or projection change must therefore agree with this table rather than
 * silently updating its own expected result.
 */
static const char *const physical_turnout_oracle
        [TC2_TRACK_START_COUNT][TC2_TRACK_DESTINATION_COUNT] = {
        {
                "8S>7C>18S>3C>2S>1S",
                "8C>17S",
                "8C>17S>14C",
                "8C>17S",
                "8C>17C>156S>155C",
                "8C",
                "8C>17C>156S>155C",
                "8S>7S"
        },
        {
                "8S>7C>18S>3C>2S>1S",
                "8C>17S",
                "8C>17S>14C",
                "8C>17S",
                "8C>17C>156S>155C",
                "8C",
                "8C>17C>156S>155C",
                "8S>7S"
        },
        {
                "5C>9S>11S>12C>4S",
                "5C>9C>10S",
                "5C>9C>10S",
                "5C>9C>10S>15C",
                "5C>9C",
                "5C>9C>10C>154C",
                "5C>9C>10C>154C",
                "5C>9C>10S>15S>6S"
        },
        {
                "5C>9S>11S>12C>4S",
                "5C>9C>10S",
                "5C>9C>10S",
                "5C>9C>10S>15C",
                "5C>9C",
                "5C>9C>10C>154C",
                "5C>9C>10C>154C",
                "5C>9C>10S>15S>6S"
        },
        {
                "5C>9S>11S>12C>4S",
                "5C>9C>10S",
                "5C>9C>10S",
                "5C>9C>10S>15C",
                "5C>9C",
                "5C>9C>10C>154C",
                "5C>9C>10C>154C",
                "5C>9C>10S>15S>6S"
        },
        {
                "18S>3C>2S>1S",
                "18C",
                "18C>14C",
                "18C>14C>13S>8C>17S",
                "18C>14C>13S",
                "18C>14C>13C>154C",
                "18C>14C>13S",
                "18C>14C>13S>8S>7S"
        }
};

static int text_starts_with(const char *text, const char *prefix) {
        if (!text || !prefix) return 0;
        while (*prefix) {
                if (*text++ != *prefix++) return 0;
        }
        return 1;
}

static int append_turnout(
        char sequence[ORACLE_SEQUENCE_CAPACITY], size_t *length,
        int switch_number, char direction) {
        if (!sequence || !length ||
            *length >= ORACLE_SEQUENCE_CAPACITY ||
            switch_number < 1 ||
            (direction != 'S' && direction != 'C')) {
                return -1;
        }
        int written = snprintf(
                sequence + *length,
                ORACLE_SEQUENCE_CAPACITY - *length,
                "%s%d%c", *length == 0 ? "" : ">",
                switch_number, direction);
        if (written < 0 ||
            (size_t)written >=
                    ORACLE_SEQUENCE_CAPACITY - *length) {
                return -1;
        }
        *length += (size_t)written;
        return 0;
}

static int projected_turnout_sequence(
        const tc2_offline_plan *plan,
        char sequence[ORACLE_SEQUENCE_CAPACITY]) {
        if (!plan || !sequence || !plan->valid ||
            plan->projection_header.waypoint_count < 1 ||
            plan->projection_header.waypoint_count >
                    TC2_OFFLINE_PLANNER_MAX_WAYPOINTS) {
                return -1;
        }

        sequence[0] = '\0';
        size_t length = 0;
        for (int index = 0;
             index < plan->projection_header.waypoint_count; ++index) {
                const tc2_dispatch_projection_waypoint *point =
                        &plan->waypoints[index];
                if (point->kind != TC2_ROUTE_WAYPOINT_TURNOUT) {
                        continue;
                }
                char direction =
                        Tc2TrackPhysicalTurnoutDirection(
                                point->turnout_direction);
                if (append_turnout(
                            sequence, &length,
                            point->switch_number,
                            direction) < 0) {
                        return -1;
                }
        }
        return length > 0 ? 0 : -1;
}

static int oracle_prefixes_are_physical(void) {
        for (int destination = 0;
             destination < TC2_TRACK_DESTINATION_COUNT;
             ++destination) {
                if (!text_starts_with(
                            physical_turnout_oracle[2][destination],
                            "5C")) {
                        return 0;
                }
        }
        if (!text_starts_with(
                    physical_turnout_oracle[5][0], "18S")) {
                return 0;
        }
        for (int destination = 1;
             destination < TC2_TRACK_DESTINATION_COUNT;
             ++destination) {
                if (!text_starts_with(
                            physical_turnout_oracle[5][destination],
                            "18C")) {
                        return 0;
                }
        }
        return 1;
}

int main(void) {
        static const int representative_speed[] = {20, 80, 120};
        track_node track[TRACK_MAX];
        int checked = 0;

        init_trackb(track);
        if (Tc2TrackCatalogValidate(track) < 0 ||
            !oracle_prefixes_are_physical()) {
                fprintf(stderr,
                        "tc2_physical_turnout_oracle_test: "
                        "catalog or independent oracle is invalid\n");
                return 1;
        }

        for (int start = 0;
             start < TC2_TRACK_START_COUNT; ++start) {
                for (int destination = 0;
                     destination < TC2_TRACK_DESTINATION_COUNT;
                     ++destination) {
                        const char *expected =
                                physical_turnout_oracle
                                        [start][destination];
                        if (!expected || !*expected) {
                                fprintf(stderr,
                                        "missing physical oracle "
                                        "%c->d%d\n",
                                        'A' + start,
                                        destination + 1);
                                return 1;
                        }
                        for (size_t train_index = 0;
                             train_index <
                                     sizeof(oracle_trains) /
                                     sizeof(oracle_trains[0]);
                             ++train_index) {
                                for (size_t speed_index = 0;
                                     speed_index <
                                             sizeof(representative_speed) /
                                             sizeof(representative_speed[0]);
                                     ++speed_index) {
                                        int train =
                                                oracle_trains[train_index];
                                        int speed = representative_speed[
                                                speed_index];
                                        tc2_offline_plan plan;
                                        char actual[
                                                ORACLE_SEQUENCE_CAPACITY];
                                        int status = Tc2OfflinePlannerBuild(
                                                track, train,
                                                start, speed,
                                                destination, &plan);
                                        if (status != TC2_OFFLINE_PLAN_OK ||
                                            plan.selected_motion_route
                                                            .reversal_count !=
                                                    0 ||
                                            projected_turnout_sequence(
                                                    &plan, actual) < 0 ||
                                            strcmp(actual, expected) != 0) {
                                                fprintf(
                                                        stderr,
                                                        "physical turnout mismatch "
                                                        "T%d %c->d%d speed=%d "
                                                        "status=%d expected=%s "
                                                        "actual=%s\n",
                                                        train,
                                                        'A' + start,
                                                        destination + 1,
                                                        speed, status,
                                                        expected,
                                                        status ==
                                                                        TC2_OFFLINE_PLAN_OK ?
                                                                actual :
                                                                "<none>");
                                                return 1;
                                        }

                                        if ((start == 2 &&
                                             !text_starts_with(
                                                     actual, "5C")) ||
                                            (start == 5 &&
                                             destination == 0 &&
                                             !text_starts_with(
                                                     actual, "18S")) ||
                                            (start == 5 &&
                                             destination > 0 &&
                                             !text_starts_with(
                                                     actual, "18C"))) {
                                                fprintf(
                                                        stderr,
                                                        "critical physical turnout "
                                                        "prefix missing "
                                                        "%c->d%d: %s\n",
                                                        'A' + start,
                                                        destination + 1,
                                                        actual);
                                                return 1;
                                        }
                                        ++checked;
                                }
                        }
                }
        }

        if (checked !=
                    TC2_TRACK_START_COUNT *
                    TC2_TRACK_DESTINATION_COUNT *
                    (int)(sizeof(oracle_trains) /
                          sizeof(oracle_trains[0])) *
                    (int)(sizeof(representative_speed) /
                          sizeof(representative_speed[0]))) {
                fprintf(stderr,
                        "physical turnout oracle cardinality mismatch: "
                        "%d\n", checked);
                return 1;
        }

        printf("tc2_physical_turnout_oracle_test: PASS "
               "(4 trains, 48 physical routes, 3 speeds, "
               "C=5C, F/d1=18S, F/d2-d8=18C)\n");
        return 0;
}

#else

int main(void) {
        return 0;
}

#endif
