#include <stdio.h>
#include "track_data.h"
#include "track_route.h"

static const char *const starts[] = {
        "EN5", "EN4", "EN7", "EN10", "EN9", "EN3"
};

static const char *const destinations[] = {
        "A12", "B15", "C12", "C9", "E5", "E14", "D7", "C15"
};

int main(void) {
        track_node track[TRACK_MAX];
        track_route route;
        int checked = 0;
        int minimum_watchdog_margin_ticks[2] = {1000000, 1000000};

        init_trackb(track);
        for (unsigned int source = 0;
             source < sizeof(starts) / sizeof(starts[0]); ++source) {
                int from = TrackFindNodeByName(track, starts[source]);
                if (from < 0 || track[from].type != NODE_ENTER) {
                        fprintf(stderr, "invalid start %s\n", starts[source]);
                        return 1;
                }
                for (unsigned int destination = 0;
                     destination <
                             sizeof(destinations) / sizeof(destinations[0]);
                     ++destination) {
                        int to = TrackFindNodeByName(
                                track, destinations[destination]);
                        if (to < 0 ||
                            TrackFindShortestRoute(track, from, to,
                                                   &route) < 0 ||
                            route.node_count < 2 ||
                            route.reversal_count != 0) {
                                fprintf(stderr, "no forward route %s -> %s\n",
                                        starts[source],
                                        destinations[destination]);
                                return 1;
                        }
                        int sensors_before_target = 0;
                        for (int node = 0; node + 1 < route.node_count; ++node) {
                                if (track[route.nodes[node]].type ==
                                    NODE_SENSOR) {
                                        ++sensors_before_target;
                                }
                        }
                        if (sensors_before_target == 0) {
                                fprintf(stderr,
                                        "no progress sensor before target %s -> %s\n",
                                        starts[source],
                                        destinations[destination]);
                                return 1;
                        }
                        const int representative_speeds[] = {8, 120};
                        for (unsigned int speed_index = 0;
                             speed_index <
                                     sizeof(representative_speeds) /
                                             sizeof(representative_speeds[0]);
                             ++speed_index) {
                                track_route guarded = route;
                                track_turnout_plan plan;
                                int speed = representative_speeds[speed_index];
                                int velocity_index =
                                        (speed * 14 + 60) / 120;
                                static const int
                                        seed_velocity_tenths[15] = {
                                        0, 12, 18, 24, 31,
                                        38, 45, 52, 59, 66,
                                        73, 80, 87, 94, 101
                                };
                                if (velocity_index < 1) velocity_index = 1;
                                if (velocity_index > 14) velocity_index = 14;
                                int watchdog_margin_ticks =
                                        speed <= 20 ? 100 : 60;
                                int braking_guard =
                                        250 + speed * 12;
                                int desired_guard_distance =
                                        250 + speed * 12 +
                                        (seed_velocity_tenths[
                                                 velocity_index] *
                                                 watchdog_margin_ticks +
                                         9) / 10;
                                int represented_guard =
                                        TrackExtendRouteAhead(
                                                track, &guarded,
                                                desired_guard_distance);
                                if (represented_guard < braking_guard) {
                                        fprintf(stderr,
                                                "cannot extend braking guard speed=%d %s -> %s\n",
                                                speed, starts[source],
                                                destinations[destination]);
                                        return 1;
                                }
                                int represented_margin_ticks =
                                        (represented_guard -
                                         braking_guard) * 10 /
                                        seed_velocity_tenths[
                                                velocity_index];
                                if (represented_margin_ticks <
                                    minimum_watchdog_margin_ticks[
                                            speed_index]) {
                                        minimum_watchdog_margin_ticks[
                                                speed_index] =
                                                represented_margin_ticks;
                                }
                                if (TrackBuildTurnoutPlan(
                                            track, &guarded, &plan) < 0) {
                                        fprintf(stderr,
                                                "invalid braking turnout plan speed=%d %s -> %s\n",
                                                speed, starts[source],
                                                destinations[destination]);
                                        return 1;
                                }
                                for (int first = 0;
                                     first < plan.action_count; ++first) {
                                        for (int second = first + 1;
                                             second < plan.action_count;
                                             ++second) {
                                                if (plan.actions[first]
                                                                    .switch_number ==
                                                            plan.actions[second]
                                                                    .switch_number &&
                                                    plan.actions[first]
                                                                    .direction !=
                                                            plan.actions[second]
                                                                    .direction) {
                                                        fprintf(
                                                                stderr,
                                                                "contradictory guard turnout speed=%d %s -> %s\n",
                                                                speed,
                                                                starts[source],
                                                                destinations[
                                                                        destination]);
                                                        return 1;
                                                }
                                        }
                                }
                        }
                        ++checked;
                }
        }

        printf("validated %d A-F to d1-d8 forward routes with speed 8/120 guards; min watchdog margins=%d/%d ticks\n",
               checked, minimum_watchdog_margin_ticks[0],
               minimum_watchdog_margin_ticks[1]);
        return checked == 48 ? 0 : 1;
}
