#include <stdio.h>

#include "../tc2_offline_planner.h"
#include "../tc2_track_model.h"
#include "../track_data.h"

#ifdef MODE_TC2

enum {
        REPORT_TRAIN = 14,
        REPORT_SPEED = 80
};

static int validate_all_speeds(
        track_node track[TRACK_MAX], int *validated) {
        *validated = 0;
        for (int start = 0;
             start < TC2_TRACK_START_COUNT; ++start) {
                for (int destination = 0;
                     destination <
                             TC2_TRACK_DESTINATION_COUNT;
                     ++destination) {
                        for (int speed = 1; speed <= 120; ++speed) {
                                tc2_offline_plan plan;
                                if (Tc2OfflinePlannerBuild(
                                            track, REPORT_TRAIN,
                                            start, speed,
                                            destination, &plan) !=
                                                    TC2_OFFLINE_PLAN_OK ||
                                    Tc2OfflinePlannerValidate(
                                            track, REPORT_TRAIN,
                                            start, speed,
                                            destination, &plan) !=
                                                    TC2_OFFLINE_PLAN_OK) {
                                        fprintf(
                                                stderr,
                                                "simulation failed: "
                                                "%c->d%d speed=%d "
                                                "status=%d\n",
                                                'A' + start,
                                                destination + 1,
                                                speed, plan.status);
                                        return -1;
                                }
                                ++*validated;
                        }
                }
        }
        return 0;
}

static void print_turnout_sequence(
        const tc2_offline_plan *plan) {
        int printed = 0;
        for (int index = 0;
             index < plan->projection_header.waypoint_count;
             ++index) {
                const tc2_dispatch_projection_waypoint *point =
                        &plan->waypoints[index];
                if (point->kind != TC2_ROUTE_WAYPOINT_TURNOUT ||
                    point->switch_number < 1 ||
                    (point->turnout_direction != DIR_STRAIGHT &&
                     point->turnout_direction != DIR_CURVED)) {
                        continue;
                }
                if (printed) putchar('>');
                printf("%d%c", point->switch_number,
                       Tc2TrackPhysicalTurnoutDirection(
                               point->turnout_direction));
                printed = 1;
        }
        if (!printed) fputs("NONE", stdout);
}

static void print_sensor_sequence(
        track_node track[TRACK_MAX],
        const tc2_offline_plan *plan) {
        int printed = 0;
        for (int index = 0;
             index < plan->projection_header.waypoint_count;
             ++index) {
                const tc2_dispatch_projection_waypoint *point =
                        &plan->waypoints[index];
                if (point->kind != TC2_ROUTE_WAYPOINT_SENSOR ||
                    point->graph_node < 0 ||
                    point->graph_node >= TRACK_MAX ||
                    !track[point->graph_node].name) {
                        continue;
                }
                if (printed) putchar('>');
                fputs(track[point->graph_node].name, stdout);
                printed = 1;
        }
        if (!printed) fputs("NONE", stdout);
}

int main(void) {
        track_node track[TRACK_MAX];
        int validated;
        init_trackb(track);

        if (Tc2TrackCatalogValidate(track) < 0 ||
            validate_all_speeds(track, &validated) < 0 ||
            validated !=
                    TC2_TRACK_START_COUNT *
                    TC2_TRACK_DESTINATION_COUNT * 120) {
                return 1;
        }

        puts("start,destination,simulated_speed,selected_side,"
             "approach_anchor,operator_offset_mm,"
             "route_to_anchor_mm,operator_distance_mm,"
             "corrected_endpoint_mm,brake_command_mm,"
             "point_stop_mm,velocity_um_per_tick,"
             "expected_sensor_sequence,expected_turnout_sequence,"
             "simulation_status,"
             "physical_status");
        for (int start = 0;
             start < TC2_TRACK_START_COUNT; ++start) {
                for (int destination = 0;
                     destination <
                             TC2_TRACK_DESTINATION_COUNT;
                     ++destination) {
                        tc2_offline_plan plan;
                        if (Tc2OfflinePlannerBuild(
                                    track, REPORT_TRAIN,
                                    start, REPORT_SPEED,
                                    destination, &plan) !=
                                            TC2_OFFLINE_PLAN_OK) {
                                return 1;
                        }
                        const tc2_track_destination_side *side =
                                Tc2TrackDestinationSide(
                                        destination,
                                        plan.selected_side);
                        if (!side) return 1;

                        printf(
                                "%c,d%d,%d,%d,%s,%d,%d,%lld,"
                                "%lld,%lld,%lld,%d,",
                                'A' + start, destination + 1,
                                REPORT_SPEED, plan.selected_side,
                                side->anchor_sensor,
                                side->base_offset_mm,
                                (int)(plan.prediction_plan
                                                .physical_destination_distance_um /
                                        1000) -
                                        side->base_offset_mm,
                                (long long)(plan.prediction_plan
                                        .physical_destination_distance_um /
                                        1000),
                                (long long)(plan.prediction_plan
                                        .corrected_endpoint_distance_um /
                                        1000),
                                (long long)(plan.prediction_plan
                                        .command_distance_um / 1000),
                                (long long)(plan.prediction_plan
                                        .point_stop_distance_um / 1000),
                                plan.prediction_plan
                                        .velocity_um_per_tick);
                        print_sensor_sequence(track, &plan);
                        putchar(',');
                        print_turnout_sequence(&plan);
                        puts(",PASS,PROVISIONAL_NEEDS_PHYSICAL_RUN");
                }
        }
        fprintf(stderr,
                "PASS: %d dispatch simulations; "
                "48 speed-80 calibration rows emitted\n",
                validated);
        return 0;
}

#else

int main(void) {
        return 0;
}

#endif
