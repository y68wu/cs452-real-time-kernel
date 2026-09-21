#include <stdio.h>

#include "track_data.h"
#include "track_reservation.h"

static track_route footprint(const int *nodes, int count) {
        track_route route;
        route.node_count = count;
        route.distance_mm = 0;
        route.optimization_cost_mm = 0;
        route.reversal_count = 0;
        for (int i = 0; i < TRACK_MAX; ++i) route.nodes[i] = -1;
        for (int i = 0; i < count; ++i) route.nodes[i] = nodes[i];
        return route;
}

static int owner_pair_is(const track_reservation_table *table,
                         int sensor, int train) {
        int reverse = sensor ^ 1;
        return table->owner_by_node[sensor] == train &&
                table->owner_by_node[reverse] == train;
}

static int tables_match(const track_reservation_table *left,
                        const track_reservation_table *right) {
        for (int node = 0; node < TRACK_MAX; ++node) {
                if (left->owner_by_node[node] != right->owner_by_node[node]) {
                        return 0;
                }
        }
        for (int train = 0; train < 256; ++train) {
                if (left->destination_by_train[train] !=
                            right->destination_by_train[train] ||
                    left->generation_by_train[train] !=
                            right->generation_by_train[train]) {
                        return 0;
                }
        }
        return 1;
}

static int check_rolling_authority_windows(track_node *track) {
        track_reservation_table table;
        track_reservation_conflict conflict;
        unsigned int generation = 0;
        const int initial_nodes[] = {0, 2};
        const int extended_nodes[] = {0, 2, 4};
        const int blocker_nodes[] = {6};
        const int conflicting_nodes[] = {2, 4, 6};
        const int shrunk_nodes[] = {4};
        const int legacy_nodes[] = {8};
        track_route initial = footprint(initial_nodes, 2);
        track_route extended = footprint(extended_nodes, 3);
        track_route blocker = footprint(blocker_nodes, 1);
        track_route conflicting = footprint(conflicting_nodes, 3);
        track_route shrunk = footprint(shrunk_nodes, 1);
        track_route legacy = footprint(legacy_nodes, 1);
        const int plan_destination = 20;

        TrackReservationInit(&table);
        if (TrackReservationTryWindowToGeneration(
                    &table, track, &initial, 77, plan_destination,
                    &conflict, &generation) != 0 ||
            generation == 0 ||
            table.destination_by_train[77] != plan_destination ||
            !owner_pair_is(&table, 0, 77) ||
            !owner_pair_is(&table, 2, 77) ||
            table.owner_by_node[plan_destination] != 0) {
                fprintf(stderr,
                        "initial destination-independent window failed\n");
                return -1;
        }
        unsigned int plan_generation = generation;

        /*
         * The old API remains strict: a destination absent from the supplied
         * route is invalid and must not create a plan.
         */
        track_reservation_table before_legacy = table;
        generation = 1234;
        conflict.node_index = 1234;
        conflict.owner_train = 1234;
        if (TrackReservationTryRouteToGeneration(
                    &table, track, &legacy, 79, plan_destination,
                    &conflict, &generation) != -1 ||
            generation != 0 || conflict.node_index != -1 ||
            conflict.owner_train != 0 ||
            !tables_match(&table, &before_legacy)) {
                fprintf(stderr,
                        "legacy route API accepted an absent destination\n");
                return -1;
        }

        if (TrackReservationUpdateWindow(
                    &table, track, &extended, 77, plan_destination,
                    plan_generation, &conflict, &generation) != 0 ||
            generation != plan_generation ||
            !owner_pair_is(&table, 0, 77) ||
            !owner_pair_is(&table, 4, 77) ||
            table.owner_by_node[plan_destination] != 0) {
                fprintf(stderr,
                        "rolling window extension changed plan identity\n");
                return -1;
        }

        if (TrackReservationTryRouteTo(
                    &table, track, &blocker, 78, 6, &conflict) != 0) {
                fprintf(stderr, "rolling window blocker setup failed\n");
                return -1;
        }
        track_reservation_table before_conflict = table;
        generation = 1234;
        if (TrackReservationTryWindowToGeneration(
                    &table, track, &conflicting, 77,
                    plan_destination + 2, &conflict,
                    &generation) != 1 ||
            conflict.node_index != 6 ||
            conflict.owner_train != 78 ||
            generation != 0 ||
            !tables_match(&table, &before_conflict)) {
                fprintf(stderr,
                        "conflicting initial window replaced an old plan\n");
                return -1;
        }

        generation = 1234;
        if (TrackReservationUpdateWindow(
                    &table, track, &conflicting, 77,
                    plan_destination, plan_generation,
                    &conflict, &generation) != 1 ||
            conflict.node_index != 6 ||
            conflict.owner_train != 78 ||
            generation != 0 ||
            !tables_match(&table, &before_conflict)) {
                fprintf(stderr,
                        "conflicting rolling extension was not atomic\n");
                return -1;
        }

        generation = 1234;
        conflict.node_index = 1234;
        conflict.owner_train = 1234;
        if (TrackReservationUpdateWindow(
                    &table, track, &shrunk, 77,
                    plan_destination, plan_generation + 1,
                    &conflict, &generation) != -1 ||
            generation != 0 || conflict.node_index != -1 ||
            conflict.owner_train != 0 ||
            !tables_match(&table, &before_conflict)) {
                fprintf(stderr,
                        "stale rolling update changed reservation state\n");
                return -1;
        }

        if (TrackReservationUpdateWindow(
                    &table, track, &shrunk, 77,
                    plan_destination + 2, plan_generation,
                    &conflict, &generation) != -1 ||
            !tables_match(&table, &before_conflict)) {
                fprintf(stderr,
                        "wrong rolling destination changed reservation state\n");
                return -1;
        }

        if (TrackReservationUpdateWindow(
                    &table, track, &shrunk, 77,
                    plan_destination, plan_generation,
                    &conflict, &generation) != 0 ||
            generation != plan_generation ||
            table.destination_by_train[77] != plan_destination ||
            table.owner_by_node[0] != 0 ||
            table.owner_by_node[1] != 0 ||
            table.owner_by_node[2] != 0 ||
            table.owner_by_node[3] != 0 ||
            !owner_pair_is(&table, 4, 77)) {
                fprintf(stderr,
                        "rolling window shrink did not release behind\n");
                return -1;
        }
        return 0;
}

static int check_atomic_expected_window_replacement(track_node *track) {
        track_reservation_table table;
        track_reservation_conflict conflict;
        unsigned int generation = 0;
        const int old_nodes[] = {0, 2};
        const int peer_nodes[] = {8};
        const int replacement_nodes[] = {4, 6};
        const int conflicting_nodes[] = {4, 8};
        track_route old_window = footprint(old_nodes, 2);
        track_route peer = footprint(peer_nodes, 1);
        track_route replacement_window = footprint(replacement_nodes, 2);
        track_route conflicting_window = footprint(conflicting_nodes, 2);
        const int old_destination = 20;
        const int new_destination = 22;

        TrackReservationInit(&table);
        if (TrackReservationTryWindowToGeneration(
                    &table, track, &old_window, 77, old_destination,
                    &conflict, &generation) != 0 ||
            generation == 0 ||
            !owner_pair_is(&table, 0, 77) ||
            !owner_pair_is(&table, 2, 77)) {
                fprintf(stderr,
                        "expected window replacement old-plan setup failed\n");
                return -1;
        }
        unsigned int old_generation = generation;

        if (TrackReservationTryRouteToGeneration(
                    &table, track, &peer, 78, 8,
                    &conflict, &generation) != 0 ||
            generation == 0 || !owner_pair_is(&table, 8, 78)) {
                fprintf(stderr,
                        "expected window replacement peer setup failed\n");
                return -1;
        }
        unsigned int peer_generation = generation;
        track_reservation_table before_replacement = table;

        generation = 1234;
        conflict.node_index = 1234;
        conflict.owner_train = 1234;
        if (TrackReservationReplaceWindowExpected(
                    &table, track, &replacement_window, 77,
                    old_destination, old_generation + 1, new_destination,
                    &conflict, &generation) != -1 ||
            generation != 0 || conflict.node_index != -1 ||
            conflict.owner_train != 0 ||
            !tables_match(&table, &before_replacement)) {
                fprintf(stderr,
                        "stale generation changed expected window state\n");
                return -1;
        }

        generation = 1234;
        conflict.node_index = 1234;
        conflict.owner_train = 1234;
        if (TrackReservationReplaceWindowExpected(
                    &table, track, &replacement_window, 77,
                    old_destination + 2, old_generation, new_destination,
                    &conflict, &generation) != -1 ||
            generation != 0 || conflict.node_index != -1 ||
            conflict.owner_train != 0 ||
            !tables_match(&table, &before_replacement)) {
                fprintf(stderr,
                        "stale destination changed expected window state\n");
                return -1;
        }

        generation = 1234;
        conflict.node_index = 1234;
        conflict.owner_train = 1234;
        if (TrackReservationReplaceWindowExpected(
                    &table, track, &conflicting_window, 77,
                    old_destination, old_generation, new_destination,
                    &conflict, &generation) != 1 ||
            generation != 0 || conflict.node_index != 8 ||
            conflict.owner_train != 78 ||
            !tables_match(&table, &before_replacement)) {
                fprintf(stderr,
                        "peer conflict changed expected window state\n");
                return -1;
        }

        track_reservation_table expected = before_replacement;
        for (int node = 0; node < TRACK_MAX; ++node) {
                if (expected.owner_by_node[node] == 77) {
                        expected.owner_by_node[node] = 0;
                }
        }
        expected.owner_by_node[4] = 77;
        expected.owner_by_node[5] = 77;
        expected.owner_by_node[6] = 77;
        expected.owner_by_node[7] = 77;
        expected.destination_by_train[77] = new_destination;
        expected.generation_by_train[77] = old_generation + 1;

        generation = 0;
        if (TrackReservationReplaceWindowExpected(
                    &table, track, &replacement_window, 77,
                    old_destination, old_generation, new_destination,
                    &conflict, &generation) != 0 ||
            generation != old_generation + 1 ||
            table.destination_by_train[77] != new_destination ||
            table.owner_by_node[new_destination] != 0 ||
            table.destination_by_train[78] != 8 ||
            table.generation_by_train[78] != peer_generation ||
            !owner_pair_is(&table, 8, 78) ||
            !tables_match(&table, &expected)) {
                fprintf(stderr,
                        "partial expected window replacement was not atomic\n");
                return -1;
        }
        return 0;
}

int main(void) {
        track_node track[TRACK_MAX];
        track_reservation_table table;
        track_reservation_conflict conflict;
        unsigned int generation = 0;
        const int initial_nodes[] = {0, 2, 4, 6};
        const int window_nodes[] = {2, 4, 6};
        const int blocker_nodes[] = {8};
        const int conflicting_nodes[] = {4, 6, 8};
        const int replacement_nodes[] = {4, 6, 10};
        const int next_replacement_nodes[] = {10, 12};
        const int conflicting_replacement_nodes[] = {6, 8, 12};
        const int new_plan_nodes[] = {14};
        track_route initial = footprint(initial_nodes, 4);
        track_route window = footprint(window_nodes, 3);
        track_route blocker = footprint(blocker_nodes, 1);
        track_route conflicting = footprint(conflicting_nodes, 3);
        track_route replacement = footprint(replacement_nodes, 3);
        track_route next_replacement = footprint(next_replacement_nodes, 2);
        track_route conflicting_replacement =
                footprint(conflicting_replacement_nodes, 3);
        track_route new_plan = footprint(new_plan_nodes, 1);

        init_trackb(track);
        TrackReservationInit(&table);
        if (check_rolling_authority_windows(track) < 0) {
                return 1;
        }
        if (check_atomic_expected_window_replacement(track) < 0) {
                return 1;
        }

        if (TrackReservationTryRouteToGeneration(
                    &table, track, &initial, 77, 6, &conflict,
                    &generation) != 0 ||
            generation == 0 || !owner_pair_is(&table, 0, 77) ||
            !owner_pair_is(&table, 6, 77)) {
                fprintf(stderr, "initial reservation failed\n");
                return 1;
        }
        unsigned int plan_generation = generation;

        if (TrackReservationUpdateFootprint(
                    &table, track, &window, 77, 6, plan_generation,
                    &conflict, &generation) != 0 ||
            generation != plan_generation ||
            table.owner_by_node[0] != 0 || table.owner_by_node[1] != 0 ||
            !owner_pair_is(&table, 2, 77) ||
            !owner_pair_is(&table, 6, 77)) {
                fprintf(stderr, "sliding footprint did not release behind\n");
                return 1;
        }

        if (TrackReservationTryRouteTo(
                    &table, track, &blocker, 78, 8, &conflict) != 0 ||
            TrackReservationUpdateFootprint(
                    &table, track, &conflicting, 77, 6, plan_generation,
                    &conflict, &generation) != 1 ||
            conflict.owner_train != 78 || conflict.node_index != 8 ||
            !owner_pair_is(&table, 2, 77) ||
            !owner_pair_is(&table, 6, 77)) {
                fprintf(stderr, "conflicting update was not atomic\n");
                return 1;
        }

        if (TrackReservationUpdateFootprint(
                    &table, track, &window, 77, 6,
                    plan_generation + 1, &conflict, &generation) != -1 ||
            !owner_pair_is(&table, 2, 77) ||
            !owner_pair_is(&table, 6, 77)) {
                fprintf(stderr, "stale generation changed the footprint\n");
                return 1;
        }

        if (TrackReservationReplacePlanExpected(
                    &table, track, &replacement, 77, 6, plan_generation,
                    10, &conflict, &generation) != 0 ||
            generation != plan_generation + 1 ||
            table.destination_by_train[77] != 10 ||
            table.owner_by_node[2] != 0 || table.owner_by_node[3] != 0 ||
            !owner_pair_is(&table, 4, 77) ||
            !owner_pair_is(&table, 6, 77) ||
            !owner_pair_is(&table, 10, 77)) {
                fprintf(stderr, "expected replacement did not commit atomically\n");
                return 1;
        }
        unsigned int replacement_generation = generation;
        track_reservation_table before_failed_replacement = table;

        generation = 1234;
        conflict.node_index = 1234;
        conflict.owner_train = 1234;
        if (TrackReservationReplacePlanExpected(
                    &table, track, &next_replacement, 77, 10,
                    plan_generation, 12, &conflict, &generation) != -1 ||
            generation != 0 || conflict.node_index != -1 ||
            conflict.owner_train != 0 ||
            !tables_match(&table, &before_failed_replacement)) {
                fprintf(stderr, "stale replacement changed reservation state\n");
                return 1;
        }

        if (TrackReservationReplacePlanExpected(
                    &table, track, &next_replacement, 77, 6,
                    replacement_generation, 12,
                    &conflict, &generation) != -1 ||
            !tables_match(&table, &before_failed_replacement)) {
                fprintf(stderr, "wrong old destination changed reservation state\n");
                return 1;
        }

        if (TrackReservationReplacePlanExpected(
                    &table, track, &next_replacement, 77, 10,
                    replacement_generation, 14,
                    &conflict, &generation) != -1 ||
            !tables_match(&table, &before_failed_replacement)) {
                fprintf(stderr, "invalid new destination changed reservation state\n");
                return 1;
        }

        if (TrackReservationReplacePlanExpected(
                    &table, track, &conflicting_replacement, 77, 10,
                    replacement_generation, 12,
                    &conflict, &generation) != 1 ||
            conflict.node_index != 8 || conflict.owner_train != 78 ||
            generation != 0 ||
            !tables_match(&table, &before_failed_replacement)) {
                fprintf(stderr, "conflicting replacement was not atomic\n");
                return 1;
        }

        if (TrackReservationTryRouteToGeneration(
                    &table, track, &new_plan, 77, 14, &conflict,
                    &generation) != 0 ||
            generation == replacement_generation ||
            table.owner_by_node[4] != 0 || table.owner_by_node[5] != 0 ||
            table.owner_by_node[10] != 0 || table.owner_by_node[11] != 0 ||
            !owner_pair_is(&table, 14, 77) ||
            table.destination_by_train[77] != 14) {
                fprintf(stderr, "new plan did not replace the old footprint\n");
                return 1;
        }

        puts("validated atomic route/window footprints and destination+generation CAS plan replacement");
        return 0;
}
