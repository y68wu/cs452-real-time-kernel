#include "track_reservation.h"

static int reverse_index(track_node *track, int node_index) {
        track_node *reverse = track[node_index].reverse;
        if (!reverse) return -1;

        int index = (int)(reverse - track);
        if (index < 0 || index >= TRACK_MAX) return -1;
        return index;
}

static int route_is_valid(const track_route *route) {
        if (!route || route->node_count < 1 || route->node_count > TRACK_MAX) return 0;
        for (int i = 0; i < route->node_count; ++i) {
                if (route->nodes[i] < 0 || route->nodes[i] >= TRACK_MAX) return 0;
        }
        return 1;
}

static int route_contains_node(const track_route *route, int node_index) {
        for (int i = 0; i < route->node_count; ++i) {
                if (route->nodes[i] == node_index) return 1;
        }
        return 0;
}

static void clear_train_owners(track_reservation_table *table, int train) {
        // 只清除这个train的ownership；destination由caller在同一个atomic update里处理。
        for (int i = 0; i < TRACK_MAX; ++i) {
                if (table->owner_by_node[i] == train) table->owner_by_node[i] = 0;
        }
}

static void advance_train_generation(track_reservation_table *table, int train) {
        // Generation 0保留给no plan；wrap后从1继续。
        table->generation_by_train[train]++;
        if (table->generation_by_train[train] == 0) {
                table->generation_by_train[train] = 1;
        }
}

void TrackReservationInit(track_reservation_table *table) {
        if (!table) return;
        for (int i = 0; i < TRACK_MAX; ++i) table->owner_by_node[i] = 0;
        for (int train = 0; train < 256; ++train) {
                table->destination_by_train[train] = -1;
                table->generation_by_train[train] = 0;
        }
}

static int try_route_to(track_reservation_table *table, track_node *track,
                        const track_route *route, int train,
                        int destination_node,
                        int require_destination_in_route,
                        int require_expected_plan,
                        int expected_destination_node,
                        unsigned int expected_generation,
                        int advance_generation,
                        track_reservation_conflict *conflict,
                        unsigned int *generation) {
        if (conflict) {
                conflict->node_index = -1;
                conflict->owner_train = 0;
        }
        if (generation) *generation = 0;
        if (!table || !track || !route_is_valid(route) || train < 1 || train > 255 ||
            destination_node < 0 || destination_node >= TRACK_MAX ||
            (require_destination_in_route &&
             !route_contains_node(route, destination_node))) {
                return -1;
        }
        if (require_expected_plan &&
            (expected_generation == 0 ||
             expected_destination_node < 0 ||
             expected_destination_node >= TRACK_MAX ||
             table->generation_by_train[train] != expected_generation ||
             table->destination_by_train[train] != expected_destination_node)) {
                return -1;
        }

        // First pass只检查conflict，保证reservation update是atomic all-or-nothing。
        for (int i = 0; i < route->node_count; ++i) {
                int node = route->nodes[i];
                int reverse = reverse_index(track, node);
                int owner = table->owner_by_node[node];
                if (owner != 0 && owner != train) {
                        if (conflict) {
                                conflict->node_index = node;
                                conflict->owner_train = owner;
                        }
                        return 1;
                }
                if (reverse >= 0) {
                        owner = table->owner_by_node[reverse];
                        if (owner != 0 && owner != train) {
                                if (conflict) {
                                        conflict->node_index = reverse;
                                        conflict->owner_train = owner;
                                }
                                return 1;
                        }
                }
        }

        // Validation成功后先移除old plan，再commit new plan，形成atomic route replacement。
        clear_train_owners(table, train);

        // Second pass commits node pairs，保守地reserve整条route作为collision baseline。
        for (int i = 0; i < route->node_count; ++i) {
                int node = route->nodes[i];
                int reverse = reverse_index(track, node);
                table->owner_by_node[node] = train;
                if (reverse >= 0) table->owner_by_node[reverse] = train;
        }
        table->destination_by_train[train] = destination_node;
        if (advance_generation) {
                // 新plan推进generation，旧sensor evidence不能匹配新的行程。
                advance_train_generation(table, train);
        }
        if (generation) *generation = table->generation_by_train[train];
        return 0;
}

int TrackReservationTryRouteToGeneration(
        track_reservation_table *table, track_node *track,
        const track_route *route, int train, int destination_node,
        track_reservation_conflict *conflict, unsigned int *generation) {
        return try_route_to(table, track, route, train, destination_node,
                            1, 0, -1, 0, 1, conflict, generation);
}

int TrackReservationTryWindowToGeneration(
        track_reservation_table *table, track_node *track,
        const track_route *footprint, int train,
        int plan_destination_node,
        track_reservation_conflict *conflict, unsigned int *generation) {
        return try_route_to(table, track, footprint, train,
                            plan_destination_node,
                            0, 0, -1, 0, 1, conflict, generation);
}

int TrackReservationTryRouteTo(track_reservation_table *table, track_node *track,
                               const track_route *route, int train,
                               int destination_node,
                               track_reservation_conflict *conflict) {
        return TrackReservationTryRouteToGeneration(
                table, track, route, train, destination_node, conflict, 0);
}

int TrackReservationUpdateFootprint(
        track_reservation_table *table, track_node *track,
        const track_route *footprint, int train, int destination_node,
        unsigned int expected_generation,
        track_reservation_conflict *conflict, unsigned int *generation) {
        return try_route_to(table, track, footprint, train, destination_node,
                            1, 1, destination_node,
                            expected_generation, 0,
                            conflict, generation);
}

int TrackReservationUpdateWindow(
        track_reservation_table *table, track_node *track,
        const track_route *footprint, int train,
        int plan_destination_node, unsigned int expected_generation,
        track_reservation_conflict *conflict, unsigned int *generation) {
        return try_route_to(table, track, footprint, train,
                            plan_destination_node,
                            0, 1, plan_destination_node,
                            expected_generation, 0,
                            conflict, generation);
}

int TrackReservationReplacePlanExpected(
        track_reservation_table *table, track_node *track,
        const track_route *replacement, int train,
        int expected_destination_node, unsigned int expected_generation,
        int new_destination_node,
        track_reservation_conflict *conflict, unsigned int *generation) {
        return try_route_to(table, track, replacement, train,
                            new_destination_node, 1, 1,
                            expected_destination_node, expected_generation, 1,
                            conflict, generation);
}

int TrackReservationReplaceWindowExpected(
        track_reservation_table *table, track_node *track,
        const track_route *replacement_window, int train,
        int expected_destination_node, unsigned int expected_generation,
        int new_destination_node,
        track_reservation_conflict *conflict, unsigned int *generation) {
        return try_route_to(table, track, replacement_window, train,
                            new_destination_node, 0, 1,
                            expected_destination_node, expected_generation, 1,
                            conflict, generation);
}

int TrackReservationTryRoute(track_reservation_table *table, track_node *track,
                             const track_route *route, int train,
                             track_reservation_conflict *conflict) {
        if (!route_is_valid(route)) return -1;
        return TrackReservationTryRouteTo(table, track, route, train,
                                          route->nodes[route->node_count - 1],
                                          conflict);
}

void TrackReservationReleaseTrain(track_reservation_table *table, int train) {
        if (!table || train < 1 || train > 255) return;
        int had_plan = table->destination_by_train[train] >= 0;
        if (!had_plan) {
                for (int node = 0; node < TRACK_MAX; ++node) {
                        if (table->owner_by_node[node] == train) {
                                had_plan = 1;
                                break;
                        }
                }
        }
        clear_train_owners(table, train);
        table->destination_by_train[train] = -1;
        if (had_plan) {
                // Release也改变这个train的plan identity；重复release保持generation稳定。
                advance_train_generation(table, train);
        }
}

int TrackReservationCountForTrain(const track_reservation_table *table, int train) {
        if (!table || train < 1 || train > 255) return 0;

        int count = 0;
        for (int i = 0; i < TRACK_MAX; ++i) {
                if (table->owner_by_node[i] == train) ++count;
        }
        return count;
}

int TrackReservationLookupNode(const track_reservation_table *table,
                               int node_index, int *owner_train,
                               int *destination_node,
                               unsigned int *generation) {
        if (!table || node_index < 0 || node_index >= TRACK_MAX || !owner_train) {
                return -1;
        }

        int owner = table->owner_by_node[node_index];
        *owner_train = owner;
        if (destination_node) {
                *destination_node = owner > 0 ?
                        table->destination_by_train[owner] : -1;
        }
        if (generation) {
                *generation = owner > 0 ?
                        table->generation_by_train[owner] : 0;
        }
        return 0;
}
