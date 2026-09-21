#ifndef _track_reservation_h_
#define _track_reservation_h_ 1

#include "track_route.h"

typedef struct {
        int owner_by_node[TRACK_MAX];
        int destination_by_train[256];
        unsigned int generation_by_train[256];
} track_reservation_table;

typedef struct {
        int node_index;
        int owner_train;
} track_reservation_conflict;

void TrackReservationInit(track_reservation_table *table);
int TrackReservationTryRoute(track_reservation_table *table, track_node *track,
                             const track_route *route, int train,
                             track_reservation_conflict *conflict);
int TrackReservationTryRouteTo(track_reservation_table *table, track_node *track,
                               const track_route *route, int train,
                               int destination_node,
                               track_reservation_conflict *conflict);
int TrackReservationTryRouteToGeneration(
        track_reservation_table *table, track_node *track,
        const track_route *route, int train, int destination_node,
        track_reservation_conflict *conflict, unsigned int *generation);
/*
 * Reserve an initial rolling-authority footprint. The plan destination is
 * identity metadata and need not be present in the current footprint.
 */
int TrackReservationTryWindowToGeneration(
        track_reservation_table *table, track_node *track,
        const track_route *footprint, int train,
        int plan_destination_node,
        track_reservation_conflict *conflict, unsigned int *generation);
int TrackReservationUpdateFootprint(
        track_reservation_table *table, track_node *track,
        const track_route *footprint, int train, int destination_node,
        unsigned int expected_generation,
        track_reservation_conflict *conflict, unsigned int *generation);
/*
 * Atomically extend or shrink a rolling-authority footprint. A conflict or
 * stale plan identity leaves the old footprint unchanged.
 */
int TrackReservationUpdateWindow(
        track_reservation_table *table, track_node *track,
        const track_route *footprint, int train,
        int plan_destination_node, unsigned int expected_generation,
        track_reservation_conflict *conflict, unsigned int *generation);
int TrackReservationReplacePlanExpected(
        track_reservation_table *table, track_node *track,
        const track_route *replacement, int train,
        int expected_destination_node, unsigned int expected_generation,
        int new_destination_node,
        track_reservation_conflict *conflict, unsigned int *generation);
/*
 * Atomically replace an old plan with the first bounded rolling-authority
 * window of a new plan. The new destination remains plan identity metadata
 * and therefore need not occur inside this initial footprint.
 */
int TrackReservationReplaceWindowExpected(
        track_reservation_table *table, track_node *track,
        const track_route *replacement_window, int train,
        int expected_destination_node, unsigned int expected_generation,
        int new_destination_node,
        track_reservation_conflict *conflict, unsigned int *generation);
void TrackReservationReleaseTrain(track_reservation_table *table, int train);
int TrackReservationCountForTrain(const track_reservation_table *table, int train);
// Single-node lookup返回owner、destination和plan generation，避免复制whole table。
int TrackReservationLookupNode(const track_reservation_table *table,
                               int node_index, int *owner_train,
                               int *destination_node,
                               unsigned int *generation);

// Reservation同时占用node和reverse node，阻止opposite-direction physical overlap。
// 新plan成功时推进generation；同一plan的sliding footprint update保留generation。
// Expected replacement同时compare旧destination和generation；成功才换plan并推进generation。
// 所有replacement都是atomic，conflict或stale expected identity会保留old footprint。
// Window API把destination仅作为plan identity；legacy route API仍要求destination在route中。

#endif
