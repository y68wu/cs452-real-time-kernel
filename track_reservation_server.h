#ifndef _track_reservation_server_h_
#define _track_reservation_server_h_ 1

#include "track_reservation.h"

#define TRACK_RESERVATION_SERVER_NAME "track-reserve"

typedef struct {
        int owner_by_node[TRACK_MAX];
        int destination_by_train[256];
        unsigned int generation_by_train[256];
} track_reservation_snapshot;

int TrackReservationServerReserve(int tid, const track_route *route, int train,
                                  int destination_node,
                                  track_reservation_conflict *conflict);
int TrackReservationServerReserveWithGeneration(
        int tid, const track_route *route, int train, int destination_node,
        track_reservation_conflict *conflict, unsigned int *generation);
int TrackReservationServerReserveWindowWithGeneration(
        int tid, const track_route *footprint, int train,
        int plan_destination_node,
        track_reservation_conflict *conflict, unsigned int *generation);
int TrackReservationServerUpdateFootprint(
        int tid, const track_route *footprint, int train, int destination_node,
        unsigned int expected_generation,
        track_reservation_conflict *conflict, unsigned int *generation);
int TrackReservationServerUpdateWindow(
        int tid, const track_route *footprint, int train,
        int plan_destination_node, unsigned int expected_generation,
        track_reservation_conflict *conflict, unsigned int *generation);
int TrackReservationServerReplacePlanExpected(
        int tid, const track_route *replacement, int train,
        int expected_destination_node, unsigned int expected_generation,
        int new_destination_node,
        track_reservation_conflict *conflict, unsigned int *generation);
int TrackReservationServerReplaceWindowExpected(
        int tid, const track_route *replacement_window, int train,
        int expected_destination_node, unsigned int expected_generation,
        int new_destination_node,
        track_reservation_conflict *conflict, unsigned int *generation);
int TrackReservationServerRelease(int tid, int train);
int TrackReservationServerSnapshot(int tid, track_reservation_snapshot *snapshot);
// LookupNode使用small IPC hot path，返回attribution所需的owner和plan identity。
int TrackReservationServerLookupNode(int tid, int node_index,
                                     int *owner_train,
                                     int *destination_node,
                                     unsigned int *generation);
void TrackReservationServerTask(void);

// 这个server通过Send/Reply serialize ownership changes，clients不共享reservation memory。

#endif
