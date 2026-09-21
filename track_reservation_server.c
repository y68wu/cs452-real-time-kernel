#include "track_reservation_server.h"
#include "nameserver.h"
#include "syscall.h"

#define RESERVATION_MSG_RESERVE 1
#define RESERVATION_MSG_RELEASE 2
#define RESERVATION_MSG_SNAPSHOT 3
#define RESERVATION_MSG_LOOKUP_NODE 4
#define RESERVATION_MSG_UPDATE_FOOTPRINT 5
#define RESERVATION_MSG_REPLACE_PLAN_EXPECTED 6
#define RESERVATION_MSG_RESERVE_WINDOW 7
#define RESERVATION_MSG_UPDATE_WINDOW 8
#define RESERVATION_MSG_REPLACE_WINDOW_EXPECTED 9

typedef struct {
        int type;
        int train;
        int destination_node;
        int expected_destination_node;
        unsigned int expected_generation;
        track_route route;
} reservation_request;

typedef struct {
        int status;
        track_reservation_conflict conflict;
        unsigned int generation;
} reservation_reply;

typedef struct {
        int status;
        int owner_train;
        int destination_node;
        unsigned int generation;
} reservation_node_reply;

typedef struct {
        int type;
        int node_index;
} reservation_node_request;

typedef union {
        reservation_request full;
        reservation_node_request node;
} reservation_request_buffer;

static void clear_request(reservation_request *request) {
        request->type = 0;
        request->train = 0;
        request->destination_node = -1;
        request->expected_destination_node = -1;
        request->expected_generation = 0;
        request->route.node_count = 0;
        request->route.distance_mm = 0;
        request->route.optimization_cost_mm = 0;
        request->route.reversal_count = 0;
        for (int i = 0; i < TRACK_MAX; ++i) request->route.nodes[i] = -1;
}

static int send_route_request(int tid, int type, const track_route *route,
                              int train, int destination_node,
                              int expected_destination_node,
                              unsigned int expected_generation,
                              track_reservation_conflict *conflict,
                              unsigned int *generation) {
        reservation_request request;
        reservation_reply reply;

        if (!route || !conflict ||
            destination_node < 0 || destination_node >= TRACK_MAX) {
                return -1;
        }
        clear_request(&request);
        request.type = type;
        request.train = train;
        request.destination_node = destination_node;
        request.expected_destination_node = expected_destination_node;
        request.expected_generation = expected_generation;
        request.route.node_count = route->node_count;
        request.route.distance_mm = route->distance_mm;
        request.route.optimization_cost_mm = route->optimization_cost_mm;
        request.route.reversal_count = route->reversal_count;
        // 只copy有效route nodes，unused IPC bytes保持deterministic -1。
        if (route->node_count >= 1 && route->node_count <= TRACK_MAX) {
                for (int i = 0; i < route->node_count; ++i) {
                        request.route.nodes[i] = route->nodes[i];
                }
        }
        int ret = Send(tid, (const char *)&request, sizeof(request),
                       (char *)&reply, sizeof(reply));
        if (ret != (int)sizeof(reply)) return -1;
        *conflict = reply.conflict;
        if (generation) *generation = reply.generation;
        return reply.status;
}

int TrackReservationServerReserveWithGeneration(
        int tid, const track_route *route, int train, int destination_node,
        track_reservation_conflict *conflict, unsigned int *generation) {
        return send_route_request(tid, RESERVATION_MSG_RESERVE, route, train,
                                  destination_node, -1, 0,
                                  conflict, generation);
}

int TrackReservationServerReserve(int tid, const track_route *route, int train,
                                  int destination_node,
                                  track_reservation_conflict *conflict) {
        return TrackReservationServerReserveWithGeneration(
                tid, route, train, destination_node, conflict, 0);
}

int TrackReservationServerReserveWindowWithGeneration(
        int tid, const track_route *footprint, int train,
        int plan_destination_node,
        track_reservation_conflict *conflict, unsigned int *generation) {
        return send_route_request(
                tid, RESERVATION_MSG_RESERVE_WINDOW, footprint, train,
                plan_destination_node, -1, 0, conflict, generation);
}

int TrackReservationServerUpdateFootprint(
        int tid, const track_route *footprint, int train, int destination_node,
        unsigned int expected_generation,
        track_reservation_conflict *conflict, unsigned int *generation) {
        return send_route_request(tid, RESERVATION_MSG_UPDATE_FOOTPRINT,
                                  footprint, train, destination_node,
                                  destination_node, expected_generation,
                                  conflict, generation);
}

int TrackReservationServerUpdateWindow(
        int tid, const track_route *footprint, int train,
        int plan_destination_node, unsigned int expected_generation,
        track_reservation_conflict *conflict, unsigned int *generation) {
        return send_route_request(
                tid, RESERVATION_MSG_UPDATE_WINDOW, footprint, train,
                plan_destination_node, plan_destination_node,
                expected_generation, conflict, generation);
}

int TrackReservationServerReplacePlanExpected(
        int tid, const track_route *replacement, int train,
        int expected_destination_node, unsigned int expected_generation,
        int new_destination_node,
        track_reservation_conflict *conflict, unsigned int *generation) {
        return send_route_request(tid, RESERVATION_MSG_REPLACE_PLAN_EXPECTED,
                                  replacement, train, new_destination_node,
                                  expected_destination_node,
                                  expected_generation, conflict, generation);
}

int TrackReservationServerReplaceWindowExpected(
        int tid, const track_route *replacement_window, int train,
        int expected_destination_node, unsigned int expected_generation,
        int new_destination_node,
        track_reservation_conflict *conflict, unsigned int *generation) {
        return send_route_request(
                tid, RESERVATION_MSG_REPLACE_WINDOW_EXPECTED,
                replacement_window, train, new_destination_node,
                expected_destination_node, expected_generation,
                conflict, generation);
}

int TrackReservationServerRelease(int tid, int train) {
        reservation_request request;
        reservation_reply reply;

        clear_request(&request);
        request.type = RESERVATION_MSG_RELEASE;
        request.train = train;
        int ret = Send(tid, (const char *)&request, sizeof(request),
                       (char *)&reply, sizeof(reply));
        return ret == (int)sizeof(reply) ? reply.status : -1;
}

int TrackReservationServerSnapshot(int tid, track_reservation_snapshot *snapshot) {
        reservation_request request;

        if (!snapshot) return -1;
        clear_request(&request);
        request.type = RESERVATION_MSG_SNAPSHOT;
        int ret = Send(tid, (const char *)&request, sizeof(request),
                       (char *)snapshot, sizeof(*snapshot));
        return ret == (int)sizeof(*snapshot) ? 0 : -1;
}

int TrackReservationServerLookupNode(int tid, int node_index,
                                     int *owner_train,
                                     int *destination_node,
                                     unsigned int *generation) {
        reservation_node_request request;
        reservation_node_reply reply;

        if (node_index < 0 || node_index >= TRACK_MAX || !owner_train) return -1;
        request.type = RESERVATION_MSG_LOOKUP_NODE;
        request.node_index = node_index;
        int ret = Send(tid, (const char *)&request, sizeof(request),
                       (char *)&reply, sizeof(reply));
        if (ret != (int)sizeof(reply) || reply.status < 0) return -1;
        *owner_train = reply.owner_train;
        if (destination_node) *destination_node = reply.destination_node;
        if (generation) *generation = reply.generation;
        return 0;
}

void TrackReservationServerTask(void) {
        track_node track[TRACK_MAX];
        track_reservation_table table;
        reservation_request_buffer request;
        reservation_reply reply;
        reservation_node_reply node_reply;
        track_reservation_snapshot snapshot;
        int sender_tid;

        init_trackb(track);
        TrackReservationInit(&table);
        RegisterAs(TRACK_RESERVATION_SERVER_NAME);

        // Server owns all reservation state，Receive order提供single-writer serialization。
        for (;;) {
                int request_len = Receive(&sender_tid, (char *)&request, sizeof(request));
                if (request_len == (int)sizeof(reservation_node_request) &&
                    request.node.type == RESERVATION_MSG_LOOKUP_NODE) {
                        // Lookup request/reply保持bounded，并返回owner当前plan generation。
                        int node = request.node.node_index;
                        if (TrackReservationLookupNode(
                                    &table, node,
                                    &node_reply.owner_train,
                                    &node_reply.destination_node,
                                    &node_reply.generation) < 0) {
                                node_reply.status = -1;
                                node_reply.owner_train = 0;
                                node_reply.destination_node = -1;
                                node_reply.generation = 0;
                        } else {
                                node_reply.status = 0;
                        }
                        Reply(sender_tid, (const char *)&node_reply,
                              sizeof(node_reply));
                        continue;
                }
                if (request_len != (int)sizeof(request)) {
                        reply.status = -1;
                        reply.conflict.node_index = -1;
                        reply.conflict.owner_train = 0;
                        reply.generation = 0;
                        Reply(sender_tid, (const char *)&reply, sizeof(reply));
                        continue;
                }

                if (request.full.type == RESERVATION_MSG_RESERVE) {
                        reply.status = TrackReservationTryRouteToGeneration(
                                &table, track, &request.full.route,
                                request.full.train,
                                request.full.destination_node,
                                &reply.conflict, &reply.generation);
                        Reply(sender_tid, (const char *)&reply, sizeof(reply));
                } else if (request.full.type ==
                           RESERVATION_MSG_RESERVE_WINDOW) {
                        reply.status = TrackReservationTryWindowToGeneration(
                                &table, track, &request.full.route,
                                request.full.train,
                                request.full.destination_node,
                                &reply.conflict, &reply.generation);
                        Reply(sender_tid, (const char *)&reply, sizeof(reply));
                } else if (request.full.type ==
                           RESERVATION_MSG_UPDATE_FOOTPRINT) {
                        reply.status = TrackReservationUpdateFootprint(
                                &table, track, &request.full.route,
                                request.full.train,
                                request.full.destination_node,
                                request.full.expected_generation,
                                &reply.conflict, &reply.generation);
                        Reply(sender_tid, (const char *)&reply, sizeof(reply));
                } else if (request.full.type ==
                           RESERVATION_MSG_UPDATE_WINDOW) {
                        reply.status = TrackReservationUpdateWindow(
                                &table, track, &request.full.route,
                                request.full.train,
                                request.full.destination_node,
                                request.full.expected_generation,
                                &reply.conflict, &reply.generation);
                        Reply(sender_tid, (const char *)&reply, sizeof(reply));
                } else if (request.full.type ==
                           RESERVATION_MSG_REPLACE_PLAN_EXPECTED) {
                        reply.status = TrackReservationReplacePlanExpected(
                                &table, track, &request.full.route,
                                request.full.train,
                                request.full.expected_destination_node,
                                request.full.expected_generation,
                                request.full.destination_node,
                                &reply.conflict, &reply.generation);
                        Reply(sender_tid, (const char *)&reply, sizeof(reply));
                } else if (request.full.type ==
                           RESERVATION_MSG_REPLACE_WINDOW_EXPECTED) {
                        reply.status = TrackReservationReplaceWindowExpected(
                                &table, track, &request.full.route,
                                request.full.train,
                                request.full.expected_destination_node,
                                request.full.expected_generation,
                                request.full.destination_node,
                                &reply.conflict, &reply.generation);
                        Reply(sender_tid, (const char *)&reply, sizeof(reply));
                } else if (request.full.type == RESERVATION_MSG_RELEASE) {
                        TrackReservationReleaseTrain(&table, request.full.train);
                        reply.status = request.full.train >= 1 &&
                                request.full.train <= 255 ? 0 : -1;
                        reply.conflict.node_index = -1;
                        reply.conflict.owner_train = 0;
                        reply.generation =
                                request.full.train >= 1 &&
                                        request.full.train <= 255 ?
                                table.generation_by_train[
                                        request.full.train] : 0;
                        Reply(sender_tid, (const char *)&reply, sizeof(reply));
                } else if (request.full.type == RESERVATION_MSG_SNAPSHOT) {
                        for (int i = 0; i < TRACK_MAX; ++i) {
                                snapshot.owner_by_node[i] = table.owner_by_node[i];
                        }
                        for (int train = 0; train < 256; ++train) {
                                snapshot.destination_by_train[train] =
                                        table.destination_by_train[train];
                                snapshot.generation_by_train[train] =
                                        table.generation_by_train[train];
                        }
                        Reply(sender_tid, (const char *)&snapshot, sizeof(snapshot));
                } else {
                        reply.status = -1;
                        reply.conflict.node_index = -1;
                        reply.conflict.owner_train = 0;
                        reply.generation = 0;
                        Reply(sender_tid, (const char *)&reply, sizeof(reply));
                }
        }
}
