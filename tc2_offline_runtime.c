#include "tc2_offline_runtime.h"

#ifndef MODE_TC2

typedef int tc2_offline_runtime_disabled_translation_unit;

#else

#include <stddef.h>
#include <stdint.h>

#include "tc2_motion_model.h"
#include "tc2_track_d_layout.h"
#include "track_data.h"

/*
 * This translation unit is intentionally self-contained: no malloc, no
 * dispatcher service, no reservation service, no terminal service, and no
 * CAN/train-control call is reachable from here.
 */

#define TC2_OFFLINE_RESOURCE_TAG_SHIFT 56u
#define TC2_OFFLINE_RESOURCE_GRAPH 1u
#define TC2_OFFLINE_RESOURCE_SENSOR_PAIR 2u
#define TC2_OFFLINE_RESOURCE_TURNOUT 3u
#define TC2_OFFLINE_RESOURCE_DESTINATION 4u
#define TC2_OFFLINE_RESOURCE_EDGE 5u

#define TC2_OFFLINE_PHYSICAL_GRAPH 1u
#define TC2_OFFLINE_PHYSICAL_SENSOR_PAIR 2u
#define TC2_OFFLINE_PHYSICAL_TURNOUT 3u

_Static_assert(TC2_OFFLINE_RUNTIME_MAX_TRAINS > 0,
               "offline runtime requires train slots");
_Static_assert(TC2_OFFLINE_RUNTIME_MAX_WAYPOINTS >= 2,
               "offline runtime requires start and destination");
_Static_assert(TC2_OFFLINE_RUNTIME_MAX_CLAIMS >=
                       3 * TC2_OFFLINE_RUNTIME_MAX_WAYPOINTS,
               "offline resource bound is too small");

static void zero_bytes(void *destination, size_t count) {
        unsigned char *bytes = (unsigned char *)destination;
        if (!bytes) return;
        for (size_t index = 0; index < count; ++index) {
                bytes[index] = 0;
        }
}

static int bytes_are_zero(const void *source, size_t count) {
        const unsigned char *bytes =
                (const unsigned char *)source;
        if (!bytes) return 0;
        for (size_t index = 0; index < count; ++index) {
                if (bytes[index] != 0) return 0;
        }
        return 1;
}

static int ranges_overlap(const void *first, size_t first_size,
                          const void *second, size_t second_size) {
        if (!first || !second || first_size == 0 ||
            second_size == 0) {
                return 0;
        }
        uintptr_t first_begin = (uintptr_t)first;
        uintptr_t second_begin = (uintptr_t)second;
        if (first_begin > UINTPTR_MAX - first_size ||
            second_begin > UINTPTR_MAX - second_size) {
                return 1;
        }
        uintptr_t first_end = first_begin + first_size;
        uintptr_t second_end = second_begin + second_size;
        return first_begin < second_end &&
               second_begin < first_end;
}

static int runtime_ready(const tc2_offline_runtime *runtime) {
        return runtime && runtime->initialized;
}

static int valid_switch_number(int number) {
        return (number >= 1 && number <= 18) ||
               (number >= 153 && number <= 156);
}

static int valid_job_state(int state) {
        return state >= TC2_JOB_EMPTY &&
               state <= TC2_JOB_TRAFFIC_HOLD;
}

static int valid_header(
        const tc2_dispatch_projection_header *header) {
        if (!header ||
            header->status != TC2_DISPATCH_PROJECTION_OK ||
            header->valid != 1 ||
            header->train < 1 || header->train > 255 ||
            !valid_job_state(header->job_state) ||
            header->publication_serial == 0 ||
            header->physical_destination_distance_um < 0 ||
            header->plan_generation == 0 ||
            header->launch_epoch == 0 ||
            (header->scheduler_healthy != 0 &&
             header->scheduler_healthy != 1) ||
            header->start_index < 0 ||
            header->start_index >= TC2_DISPATCH_START_COUNT ||
            header->destination_index < 0 ||
            header->destination_index >=
                    TC2_DISPATCH_DESTINATION_COUNT ||
            header->destination_side < 0 ||
            header->destination_side >=
                    TC2_TRACK_DESTINATION_SIDE_COUNT ||
            header->speed < 1 || header->speed > 120 ||
            header->waypoint_count < 2 ||
            header->waypoint_count >
                    TC2_OFFLINE_RUNTIME_MAX_WAYPOINTS ||
            header->sensor_count < 0 ||
            header->turnout_count < 0 ||
            header->reversal_count < 0 ||
            header->geometry_anchor_route_offset < 0 ||
            header->physical_destination_route_offset <
                    header->geometry_anchor_route_offset ||
            header->last_visible_route_offset <
                    header->geometry_anchor_route_offset ||
            header->last_visible_route_offset >
                    header->physical_destination_route_offset ||
            header->operator_destination_waypoint_index !=
                    header->waypoint_count - 1 ||
            header->physical_destination_offset_mm < 0 ||
            Tc2MotionProvisionalVelocityUmPerTick(
                    header->speed) <= 0) {
                return 0;
        }
        int semantic_count =
                2 + header->sensor_count +
                header->turnout_count +
                header->reversal_count;
        return semantic_count == header->waypoint_count;
}

static int valid_ui_cell(
        const tc2_dispatch_projection_waypoint *waypoint) {
        if (!waypoint || waypoint->ui_width == 0 ||
            waypoint->ui_row >= TC2_TRACK_D_LAYOUT_ROWS ||
            waypoint->ui_column >=
                    TC2_TRACK_D_LAYOUT_MAP_COLUMNS) {
                return 0;
        }
        return (int)waypoint->ui_column +
                       (int)waypoint->ui_width <=
               TC2_TRACK_D_LAYOUT_MAP_COLUMNS;
}

static int valid_waypoint_kind_fields(
        const tc2_dispatch_projection_header *header,
        const tc2_dispatch_projection_waypoint *waypoint,
        int index) {
        if (!header || !waypoint ||
            waypoint->reserved != 0 ||
            waypoint->graph_node < 0 ||
            waypoint->graph_node >= TRACK_MAX ||
            !valid_ui_cell(waypoint)) {
                return 0;
        }
        if (waypoint->kind == TC2_ROUTE_WAYPOINT_START) {
                return index == 0 &&
                       waypoint->route_offset == 0 &&
                       waypoint->distance_um == 0 &&
                       waypoint->sensor_index == -1 &&
                       waypoint->switch_number == -1 &&
                       waypoint->turnout_direction == -1 &&
                       waypoint->destination_index == -1;
        }
        if (waypoint->kind == TC2_ROUTE_WAYPOINT_SENSOR ||
            waypoint->kind == TC2_ROUTE_WAYPOINT_REVERSAL) {
                return index > 0 &&
                       waypoint->sensor_index >= 0 &&
                       waypoint->sensor_index <
                               TC2_TRACK_D_DIRECTED_SENSOR_COUNT &&
                       waypoint->switch_number == -1 &&
                       waypoint->turnout_direction == -1 &&
                       waypoint->destination_index == -1;
        }
        if (waypoint->kind == TC2_ROUTE_WAYPOINT_TURNOUT) {
                return index > 0 &&
                       waypoint->sensor_index == -1 &&
                       valid_switch_number(
                               waypoint->switch_number) &&
                       (waypoint->turnout_direction == 0 ||
                        waypoint->turnout_direction == 1) &&
                       waypoint->destination_index == -1;
        }
        if (waypoint->kind ==
                    TC2_ROUTE_WAYPOINT_DESTINATION) {
                return index == header->waypoint_count - 1 &&
                       waypoint->sensor_index == -1 &&
                       waypoint->switch_number == -1 &&
                       waypoint->turnout_direction == -1 &&
                       waypoint->destination_index ==
                               header->destination_index;
        }
        return 0;
}

static int validate_projection(
        const tc2_dispatch_projection_header *header,
        const tc2_dispatch_projection_waypoint *waypoints,
        int waypoint_count) {
        if (!valid_header(header) || !waypoints ||
            waypoint_count != header->waypoint_count) {
                return TC2_OFFLINE_MALFORMED_PROJECTION;
        }

        int sensors = 0;
        int turnouts = 0;
        int reversals = 0;
        int starts = 0;
        int destinations = 0;
        int64_t prior_distance = -1;
        int prior_route_offset = -1;

        for (int index = 0; index < waypoint_count; ++index) {
                const tc2_dispatch_projection_waypoint *waypoint =
                        &waypoints[index];
                if (!valid_waypoint_kind_fields(
                            header, waypoint, index) ||
                    waypoint->distance_um < 0 ||
                    waypoint->distance_um >
                            header->
                              physical_destination_distance_um ||
                    waypoint->route_offset < 0 ||
                    waypoint->distance_um < prior_distance ||
                    waypoint->route_offset <
                            prior_route_offset) {
                        return TC2_OFFLINE_MALFORMED_PROJECTION;
                }
                if (waypoint->kind ==
                            TC2_ROUTE_WAYPOINT_START) {
                        ++starts;
                } else if (waypoint->kind ==
                                   TC2_ROUTE_WAYPOINT_SENSOR) {
                        ++sensors;
                } else if (waypoint->kind ==
                                   TC2_ROUTE_WAYPOINT_TURNOUT) {
                        ++turnouts;
                } else if (waypoint->kind ==
                                   TC2_ROUTE_WAYPOINT_REVERSAL) {
                        ++reversals;
                } else if (waypoint->kind ==
                                   TC2_ROUTE_WAYPOINT_DESTINATION) {
                        ++destinations;
                }
                prior_distance = waypoint->distance_um;
                prior_route_offset = waypoint->route_offset;
        }

        const tc2_dispatch_projection_waypoint *last =
                &waypoints[waypoint_count - 1];
        if (starts != 1 || destinations != 1 ||
            sensors != header->sensor_count ||
            turnouts != header->turnout_count ||
            reversals != header->reversal_count ||
            last->kind != TC2_ROUTE_WAYPOINT_DESTINATION ||
            last->distance_um !=
                    header->physical_destination_distance_um ||
            last->route_offset !=
                    header->last_visible_route_offset) {
                return TC2_OFFLINE_MALFORMED_PROJECTION;
        }
        return TC2_OFFLINE_OK;
}

static int find_train_slot(
        const tc2_offline_runtime *runtime, int train) {
        if (!runtime) return -1;
        for (int slot = 0;
             slot < TC2_OFFLINE_RUNTIME_MAX_TRAINS; ++slot) {
                if (runtime->trains[slot].state !=
                            TC2_OFFLINE_TRAIN_EMPTY &&
                    runtime->trains[slot].train == train) {
                        return slot;
                }
        }
        return -1;
}

static int find_empty_slot(
        const tc2_offline_runtime *runtime) {
        if (!runtime) return -1;
        for (int slot = 0;
             slot < TC2_OFFLINE_RUNTIME_MAX_TRAINS; ++slot) {
                if (runtime->trains[slot].state ==
                            TC2_OFFLINE_TRAIN_EMPTY) {
                        return slot;
                }
        }
        return -1;
}

static void clear_train(tc2_offline_train *train) {
        if (!train) return;
        zero_bytes(train, sizeof(*train));
        train->state = TC2_OFFLINE_TRAIN_EMPTY;
        train->train = TC2_OFFLINE_RUNTIME_INVALID_TRAIN;
        train->conflict_train =
                TC2_OFFLINE_RUNTIME_INVALID_TRAIN;
        train->last_sensor_index =
                TC2_OFFLINE_RUNTIME_INVALID_INDEX;
        train->next_waypoint_index =
                TC2_OFFLINE_RUNTIME_INVALID_INDEX;
}

static void fail_loading_train(
        tc2_offline_train *train, int error) {
        if (!train) return;
        train->state = TC2_OFFLINE_TRAIN_FAILED;
        train->last_error = error;
        train->start_requested = 0;
        train->entered_route = 0;
        train->claim_count = 0;
        train->conflict_train =
                TC2_OFFLINE_RUNTIME_INVALID_TRAIN;
}

static uint64_t resource_key(
        unsigned int tag, uint64_t payload) {
        return ((uint64_t)tag <<
                        TC2_OFFLINE_RESOURCE_TAG_SHIFT) |
               (payload & UINT64_C(0x00ffffffffffffff));
}

static uint32_t physical_vertex(
        const tc2_dispatch_projection_waypoint *waypoint) {
        unsigned int type = TC2_OFFLINE_PHYSICAL_GRAPH;
        unsigned int value = 0;
        if (!waypoint) return 0;
        if (waypoint->kind == TC2_ROUTE_WAYPOINT_SENSOR ||
            waypoint->kind == TC2_ROUTE_WAYPOINT_REVERSAL) {
                type = TC2_OFFLINE_PHYSICAL_SENSOR_PAIR;
                value = (unsigned int)waypoint->sensor_index / 2u;
        } else if (waypoint->kind ==
                           TC2_ROUTE_WAYPOINT_TURNOUT) {
                type = TC2_OFFLINE_PHYSICAL_TURNOUT;
                value = (unsigned int)waypoint->switch_number;
        } else {
                value = (unsigned int)waypoint->graph_node;
        }
        return (type << 24u) | (value & UINT32_C(0x00ffffff));
}

static int add_claim(tc2_offline_resource_claim *claims,
                     int *claim_count, uint64_t key) {
        if (!claims || !claim_count || key == 0 ||
            *claim_count < 0 ||
            *claim_count > TC2_OFFLINE_RUNTIME_MAX_CLAIMS) {
                return TC2_OFFLINE_OVERFLOW;
        }
        for (int index = 0; index < *claim_count; ++index) {
                if (claims[index].key == key) {
                        return TC2_OFFLINE_OK;
                }
        }
        if (*claim_count >= TC2_OFFLINE_RUNTIME_MAX_CLAIMS) {
                return TC2_OFFLINE_OVERFLOW;
        }
        claims[*claim_count].key = key;
        ++*claim_count;
        return TC2_OFFLINE_OK;
}

static int add_waypoint_claims(
        const tc2_offline_train *train, int waypoint_index,
        tc2_offline_resource_claim *claims, int *claim_count) {
        if (!train || waypoint_index < 0 ||
            waypoint_index >= train->loaded_waypoint_count) {
                return TC2_OFFLINE_BOUNDS;
        }
        const tc2_dispatch_projection_waypoint *waypoint =
                &train->waypoints[waypoint_index];
        int status = add_claim(
                claims, claim_count,
                resource_key(TC2_OFFLINE_RESOURCE_GRAPH,
                             (uint64_t)(unsigned int)
                                     waypoint->graph_node));
        if (status != TC2_OFFLINE_OK) return status;

        if (waypoint->kind == TC2_ROUTE_WAYPOINT_SENSOR ||
            waypoint->kind == TC2_ROUTE_WAYPOINT_REVERSAL) {
                status = add_claim(
                        claims, claim_count,
                        resource_key(
                                TC2_OFFLINE_RESOURCE_SENSOR_PAIR,
                                (uint64_t)(unsigned int)
                                        waypoint->sensor_index /
                                        2u));
        } else if (waypoint->kind ==
                           TC2_ROUTE_WAYPOINT_TURNOUT) {
                status = add_claim(
                        claims, claim_count,
                        resource_key(
                                TC2_OFFLINE_RESOURCE_TURNOUT,
                                (uint64_t)(unsigned int)
                                        waypoint->
                                          switch_number));
        } else if (waypoint->kind ==
                           TC2_ROUTE_WAYPOINT_DESTINATION) {
                status = add_claim(
                        claims, claim_count,
                        resource_key(
                                TC2_OFFLINE_RESOURCE_DESTINATION,
                                (uint64_t)(unsigned int)
                                        waypoint->
                                          destination_index));
        }
        return status;
}

static int add_edge_claim(
        const tc2_offline_train *train, int first_index,
        int second_index, tc2_offline_resource_claim *claims,
        int *claim_count) {
        if (!train || first_index < 0 || second_index < 0 ||
            first_index >= train->loaded_waypoint_count ||
            second_index >= train->loaded_waypoint_count) {
                return TC2_OFFLINE_BOUNDS;
        }
        uint32_t first =
                physical_vertex(&train->waypoints[first_index]);
        uint32_t second =
                physical_vertex(&train->waypoints[second_index]);
        if (second < first) {
                uint32_t swap = first;
                first = second;
                second = swap;
        }
        uint64_t payload =
                ((uint64_t)first << 28u) |
                ((uint64_t)second &
                 UINT64_C(0x000000000fffffff));
        return add_claim(
                claims, claim_count,
                resource_key(TC2_OFFLINE_RESOURCE_EDGE,
                             payload));
}

static int previous_waypoint_for_progress(
        const tc2_offline_train *train, int64_t progress_um) {
        if (!train || train->loaded_waypoint_count < 1) {
                return TC2_OFFLINE_RUNTIME_INVALID_INDEX;
        }
        int previous = 0;
        for (int index = 1;
             index < train->loaded_waypoint_count; ++index) {
                if (train->waypoints[index].distance_um >
                            progress_um) {
                        break;
                }
                previous = index;
        }
        return previous;
}

static int build_claim_range(
        const tc2_offline_train *train, int first, int last,
        tc2_offline_resource_claim *claims, int *claim_count) {
        if (!train || !claims || !claim_count || first < 0 ||
            last < first ||
            last >= train->loaded_waypoint_count) {
                return TC2_OFFLINE_BOUNDS;
        }
        *claim_count = 0;
        for (int index = first; index <= last; ++index) {
                int status = add_waypoint_claims(
                        train, index, claims, claim_count);
                if (status != TC2_OFFLINE_OK) return status;
                if (index > first) {
                        status = add_edge_claim(
                                train, index - 1, index,
                                claims, claim_count);
                        if (status != TC2_OFFLINE_OK) {
                                return status;
                        }
                }
        }
        return TC2_OFFLINE_OK;
}

static int build_transit_claims(
        const tc2_offline_train *train, int64_t candidate_um,
        tc2_offline_resource_claim *claims, int *claim_count) {
        if (!train || !claims || !claim_count ||
            train->loaded_waypoint_count < 2 ||
            candidate_um < train->progress_um ||
            candidate_um >
                    train->header.
                      physical_destination_distance_um) {
                return TC2_OFFLINE_BOUNDS;
        }
        int first = previous_waypoint_for_progress(
                train, train->progress_um);
        int candidate_previous =
                previous_waypoint_for_progress(
                        train, candidate_um);
        if (first < 0 || candidate_previous < first) {
                return TC2_OFFLINE_MALFORMED_PROJECTION;
        }
        int last = candidate_previous + 2;
        if (last >= train->loaded_waypoint_count) {
                last = train->loaded_waypoint_count - 1;
        }
        return build_claim_range(
                train, first, last, claims, claim_count);
}

static int build_stationary_claims(
        const tc2_offline_train *train,
        tc2_offline_resource_claim *claims, int *claim_count) {
        if (!train || !claims || !claim_count ||
            train->loaded_waypoint_count < 2) {
                return TC2_OFFLINE_BOUNDS;
        }
        if (train->state == TC2_OFFLINE_TRAIN_ARRIVED) {
                int destination =
                        train->loaded_waypoint_count - 1;
                *claim_count = 0;
                return add_waypoint_claims(
                        train, destination, claims, claim_count);
        }
        int first = previous_waypoint_for_progress(
                train, train->progress_um);
        if (first < 0) {
                return TC2_OFFLINE_MALFORMED_PROJECTION;
        }
        int last = first + 2;
        if (last >= train->loaded_waypoint_count) {
                last = train->loaded_waypoint_count - 1;
        }
        return build_claim_range(
                train, first, last, claims, claim_count);
}

static int claims_conflict(
        const tc2_offline_resource_claim *first,
        int first_count,
        const tc2_offline_resource_claim *second,
        int second_count) {
        if (!first || !second || first_count < 0 ||
            second_count < 0) {
                return 1;
        }
        for (int first_index = 0;
             first_index < first_count; ++first_index) {
                for (int second_index = 0;
                     second_index < second_count;
                     ++second_index) {
                        if (first[first_index].key ==
                                    second[second_index].key) {
                                return 1;
                        }
                }
        }
        return 0;
}

static int copy_claims(
        tc2_offline_resource_claim *destination,
        const tc2_offline_resource_claim *source, int count) {
        if (!destination || !source || count < 0 ||
            count > TC2_OFFLINE_RUNTIME_MAX_CLAIMS) {
                return TC2_OFFLINE_OVERFLOW;
        }
        for (int index = 0; index < count; ++index) {
                destination[index] = source[index];
        }
        return TC2_OFFLINE_OK;
}

static int event_waypoint_index(
        const tc2_offline_train *train) {
        if (!train || train->loaded_waypoint_count < 1) {
                return TC2_OFFLINE_RUNTIME_INVALID_INDEX;
        }
        int index = train->next_waypoint_index;
        if (index < 0) index = 0;
        if (index >= train->loaded_waypoint_count) {
                index = train->loaded_waypoint_count - 1;
        }
        return index;
}

static void emit_event(
        tc2_offline_runtime *runtime,
        tc2_offline_train *train,
        tc2_offline_event_type type, int waypoint_index,
        int conflict_train, tc2_offline_event_sink sink,
        void *context) {
        if (!runtime || !train || type == TC2_OFFLINE_EVENT_NONE) {
                return;
        }
        tc2_offline_event event;
        zero_bytes(&event, sizeof(event));
        ++runtime->event_serial;
        if (runtime->event_serial == 0) {
                ++runtime->event_serial;
        }
        event.type = type;
        event.source =
                TC2_OFFLINE_EVENT_SOURCE_PREDICTED_OFFLINE;
        event.tick = runtime->tick;
        event.serial = runtime->event_serial;
        event.train = train->train;
        event.conflict_train = conflict_train;
        event.waypoint_index = waypoint_index;
        event.route_offset =
                TC2_OFFLINE_RUNTIME_INVALID_INDEX;
        event.graph_node =
                TC2_OFFLINE_RUNTIME_INVALID_INDEX;
        event.sensor_index =
                TC2_OFFLINE_RUNTIME_INVALID_INDEX;
        event.switch_number =
                TC2_OFFLINE_RUNTIME_INVALID_INDEX;
        event.turnout_direction =
                TC2_OFFLINE_RUNTIME_INVALID_INDEX;
        event.destination_index =
                TC2_OFFLINE_RUNTIME_INVALID_INDEX;
        event.distance_um = train->progress_um;

        if (waypoint_index >= 0 &&
            waypoint_index < train->loaded_waypoint_count) {
                const tc2_dispatch_projection_waypoint *waypoint =
                        &train->waypoints[waypoint_index];
                event.route_offset = waypoint->route_offset;
                event.graph_node = waypoint->graph_node;
                event.sensor_index = waypoint->sensor_index;
                event.switch_number = waypoint->switch_number;
                event.turnout_direction =
                        waypoint->turnout_direction;
                event.destination_index =
                        waypoint->destination_index;
                event.distance_um = waypoint->distance_um;
                event.ui_row = waypoint->ui_row;
                event.ui_column = waypoint->ui_column;
                event.ui_width = waypoint->ui_width;
        }
        event.is_live = 0;
        train->last_event_tick = runtime->tick;
        if (sink) sink(&event, context);
}

static void fail_runtime_train(
        tc2_offline_runtime *runtime, tc2_offline_train *train,
        int error, tc2_offline_event_sink sink, void *context) {
        if (!runtime || !train) return;
        train->state = TC2_OFFLINE_TRAIN_FAILED;
        train->last_error = error;
        train->claim_count = 0;
        train->start_requested = 0;
        train->conflict_train =
                TC2_OFFLINE_RUNTIME_INVALID_TRAIN;
        ++train->motion_revision;
        emit_event(
                runtime, train, TC2_OFFLINE_EVENT_FAILED,
                event_waypoint_index(train),
                TC2_OFFLINE_RUNTIME_INVALID_TRAIN,
                sink, context);
}

static int candidate_priority_before(
        const tc2_offline_train *first,
        const tc2_offline_train *second) {
        uint32_t launch_delta;
        if (first->wait_ticks != second->wait_ticks) {
                return first->wait_ticks > second->wait_ticks;
        }
        if (first->header.launch_epoch !=
                    second->header.launch_epoch) {
                launch_delta =
                        first->header.launch_epoch -
                        second->header.launch_epoch;
                /*
                 * RFC-1982-style serial ordering is intentionally undefined
                 * at exactly half the uint32 range.  Fall through to the
                 * stable train-number tie break instead of allowing both
                 * directions to compare as "before".
                 */
                if (launch_delta != 0x80000000u) {
                        return (int32_t)launch_delta < 0;
                }
        }
        return first->train < second->train;
}

static int active_candidate(
        const tc2_offline_train *train) {
        if (!train || !train->start_requested) return 0;
        return train->state == TC2_OFFLINE_TRAIN_READY ||
               train->state == TC2_OFFLINE_TRAIN_RUNNING ||
               train->state ==
                       TC2_OFFLINE_TRAIN_WAIT_CONFLICT;
}

static void note_wait(
        tc2_offline_runtime *runtime, tc2_offline_train *train,
        int conflict_train, tc2_offline_event_sink sink,
        void *context) {
        if (!runtime || !train) return;
        /*
         * A train emits one WAIT event when it enters the wait state.  The
         * blocking owner can legitimately change while an older train clears
         * successive overlapping windows; updating that diagnostic must not
         * manufacture a second wait transition.
         */
        int changed =
                train->state !=
                        TC2_OFFLINE_TRAIN_WAIT_CONFLICT;
        train->state = TC2_OFFLINE_TRAIN_WAIT_CONFLICT;
        train->conflict_train = conflict_train;
        if (train->wait_ticks != UINT32_MAX) {
                ++train->wait_ticks;
        }
        if (changed) {
                ++train->motion_revision;
                emit_event(
                        runtime, train,
                        TC2_OFFLINE_EVENT_CONFLICT_WAIT,
                        event_waypoint_index(train),
                        conflict_train, sink, context);
        }
}

static void process_crossed_waypoints(
        tc2_offline_runtime *runtime, tc2_offline_train *train,
        tc2_offline_event_sink sink, void *context) {
        if (!runtime || !train) return;
        while (train->next_waypoint_index >= 0 &&
               train->next_waypoint_index <
                       train->loaded_waypoint_count &&
               train->waypoints[
                       train->next_waypoint_index].
                       distance_um <= train->progress_um) {
                int index = train->next_waypoint_index;
                const tc2_dispatch_projection_waypoint *waypoint =
                        &train->waypoints[index];
                ++train->next_waypoint_index;

                if (waypoint->kind ==
                            TC2_ROUTE_WAYPOINT_SENSOR) {
                        train->last_sensor_index =
                                waypoint->sensor_index;
                        train->last_sensor_tick = runtime->tick;
                        emit_event(
                                runtime, train,
                                TC2_OFFLINE_EVENT_PREDICTED_SENSOR_PULSE,
                                index,
                                TC2_OFFLINE_RUNTIME_INVALID_TRAIN,
                                sink, context);
                } else if (waypoint->kind ==
                                   TC2_ROUTE_WAYPOINT_REVERSAL) {
                        train->last_sensor_index =
                                waypoint->sensor_index;
                        train->last_sensor_tick = runtime->tick;
                        emit_event(
                                runtime, train,
                                TC2_OFFLINE_EVENT_PREDICTED_SENSOR_PULSE,
                                index,
                                TC2_OFFLINE_RUNTIME_INVALID_TRAIN,
                                sink, context);
                        emit_event(
                                runtime, train,
                                TC2_OFFLINE_EVENT_PREDICTED_REVERSAL,
                                index,
                                TC2_OFFLINE_RUNTIME_INVALID_TRAIN,
                                sink, context);
                } else if (waypoint->kind ==
                                   TC2_ROUTE_WAYPOINT_TURNOUT) {
                        emit_event(
                                runtime, train,
                                TC2_OFFLINE_EVENT_PREDICTED_TURNOUT,
                                index,
                                TC2_OFFLINE_RUNTIME_INVALID_TRAIN,
                                sink, context);
                } else if (waypoint->kind ==
                                   TC2_ROUTE_WAYPOINT_DESTINATION) {
                        train->state =
                                TC2_OFFLINE_TRAIN_ARRIVED;
                        train->start_requested = 0;
                        train->conflict_train =
                                TC2_OFFLINE_RUNTIME_INVALID_TRAIN;
                        train->wait_ticks = 0;
                        emit_event(
                                runtime, train,
                                TC2_OFFLINE_EVENT_ARRIVED,
                                index,
                                TC2_OFFLINE_RUNTIME_INVALID_TRAIN,
                                sink, context);
                }
        }
}

static void accept_candidate(
        tc2_offline_runtime *runtime, tc2_offline_train *train,
        int64_t candidate_um, int candidate_claim_count,
        tc2_offline_event_sink sink, void *context) {
        if (!runtime || !train) return;
        tc2_offline_train_state old_state = train->state;
        if (copy_claims(
                    train->claims, runtime->scratch_claims,
                    candidate_claim_count) != TC2_OFFLINE_OK) {
                fail_runtime_train(
                        runtime, train, TC2_OFFLINE_OVERFLOW,
                        sink, context);
                return;
        }
        train->claim_count = candidate_claim_count;
        train->conflict_train =
                TC2_OFFLINE_RUNTIME_INVALID_TRAIN;
        train->wait_ticks = 0;
        train->state = TC2_OFFLINE_TRAIN_RUNNING;
        if (old_state ==
                    TC2_OFFLINE_TRAIN_WAIT_CONFLICT) {
                emit_event(
                        runtime, train,
                        TC2_OFFLINE_EVENT_CONFLICT_RESUMED,
                        event_waypoint_index(train),
                        TC2_OFFLINE_RUNTIME_INVALID_TRAIN,
                        sink, context);
        }
        if (!train->entered_route) {
                train->entered_route = 1;
                emit_event(
                        runtime, train,
                        TC2_OFFLINE_EVENT_STARTED, 0,
                        TC2_OFFLINE_RUNTIME_INVALID_TRAIN,
                        sink, context);
        }

        if (candidate_um != train->progress_um) {
                train->progress_um = candidate_um;
                ++train->motion_revision;
        }
        process_crossed_waypoints(
                runtime, train, sink, context);
}

static int conflict_owner_for_candidate(
        const tc2_offline_runtime *runtime, int candidate_slot,
        const tc2_offline_resource_claim *claims,
        int claim_count) {
        if (!runtime || !claims) {
                return TC2_OFFLINE_RUNTIME_INVALID_TRAIN;
        }
        int owner = TC2_OFFLINE_RUNTIME_INVALID_TRAIN;
        for (int slot = 0;
             slot < TC2_OFFLINE_RUNTIME_MAX_TRAINS; ++slot) {
                if (slot == candidate_slot) continue;
                const tc2_offline_train *other =
                        &runtime->trains[slot];
                if (other->state == TC2_OFFLINE_TRAIN_EMPTY ||
                    other->state == TC2_OFFLINE_TRAIN_LOADING ||
                    other->state == TC2_OFFLINE_TRAIN_READY ||
                    other->state == TC2_OFFLINE_TRAIN_FAILED ||
                    other->claim_count <= 0) {
                        continue;
                }
                if (claims_conflict(
                            claims, claim_count,
                            other->claims,
                            other->claim_count) &&
                    (owner ==
                             TC2_OFFLINE_RUNTIME_INVALID_TRAIN ||
                     other->train < owner)) {
                        owner = other->train;
                }
        }
        return owner;
}

static void normalize_accepted_claims(
        tc2_offline_runtime *runtime, tc2_offline_train *train,
        tc2_offline_event_sink sink, void *context) {
        if (!runtime || !train ||
            (train->state != TC2_OFFLINE_TRAIN_RUNNING &&
             train->state != TC2_OFFLINE_TRAIN_ARRIVED)) {
                return;
        }
        int claim_count = 0;
        int status = build_stationary_claims(
                train, runtime->scratch_claims, &claim_count);
        if (status != TC2_OFFLINE_OK ||
            copy_claims(
                    train->claims, runtime->scratch_claims,
                    claim_count) != TC2_OFFLINE_OK) {
                fail_runtime_train(
                        runtime, train,
                        status == TC2_OFFLINE_OK ?
                                TC2_OFFLINE_OVERFLOW : status,
                        sink, context);
                return;
        }
        train->claim_count = claim_count;
}

static void advance_one_tick(
        tc2_offline_runtime *runtime,
        tc2_offline_event_sink sink, void *context) {
        int order[TC2_OFFLINE_RUNTIME_MAX_TRAINS];
        int order_count = 0;

        for (int slot = 0;
             slot < TC2_OFFLINE_RUNTIME_MAX_TRAINS; ++slot) {
                if (!active_candidate(&runtime->trains[slot])) {
                        continue;
                }
                int insert = order_count;
                while (insert > 0 &&
                       candidate_priority_before(
                               &runtime->trains[slot],
                               &runtime->trains[
                                       order[insert - 1]])) {
                        order[insert] = order[insert - 1];
                        --insert;
                }
                order[insert] = slot;
                ++order_count;
        }

        int accepted[TC2_OFFLINE_RUNTIME_MAX_TRAINS];
        zero_bytes(accepted, sizeof(accepted));

        for (int order_index = 0;
             order_index < order_count; ++order_index) {
                int slot = order[order_index];
                tc2_offline_train *train =
                        &runtime->trains[slot];
                int64_t endpoint =
                        train->header.
                          physical_destination_distance_um;
                int velocity =
                        Tc2MotionProvisionalVelocityUmPerTick(
                                train->header.speed);
                if (velocity <= 0 ||
                    train->progress_um < 0 ||
                    train->progress_um > endpoint ||
                    train->progress_um >
                            INT64_MAX - velocity) {
                        fail_runtime_train(
                                runtime, train,
                                TC2_OFFLINE_OVERFLOW,
                                sink, context);
                        continue;
                }
                train->velocity_um_per_tick = velocity;
                int64_t candidate =
                        train->progress_um + velocity;
                if (candidate > endpoint) candidate = endpoint;

                int claim_count = 0;
                int status = build_transit_claims(
                        train, candidate,
                        runtime->scratch_claims,
                        &claim_count);
                if (status != TC2_OFFLINE_OK) {
                        fail_runtime_train(
                                runtime, train, status,
                                sink, context);
                        continue;
                }
                int conflict_train =
                        conflict_owner_for_candidate(
                                runtime, slot,
                                runtime->scratch_claims,
                                claim_count);
                if (conflict_train !=
                            TC2_OFFLINE_RUNTIME_INVALID_TRAIN) {
                        note_wait(
                                runtime, train, conflict_train,
                                sink, context);
                        continue;
                }
                accept_candidate(
                        runtime, train, candidate, claim_count,
                        sink, context);
                accepted[slot] = 1;
        }

        /*
         * Transit claims remain conservative through the complete arbitration
         * pass.  Shrinking them afterward cannot introduce a new overlap.
         */
        for (int slot = 0;
             slot < TC2_OFFLINE_RUNTIME_MAX_TRAINS; ++slot) {
                if (accepted[slot]) {
                        normalize_accepted_claims(
                                runtime,
                                &runtime->trains[slot],
                                sink, context);
                }
        }
}

int Tc2OfflineRuntimeInitialize(
        tc2_offline_runtime *runtime, uint32_t initial_tick) {
        if (!runtime) return TC2_OFFLINE_INVALID_ARGUMENT;
        zero_bytes(runtime, sizeof(*runtime));
        runtime->tick = initial_tick;
        runtime->initialized = 1;
        for (int slot = 0;
             slot < TC2_OFFLINE_RUNTIME_MAX_TRAINS; ++slot) {
                clear_train(&runtime->trains[slot]);
        }
        return TC2_OFFLINE_OK;
}

void Tc2OfflineRuntimeReset(
        tc2_offline_runtime *runtime, uint32_t initial_tick) {
        if (!runtime) return;
        (void)Tc2OfflineRuntimeInitialize(runtime, initial_tick);
}

int Tc2OfflineRuntimeBeginTrip(
        tc2_offline_runtime *runtime,
        const tc2_dispatch_projection_header *header) {
        if (!runtime_ready(runtime) || !header) {
                return TC2_OFFLINE_INVALID_ARGUMENT;
        }
        if (runtime->in_step) return TC2_OFFLINE_INVALID_STATE;
        if (ranges_overlap(
                    runtime, sizeof(*runtime),
                    header, sizeof(*header))) {
                return TC2_OFFLINE_ALIAS;
        }
        if (!valid_header(header)) {
                return TC2_OFFLINE_MALFORMED_PROJECTION;
        }
        if (find_train_slot(runtime, header->train) >= 0) {
                return TC2_OFFLINE_DUPLICATE_TRAIN;
        }
        int slot = find_empty_slot(runtime);
        if (slot < 0) return TC2_OFFLINE_CAPACITY;

        tc2_offline_train *train = &runtime->trains[slot];
        clear_train(train);
        train->state = TC2_OFFLINE_TRAIN_LOADING;
        train->train = header->train;
        train->header = *header;
        train->velocity_um_per_tick =
                Tc2MotionProvisionalVelocityUmPerTick(
                        header->speed);
        train->conflict_train =
                TC2_OFFLINE_RUNTIME_INVALID_TRAIN;
        train->last_sensor_index =
                TC2_OFFLINE_RUNTIME_INVALID_INDEX;
        train->next_waypoint_index =
                TC2_OFFLINE_RUNTIME_INVALID_INDEX;
        return TC2_OFFLINE_OK;
}

int Tc2OfflineRuntimeAppendProjectionPage(
        tc2_offline_runtime *runtime,
        const tc2_dispatch_projection_page *page) {
        if (!runtime_ready(runtime) || !page) {
                return TC2_OFFLINE_INVALID_ARGUMENT;
        }
        if (runtime->in_step) return TC2_OFFLINE_INVALID_STATE;
        if (ranges_overlap(
                    runtime, sizeof(*runtime),
                    page, sizeof(*page))) {
                return TC2_OFFLINE_ALIAS;
        }
        if (page->train < 1 || page->train > 255) {
                return TC2_OFFLINE_INVALID_ARGUMENT;
        }
        int slot = find_train_slot(runtime, page->train);
        if (slot < 0) return TC2_OFFLINE_NOT_FOUND;
        tc2_offline_train *train = &runtime->trains[slot];
        if (train->state != TC2_OFFLINE_TRAIN_LOADING) {
                return train->state ==
                                TC2_OFFLINE_TRAIN_FAILED ?
                        train->last_error :
                        TC2_OFFLINE_INVALID_STATE;
        }
        if (page->status != TC2_DISPATCH_PROJECTION_OK ||
            page->publication_serial !=
                    train->header.publication_serial ||
            page->plan_generation !=
                    train->header.plan_generation ||
            page->launch_epoch !=
                    train->header.launch_epoch) {
                fail_loading_train(train, TC2_OFFLINE_STALE);
                return TC2_OFFLINE_STALE;
        }
        if (page->first_waypoint !=
                    train->loaded_waypoint_count ||
            page->total_waypoints !=
                    train->header.waypoint_count ||
            page->count < 1 ||
            page->count >
                    TC2_DISPATCH_PROJECTION_PAGE_CAPACITY ||
            page->first_waypoint < 0 ||
            page->first_waypoint >
                    train->header.waypoint_count -
                    page->count) {
                fail_loading_train(train, TC2_OFFLINE_BOUNDS);
                return TC2_OFFLINE_BOUNDS;
        }
        int expected_more =
                page->first_waypoint + page->count <
                page->total_waypoints;
        if (page->has_more != expected_more) {
                fail_loading_train(
                        train,
                        TC2_OFFLINE_MALFORMED_PROJECTION);
                return TC2_OFFLINE_MALFORMED_PROJECTION;
        }
        for (int index = page->count;
             index < TC2_DISPATCH_PROJECTION_PAGE_CAPACITY;
             ++index) {
                if (!bytes_are_zero(
                            &page->waypoints[index],
                            sizeof(page->waypoints[index]))) {
                        fail_loading_train(
                                train,
                                TC2_OFFLINE_MALFORMED_PROJECTION);
                        return TC2_OFFLINE_MALFORMED_PROJECTION;
                }
        }
        for (int index = 0; index < page->count; ++index) {
                train->waypoints[
                        train->loaded_waypoint_count + index] =
                        page->waypoints[index];
        }
        train->loaded_waypoint_count += page->count;
        return TC2_OFFLINE_OK;
}

int Tc2OfflineRuntimeCommitTrip(
        tc2_offline_runtime *runtime, int train_number) {
        if (!runtime_ready(runtime)) {
                return TC2_OFFLINE_INVALID_ARGUMENT;
        }
        if (runtime->in_step) return TC2_OFFLINE_INVALID_STATE;
        int slot = find_train_slot(runtime, train_number);
        if (slot < 0) return TC2_OFFLINE_NOT_FOUND;
        tc2_offline_train *train = &runtime->trains[slot];
        if (train->state != TC2_OFFLINE_TRAIN_LOADING) {
                return train->state ==
                                TC2_OFFLINE_TRAIN_FAILED ?
                        train->last_error :
                        TC2_OFFLINE_INVALID_STATE;
        }
        if (train->loaded_waypoint_count !=
                    train->header.waypoint_count) {
                fail_loading_train(train, TC2_OFFLINE_BOUNDS);
                return TC2_OFFLINE_BOUNDS;
        }
        int status = validate_projection(
                &train->header, train->waypoints,
                train->loaded_waypoint_count);
        if (status != TC2_OFFLINE_OK) {
                fail_loading_train(train, status);
                return status;
        }
        train->state = TC2_OFFLINE_TRAIN_READY;
        train->last_error = TC2_OFFLINE_OK;
        train->next_waypoint_index = 1;
        train->progress_um = 0;
        train->claim_count = 0;
        train->start_requested = 0;
        train->entered_route = 0;
        train->wait_ticks = 0;
        return TC2_OFFLINE_OK;
}

int Tc2OfflineRuntimeAbortTrip(
        tc2_offline_runtime *runtime, int train_number) {
        if (!runtime_ready(runtime)) {
                return TC2_OFFLINE_INVALID_ARGUMENT;
        }
        if (runtime->in_step) return TC2_OFFLINE_INVALID_STATE;
        int slot = find_train_slot(runtime, train_number);
        if (slot < 0) return TC2_OFFLINE_NOT_FOUND;
        tc2_offline_train *train = &runtime->trains[slot];
        if (train->state != TC2_OFFLINE_TRAIN_LOADING &&
            train->state != TC2_OFFLINE_TRAIN_FAILED) {
                return TC2_OFFLINE_INVALID_STATE;
        }
        clear_train(train);
        return TC2_OFFLINE_OK;
}

int Tc2OfflineRuntimeAddTrip(
        tc2_offline_runtime *runtime,
        const tc2_dispatch_projection_header *header,
        const tc2_dispatch_projection_waypoint *waypoints,
        int waypoint_count) {
        if (!runtime_ready(runtime) || !header || !waypoints ||
            waypoint_count < 0) {
                return TC2_OFFLINE_INVALID_ARGUMENT;
        }
        if (runtime->in_step) return TC2_OFFLINE_INVALID_STATE;
        if (waypoint_count >
                    TC2_OFFLINE_RUNTIME_MAX_WAYPOINTS) {
                return TC2_OFFLINE_BOUNDS;
        }
        if (ranges_overlap(
                    runtime, sizeof(*runtime),
                    header, sizeof(*header)) ||
            ranges_overlap(
                    runtime, sizeof(*runtime),
                    waypoints,
                    (size_t)waypoint_count *
                            sizeof(*waypoints))) {
                return TC2_OFFLINE_ALIAS;
        }
        int validation = validate_projection(
                header, waypoints, waypoint_count);
        if (validation != TC2_OFFLINE_OK) return validation;

        int status = Tc2OfflineRuntimeBeginTrip(
                runtime, header);
        if (status != TC2_OFFLINE_OK) return status;
        int slot = find_train_slot(runtime, header->train);
        if (slot < 0) return TC2_OFFLINE_NOT_FOUND;
        tc2_offline_train *train = &runtime->trains[slot];
        for (int index = 0; index < waypoint_count; ++index) {
                train->waypoints[index] = waypoints[index];
        }
        train->loaded_waypoint_count = waypoint_count;
        status = Tc2OfflineRuntimeCommitTrip(
                runtime, header->train);
        if (status != TC2_OFFLINE_OK) {
                clear_train(train);
        }
        return status;
}

int Tc2OfflineRuntimeStartTrip(
        tc2_offline_runtime *runtime, int train_number) {
        if (!runtime_ready(runtime)) {
                return TC2_OFFLINE_INVALID_ARGUMENT;
        }
        if (runtime->in_step) return TC2_OFFLINE_INVALID_STATE;
        int slot = find_train_slot(runtime, train_number);
        if (slot < 0) return TC2_OFFLINE_NOT_FOUND;
        tc2_offline_train *train = &runtime->trains[slot];
        if (train->state != TC2_OFFLINE_TRAIN_READY) {
                return TC2_OFFLINE_INVALID_STATE;
        }
        train->start_requested = 1;
        return TC2_OFFLINE_OK;
}

int Tc2OfflineRuntimeStartAll(tc2_offline_runtime *runtime) {
        if (!runtime_ready(runtime)) {
                return TC2_OFFLINE_INVALID_ARGUMENT;
        }
        if (runtime->in_step) return TC2_OFFLINE_INVALID_STATE;
        for (int slot = 0;
             slot < TC2_OFFLINE_RUNTIME_MAX_TRAINS; ++slot) {
                if (runtime->trains[slot].state ==
                            TC2_OFFLINE_TRAIN_READY) {
                        runtime->trains[slot].
                                start_requested = 1;
                }
        }
        return TC2_OFFLINE_OK;
}

int Tc2OfflineRuntimeRemoveTrip(
        tc2_offline_runtime *runtime, int train_number) {
        if (!runtime_ready(runtime)) {
                return TC2_OFFLINE_INVALID_ARGUMENT;
        }
        if (runtime->in_step) return TC2_OFFLINE_INVALID_STATE;
        int slot = find_train_slot(runtime, train_number);
        if (slot < 0) return TC2_OFFLINE_NOT_FOUND;
        clear_train(&runtime->trains[slot]);
        return TC2_OFFLINE_OK;
}

int Tc2OfflineRuntimeStep(
        tc2_offline_runtime *runtime, uint32_t now_tick,
        tc2_offline_event_sink sink, void *context) {
        if (!runtime_ready(runtime)) {
                return TC2_OFFLINE_INVALID_ARGUMENT;
        }
        if (runtime->in_step) return TC2_OFFLINE_INVALID_STATE;
        uint32_t elapsed = now_tick - runtime->tick;
        if (elapsed > TC2_OFFLINE_RUNTIME_MAX_STEP_TICKS) {
                return TC2_OFFLINE_BOUNDS;
        }
        if (elapsed == 0) return TC2_OFFLINE_OK;

        runtime->in_step = 1;
        for (uint32_t step = 0; step < elapsed; ++step) {
                ++runtime->tick;
                advance_one_tick(runtime, sink, context);
        }
        runtime->in_step = 0;
        return TC2_OFFLINE_OK;
}

static void fill_train_snapshot(
        const tc2_offline_train *train,
        tc2_offline_train_snapshot *snapshot) {
        zero_bytes(snapshot, sizeof(*snapshot));
        snapshot->state = train->state;
        snapshot->last_error = train->last_error;
        snapshot->start_requested = train->start_requested;
        snapshot->train = train->train;
        snapshot->speed = train->header.speed;
        snapshot->start_index = train->header.start_index;
        snapshot->destination_index =
                train->header.destination_index;
        snapshot->destination_side =
                train->header.destination_side;
        snapshot->conflict_train = train->conflict_train;
        snapshot->wait_ticks = train->wait_ticks;
        snapshot->motion_revision = train->motion_revision;
        snapshot->last_event_tick = train->last_event_tick;
        snapshot->last_sensor_tick = train->last_sensor_tick;
        snapshot->last_sensor_index = train->last_sensor_index;
        snapshot->publication_serial =
                train->header.publication_serial;
        snapshot->plan_generation =
                train->header.plan_generation;
        snapshot->launch_epoch = train->header.launch_epoch;
        snapshot->progress_um = train->progress_um;
        snapshot->destination_distance_um =
                train->header.
                  physical_destination_distance_um;
        snapshot->velocity_um_per_tick =
                train->velocity_um_per_tick;
        snapshot->previous_waypoint_index =
                TC2_OFFLINE_RUNTIME_INVALID_INDEX;
        snapshot->next_waypoint_index =
                TC2_OFFLINE_RUNTIME_INVALID_INDEX;
        snapshot->route_offset =
                TC2_OFFLINE_RUNTIME_INVALID_INDEX;
        snapshot->graph_node =
                TC2_OFFLINE_RUNTIME_INVALID_INDEX;
        snapshot->next_graph_node =
                TC2_OFFLINE_RUNTIME_INVALID_INDEX;

        if (train->loaded_waypoint_count < 1) return;
        int previous = previous_waypoint_for_progress(
                train, train->progress_um);
        if (previous < 0) previous = 0;
        int next = previous + 1;
        if (next >= train->loaded_waypoint_count) {
                next = TC2_OFFLINE_RUNTIME_INVALID_INDEX;
        }
        snapshot->previous_waypoint_index = previous;
        snapshot->next_waypoint_index = next;
        snapshot->route_offset =
                train->waypoints[previous].route_offset;
        snapshot->graph_node =
                train->waypoints[previous].graph_node;
        snapshot->ui_row_milli =
                (int)train->waypoints[previous].ui_row *
                TC2_OFFLINE_RUNTIME_UI_SCALE;
        snapshot->ui_column_milli =
                (int)train->waypoints[previous].ui_column *
                TC2_OFFLINE_RUNTIME_UI_SCALE;
        snapshot->ui_width =
                train->waypoints[previous].ui_width;
        if (next < 0) return;

        snapshot->next_graph_node =
                train->waypoints[next].graph_node;
        int64_t first_distance =
                train->waypoints[previous].distance_um;
        int64_t second_distance =
                train->waypoints[next].distance_um;
        int64_t span = second_distance - first_distance;
        int64_t offset = train->progress_um - first_distance;
        if (span <= 0 || offset <= 0) return;
        if (offset > span) offset = span;
        int row_delta =
                (int)train->waypoints[next].ui_row -
                (int)train->waypoints[previous].ui_row;
        int column_delta =
                (int)train->waypoints[next].ui_column -
                (int)train->waypoints[previous].ui_column;
        snapshot->ui_row_milli +=
                (int)(((int64_t)row_delta *
                       TC2_OFFLINE_RUNTIME_UI_SCALE *
                       offset) / span);
        snapshot->ui_column_milli +=
                (int)(((int64_t)column_delta *
                       TC2_OFFLINE_RUNTIME_UI_SCALE *
                       offset) / span);
        {
                int row =
                        (snapshot->ui_row_milli +
                         TC2_OFFLINE_RUNTIME_UI_SCALE / 2) /
                        TC2_OFFLINE_RUNTIME_UI_SCALE;
                int column =
                        (snapshot->ui_column_milli +
                         TC2_OFFLINE_RUNTIME_UI_SCALE / 2) /
                        TC2_OFFLINE_RUNTIME_UI_SCALE;
                if (!Tc2TrackDLayoutCellIsOccupied(row, column)) {
                        int selected =
                                offset * 2 >= span ? next : previous;
                        snapshot->ui_row_milli =
                                (int)train->waypoints[selected].ui_row *
                                TC2_OFFLINE_RUNTIME_UI_SCALE;
                        snapshot->ui_column_milli =
                                (int)train->waypoints[selected].ui_column *
                                TC2_OFFLINE_RUNTIME_UI_SCALE;
                        snapshot->ui_width =
                                train->waypoints[selected].ui_width;
                }
        }
}

int Tc2OfflineRuntimeGetTrainSnapshot(
        const tc2_offline_runtime *runtime, int train_number,
        tc2_offline_train_snapshot *snapshot) {
        if (!runtime_ready(runtime) || !snapshot) {
                return TC2_OFFLINE_INVALID_ARGUMENT;
        }
        if (ranges_overlap(
                    runtime, sizeof(*runtime),
                    snapshot, sizeof(*snapshot))) {
                return TC2_OFFLINE_ALIAS;
        }
        int slot = find_train_slot(runtime, train_number);
        if (slot < 0) return TC2_OFFLINE_NOT_FOUND;
        fill_train_snapshot(&runtime->trains[slot], snapshot);
        return TC2_OFFLINE_OK;
}

int Tc2OfflineRuntimeGetSnapshot(
        const tc2_offline_runtime *runtime,
        tc2_offline_snapshot *snapshot) {
        if (!runtime_ready(runtime) || !snapshot) {
                return TC2_OFFLINE_INVALID_ARGUMENT;
        }
        if (ranges_overlap(
                    runtime, sizeof(*runtime),
                    snapshot, sizeof(*snapshot))) {
                return TC2_OFFLINE_ALIAS;
        }
        zero_bytes(snapshot, sizeof(*snapshot));
        snapshot->tick = runtime->tick;
        snapshot->event_serial = runtime->event_serial;
        for (int slot = 0;
             slot < TC2_OFFLINE_RUNTIME_MAX_TRAINS; ++slot) {
                const tc2_offline_train *train =
                        &runtime->trains[slot];
                if (train->state ==
                            TC2_OFFLINE_TRAIN_EMPTY) {
                        continue;
                }
                int output = snapshot->train_count;
                fill_train_snapshot(
                        train, &snapshot->trains[output]);
                ++snapshot->train_count;
                if (train->state ==
                            TC2_OFFLINE_TRAIN_RUNNING) {
                        ++snapshot->running_count;
                } else if (train->state ==
                                   TC2_OFFLINE_TRAIN_WAIT_CONFLICT) {
                        ++snapshot->waiting_count;
                } else if (train->state ==
                                   TC2_OFFLINE_TRAIN_ARRIVED) {
                        ++snapshot->arrived_count;
                }
        }
        return TC2_OFFLINE_OK;
}

const char *Tc2OfflineRuntimeStateName(
        tc2_offline_train_state state) {
        if (state == TC2_OFFLINE_TRAIN_EMPTY) return "EMPTY";
        if (state == TC2_OFFLINE_TRAIN_LOADING) return "LOADING";
        if (state == TC2_OFFLINE_TRAIN_READY) return "READY";
        if (state == TC2_OFFLINE_TRAIN_RUNNING) return "RUNNING";
        if (state == TC2_OFFLINE_TRAIN_WAIT_CONFLICT) {
                return "WAIT_CONFLICT";
        }
        if (state == TC2_OFFLINE_TRAIN_ARRIVED) return "ARRIVED";
        if (state == TC2_OFFLINE_TRAIN_FAILED) return "FAILED";
        return "INVALID";
}

const char *Tc2OfflineRuntimeEventName(
        tc2_offline_event_type type) {
        if (type == TC2_OFFLINE_EVENT_NONE) return "NONE";
        if (type == TC2_OFFLINE_EVENT_STARTED) return "STARTED";
        if (type == TC2_OFFLINE_EVENT_CONFLICT_WAIT) {
                return "CONFLICT_WAIT";
        }
        if (type == TC2_OFFLINE_EVENT_CONFLICT_RESUMED) {
                return "CONFLICT_RESUMED";
        }
        if (type == TC2_OFFLINE_EVENT_PREDICTED_SENSOR_PULSE) {
                return "PREDICTED_SENSOR_PULSE";
        }
        if (type == TC2_OFFLINE_EVENT_PREDICTED_TURNOUT) {
                return "PREDICTED_TURNOUT";
        }
        if (type == TC2_OFFLINE_EVENT_PREDICTED_REVERSAL) {
                return "PREDICTED_REVERSAL";
        }
        if (type == TC2_OFFLINE_EVENT_ARRIVED) return "ARRIVED";
        if (type == TC2_OFFLINE_EVENT_FAILED) return "FAILED";
        return "INVALID";
}

#endif
