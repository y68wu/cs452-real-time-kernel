#include <limits.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "../tc2_offline_runtime.h"
#include "../tc2_track_d_layout.h"
#include "../tc2_ui_overlay.h"

enum {
        EVENT_LOG_CAPACITY = 512,
        MATRIX_TRIP_COUNT =
                TC2_TRACK_D_START_COUNT *
                TC2_TRACK_D_DESTINATION_COUNT * 120
};

typedef struct {
        tc2_dispatch_projection_header header;
        tc2_dispatch_projection_waypoint
                waypoints[TC2_OFFLINE_RUNTIME_MAX_WAYPOINTS];
} projection_fixture;

typedef struct {
        tc2_offline_event events[EVENT_LOG_CAPACITY];
        int count;
        int overflow;
        int invalid_provenance;
} event_log;

typedef struct {
        event_log log;
        tc2_ui_overlay *overlay;
        projection_fixture *fixtures;
        int fixture_count;
        int overlay_error;
} overlay_event_log;

static int fail(const char *message) {
        fprintf(stderr, "tc2_offline_runtime_test: %s\n", message);
        return 1;
}

static void capture_event(
        const tc2_offline_event *event, void *context) {
        event_log *log = (event_log *)context;
        if (!event || !log) return;
        if (event->source !=
                    TC2_OFFLINE_EVENT_SOURCE_PREDICTED_OFFLINE ||
            event->is_live != 0 || event->serial == 0) {
                log->invalid_provenance = 1;
        }
        if (log->count >= EVENT_LOG_CAPACITY) {
                log->overflow = 1;
                return;
        }
        log->events[log->count++] = *event;
}

static const projection_fixture *fixture_for_train(
        const overlay_event_log *context, int train) {
        if (!context || !context->fixtures) return 0;
        for (int index = 0;
             index < context->fixture_count; ++index) {
                if (context->fixtures[index].header.train == train) {
                        return &context->fixtures[index];
                }
        }
        return 0;
}

static void capture_overlay_event(
        const tc2_offline_event *event, void *opaque) {
        overlay_event_log *context =
                (overlay_event_log *)opaque;
        const projection_fixture *fixture;
        int status = 0;

        if (!context) return;
        capture_event(event, &context->log);
        if (!event || !context->overlay) return;
        fixture = fixture_for_train(context, event->train);
        if (!fixture) {
                context->overlay_error = 1;
                return;
        }

        if (event->type ==
                    TC2_OFFLINE_EVENT_PREDICTED_SENSOR_PULSE) {
                status = Tc2UiOverlayUpdateAtSensor(
                        context->overlay, event->train,
                        event->sensor_index,
                        fixture->header.plan_generation,
                        event->serial, event->serial,
                        TC2_UI_QUALITY_PREDICTED,
                        event->tick);
        } else if (event->type ==
                           TC2_OFFLINE_EVENT_PREDICTED_TURNOUT ||
                   event->type ==
                           TC2_OFFLINE_EVENT_PREDICTED_REVERSAL) {
                status = Tc2UiOverlayUpdatePredictedCell(
                        context->overlay, event->train,
                        event->ui_row, event->ui_column,
                        fixture->header.plan_generation,
                        event->serial, event->tick);
        } else if (event->type ==
                           TC2_OFFLINE_EVENT_ARRIVED) {
                status = Tc2UiOverlayUpdateAtDestination(
                        context->overlay, event->train,
                        fixture->header.destination_index,
                        fixture->header.plan_generation,
                        event->serial,
                        TC2_UI_QUALITY_ESTIMATED,
                        event->tick);
        }
        if (status < 0) context->overlay_error = 1;
}

static void initialize_waypoint(
        tc2_dispatch_projection_waypoint *waypoint) {
        if (!waypoint) return;
        memset(waypoint, 0, sizeof(*waypoint));
        waypoint->route_offset = -1;
        waypoint->graph_node = -1;
        waypoint->sensor_index = -1;
        waypoint->switch_number = -1;
        waypoint->turnout_direction = -1;
        waypoint->destination_index = -1;
}

static int set_start_waypoint(
        tc2_dispatch_projection_waypoint *waypoint,
        int start_index, int graph_node, int route_offset,
        int64_t distance_um) {
        const tc2_track_d_start_layout *layout =
                Tc2TrackDStartLayout((size_t)start_index);
        if (!waypoint || !layout) return -1;
        initialize_waypoint(waypoint);
        waypoint->kind = TC2_ROUTE_WAYPOINT_START;
        waypoint->route_offset = (int16_t)route_offset;
        waypoint->graph_node = (int16_t)graph_node;
        waypoint->distance_um = distance_um;
        waypoint->ui_row = layout->row;
        waypoint->ui_column = layout->column;
        waypoint->ui_width = 1;
        return 0;
}

static int set_sensor_waypoint(
        tc2_dispatch_projection_waypoint *waypoint,
        int sensor_index, int graph_node, int route_offset,
        int64_t distance_um) {
        tc2_track_d_directed_sensor_cell cell;
        if (!waypoint ||
            Tc2TrackDDirectedSensorCell(sensor_index, &cell) < 0) {
                return -1;
        }
        initialize_waypoint(waypoint);
        waypoint->kind = TC2_ROUTE_WAYPOINT_SENSOR;
        waypoint->route_offset = (int16_t)route_offset;
        waypoint->graph_node = (int16_t)graph_node;
        waypoint->sensor_index = (int16_t)sensor_index;
        waypoint->distance_um = distance_um;
        waypoint->ui_row = cell.row;
        waypoint->ui_column = cell.column;
        waypoint->ui_width = cell.width;
        return 0;
}

static int set_destination_waypoint(
        tc2_dispatch_projection_waypoint *waypoint,
        int destination_index, int graph_node, int route_offset,
        int64_t distance_um) {
        const tc2_track_d_destination_layout *layout =
                Tc2TrackDDestinationLayout(
                        (size_t)destination_index);
        if (!waypoint || !layout) return -1;
        initialize_waypoint(waypoint);
        waypoint->kind = TC2_ROUTE_WAYPOINT_DESTINATION;
        waypoint->route_offset = (int16_t)route_offset;
        waypoint->graph_node = (int16_t)graph_node;
        waypoint->destination_index =
                (int8_t)destination_index;
        waypoint->distance_um = distance_um;
        waypoint->ui_row = layout->row;
        waypoint->ui_column = layout->column;
        waypoint->ui_width = 2;
        return 0;
}

static int initialize_header(
        projection_fixture *fixture, int train,
        int start_index, int destination_index,
        int destination_side, int speed,
        uint64_t publication_serial,
        uint32_t plan_generation, uint32_t launch_epoch,
        int waypoint_count, int sensor_count,
        int geometry_anchor_route_offset,
        int destination_route_offset,
        int64_t destination_distance_um) {
        tc2_dispatch_projection_header *header;
        if (!fixture || train < 1 || train > 255 ||
            start_index < 0 ||
            start_index >= TC2_TRACK_D_START_COUNT ||
            destination_index < 0 ||
            destination_index >=
                    TC2_TRACK_D_DESTINATION_COUNT ||
            destination_side < 0 ||
            destination_side >=
                    TC2_TRACK_DESTINATION_SIDE_COUNT ||
            speed < 1 || speed > 120 ||
            publication_serial == 0 ||
            plan_generation == 0 || launch_epoch == 0 ||
            waypoint_count < 2 ||
            waypoint_count >
                    TC2_OFFLINE_RUNTIME_MAX_WAYPOINTS ||
            sensor_count < 0 ||
            destination_distance_um <= 0) {
                return -1;
        }

        memset(fixture, 0, sizeof(*fixture));
        header = &fixture->header;
        header->status = TC2_DISPATCH_PROJECTION_OK;
        header->valid = 1;
        header->train = train;
        header->job_state = TC2_JOB_READY;
        header->publication_serial = publication_serial;
        header->physical_destination_distance_um =
                destination_distance_um;
        header->plan_generation = plan_generation;
        header->launch_epoch = launch_epoch;
        header->scheduler_healthy = 1;
        header->start_index = start_index;
        header->destination_index = destination_index;
        header->destination_side = destination_side;
        header->speed = speed;
        header->waypoint_count = waypoint_count;
        header->sensor_count = sensor_count;
        header->turnout_count = 0;
        header->reversal_count = 0;
        header->geometry_anchor_route_offset =
                geometry_anchor_route_offset;
        header->physical_destination_route_offset =
                destination_route_offset;
        header->last_visible_route_offset =
                destination_route_offset;
        header->operator_destination_waypoint_index =
                waypoint_count - 1;
        header->physical_destination_offset_mm =
                (int)(destination_distance_um / 1000);
        return 0;
}

static int make_basic_fixture(
        projection_fixture *fixture, int train,
        int start_index, int destination_index,
        int destination_side, int speed,
        uint64_t serial, uint32_t generation,
        uint32_t launch_epoch, int lane,
        int64_t destination_distance_um) {
        int graph_base = lane * 3;
        int sensor_index =
                (start_index * 16 + destination_index * 2 +
                 destination_side) %
                TC2_TRACK_D_DIRECTED_SENSOR_COUNT;
        if (initialize_header(
                    fixture, train, start_index,
                    destination_index, destination_side, speed,
                    serial, generation, launch_epoch, 3, 1, 1,
                    2, destination_distance_um) < 0 ||
            set_start_waypoint(
                    &fixture->waypoints[0], start_index,
                    graph_base, 0, 0) < 0 ||
            set_sensor_waypoint(
                    &fixture->waypoints[1], sensor_index,
                    graph_base + 1, 1,
                    destination_distance_um / 2) < 0 ||
            set_destination_waypoint(
                    &fixture->waypoints[2],
                    destination_index, graph_base + 2, 2,
                    destination_distance_um) < 0) {
                return -1;
        }
        return 0;
}

static int make_sensor_fixture(
        projection_fixture *fixture, int train, int speed,
        int sensor_count, uint64_t serial,
        uint32_t generation, uint32_t launch_epoch) {
        int waypoint_count = sensor_count + 2;
        int destination_index = 7;
        int destination_offset = sensor_count + 1;
        int64_t destination_distance_um =
                (int64_t)(sensor_count + 2) * 10;
        if (sensor_count < 1 ||
            sensor_count >
                    TC2_TRACK_D_DIRECTED_SENSOR_COUNT ||
            initialize_header(
                    fixture, train, 0, destination_index, 0,
                    speed, serial, generation, launch_epoch,
                    waypoint_count, sensor_count,
                    sensor_count, destination_offset,
                    destination_distance_um) < 0 ||
            set_start_waypoint(
                    &fixture->waypoints[0], 0, 100, 0, 0) < 0) {
                return -1;
        }
        for (int sensor = 0; sensor < sensor_count; ++sensor) {
                if (set_sensor_waypoint(
                            &fixture->waypoints[sensor + 1],
                            sensor, sensor, sensor + 1,
                            (int64_t)(sensor + 1) * 10) < 0) {
                        return -1;
                }
        }
        if (set_destination_waypoint(
                    &fixture->waypoints[waypoint_count - 1],
                    destination_index, 101,
                    destination_offset,
                    destination_distance_um) < 0) {
                return -1;
        }
        return 0;
}

static int event_count(
        const event_log *log, int train,
        tc2_offline_event_type type) {
        int count = 0;
        if (!log) return 0;
        for (int index = 0; index < log->count; ++index) {
                if ((train < 0 ||
                     log->events[index].train == train) &&
                    log->events[index].type == type) {
                        ++count;
                }
        }
        return count;
}

static int full_matrix_test(void) {
        tc2_offline_runtime runtime;
        projection_fixture fixture;
        event_log log;
        int matrix_count = 0;

        for (int start = 0;
             start < TC2_TRACK_D_START_COUNT; ++start) {
                for (int destination = 0;
                     destination <
                             TC2_TRACK_D_DESTINATION_COUNT;
                     ++destination) {
                        int previous_velocity = 0;
                        for (int speed = 1; speed <= 120;
                             ++speed) {
                                tc2_offline_train_snapshot snapshot;
                                int side =
                                        (start + destination) %
                                        TC2_TRACK_DESTINATION_SIDE_COUNT;
                                uint64_t serial =
                                        (uint64_t)matrix_count + 1u;
                                memset(&log, 0, sizeof(log));
                                if (make_basic_fixture(
                                            &fixture, 14, start,
                                            destination, side,
                                            speed, serial, 1u,
                                            1u, 0, 2048) < 0 ||
                                    Tc2OfflineRuntimeInitialize(
                                            &runtime, 0u) !=
                                            TC2_OFFLINE_OK ||
                                    Tc2OfflineRuntimeAddTrip(
                                            &runtime,
                                            &fixture.header,
                                            fixture.waypoints,
                                            fixture.header
                                                    .waypoint_count) !=
                                            TC2_OFFLINE_OK ||
                                    Tc2OfflineRuntimeStartTrip(
                                            &runtime, 14) !=
                                            TC2_OFFLINE_OK ||
                                    Tc2OfflineRuntimeStep(
                                            &runtime,
                                            TC2_OFFLINE_RUNTIME_MAX_STEP_TICKS,
                                            capture_event,
                                            &log) !=
                                            TC2_OFFLINE_OK ||
                                    Tc2OfflineRuntimeGetTrainSnapshot(
                                            &runtime, 14,
                                            &snapshot) !=
                                            TC2_OFFLINE_OK) {
                                        return fail(
                                                "A-F/d1-d8/speed matrix execution failed");
                                }
                                if (snapshot.state !=
                                            TC2_OFFLINE_TRAIN_ARRIVED ||
                                    snapshot.start_index != start ||
                                    snapshot.destination_index !=
                                            destination ||
                                    snapshot.destination_side !=
                                            side ||
                                    snapshot.speed != speed ||
                                    snapshot.progress_um != 2048 ||
                                    snapshot.destination_distance_um !=
                                            2048 ||
                                    snapshot.previous_waypoint_index !=
                                            2 ||
                                    snapshot.next_waypoint_index !=
                                            TC2_OFFLINE_RUNTIME_INVALID_INDEX ||
                                    snapshot.motion_revision == 0 ||
                                    snapshot.velocity_um_per_tick <= 0 ||
                                    snapshot.velocity_um_per_tick <
                                            previous_velocity ||
                                    event_count(
                                            &log, 14,
                                            TC2_OFFLINE_EVENT_STARTED) !=
                                            1 ||
                                    event_count(
                                            &log, 14,
                                            TC2_OFFLINE_EVENT_PREDICTED_SENSOR_PULSE) !=
                                            1 ||
                                    event_count(
                                            &log, 14,
                                            TC2_OFFLINE_EVENT_ARRIVED) !=
                                            1 ||
                                    log.overflow ||
                                    log.invalid_provenance) {
                                        return fail(
                                                "matrix trip did not finish deterministically");
                                }
                                previous_velocity =
                                        snapshot.velocity_um_per_tick;
                                {
                                        int old_count = log.count;
                                        if (Tc2OfflineRuntimeStep(
                                                    &runtime,
                                                    TC2_OFFLINE_RUNTIME_MAX_STEP_TICKS,
                                                    capture_event,
                                                    &log) !=
                                                    TC2_OFFLINE_OK ||
                                            log.count != old_count) {
                                                return fail(
                                                        "arrived matrix trip emitted duplicate events");
                                        }
                                }
                                ++matrix_count;
                        }
                }
        }
        if (matrix_count != MATRIX_TRIP_COUNT) {
                return fail("matrix did not cover all 5760 trips");
        }
        return 0;
}

static int nonconflicting_cardinality_test(void) {
        static const int cardinalities[] = {1, 2, 3, 4, 6};

        for (size_t cardinality_index = 0;
             cardinality_index <
                     sizeof(cardinalities) /
                             sizeof(cardinalities[0]);
             ++cardinality_index) {
                int count = cardinalities[cardinality_index];
                tc2_offline_runtime runtime;
                tc2_offline_snapshot snapshot;
                projection_fixture
                        fixtures[TC2_OFFLINE_RUNTIME_MAX_TRAINS];
                tc2_ui_overlay overlay;
                overlay_event_log events;
                int used_colors[TC2_UI_COLOR_COUNT] = {0};

                memset(&events, 0, sizeof(events));
                events.fixtures = fixtures;
                events.fixture_count = count;
                events.overlay = &overlay;
                Tc2UiOverlayInit(
                        &overlay,
                        TC2_UI_SOURCE_OFFLINE_SIMULATION);
                if (Tc2OfflineRuntimeInitialize(&runtime, 0u) !=
                    TC2_OFFLINE_OK) {
                        return fail(
                                "cardinality runtime initialization failed");
                }

                for (int index = 0; index < count; ++index) {
                        int train = 30 + index;
                        if (make_basic_fixture(
                                    &fixtures[index], train,
                                    index % 6, index % 8,
                                    index % 2, 20 + index,
                                    100u + (uint64_t)index,
                                    1u, 1u, index, 2048) < 0 ||
                            Tc2OfflineRuntimeAddTrip(
                                    &runtime,
                                    &fixtures[index].header,
                                    fixtures[index].waypoints,
                                    fixtures[index].header
                                            .waypoint_count) !=
                                    TC2_OFFLINE_OK ||
                            Tc2UiOverlayUpsertAtStart(
                                    &overlay, train, index % 6,
                                    index % 8, 1u,
                                    TC2_UI_QUALITY_PREDICTED,
                                    0u) != 1) {
                                return fail(
                                        "could not add independent cardinality trip");
                        }
                        {
                                const tc2_ui_train_overlay *entry =
                                        Tc2UiOverlayFindTrain(
                                                &overlay, train);
                                if (!entry ||
                                    entry->color_slot < 0 ||
                                    entry->color_slot >=
                                            TC2_UI_COLOR_COUNT ||
                                    used_colors[entry->color_slot]++) {
                                        return fail(
                                                "multi-train colors are not stable and unique");
                                }
                        }
                }

                if (Tc2OfflineRuntimeStartAll(&runtime) !=
                            TC2_OFFLINE_OK ||
                    Tc2OfflineRuntimeStep(
                            &runtime,
                            TC2_OFFLINE_RUNTIME_MAX_STEP_TICKS,
                            capture_overlay_event,
                            &events) != TC2_OFFLINE_OK ||
                    Tc2OfflineRuntimeGetSnapshot(
                            &runtime, &snapshot) !=
                            TC2_OFFLINE_OK ||
                    snapshot.train_count != count ||
                    snapshot.arrived_count != count ||
                    snapshot.running_count != 0 ||
                    snapshot.waiting_count != 0 ||
                    events.overlay_error ||
                    events.log.overflow ||
                    events.log.invalid_provenance) {
                        return fail(
                                "1/2/3/4/6-train independent advance failed");
                }

                for (int index = 0; index < count; ++index) {
                        const tc2_ui_train_overlay *entry =
                                Tc2UiOverlayFindTrain(
                                        &overlay, 30 + index);
                        if (!entry ||
                            entry->position_kind !=
                                    TC2_UI_POSITION_DESTINATION ||
                            entry->position_quality !=
                                    TC2_UI_QUALITY_ESTIMATED ||
                            entry->position_revision == 0) {
                                return fail(
                                        "offline arrival did not update UI overlay");
                        }
                }
        }
        return 0;
}

static int directional_sensor_test(void) {
        tc2_offline_runtime runtime;
        projection_fixture fixture;
        tc2_ui_overlay overlay;
        overlay_event_log context;
        const tc2_offline_event *a11_event = 0;
        const tc2_offline_event *a12_event = 0;
        const tc2_ui_train_overlay *marker;
        int pulse_count[TC2_TRACK_D_DIRECTED_SENSOR_COUNT] =
                {0};
        int stable_color;

        memset(&context, 0, sizeof(context));
        if (make_sensor_fixture(
                    &fixture, 14, 120,
                    TC2_TRACK_D_DIRECTED_SENSOR_COUNT,
                    200u, 1u, 1u) < 0) {
                return fail(
                        "could not build eighty-sensor projection");
        }
        context.fixtures = &fixture;
        context.fixture_count = 1;
        context.overlay = &overlay;
        Tc2UiOverlayInit(
                &overlay, TC2_UI_SOURCE_OFFLINE_SIMULATION);
        if (Tc2UiOverlayUpsertAtStart(
                    &overlay, 14, 0, 7, 1u,
                    TC2_UI_QUALITY_PREDICTED, 0u) != 1) {
                return fail("could not create sensor UI marker");
        }
        marker = Tc2UiOverlayFindTrain(&overlay, 14);
        stable_color = marker ? marker->color_slot : -1;

        if (Tc2OfflineRuntimeInitialize(&runtime, 0u) !=
                    TC2_OFFLINE_OK ||
            Tc2OfflineRuntimeAddTrip(
                    &runtime, &fixture.header,
                    fixture.waypoints,
                    fixture.header.waypoint_count) !=
                    TC2_OFFLINE_OK ||
            Tc2OfflineRuntimeStartTrip(&runtime, 14) !=
                    TC2_OFFLINE_OK ||
            Tc2OfflineRuntimeStep(
                    &runtime, 100u,
                    capture_overlay_event, &context) !=
                    TC2_OFFLINE_OK) {
                return fail(
                        "eighty-sensor projection did not execute");
        }

        for (int index = 0;
             index < context.log.count; ++index) {
                const tc2_offline_event *event =
                        &context.log.events[index];
                if (event->type !=
                    TC2_OFFLINE_EVENT_PREDICTED_SENSOR_PULSE) {
                        continue;
                }
                if (event->sensor_index < 0 ||
                    event->sensor_index >=
                            TC2_TRACK_D_DIRECTED_SENSOR_COUNT) {
                        return fail(
                                "predicted pulse has invalid sensor");
                }
                ++pulse_count[event->sensor_index];
                if (event->sensor_index == 10) {
                        a11_event = event;
                } else if (event->sensor_index == 11) {
                        a12_event = event;
                }
        }
        for (int sensor = 0;
             sensor < TC2_TRACK_D_DIRECTED_SENSOR_COUNT;
             ++sensor) {
                const tc2_ui_sensor_overlay *flash =
                        Tc2UiOverlaySensor(&overlay, sensor);
                if (pulse_count[sensor] != 1 || !flash ||
                    !flash->active || flash->train != 14 ||
                    flash->evidence_source !=
                            TC2_UI_SOURCE_OFFLINE_SIMULATION ||
                    flash->evidence_quality !=
                            TC2_UI_QUALITY_PREDICTED ||
                    flash->sequence == 0) {
                        return fail(
                                "directional sensor did not pulse exactly once");
                }
        }
        if (!a11_event || !a12_event ||
            a11_event->ui_row != a12_event->ui_row ||
            a11_event->ui_column != a12_event->ui_column ||
            Tc2UiOverlaySensor(&overlay, 10)->column !=
                    Tc2UiOverlaySensor(&overlay, 11)->column) {
                return fail(
                        "A11/A12 identity or shared physical cell was lost");
        }
        marker = Tc2UiOverlayFindTrain(&overlay, 14);
        if (!marker ||
            marker->position_kind !=
                    TC2_UI_POSITION_DESTINATION ||
            marker->color_slot != stable_color ||
            marker->position_revision == 0 ||
            context.overlay_error ||
            context.log.invalid_provenance ||
            event_count(
                    &context.log, 14,
                    TC2_OFFLINE_EVENT_PREDICTED_SENSOR_PULSE) !=
                    TC2_TRACK_D_DIRECTED_SENSOR_COUNT ||
            event_count(
                    &context.log, 14,
                    TC2_OFFLINE_EVENT_ARRIVED) != 1) {
                return fail(
                        "sensor event/UI bridge lost final position");
        }
        {
                int old_count = context.log.count;
                if (Tc2OfflineRuntimeStep(
                            &runtime, 100u,
                            capture_overlay_event, &context) !=
                            TC2_OFFLINE_OK ||
                    context.log.count != old_count) {
                        return fail(
                                "same-tick step duplicated sensor pulses");
                }
        }
        return 0;
}

static int conflict_fairness_test(void) {
        tc2_offline_runtime runtime;
        projection_fixture
                fixtures[TC2_OFFLINE_RUNTIME_MAX_TRAINS];
        event_log log;
        uint32_t now = 0;
        int arrival_order[TC2_OFFLINE_RUNTIME_MAX_TRAINS];
        int arrived_total = 0;

        memset(&log, 0, sizeof(log));
        if (Tc2OfflineRuntimeInitialize(&runtime, now) !=
            TC2_OFFLINE_OK) {
                return fail("conflict runtime initialization failed");
        }
        for (int index = 0;
             index < TC2_OFFLINE_RUNTIME_MAX_TRAINS;
             ++index) {
                int train = 100 + index;
                if (make_basic_fixture(
                            &fixtures[index], train, 0, 0, 0,
                            1, 300u + (uint64_t)index,
                            1u, 1u, 0, 2048) < 0 ||
                    Tc2OfflineRuntimeAddTrip(
                            &runtime, &fixtures[index].header,
                            fixtures[index].waypoints, 3) !=
                            TC2_OFFLINE_OK) {
                        return fail(
                                "could not fill sixteen-train conflict queue");
                }
        }
        if (Tc2OfflineRuntimeStartAll(&runtime) !=
                    TC2_OFFLINE_OK ||
            Tc2OfflineRuntimeStep(
                    &runtime, 1u, capture_event, &log) !=
                    TC2_OFFLINE_OK) {
                return fail("conflict queue did not start");
        }
        {
                tc2_offline_snapshot snapshot;
                if (Tc2OfflineRuntimeGetSnapshot(
                            &runtime, &snapshot) !=
                            TC2_OFFLINE_OK ||
                    snapshot.train_count !=
                            TC2_OFFLINE_RUNTIME_MAX_TRAINS ||
                    snapshot.running_count != 1 ||
                    snapshot.waiting_count !=
                            TC2_OFFLINE_RUNTIME_MAX_TRAINS - 1 ||
                    snapshot.arrived_count != 0 ||
                    event_count(
                            &log,
                            TC2_OFFLINE_RUNTIME_INVALID_TRAIN,
                            TC2_OFFLINE_EVENT_CONFLICT_WAIT) <
                            TC2_OFFLINE_RUNTIME_MAX_TRAINS - 1) {
                        return fail(
                                "conflicting launch did not produce a single owner and wait queue");
                }
        }

        while (arrived_total <
               TC2_OFFLINE_RUNTIME_MAX_TRAINS) {
                tc2_offline_snapshot snapshot;
                int arrived_train =
                        TC2_OFFLINE_RUNTIME_INVALID_TRAIN;
                now += TC2_OFFLINE_RUNTIME_MAX_STEP_TICKS;
                if (Tc2OfflineRuntimeStep(
                            &runtime, now,
                            capture_event, &log) !=
                            TC2_OFFLINE_OK ||
                    Tc2OfflineRuntimeGetSnapshot(
                            &runtime, &snapshot) !=
                            TC2_OFFLINE_OK) {
                        return fail(
                                "conflict queue advance failed");
                }
                if (snapshot.running_count > 1 ||
                    snapshot.arrived_count != 1) {
                        return fail(
                                "overlapping conflict claims moved together");
                }
                for (int index = 0;
                     index < snapshot.train_count; ++index) {
                        if (snapshot.trains[index].state ==
                            TC2_OFFLINE_TRAIN_ARRIVED) {
                                arrived_train =
                                        snapshot.trains[index]
                                                .train;
                                break;
                        }
                }
                if (arrived_train != 100 + arrived_total) {
                        return fail(
                                "conflict queue was not deterministic FIFO");
                }
                arrival_order[arrived_total++] = arrived_train;
                if (Tc2OfflineRuntimeRemoveTrip(
                            &runtime, arrived_train) !=
                    TC2_OFFLINE_OK) {
                        return fail(
                                "arrived conflict owner did not release");
                }
        }
        for (int index = 0;
             index < TC2_OFFLINE_RUNTIME_MAX_TRAINS; ++index) {
                if (arrival_order[index] != 100 + index ||
                    event_count(
                            &log, 100 + index,
                            TC2_OFFLINE_EVENT_STARTED) != 1 ||
                    event_count(
                            &log, 100 + index,
                            TC2_OFFLINE_EVENT_ARRIVED) != 1 ||
                    (index > 0 &&
                     event_count(
                             &log, 100 + index,
                             TC2_OFFLINE_EVENT_CONFLICT_WAIT) < 1)) {
                        return fail(
                                "sixteen-train FIFO wait/recovery evidence is wrong");
                }
        }
        {
                tc2_offline_snapshot snapshot;
                if (Tc2OfflineRuntimeGetSnapshot(
                            &runtime, &snapshot) !=
                            TC2_OFFLINE_OK ||
                    snapshot.train_count != 0 ||
                    log.overflow || log.invalid_provenance) {
                        return fail(
                                "conflict queue did not drain safely");
                }
        }
        return 0;
}

static int half_range_epoch_tie_test(void) {
        tc2_offline_runtime runtime;
        projection_fixture lower_train;
        projection_fixture higher_train;
        tc2_offline_snapshot snapshot;

        if (Tc2OfflineRuntimeInitialize(&runtime, 0u) !=
                    TC2_OFFLINE_OK ||
            make_basic_fixture(
                    &higher_train, 200, 0, 0, 0, 1,
                    700u, 1u, 1u, 0, 2048) < 0 ||
            make_basic_fixture(
                    &lower_train, 100, 0, 0, 0, 1,
                    701u, 1u, 0x80000001u, 0, 2048) < 0 ||
            Tc2OfflineRuntimeAddTrip(
                    &runtime, &higher_train.header,
                    higher_train.waypoints, 3) !=
                    TC2_OFFLINE_OK ||
            Tc2OfflineRuntimeAddTrip(
                    &runtime, &lower_train.header,
                    lower_train.waypoints, 3) !=
                    TC2_OFFLINE_OK ||
            Tc2OfflineRuntimeStartAll(&runtime) !=
                    TC2_OFFLINE_OK ||
            Tc2OfflineRuntimeStep(
                    &runtime, 1u, 0, 0) !=
                    TC2_OFFLINE_OK ||
            Tc2OfflineRuntimeGetSnapshot(
                    &runtime, &snapshot) !=
                    TC2_OFFLINE_OK ||
            snapshot.running_count != 1 ||
            snapshot.waiting_count != 1) {
                return fail(
                        "half-range launch epoch arbitration failed");
        }
        for (int index = 0;
             index < snapshot.train_count; ++index) {
                if ((snapshot.trains[index].train == 100 &&
                     snapshot.trains[index].state !=
                             TC2_OFFLINE_TRAIN_RUNNING) ||
                    (snapshot.trains[index].train == 200 &&
                     snapshot.trains[index].state !=
                             TC2_OFFLINE_TRAIN_WAIT_CONFLICT)) {
                        return fail(
                                "half-range launch epoch lacked stable train tie break");
                }
        }
        return 0;
}

static int wrapping_tick_test(void) {
        tc2_offline_runtime runtime;
        projection_fixture fixture;
        tc2_offline_snapshot before;
        tc2_offline_snapshot after;
        event_log log;
        uint32_t initial = UINT32_MAX - 100u;
        uint32_t wrapped =
                initial +
                TC2_OFFLINE_RUNTIME_MAX_STEP_TICKS;

        memset(&log, 0, sizeof(log));
        if (make_basic_fixture(
                    &fixture, 14, 0, 6, 0, 120,
                    500u, 1u, 1u, 0, 2048) < 0 ||
            Tc2OfflineRuntimeInitialize(
                    &runtime, initial) != TC2_OFFLINE_OK ||
            Tc2OfflineRuntimeAddTrip(
                    &runtime, &fixture.header,
                    fixture.waypoints, 3) !=
                    TC2_OFFLINE_OK ||
            Tc2OfflineRuntimeStartTrip(
                    &runtime, 14) != TC2_OFFLINE_OK ||
            Tc2OfflineRuntimeStep(
                    &runtime, wrapped,
                    capture_event, &log) != TC2_OFFLINE_OK ||
            Tc2OfflineRuntimeGetSnapshot(
                    &runtime, &before) != TC2_OFFLINE_OK ||
            before.tick != wrapped ||
            before.arrived_count != 1 ||
            event_count(
                    &log, 14,
                    TC2_OFFLINE_EVENT_ARRIVED) != 1) {
                return fail("uint32 tick wrap did not advance");
        }
        if (Tc2OfflineRuntimeStep(
                    &runtime, wrapped - 1u,
                    capture_event, &log) == TC2_OFFLINE_OK ||
            Tc2OfflineRuntimeGetSnapshot(
                    &runtime, &after) != TC2_OFFLINE_OK ||
            memcmp(&before, &after, sizeof(before)) != 0) {
                return fail(
                        "ambiguous backwards step mutated runtime");
        }
        return 0;
}

static void make_page(
        const projection_fixture *fixture, int first,
        int count, tc2_dispatch_projection_page *page) {
        memset(page, 0, sizeof(*page));
        page->status = TC2_DISPATCH_PROJECTION_OK;
        page->train = fixture->header.train;
        page->publication_serial =
                fixture->header.publication_serial;
        page->plan_generation =
                fixture->header.plan_generation;
        page->launch_epoch = fixture->header.launch_epoch;
        page->first_waypoint = first;
        page->count = count;
        page->total_waypoints =
                fixture->header.waypoint_count;
        page->has_more =
                first + count <
                fixture->header.waypoint_count;
        for (int index = 0; index < count; ++index) {
                page->waypoints[index] =
                        fixture->waypoints[first + index];
        }
}

static int paged_fail_closed_test(void) {
        tc2_offline_runtime runtime;
        projection_fixture fixture;
        tc2_dispatch_projection_page first;
        tc2_dispatch_projection_page second;
        tc2_offline_train_snapshot snapshot;

        if (make_sensor_fixture(
                    &fixture, 14, 40, 10,
                    600u, 7u, 9u) < 0 ||
            Tc2OfflineRuntimeInitialize(
                    &runtime, 0u) != TC2_OFFLINE_OK ||
            Tc2OfflineRuntimeBeginTrip(
                    &runtime, &fixture.header) !=
                    TC2_OFFLINE_OK) {
                return fail("could not begin paged projection");
        }
        make_page(
                &fixture, 0,
                TC2_DISPATCH_PROJECTION_PAGE_CAPACITY,
                &first);
        make_page(
                &fixture,
                TC2_DISPATCH_PROJECTION_PAGE_CAPACITY,
                fixture.header.waypoint_count -
                        TC2_DISPATCH_PROJECTION_PAGE_CAPACITY,
                &second);
        if (Tc2OfflineRuntimeAppendProjectionPage(
                    &runtime, &first) != TC2_OFFLINE_OK ||
            Tc2OfflineRuntimeAppendProjectionPage(
                    &runtime, &second) != TC2_OFFLINE_OK ||
            Tc2OfflineRuntimeCommitTrip(
                    &runtime, 14) != TC2_OFFLINE_OK ||
            Tc2OfflineRuntimeGetTrainSnapshot(
                    &runtime, 14, &snapshot) !=
                    TC2_OFFLINE_OK ||
            snapshot.state != TC2_OFFLINE_TRAIN_READY ||
            snapshot.publication_serial !=
                    fixture.header.publication_serial ||
            snapshot.plan_generation !=
                    fixture.header.plan_generation ||
            snapshot.launch_epoch !=
                    fixture.header.launch_epoch) {
                return fail(
                        "valid paged publication did not commit atomically");
        }

        Tc2OfflineRuntimeReset(&runtime, 0u);
        if (Tc2OfflineRuntimeBeginTrip(
                    &runtime, &fixture.header) !=
                    TC2_OFFLINE_OK) {
                return fail("could not begin stale-token case");
        }
        ++first.publication_serial;
        if (Tc2OfflineRuntimeAppendProjectionPage(
                    &runtime, &first) != TC2_OFFLINE_STALE ||
            Tc2OfflineRuntimeGetTrainSnapshot(
                    &runtime, 14, &snapshot) !=
                    TC2_OFFLINE_OK ||
            snapshot.state != TC2_OFFLINE_TRAIN_FAILED ||
            snapshot.last_error != TC2_OFFLINE_STALE ||
            Tc2OfflineRuntimeCommitTrip(
                    &runtime, 14) == TC2_OFFLINE_OK) {
                return fail(
                        "stale page did not fail loading slot closed");
        }

        Tc2OfflineRuntimeReset(&runtime, 0u);
        make_page(
                &fixture, 0,
                TC2_DISPATCH_PROJECTION_PAGE_CAPACITY,
                &first);
        if (Tc2OfflineRuntimeBeginTrip(
                    &runtime, &fixture.header) !=
                    TC2_OFFLINE_OK ||
            Tc2OfflineRuntimeAppendProjectionPage(
                    &runtime, &first) != TC2_OFFLINE_OK ||
            Tc2OfflineRuntimeCommitTrip(
                    &runtime, 14) == TC2_OFFLINE_OK ||
            Tc2OfflineRuntimeGetTrainSnapshot(
                    &runtime, 14, &snapshot) !=
                    TC2_OFFLINE_OK ||
            snapshot.state != TC2_OFFLINE_TRAIN_FAILED ||
            snapshot.last_error == TC2_OFFLINE_OK) {
                return fail(
                        "missing page did not fail commit closed");
        }

        Tc2OfflineRuntimeReset(&runtime, 0u);
        if (Tc2OfflineRuntimeBeginTrip(
                    &runtime, &fixture.header) !=
                    TC2_OFFLINE_OK) {
                return fail("could not begin out-of-order case");
        }
        make_page(&fixture, 1, 1, &first);
        if (Tc2OfflineRuntimeAppendProjectionPage(
                    &runtime, &first) == TC2_OFFLINE_OK ||
            Tc2OfflineRuntimeGetTrainSnapshot(
                    &runtime, 14, &snapshot) !=
                    TC2_OFFLINE_OK ||
            snapshot.state != TC2_OFFLINE_TRAIN_FAILED) {
                return fail(
                        "out-of-order page did not fail closed");
        }

        Tc2OfflineRuntimeReset(&runtime, 0u);
        fixture.header.waypoint_count += 1;
        {
                tc2_offline_snapshot runtime_snapshot;

                if (Tc2OfflineRuntimeAddTrip(
                            &runtime, &fixture.header,
                            fixture.waypoints,
                            fixture.header.waypoint_count - 1) ==
                            TC2_OFFLINE_OK ||
                    Tc2OfflineRuntimeGetSnapshot(
                            &runtime,
                            &runtime_snapshot) !=
                            TC2_OFFLINE_OK ||
                    runtime_snapshot.train_count != 0) {
                        return fail(
                                "malformed full projection was accepted");
                }
        }
        return 0;
}

int main(void) {
        char layout_error[160];

        if (Tc2TrackDLayoutValidate(
                    layout_error, sizeof(layout_error)) < 0) {
                fprintf(stderr,
                        "tc2_offline_runtime_test: invalid layout: %s\n",
                        layout_error);
                return 1;
        }
        if (strcmp(
                    Tc2OfflineRuntimeStateName(
                            TC2_OFFLINE_TRAIN_WAIT_CONFLICT),
                    "WAIT_CONFLICT") != 0 ||
            strcmp(
                    Tc2OfflineRuntimeEventName(
                            TC2_OFFLINE_EVENT_PREDICTED_SENSOR_PULSE),
                    "PREDICTED_SENSOR_PULSE") != 0) {
                return fail("runtime audit labels are unstable");
        }
        if (full_matrix_test() ||
            nonconflicting_cardinality_test() ||
            directional_sensor_test() ||
            conflict_fairness_test() ||
            half_range_epoch_tie_test() ||
            wrapping_tick_test() ||
            paged_fail_closed_test()) {
                return 1;
        }

        printf(
                "tc2_offline_runtime_test: ok "
                "(%d A-F/d1-d8/speed cases; "
                "1/2/3/4/6/16 trains; 80 directional sensors; "
                "wrap/conflict/paging fail-closed)\n",
                MATRIX_TRIP_COUNT);
        return 0;
}
