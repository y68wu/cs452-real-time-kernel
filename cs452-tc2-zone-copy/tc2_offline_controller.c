#include "tc2_offline_controller.h"

#ifndef MODE_TC2

typedef int tc2_offline_controller_disabled_translation_unit;

#else

#include <stddef.h>
#include <stdint.h>

#include "tc2_track_d_layout.h"

_Static_assert(TC2_OFFLINE_CONTROLLER_MAX_TRAINS == TC2_UI_MAX_TRAINS,
               "offline controller registry must match overlay capacity");

static void zero_bytes(void *destination, size_t count) {
        unsigned char *bytes = (unsigned char *)destination;
        if (!bytes) return;
        for (size_t index = 0; index < count; ++index) {
                bytes[index] = 0;
        }
}

static uint32_t next_nonzero_u32(uint32_t *value) {
        if (!value) return 0;
        ++*value;
        if (*value == 0) ++*value;
        return *value;
}

static uint64_t next_nonzero_u64(uint64_t *value) {
        if (!value) return 0;
        ++*value;
        if (*value == 0) ++*value;
        return *value;
}

static int serial_is_newer(uint32_t candidate, uint32_t reference) {
        uint32_t distance;
        if (candidate == 0) return 0;
        if (reference == 0) return 1;
        distance = candidate - reference;
        return distance != 0 && distance < UINT32_C(0x80000000);
}

static tc2_offline_controller_train *find_train(
        tc2_offline_controller *controller, int train) {
        if (!controller || train < 1 || train > 255) return 0;
        for (int slot = 0;
             slot < TC2_OFFLINE_CONTROLLER_MAX_TRAINS; ++slot) {
                tc2_offline_controller_train *entry =
                        &controller->trains[slot];
                if (entry->active && entry->train == train) {
                        return entry;
                }
        }
        return 0;
}

static tc2_offline_controller_train *find_free_train(
        tc2_offline_controller *controller) {
        if (!controller) return 0;
        for (int slot = 0;
             slot < TC2_OFFLINE_CONTROLLER_MAX_TRAINS; ++slot) {
                if (!controller->trains[slot].active) {
                        return &controller->trains[slot];
                }
        }
        return 0;
}

static int controller_ready(
        const tc2_offline_controller *controller) {
        if (!controller || !controller->initialized) {
                return TC2_OFFLINE_CONTROLLER_NOT_INITIALIZED;
        }
        if (controller->failed_closed) {
                return TC2_OFFLINE_CONTROLLER_FAILED_CLOSED;
        }
        return TC2_OFFLINE_CONTROLLER_OK;
}

static int fail_closed(
        tc2_offline_controller *controller,
        int controller_error, int subsystem_error) {
        if (!controller) return controller_error;
        controller->failed_closed = 1;
        controller->last_error = controller_error;
        controller->last_subsystem_error = subsystem_error;
        return controller_error;
}

static int map_runtime_status(int status) {
        switch (status) {
        case TC2_OFFLINE_OK:
                return TC2_OFFLINE_CONTROLLER_OK;
        case TC2_OFFLINE_INVALID_ARGUMENT:
        case TC2_OFFLINE_ALIAS:
                return TC2_OFFLINE_CONTROLLER_INVALID_ARGUMENT;
        case TC2_OFFLINE_CAPACITY:
                return TC2_OFFLINE_CONTROLLER_CAPACITY;
        case TC2_OFFLINE_NOT_FOUND:
                return TC2_OFFLINE_CONTROLLER_NOT_FOUND;
        case TC2_OFFLINE_DUPLICATE_TRAIN:
        case TC2_OFFLINE_INVALID_STATE:
        case TC2_OFFLINE_STALE:
                return TC2_OFFLINE_CONTROLLER_INVALID_STATE;
        default:
                return TC2_OFFLINE_CONTROLLER_RUNTIME_ERROR;
        }
}

static int active_metadata_count(
        const tc2_offline_controller *controller) {
        int count = 0;
        if (!controller) return -1;
        for (int slot = 0;
             slot < TC2_OFFLINE_CONTROLLER_MAX_TRAINS; ++slot) {
                if (controller->trains[slot].active) ++count;
        }
        return count;
}

/*
 * The controller is the only publication owner.  Check all three retained
 * views before and after state-changing calls so a partial publication can
 * never be displayed as a valid predicted trip.
 */
static int validate_registry(tc2_offline_controller *controller) {
        tc2_offline_snapshot runtime_snapshot;
        int runtime_status;
        int metadata_count;
        int overlay_count;
        if (!controller || !controller->initialized) return -1;
        runtime_status = Tc2OfflineRuntimeGetSnapshot(
                &controller->runtime, &runtime_snapshot);
        if (runtime_status != TC2_OFFLINE_OK) {
                return fail_closed(
                        controller,
                        TC2_OFFLINE_CONTROLLER_RUNTIME_ERROR,
                        runtime_status);
        }
        metadata_count = active_metadata_count(controller);
        overlay_count = Tc2UiOverlayActiveTrainCount(
                &controller->overlay);
        if (metadata_count < 0 ||
            overlay_count != metadata_count ||
            runtime_snapshot.train_count != metadata_count) {
                return fail_closed(
                        controller,
                        TC2_OFFLINE_CONTROLLER_FAILED_CLOSED,
                        -1);
        }

        for (int slot = 0;
             slot < TC2_OFFLINE_CONTROLLER_MAX_TRAINS; ++slot) {
                tc2_offline_controller_train *metadata =
                        &controller->trains[slot];
                tc2_offline_train_snapshot train_snapshot;
                const tc2_ui_train_overlay *overlay_train;
                if (!metadata->active) continue;
                runtime_status = Tc2OfflineRuntimeGetTrainSnapshot(
                        &controller->runtime, metadata->train,
                        &train_snapshot);
                overlay_train = Tc2UiOverlayFindTrain(
                        &controller->overlay, metadata->train);
                if (runtime_status != TC2_OFFLINE_OK ||
                    !overlay_train ||
                    train_snapshot.train != metadata->train ||
                    train_snapshot.start_index !=
                            metadata->start_index ||
                    train_snapshot.speed != metadata->speed ||
                    train_snapshot.destination_index !=
                            metadata->destination_index ||
                    train_snapshot.plan_generation !=
                            metadata->plan_generation ||
                    train_snapshot.publication_serial !=
                            metadata->publication_serial ||
                    train_snapshot.launch_epoch !=
                            metadata->launch_epoch ||
                    overlay_train->start_index !=
                            metadata->start_index ||
                    overlay_train->destination_index !=
                            metadata->destination_index ||
                    overlay_train->plan_generation !=
                            metadata->plan_generation) {
                        return fail_closed(
                                controller,
                                TC2_OFFLINE_CONTROLLER_FAILED_CLOSED,
                                runtime_status);
                }
        }
        return TC2_OFFLINE_CONTROLLER_OK;
}

static uint32_t next_position_revision(
        tc2_offline_controller_train *train) {
        if (!train) return 0;
        return next_nonzero_u32(&train->position_revision);
}

static int clamp_cell(int value, int upper_exclusive) {
        if (value < 0) return 0;
        if (value >= upper_exclusive) return upper_exclusive - 1;
        return value;
}

static int milli_to_cell(int value) {
        if (value >= 0) return (value + 500) / 1000;
        return (value - 500) / 1000;
}

static int waypoint_has_runtime_cell(
        const tc2_dispatch_projection_waypoint *waypoint) {
        if (!waypoint || waypoint->ui_width == 0 ||
            waypoint->ui_row >= TC2_TRACK_D_LAYOUT_ROWS ||
            waypoint->ui_column >= TC2_TRACK_D_LAYOUT_MAP_COLUMNS) {
                return 0;
        }
        return (int)waypoint->ui_column +
                       (int)waypoint->ui_width <=
               TC2_TRACK_D_LAYOUT_MAP_COLUMNS;
}

static int interpolate_cell_component(
        int first, int second, int64_t numerator,
        int64_t denominator) {
        int64_t delta;
        int64_t scaled;
        if (denominator <= 0) return first;
        delta = (int64_t)second - (int64_t)first;
        scaled = delta * numerator;
        if (scaled >= 0) scaled += denominator / 2;
        else scaled -= denominator / 2;
        return first + (int)(scaled / denominator);
}

/*
 * The static layout stores turnout labels in the right-hand legend.  The
 * runtime and overlay intentionally accept map cells only.  Project just the
 * display coordinates of each turnout back between its adjacent route
 * landmarks; switch number, direction, graph node, distance, route offset,
 * reservation keys, and every other routing/collision field stay untouched.
 */
static int normalize_runtime_waypoint_cells(
        tc2_dispatch_projection_waypoint *waypoints,
        int waypoint_count) {
        int index = 0;
        if (!waypoints || waypoint_count < 2) return -1;
        while (index < waypoint_count) {
                int first_invalid;
                int next_valid;
                int previous_valid;
                int64_t first_distance;
                int64_t distance_span;
                if (waypoint_has_runtime_cell(&waypoints[index])) {
                        ++index;
                        continue;
                }
                first_invalid = index;
                while (index < waypoint_count &&
                       !waypoint_has_runtime_cell(
                               &waypoints[index])) {
                        if (waypoints[index].kind !=
                            TC2_ROUTE_WAYPOINT_TURNOUT) {
                                return -1;
                        }
                        ++index;
                }
                previous_valid = first_invalid - 1;
                next_valid = index;
                if (previous_valid < 0 ||
                    next_valid >= waypoint_count ||
                    !waypoint_has_runtime_cell(
                            &waypoints[previous_valid]) ||
                    !waypoint_has_runtime_cell(
                            &waypoints[next_valid])) {
                        return -1;
                }
                first_distance =
                        waypoints[previous_valid].distance_um;
                distance_span =
                        waypoints[next_valid].distance_um -
                        first_distance;
                for (int current = first_invalid;
                     current < next_valid; ++current) {
                        int64_t numerator =
                                waypoints[current].distance_um -
                                first_distance;
                        int row;
                        int column;
                        if (numerator < 0 || distance_span <= 0 ||
                            numerator > distance_span) {
                                return -1;
                        }
                        row = interpolate_cell_component(
                                waypoints[previous_valid].ui_row,
                                waypoints[next_valid].ui_row,
                                numerator, distance_span);
                        column = interpolate_cell_component(
                                waypoints[previous_valid].ui_column,
                                waypoints[next_valid].ui_column,
                                numerator, distance_span);
                        waypoints[current].ui_row =
                                (uint8_t)clamp_cell(
                                        row,
                                        TC2_TRACK_D_LAYOUT_ROWS);
                        waypoints[current].ui_column =
                                (uint8_t)clamp_cell(
                                        column,
                                        TC2_TRACK_D_LAYOUT_MAP_COLUMNS);
                        waypoints[current].ui_width = 1;
                }
        }
        return 0;
}

static void consume_runtime_event(
        const tc2_offline_event *event, void *context) {
        tc2_offline_controller *controller = context;
        tc2_offline_controller_train *metadata;
        uint32_t revision;
        int status = 0;
        if (!controller || controller->event_error) return;
        if (!event ||
            event->source !=
                    TC2_OFFLINE_EVENT_SOURCE_PREDICTED_OFFLINE ||
            event->is_live != 0 ||
            event->serial == 0) {
                controller->event_error = -1;
                return;
        }
        metadata = find_train(controller, event->train);
        if (!metadata ||
            !serial_is_newer(
                    event->serial, metadata->last_event_serial)) {
                controller->event_error = -2;
                return;
        }
        metadata->last_event_serial = event->serial;

        switch (event->type) {
        case TC2_OFFLINE_EVENT_STARTED:
        case TC2_OFFLINE_EVENT_CONFLICT_WAIT:
        case TC2_OFFLINE_EVENT_CONFLICT_RESUMED:
                return;
        case TC2_OFFLINE_EVENT_PREDICTED_SENSOR_PULSE:
                revision = next_position_revision(metadata);
                status = Tc2UiOverlayUpdateAtSensor(
                        &controller->overlay, metadata->train,
                        event->sensor_index,
                        metadata->plan_generation, revision,
                        event->serial,
                        TC2_UI_QUALITY_PREDICTED,
                        event->tick);
                break;
        case TC2_OFFLINE_EVENT_PREDICTED_TURNOUT:
        case TC2_OFFLINE_EVENT_PREDICTED_REVERSAL:
                revision = next_position_revision(metadata);
                status = Tc2UiOverlayUpdatePredictedCell(
                        &controller->overlay, metadata->train,
                        event->ui_row, event->ui_column,
                        metadata->plan_generation, revision,
                        event->tick);
                break;
        case TC2_OFFLINE_EVENT_ARRIVED:
                revision = next_position_revision(metadata);
                status = Tc2UiOverlayUpdateAtDestination(
                        &controller->overlay, metadata->train,
                        metadata->destination_index,
                        metadata->plan_generation, revision,
                        TC2_UI_QUALITY_ESTIMATED,
                        event->tick);
                break;
        case TC2_OFFLINE_EVENT_FAILED:
                controller->event_error = -3;
                return;
        default:
                controller->event_error = -4;
                return;
        }
        if (status != 1) {
                controller->event_error = status ? status : -5;
                return;
        }

        metadata->last_position_event_tick = event->tick;
        if (event->type == TC2_OFFLINE_EVENT_ARRIVED) {
                const tc2_track_d_destination_layout *destination =
                        Tc2TrackDDestinationLayout(
                                (size_t)metadata->
                                        destination_index);
                if (!destination) {
                        controller->event_error = -6;
                        return;
                }
                metadata->last_row = destination->row;
                metadata->last_column =
                        destination->column;
        } else {
                metadata->last_row = event->ui_row;
                metadata->last_column = event->ui_column;
        }
}

int Tc2OfflineControllerInitialize(
        tc2_offline_controller *controller, uint32_t initial_tick) {
        int status;
        if (!controller) {
                return TC2_OFFLINE_CONTROLLER_INVALID_ARGUMENT;
        }
        zero_bytes(controller, sizeof(*controller));
        status = Tc2OfflineRuntimeInitialize(
                &controller->runtime, initial_tick);
        if (status != TC2_OFFLINE_OK) {
                controller->last_error =
                        TC2_OFFLINE_CONTROLLER_RUNTIME_ERROR;
                controller->last_subsystem_error = status;
                return TC2_OFFLINE_CONTROLLER_RUNTIME_ERROR;
        }
        Tc2UiOverlayInit(
                &controller->overlay,
                TC2_UI_SOURCE_OFFLINE_SIMULATION);
        Tc2UiRendererInit(&controller->renderer);
        Tc2UiTxPumpInit(&controller->pump);
        Tc2OfflinePlannerInitialize(&controller->planner_scratch);
        controller->tick = initial_tick;
        controller->initialized = 1;
        return TC2_OFFLINE_CONTROLLER_OK;
}

void Tc2OfflineControllerReset(
        tc2_offline_controller *controller, uint32_t initial_tick) {
        if (!controller) return;
        (void)Tc2OfflineControllerInitialize(
                controller, initial_tick);
}

int Tc2OfflineControllerStageTrip(
        tc2_offline_controller *controller,
        track_node track[TRACK_MAX],
        int train, int start_index, int speed,
        int destination_index) {
        tc2_offline_controller_train *metadata;
        tc2_dispatch_projection_header *header;
        int status;
        int overlay_status;
        status = controller_ready(controller);
        if (status != TC2_OFFLINE_CONTROLLER_OK) return status;
        if (!track || train < 1 || train > 255 ||
            start_index < 0 ||
            start_index >= TC2_TRACK_D_START_COUNT ||
            speed < 1 || speed > 120 ||
            destination_index < 0 ||
            destination_index >=
                    TC2_TRACK_D_DESTINATION_COUNT) {
                return TC2_OFFLINE_CONTROLLER_INVALID_ARGUMENT;
        }
        if (find_train(controller, train)) {
                return TC2_OFFLINE_CONTROLLER_INVALID_STATE;
        }
        metadata = find_free_train(controller);
        if (!metadata) return TC2_OFFLINE_CONTROLLER_CAPACITY;
        if (validate_registry(controller) < 0) {
                return controller->last_error;
        }

        status = Tc2OfflinePlannerBuild(
                track, train, start_index, speed,
                destination_index,
                &controller->planner_scratch);
        if (status != TC2_OFFLINE_PLAN_OK ||
            Tc2OfflinePlannerValidate(
                    track, train, start_index, speed,
                    destination_index,
                    &controller->planner_scratch) !=
                            TC2_OFFLINE_PLAN_OK) {
                controller->last_subsystem_error = status;
                return TC2_OFFLINE_CONTROLLER_PLANNER_ERROR;
        }

        /*
         * The pure planner's hash proves plan bytes.  Replace only the three
         * publication ordering tokens after validation so identical trips
         * staged after removal are still strictly newer to the UI/runtime.
         */
        header = &controller->planner_scratch.projection_header;
        header->publication_serial = next_nonzero_u64(
                &controller->next_publication_serial);
        header->plan_generation = next_nonzero_u32(
                &controller->next_plan_generation);
        header->launch_epoch = next_nonzero_u32(
                &controller->next_launch_epoch);

        if (normalize_runtime_waypoint_cells(
                    controller->planner_scratch.waypoints,
                    header->waypoint_count) < 0) {
                controller->last_subsystem_error =
                        TC2_OFFLINE_MALFORMED_PROJECTION;
                return TC2_OFFLINE_CONTROLLER_PLANNER_ERROR;
        }

        status = Tc2OfflineRuntimeAddTrip(
                &controller->runtime, header,
                controller->planner_scratch.waypoints,
                header->waypoint_count);
        if (status != TC2_OFFLINE_OK) {
                controller->last_subsystem_error = status;
                return map_runtime_status(status);
        }
        overlay_status = Tc2UiOverlayUpsertAtStart(
                &controller->overlay, train, start_index,
                destination_index, header->plan_generation,
                TC2_UI_QUALITY_PREDICTED, controller->tick);
        if (overlay_status != 1) {
                (void)Tc2OfflineRuntimeRemoveTrip(
                        &controller->runtime, train);
                return fail_closed(
                        controller,
                        TC2_OFFLINE_CONTROLLER_OVERLAY_ERROR,
                        overlay_status);
        }
        if (Tc2UiOverlaySetTrainSpeed(
                    &controller->overlay, train, 0,
                    header->plan_generation,
                    controller->tick) < 0) {
                (void)Tc2OfflineRuntimeRemoveTrip(
                        &controller->runtime, train);
                return fail_closed(
                        controller,
                        TC2_OFFLINE_CONTROLLER_OVERLAY_ERROR,
                        -9);
        }

        zero_bytes(metadata, sizeof(*metadata));
        metadata->active = 1;
        metadata->train = train;
        metadata->start_index = start_index;
        metadata->speed = speed;
        metadata->destination_index = destination_index;
        metadata->plan_generation = header->plan_generation;
        metadata->publication_serial =
                header->publication_serial;
        metadata->launch_epoch = header->launch_epoch;
        {
                const tc2_track_d_start_layout *start =
                        Tc2TrackDStartLayout(
                                (size_t)start_index);
                metadata->last_row = start ? start->row : -1;
                metadata->last_column =
                        start ? start->column : -1;
        }
        status = validate_registry(controller);
        if (status != TC2_OFFLINE_CONTROLLER_OK) return status;
        return TC2_OFFLINE_CONTROLLER_OK;
}

static int sync_overlay_speeds_from_runtime(
        tc2_offline_controller *controller) {
        tc2_offline_snapshot snapshot;
        int status;
        if (!controller) return TC2_OFFLINE_CONTROLLER_INVALID_ARGUMENT;
        status = Tc2OfflineRuntimeGetSnapshot(
                &controller->runtime, &snapshot);
        if (status != TC2_OFFLINE_OK) {
                controller->last_subsystem_error = status;
                return TC2_OFFLINE_CONTROLLER_RUNTIME_ERROR;
        }
        for (int index = 0; index < snapshot.train_count; ++index) {
                const tc2_offline_train_snapshot *train =
                        &snapshot.trains[index];
                tc2_offline_controller_train *metadata =
                        find_train(controller, train->train);
                if (!metadata ||
                    Tc2UiOverlaySetTrainSpeed(
                            &controller->overlay,
                            metadata->train,
                            (train->state ==
                                     TC2_OFFLINE_TRAIN_RUNNING ||
                             (train->state ==
                                      TC2_OFFLINE_TRAIN_READY &&
                              train->start_requested)) ?
                                    metadata->speed : 0,
                            metadata->plan_generation,
                            controller->tick) < 0) {
                        return TC2_OFFLINE_CONTROLLER_OVERLAY_ERROR;
                }
        }
        return TC2_OFFLINE_CONTROLLER_OK;
}

int Tc2OfflineControllerStartTrip(
        tc2_offline_controller *controller, int train) {
        int status = controller_ready(controller);
        if (status != TC2_OFFLINE_CONTROLLER_OK) return status;
        if (validate_registry(controller) < 0) {
                return controller->last_error;
        }
        status = Tc2OfflineRuntimeStartTrip(
                &controller->runtime, train);
        if (status != TC2_OFFLINE_OK) {
                controller->last_subsystem_error = status;
                return map_runtime_status(status);
        }
        status = sync_overlay_speeds_from_runtime(controller);
        if (status != TC2_OFFLINE_CONTROLLER_OK) {
                return fail_closed(
                        controller, status,
                        controller->last_subsystem_error);
        }
        return validate_registry(controller);
}

int Tc2OfflineControllerStartAll(
        tc2_offline_controller *controller) {
        int status = controller_ready(controller);
        if (status != TC2_OFFLINE_CONTROLLER_OK) return status;
        if (validate_registry(controller) < 0) {
                return controller->last_error;
        }
        status = Tc2OfflineRuntimeStartAll(
                &controller->runtime);
        if (status != TC2_OFFLINE_OK) {
                controller->last_subsystem_error = status;
                return map_runtime_status(status);
        }
        status = sync_overlay_speeds_from_runtime(controller);
        if (status != TC2_OFFLINE_CONTROLLER_OK) {
                return fail_closed(
                        controller, status,
                        controller->last_subsystem_error);
        }
        return validate_registry(controller);
}

int Tc2OfflineControllerRemoveTrip(
        tc2_offline_controller *controller, int train) {
        tc2_offline_controller_train *metadata;
        int status = controller_ready(controller);
        if (status != TC2_OFFLINE_CONTROLLER_OK) return status;
        metadata = find_train(controller, train);
        if (!metadata) return TC2_OFFLINE_CONTROLLER_NOT_FOUND;
        if (validate_registry(controller) < 0) {
                return controller->last_error;
        }
        status = Tc2OfflineRuntimeRemoveTrip(
                &controller->runtime, train);
        if (status != TC2_OFFLINE_OK) {
                return fail_closed(
                        controller,
                        TC2_OFFLINE_CONTROLLER_RUNTIME_ERROR,
                        status);
        }
        status = Tc2UiOverlayRemoveTrain(
                &controller->overlay, train);
        if (status != 0) {
                return fail_closed(
                        controller,
                        TC2_OFFLINE_CONTROLLER_OVERLAY_ERROR,
                        status);
        }
        zero_bytes(metadata, sizeof(*metadata));
        return validate_registry(controller);
}

int Tc2OfflineControllerStep(
        tc2_offline_controller *controller, uint32_t now_tick) {
        tc2_offline_snapshot snapshot;
        int status = controller_ready(controller);
        if (status != TC2_OFFLINE_CONTROLLER_OK) return status;
        if (validate_registry(controller) < 0) {
                return controller->last_error;
        }
        controller->event_error = 0;
        status = Tc2OfflineRuntimeStep(
                &controller->runtime, now_tick,
                consume_runtime_event, controller);
        if (status != TC2_OFFLINE_OK) {
                controller->last_subsystem_error = status;
                return map_runtime_status(status);
        }
        controller->tick = now_tick;
        if (controller->event_error) {
                return fail_closed(
                        controller,
                        TC2_OFFLINE_CONTROLLER_OVERLAY_ERROR,
                        controller->event_error);
        }
        status = Tc2OfflineRuntimeGetSnapshot(
                &controller->runtime, &snapshot);
        if (status != TC2_OFFLINE_OK) {
                return fail_closed(
                        controller,
                        TC2_OFFLINE_CONTROLLER_RUNTIME_ERROR,
                        status);
        }

        for (int index = 0;
             index < snapshot.train_count; ++index) {
                const tc2_offline_train_snapshot *train =
                        &snapshot.trains[index];
                tc2_offline_controller_train *metadata =
                        find_train(controller, train->train);
                int row;
                int column;
                uint32_t revision;
                if (!metadata) {
                        return fail_closed(
                                controller,
                                TC2_OFFLINE_CONTROLLER_FAILED_CLOSED,
                                -7);
                }
                if (train->state == TC2_OFFLINE_TRAIN_FAILED) {
                        return fail_closed(
                                controller,
                                TC2_OFFLINE_CONTROLLER_RUNTIME_ERROR,
                                train->last_error);
                }
                if (Tc2UiOverlaySetTrainSpeed(
                            &controller->overlay,
                            metadata->train,
                            train->state ==
                                    TC2_OFFLINE_TRAIN_RUNNING ?
                                    metadata->speed : 0,
                            metadata->plan_generation,
                            now_tick) < 0) {
                        return fail_closed(
                                controller,
                                TC2_OFFLINE_CONTROLLER_OVERLAY_ERROR,
                                -10);
                }
                if (train->state != TC2_OFFLINE_TRAIN_RUNNING ||
                    metadata->last_position_event_tick == now_tick) {
                        continue;
                }
                row = clamp_cell(
                        milli_to_cell(train->ui_row_milli),
                        TC2_TRACK_D_LAYOUT_ROWS);
                column = clamp_cell(
                        milli_to_cell(train->ui_column_milli),
                        TC2_TRACK_D_LAYOUT_MAP_COLUMNS);
                if (row == metadata->last_row &&
                    column == metadata->last_column) {
                        continue;
                }
                revision = next_position_revision(metadata);
                status = Tc2UiOverlayUpdatePredictedCell(
                        &controller->overlay, metadata->train,
                        row, column, metadata->plan_generation,
                        revision, now_tick);
                if (status != 1) {
                        return fail_closed(
                                controller,
                                TC2_OFFLINE_CONTROLLER_OVERLAY_ERROR,
                                status);
                }
                metadata->last_position_event_tick = now_tick;
                metadata->last_row = row;
                metadata->last_column = column;
        }
        if (Tc2UiOverlayExpireSensorFlashes(
                    &controller->overlay, now_tick) < 0) {
                return fail_closed(
                        controller,
                        TC2_OFFLINE_CONTROLLER_OVERLAY_ERROR,
                        -8);
        }
        return validate_registry(controller);
}

int Tc2OfflineControllerRender(
        tc2_offline_controller *controller,
        const char *prompt_restore, size_t prompt_restore_length,
        tc2_offline_controller_render_result *result) {
        tc2_offline_controller_render_result local = {0};
        uint32_t generation;
        int status;
        int kind;
        if (!controller || !controller->initialized) {
                return TC2_OFFLINE_CONTROLLER_NOT_INITIALIZED;
        }
        if (!prompt_restore || prompt_restore_length == 0) {
                return TC2_OFFLINE_CONTROLLER_INVALID_ARGUMENT;
        }
        if (Tc2UiTxPumpNeedsFullRedraw(&controller->pump)) {
                Tc2UiRendererInvalidate(&controller->renderer);
        }
        generation = next_nonzero_u32(
                &controller->next_render_generation);
        status = Tc2UiTxPumpBeginFrame(
                &controller->pump, generation);
        if (status == TC2_UI_TX_BUSY) {
                return TC2_OFFLINE_CONTROLLER_BACKPRESSURE;
        }
        if (status != TC2_UI_TX_OK) {
                controller->last_subsystem_error = status;
                return TC2_OFFLINE_CONTROLLER_TX_ERROR;
        }
        (void)Tc2UiOverlayAdvanceRefresh(&controller->overlay);
        (void)Tc2UiOverlayExpireSensorFlashes(
                &controller->overlay, controller->tick);
        status = Tc2UiRendererRender(
                &controller->renderer, &controller->overlay,
                controller->tick, Tc2UiTxPumpStageWrite,
                &controller->pump, &local.renderer);
        if (status != 0) {
                Tc2UiTxPumpAbortFrame(&controller->pump);
                Tc2UiRendererInvalidate(&controller->renderer);
                controller->last_subsystem_error = status;
                return TC2_OFFLINE_CONTROLLER_RENDER_ERROR;
        }
        if (local.renderer.emitted_bytes == 0 &&
            Tc2UiTxPumpStageWrite(
                    &controller->pump, "\033[0m", 4) != 0) {
                Tc2UiTxPumpAbortFrame(&controller->pump);
                Tc2UiRendererInvalidate(&controller->renderer);
                return TC2_OFFLINE_CONTROLLER_TX_ERROR;
        }
        kind = local.renderer.full_redraw ?
                TC2_UI_TX_FRAME_FULL :
                TC2_UI_TX_FRAME_DELTA;
        status = Tc2UiTxPumpCommitFrame(
                &controller->pump,
                (tc2_ui_tx_frame_kind)kind,
                prompt_restore, prompt_restore_length);
        if (status == TC2_UI_TX_NEEDS_FULL_REDRAW) {
                Tc2UiRendererInvalidate(&controller->renderer);
                if (result) *result = local;
                return TC2_OFFLINE_CONTROLLER_NEEDS_FULL_REDRAW;
        }
        if (status != TC2_UI_TX_OK) {
                Tc2UiRendererInvalidate(&controller->renderer);
                controller->last_subsystem_error = status;
                return TC2_OFFLINE_CONTROLLER_TX_ERROR;
        }
        local.generation = generation;
        local.frame_kind = kind;
        local.queued = 1;
        if (result) *result = local;
        return TC2_OFFLINE_CONTROLLER_OK;
}

int Tc2OfflineControllerDrain(
        tc2_offline_controller *controller,
        tc2_ui_tx_write_fn write, void *write_context,
        unsigned int max_chunks,
        tc2_ui_tx_drain_result *result) {
        int status;
        if (!controller || !controller->initialized) {
                return TC2_OFFLINE_CONTROLLER_NOT_INITIALIZED;
        }
        if (!write || max_chunks == 0) {
                return TC2_OFFLINE_CONTROLLER_INVALID_ARGUMENT;
        }
        status = Tc2UiTxPumpDrain(
                &controller->pump, write, write_context,
                max_chunks, result);
        if (status == TC2_UI_TX_IDLE) {
                return TC2_OFFLINE_CONTROLLER_NO_CHANGE;
        }
        if (status == TC2_UI_TX_BACKPRESSURE) {
                return TC2_OFFLINE_CONTROLLER_BACKPRESSURE;
        }
        if (status == TC2_UI_TX_NEEDS_FULL_REDRAW) {
                Tc2UiRendererInvalidate(&controller->renderer);
                return TC2_OFFLINE_CONTROLLER_NEEDS_FULL_REDRAW;
        }
        if (status == TC2_UI_TX_OUTPUT_FAILURE) {
                Tc2UiRendererInvalidate(&controller->renderer);
                controller->last_subsystem_error = status;
                return TC2_OFFLINE_CONTROLLER_TX_ERROR;
        }
        if (status != TC2_UI_TX_OK) {
                controller->last_subsystem_error = status;
                return TC2_OFFLINE_CONTROLLER_TX_ERROR;
        }
        return TC2_OFFLINE_CONTROLLER_OK;
}

int Tc2OfflineControllerGetTrainSnapshot(
        const tc2_offline_controller *controller, int train,
        tc2_offline_train_snapshot *snapshot) {
        if (!controller || !controller->initialized) {
                return TC2_OFFLINE_CONTROLLER_NOT_INITIALIZED;
        }
        if (!snapshot) {
                return TC2_OFFLINE_CONTROLLER_INVALID_ARGUMENT;
        }
        return map_runtime_status(
                Tc2OfflineRuntimeGetTrainSnapshot(
                        &controller->runtime, train, snapshot));
}

int Tc2OfflineControllerGetSnapshot(
        const tc2_offline_controller *controller,
        tc2_offline_controller_snapshot *snapshot) {
        int status;
        if (!controller || !controller->initialized) {
                return TC2_OFFLINE_CONTROLLER_NOT_INITIALIZED;
        }
        if (!snapshot) {
                return TC2_OFFLINE_CONTROLLER_INVALID_ARGUMENT;
        }
        zero_bytes(snapshot, sizeof(*snapshot));
        status = Tc2OfflineRuntimeGetSnapshot(
                &controller->runtime, &snapshot->runtime);
        if (status != TC2_OFFLINE_OK) {
                return map_runtime_status(status);
        }
        snapshot->initialized = controller->initialized;
        snapshot->failed_closed = controller->failed_closed;
        snapshot->last_error = controller->last_error;
        snapshot->last_subsystem_error =
                controller->last_subsystem_error;
        snapshot->predicted_only = 1;
        snapshot->is_live = 0;
        snapshot->source =
                TC2_UI_SOURCE_OFFLINE_SIMULATION;
        snapshot->tick = controller->tick;
        snapshot->overlay_train_count =
                Tc2UiOverlayActiveTrainCount(
                        &controller->overlay);
        snapshot->output_pending =
                Tc2UiTxPumpHasWork(&controller->pump);
        snapshot->full_redraw_required =
                Tc2UiTxPumpNeedsFullRedraw(
                        &controller->pump);
        snapshot->output_failed =
                Tc2UiTxPumpOutputFailed(&controller->pump);
        snapshot->last_accepted_generation =
                Tc2UiTxPumpLastAcceptedGeneration(
                        &controller->pump);
        snapshot->last_completed_generation =
                Tc2UiTxPumpLastCompletedGeneration(
                        &controller->pump);
        return TC2_OFFLINE_CONTROLLER_OK;
}

const tc2_ui_overlay *Tc2OfflineControllerOverlay(
        const tc2_offline_controller *controller) {
        if (!controller || !controller->initialized) return 0;
        return &controller->overlay;
}

int Tc2OfflineControllerHasPendingOutput(
        const tc2_offline_controller *controller) {
        if (!controller || !controller->initialized) return 0;
        return Tc2UiTxPumpHasWork(&controller->pump);
}

int Tc2OfflineControllerNeedsFullRedraw(
        const tc2_offline_controller *controller) {
        if (!controller || !controller->initialized) return 0;
        return Tc2UiTxPumpNeedsFullRedraw(&controller->pump);
}

const char *Tc2OfflineControllerEvidenceLabel(void) {
        return TC2_OFFLINE_CONTROLLER_EVIDENCE_LABEL;
}

#endif
