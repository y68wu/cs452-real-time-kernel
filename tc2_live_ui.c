#include "tc2_live_ui.h"
#include "tc2_ui_path_geometry.h"

#ifndef MODE_TC2

typedef int tc2_live_ui_disabled_translation_unit;

#else

#define TC2_LIVE_UI_GENERATION_HANDOFF_TICKS 50U
#define TC2_LIVE_UI_JOB_STATE_GENERATION_HANDOFF (-1)

static void zero_bytes(void *value, size_t size) {
        volatile unsigned char *bytes = value;
        while (size > 0) {
                bytes[--size] = 0;
        }
}

static void copy_bytes(
        void *destination, const void *source, size_t size) {
        volatile unsigned char *to = destination;
        const volatile unsigned char *from = source;
        for (size_t index = 0; index < size; ++index) {
                to[index] = from[index];
        }
}

static uint32_t next_nonzero(uint32_t *value) {
        ++*value;
        if (*value == 0) ++*value;
        return *value;
}

static int sequence_after(uint32_t candidate, uint32_t reference) {
        if (candidate == 0 || candidate == reference) return 0;
        if (reference == 0) return 1;
        return candidate - reference < (1u << 31);
}

static int retained_current_cell_maps_to_initial_route(
        const tc2_dispatch_projection_waypoint *waypoints,
        int waypoint_count, int retained_row, int retained_column,
        int64_t *mapped_distance_um);

static int job_state_supports_physical_prediction(int state) {
        return state == TC2_JOB_PREPARING ||
                state == TC2_JOB_READY ||
                state == TC2_JOB_LAUNCHING ||
                state == TC2_JOB_RUNNING ||
                state == TC2_JOB_BRAKING ||
                state == TC2_JOB_REVERSING ||
                state == TC2_JOB_TRAFFIC_HOLD;
}

static int job_state_reports_commanded_speed(int state) {
        return state == TC2_JOB_PREPARING ||
                state == TC2_JOB_READY ||
                state == TC2_JOB_LAUNCHING ||
                state == TC2_JOB_RUNNING ||
                state == TC2_JOB_REVERSING;
}

static int job_visible_speed(
        const tc2_dispatch_job_snapshot *job,
        const tc2_live_ui_train *train) {
        if (!job || !train ||
            !job_state_reports_commanded_speed(job->state)) {
                return 0;
        }
        if (job->command_speed > 0) return job->command_speed;
        if (job->state == TC2_JOB_LAUNCHING ||
            job->state == TC2_JOB_RUNNING ||
            job->state == TC2_JOB_REVERSING) {
                return train->speed;
        }
        return 0;
}

static int job_has_physical_motion(
        const tc2_dispatch_job_snapshot *job,
        const tc2_live_ui_train *train) {
        if (!job || !train ||
            !job_state_supports_physical_prediction(job->state) ||
            job->prediction_velocity_um_per_tick <= 0) {
                return 0;
        }
        if (job->state == TC2_JOB_BRAKING ||
            job->state == TC2_JOB_TRAFFIC_HOLD) {
                return 1;
        }
        return train->current_speed > 0;
}

static tc2_live_ui_train *find_train(
        tc2_live_ui *ui, int train) {
        if (!ui) return 0;
        for (int slot = 0; slot < TC2_UI_MAX_TRAINS; ++slot) {
                if (ui->trains[slot].active &&
                    ui->trains[slot].train == train) {
                        return &ui->trains[slot];
                }
        }
        return 0;
}

static tc2_live_ui_train *allocate_train(tc2_live_ui *ui) {
        for (int slot = 0; slot < TC2_UI_MAX_TRAINS; ++slot) {
                if (!ui->trains[slot].active) return &ui->trains[slot];
        }
        return 0;
}

static const tc2_dispatch_job_snapshot *find_job(
        const tc2_dispatch_snapshot *dispatch, int train,
        uint32_t generation) {
        if (!dispatch) return 0;
        for (int slot = 0; slot < TC2_DISPATCH_MAX_JOBS; ++slot) {
                const tc2_dispatch_job_snapshot *job =
                        &dispatch->jobs[slot];
                if (job->state != TC2_JOB_EMPTY &&
                    job->train == train &&
                    job->plan_generation == generation) {
                        return job;
                }
        }
        return 0;
}

static const tc2_dispatch_job_snapshot *find_job_for_train(
        const tc2_dispatch_snapshot *dispatch, int train) {
        const tc2_dispatch_job_snapshot *selected = 0;
        if (!dispatch) return 0;
        for (int slot = 0; slot < TC2_DISPATCH_MAX_JOBS; ++slot) {
                const tc2_dispatch_job_snapshot *job =
                        &dispatch->jobs[slot];
                if (job->state == TC2_JOB_EMPTY ||
                    job->train != train ||
                    job->plan_generation == 0) {
                        continue;
                }
                if (!selected ||
                    sequence_after(
                            job->plan_generation,
                            selected->plan_generation)) {
                        selected = job;
                }
        }
        return selected;
}

static const tc2_track_d_switch_layout *switch_layout_for_number(
        int switch_number) {
        for (size_t index = 0;
             index < TC2_TRACK_D_SWITCH_COUNT; ++index) {
                const tc2_track_d_switch_layout *layout =
                        Tc2TrackDSwitchLayout(index);
                if (layout && layout->number == switch_number) {
                        return layout;
                }
        }
        return 0;
}

static int projection_payload_valid(
        const tc2_live_ui *ui,
        const tc2_dispatch_projection_header *header,
        const tc2_dispatch_projection_waypoint *waypoints,
        int waypoint_count) {
        int64_t prior_distance = -1;
        if (!ui || !header || !waypoints || waypoint_count < 1 ||
            waypoint_count != header->waypoint_count) {
                return 0;
        }
        if (waypoints[0].kind != TC2_ROUTE_WAYPOINT_START ||
            waypoints[0].distance_um != 0) {
                return 0;
        }
        if (waypoint_count == 1 &&
            header->start_index != TC2_DISPATCH_START_CURRENT) {
                return 0;
        }
        if (waypoint_count > 1 &&
            (waypoints[waypoint_count - 1].kind !=
                     TC2_ROUTE_WAYPOINT_DESTINATION ||
             waypoints[waypoint_count - 1].destination_index !=
                     header->destination_index)) {
                return 0;
        }

        for (int index = 0; index < waypoint_count; ++index) {
                const tc2_dispatch_projection_waypoint *waypoint =
                        &waypoints[index];
                if (waypoint->kind < TC2_ROUTE_WAYPOINT_START ||
                    waypoint->kind >
                            TC2_ROUTE_WAYPOINT_DESTINATION ||
                    waypoint->distance_um < 0 ||
                    waypoint->distance_um < prior_distance ||
                    waypoint->ui_width == 0) {
                        return 0;
                }
                prior_distance = waypoint->distance_um;
                switch (waypoint->kind) {
                case TC2_ROUTE_WAYPOINT_START:
                        if (index != 0) return 0;
                        if (header->start_index ==
                                    TC2_DISPATCH_START_CURRENT) {
                                const tc2_ui_train_overlay *shown =
                                        Tc2UiOverlayFindTrain(
                                                &ui->overlay,
                                                header->train);
                                if ((!shown ||
                                     !Tc2TrackDLayoutCellIsOccupied(
                                             shown->row,
                                             shown->column)) &&
                                    !Tc2TrackDLayoutCellIsOccupied(
                                            waypoint->ui_row,
                                            waypoint->ui_column)) {
                                        return 0;
                                }
                        } else {
                                const tc2_track_d_start_layout *start =
                                        Tc2TrackDStartLayout(
                                                (size_t)
                                                header->start_index);
                                if (!start ||
                                    waypoint->ui_row != start->row ||
                                    waypoint->ui_column !=
                                            start->column) {
                                        return 0;
                                }
                        }
                        break;
                case TC2_ROUTE_WAYPOINT_SENSOR:
                case TC2_ROUTE_WAYPOINT_REVERSAL: {
                        tc2_track_d_directed_sensor_cell cell;
                        if (Tc2TrackDDirectedSensorCell(
                                    waypoint->sensor_index,
                                    &cell) < 0 ||
                            waypoint->ui_row != cell.row ||
                            waypoint->ui_column != cell.column ||
                            waypoint->ui_width != cell.width) {
                                return 0;
                        }
                        break;
                }
                case TC2_ROUTE_WAYPOINT_TURNOUT: {
                        const tc2_track_d_switch_layout *layout =
                                switch_layout_for_number(
                                        waypoint->switch_number);
                        if (!layout ||
                            (waypoint->turnout_direction !=
                                     DIR_STRAIGHT &&
                             waypoint->turnout_direction !=
                                     DIR_CURVED) ||
                            waypoint->ui_row != layout->row ||
                            waypoint->ui_column != layout->column) {
                                return 0;
                        }
                        break;
                }
                case TC2_ROUTE_WAYPOINT_DESTINATION: {
                        const tc2_track_d_destination_layout *destination =
                                Tc2TrackDDestinationLayout(
                                        (size_t)
                                        header->destination_index);
                        if (index != waypoint_count - 1 ||
                            waypoint->destination_index !=
                                    header->destination_index ||
                            !destination ||
                            waypoint->ui_row != destination->row ||
                            waypoint->ui_column !=
                                    destination->column) {
                                return 0;
                        }
                        break;
                }
                default:
                        return 0;
                }
        }
        return 1;
}

static int waypoint_for_sensor(
        const tc2_live_ui_train *train, int sensor,
        int64_t minimum_distance_um) {
        int selected = -1;
        for (int index = 0; index < train->waypoint_count; ++index) {
                const tc2_dispatch_projection_waypoint *waypoint =
                        &train->waypoints[index];
                if (waypoint->sensor_index != sensor ||
                    waypoint->distance_um < minimum_distance_um) {
                        continue;
                }
                if (selected < 0 ||
                    waypoint->distance_um <
                            train->waypoints[selected].distance_um) {
                        selected = index;
                }
        }
        return selected;
}

/*
 * A physical detector is represented by two directed sensor nodes.  Prefer
 * the exact projected half, but accept its paired half when live attribution
 * reports the other direction.
 */
static int waypoint_for_physical_sensor(
        const tc2_live_ui_train *train, int sensor,
        int64_t minimum_distance_um) {
        int exact = waypoint_for_sensor(
                train, sensor, minimum_distance_um);
        if (sensor < 0 || sensor >= TRAIN_SENSOR_COUNT) {
                return exact;
        }
        int paired = waypoint_for_sensor(
                train, sensor ^ 1, minimum_distance_um);
        if (exact < 0) return paired;
        if (paired < 0) return exact;
        return train->waypoints[paired].distance_um <
                            train->waypoints[exact].distance_um ?
                paired : exact;
}

static int64_t next_unconfirmed_sensor_distance_um(
        const tc2_live_ui_train *train) {
        int64_t selected = -1;
        if (!train) return -1;
        for (int index = 0; index < train->waypoint_count; ++index) {
                const tc2_dispatch_projection_waypoint *waypoint =
                        &train->waypoints[index];
                if ((waypoint->kind != TC2_ROUTE_WAYPOINT_SENSOR &&
                     waypoint->kind != TC2_ROUTE_WAYPOINT_REVERSAL) ||
                    waypoint->sensor_index < 0 ||
                    waypoint->distance_um <=
                            train->confirmed_sensor_distance_um) {
                        continue;
                }
                if (selected < 0 ||
                    waypoint->distance_um < selected) {
                        selected = waypoint->distance_um;
                }
        }
        return selected;
}

void Tc2LiveUiInitialize(tc2_live_ui *ui, uint32_t initial_tick) {
        if (!ui) return;
        zero_bytes(ui, sizeof(*ui));
        ui->initialized = 1;
        ui->tick = initial_tick;
        Tc2UiOverlayInit(&ui->overlay, TC2_UI_SOURCE_LIVE_CAN);
        Tc2UiRendererInit(&ui->renderer);
        Tc2UiTxPumpInit(&ui->pump);
}

int Tc2LiveUiHasPlan(
        const tc2_live_ui *ui, int train, uint32_t plan_generation) {
        if (!ui || !ui->initialized || plan_generation == 0) return 0;
        for (int slot = 0; slot < TC2_UI_MAX_TRAINS; ++slot) {
                if (ui->trains[slot].active &&
                    ui->trains[slot].train == train &&
                    ui->trains[slot].plan_generation == plan_generation &&
                    !ui->trains[slot].localization_placeholder) {
                        return 1;
                }
        }
        return 0;
}

int Tc2LiveUiAcceptProjection(
        tc2_live_ui *ui,
        const tc2_dispatch_projection_header *header,
        const tc2_dispatch_projection_waypoint *waypoints,
        int waypoint_count, uint32_t now_tick) {
        if (!ui || !ui->initialized || !header || !waypoints ||
            !header->valid || header->train < 1 || header->train > 255 ||
            header->plan_generation == 0 ||
            (header->start_index !=
                         TC2_DISPATCH_START_CURRENT &&
             (header->start_index < 0 ||
              header->start_index >=
                      TC2_TRACK_D_START_COUNT)) ||
            header->destination_index < 0 ||
            header->destination_index >= TC2_TRACK_D_DESTINATION_COUNT ||
            waypoint_count < 1 ||
            waypoint_count > TC2_LIVE_UI_MAX_WAYPOINTS ||
            waypoint_count != header->waypoint_count) {
                return -1;
        }
        tc2_live_ui_train *train = find_train(ui, header->train);
        if (train &&
            !sequence_after(
                    header->plan_generation,
                    train->plan_generation) &&
            header->plan_generation != train->plan_generation) {
                return 0;
        }
        int same_generation_upgrade =
                train &&
                train->plan_generation ==
                        header->plan_generation &&
                train->localization_placeholder &&
                header->start_index ==
                        TC2_DISPATCH_START_CURRENT &&
                waypoint_count > 1;
        if (train &&
            train->plan_generation == header->plan_generation &&
            !same_generation_upgrade) {
                return 0;
        }
        if (!projection_payload_valid(
                    ui, header, waypoints, waypoint_count)) {
                return -1;
        }
        int initial_row = waypoints[0].ui_row;
        int initial_column = waypoints[0].ui_column;
        if (header->start_index ==
                    TC2_DISPATCH_START_CURRENT) {
                const tc2_ui_train_overlay *shown =
                        Tc2UiOverlayFindTrain(
                                &ui->overlay, header->train);
                if (shown &&
                    Tc2TrackDLayoutCellIsOccupied(
                            shown->row, shown->column)) {
                        initial_row = shown->row;
                        initial_column = shown->column;
                }
        }
        int newly_allocated = 0;
        if (!train) {
                train = allocate_train(ui);
                newly_allocated = 1;
        }
        if (!train) return -1;
        int overlay_status = 1;
        if (!same_generation_upgrade) {
                overlay_status =
                        header->start_index ==
                                TC2_DISPATCH_START_CURRENT ?
                        Tc2UiOverlayUpsertAtPredictedCell(
                                &ui->overlay, header->train,
                                initial_row, initial_column,
                                header->destination_index,
                                header->plan_generation,
                                TC2_UI_QUALITY_PREDICTED, now_tick) :
                        Tc2UiOverlayUpsertAtStart(
                                &ui->overlay, header->train,
                                header->start_index,
                                header->destination_index,
                                header->plan_generation,
                                TC2_UI_QUALITY_PREDICTED, now_tick);
        }
        if (overlay_status < 0) {
                if (newly_allocated) {
                        zero_bytes(train, sizeof(*train));
                }
                return -1;
        }
        if (overlay_status == 0) {
                if (newly_allocated) {
                        zero_bytes(train, sizeof(*train));
                }
                return 0;
        }
        uint32_t retained_sensor_sequence =
                !newly_allocated ?
                train->last_sensor_sequence : 0;
        uint32_t retained_position_revision =
                !newly_allocated ?
                train->position_revision :
                header->plan_generation;
        if (!same_generation_upgrade &&
            !sequence_after(retained_position_revision,
                            header->plan_generation)) {
                retained_position_revision =
                        header->plan_generation;
        }
        int64_t retained_confirmed_sensor_distance_um =
                same_generation_upgrade ?
                train->confirmed_sensor_distance_um : 0;
        int retained_last_job_state =
                !newly_allocated ?
                train->last_job_state : TC2_JOB_EMPTY;
        zero_bytes(train, sizeof(*train));
        train->active = 1;
        train->train = header->train;
        train->start_index = header->start_index;
        train->destination_index = header->destination_index;
        train->speed = header->speed;
        train->current_speed = 0;
        train->last_job_state = retained_last_job_state;
        train->plan_generation = header->plan_generation;
        train->launch_epoch = header->launch_epoch;
        train->position_revision =
                retained_position_revision;
        train->last_sensor_sequence =
                retained_sensor_sequence;
        train->confirmed_sensor_distance_um =
                retained_confirmed_sensor_distance_um;
        train->anchor_tick = now_tick;
        train->localization_placeholder =
                header->start_index ==
                        TC2_DISPATCH_START_CURRENT &&
                waypoint_count == 1;
        train->waypoint_count = waypoint_count;
        for (int index = 0; index < waypoint_count; ++index) {
                copy_bytes(
                        &train->waypoints[index], &waypoints[index],
                        sizeof(train->waypoints[index]));
        }
        if (header->start_index ==
                    TC2_DISPATCH_START_CURRENT) {
                int projection_start_row =
                        train->waypoints[0].ui_row;
                int projection_start_column =
                        train->waypoints[0].ui_column;
                int retained_cell_is_aligned =
                        initial_row == projection_start_row &&
                        initial_column == projection_start_column;
                int64_t retained_route_distance_um = 0;
                if (!retained_cell_is_aligned &&
                    waypoint_count > 1) {
                        retained_cell_is_aligned =
                                retained_current_cell_maps_to_initial_route(
                                        waypoints, waypoint_count,
                                        initial_row, initial_column,
                                        &retained_route_distance_um);
                }
                if (waypoint_count == 1) {
                        /*
                         * A localization-only CURRENT record has no route
                         * geometry yet.  Keep its sole visual cell at the
                         * last physical marker until the exact projection
                         * arrives; exact routes below remain immutable.
                         */
                        train->waypoints[0].ui_row =
                                (uint8_t)initial_row;
                        train->waypoints[0].ui_column =
                                (uint8_t)initial_column;
                        train->waypoints[0].ui_width = 1;
                } else if (retained_cell_is_aligned) {
                        train->anchor_distance_um =
                                retained_route_distance_um;
                        train->displayed_distance_um =
                                retained_route_distance_um;
                        train->confirmed_sensor_distance_um =
                                retained_route_distance_um;
                }
                train->awaiting_current_sensor =
                        waypoint_count > 1 &&
                        (initial_row != projection_start_row ||
                         initial_column !=
                                 projection_start_column) &&
                        !retained_cell_is_aligned;
        }
        if (Tc2UiOverlaySetTrainSpeed(
                    &ui->overlay, train->train, 0,
                    train->plan_generation, now_tick) < 0) {
                return -1;
        }
        return 0;
}

static int apply_new_sensor_events(
        tc2_live_ui *ui, tc2_live_ui_train *train,
        const train_sensor_snapshot_t *sensors, uint32_t now_tick) {
        int changed = 0;
        for (;;) {
                int selected_sensor = -1;
                uint32_t selected_sequence = 0;
                for (int sensor = 0; sensor < TRAIN_SENSOR_COUNT; ++sensor) {
                        uint32_t sequence =
                                sensors->last_attributed_sequence_by_sensor[
                                        sensor];
                        if (sensors->last_attributed_train_by_sensor[sensor] !=
                                    train->train ||
                            sensors->last_attributed_generation_by_sensor[
                                    sensor] != train->plan_generation ||
                            !sequence_after(
                                    sequence,
                                    train->last_sensor_sequence)) {
                                continue;
                        }
                        if (selected_sensor < 0 ||
                            sequence_after(selected_sequence, sequence)) {
                                selected_sensor = sensor;
                                selected_sequence = sequence;
                        }
                }
                if (selected_sensor < 0) break;
                int64_t strict_minimum_distance_um =
                        train->confirmed_sensor_distance_um;
                if (strict_minimum_distance_um < INT64_MAX) {
                        ++strict_minimum_distance_um;
                }
                int waypoint = waypoint_for_physical_sensor(
                        train, selected_sensor,
                        strict_minimum_distance_um);
                /*
                 * A delayed duplicate or an unexpected detector can be
                 * attributed after prediction has already passed its route
                 * position.  Consume its sequence, but never pull the train
                 * marker backwards/off the active route.  The next valid
                 * ordered route detector remains authoritative.
                 */
                if (waypoint < 0) {
                        /*
                         * A geometry-only one-cell CURRENT localization
                         * handoff deliberately freezes the marker until the
                         * exact replacement route is known. Its attributed
                         * physical detector must still flash; it just cannot
                         * be used to invent a visual path or move '@'.
                         */
                        if (Tc2UiOverlayFlashSensor(
                                    &ui->overlay, train->train,
                                    selected_sensor,
                                    train->plan_generation,
                                    selected_sequence,
                                    TC2_UI_QUALITY_SENSOR_CONFIRMED,
                                    now_tick) < 0) {
                                return -1;
                        }
                        if (train->localization_placeholder ||
                            train->awaiting_current_sensor) {
                                /*
                                 * Do not consume physical evidence while a
                                 * CURRENT route is incomplete or unaligned.
                                 * The exact replacement projection can retry
                                 * this same snapshot occurrence.
                                 */
                                return changed;
                        }
                        train->last_sensor_sequence = selected_sequence;
                        continue;
                }
                train->last_sensor_sequence = selected_sequence;
                train->awaiting_current_sensor = 0;
                if (Tc2UiOverlayFlashSensor(
                            &ui->overlay, train->train,
                            selected_sensor, train->plan_generation,
                            selected_sequence,
                            TC2_UI_QUALITY_SENSOR_CONFIRMED,
                            now_tick) < 0) {
                        return -1;
                }
                if (train->waypoints[waypoint].distance_um >
                            train->confirmed_sensor_distance_um) {
                        train->confirmed_sensor_distance_um =
                                train->waypoints[waypoint].distance_um;
                }
                /*
                 * A detector confirms route progress, but a delayed edge
                 * must never pull the continuously predicted train marker
                 * backwards. Keep the sensor flash as physical evidence and
                 * advance the motion anchor only when the detector is ahead
                 * of the distance already rendered.
                 */
                if (train->waypoints[waypoint].distance_um >=
                            train->displayed_distance_um) {
                        train->anchor_distance_um =
                                train->waypoints[waypoint].distance_um;
                        train->displayed_distance_um =
                                train->anchor_distance_um;
                        train->anchor_tick = now_tick;
                        /*
                         * A confirmed detector is also a position high-water
                         * mark.  Holds have zero predicted velocity, so move
                         * the marker directly to the matched route cell; the
                         * >= guard above ensures a delayed event cannot move
                         * it backwards.
                         */
                        ++train->position_revision;
                        if (train->position_revision == 0) {
                                ++train->position_revision;
                        }
                        if (Tc2UiOverlayUpdatePredictedCell(
                                    &ui->overlay, train->train,
                                    train->waypoints[waypoint].ui_row,
                                    train->waypoints[waypoint].ui_column,
                                    train->plan_generation,
                                    train->position_revision,
                                    now_tick) < 0) {
                                return -1;
                        }
                }
                changed = 1;
        }
        return changed;
}

static int waypoint_has_visual_cell(
        const tc2_dispatch_projection_waypoint *waypoint) {
        return waypoint &&
                waypoint->kind != TC2_ROUTE_WAYPOINT_TURNOUT &&
                waypoint->ui_width > 0 &&
                waypoint->ui_row < TC2_TRACK_D_LAYOUT_ROWS &&
                waypoint->ui_column <
                        TC2_TRACK_D_LAYOUT_MAP_COLUMNS;
}

static int interpolate_component(
        int first, int second, int64_t offset, int64_t span) {
        if (span <= 0 || offset <= 0) return first;
        if (offset >= span) return second;
        return first + (int)(((int64_t)(second - first) * offset) /
                             span);
}

static int route_step(
        int64_t offset, int64_t span, int last_step) {
        if (span <= 0 || offset <= 0) return 0;
        if (offset >= span) return last_step;
        return (int)((offset * last_step) / span);
}

/*
 * The compact ASCII map is schematic rather than Euclidean.  In particular,
 * a straight screen interpolation from E12 to C8 cuts through blank space
 * and then snaps between the lower and upper rows.  Walk the drawn Track-D
 * cells explicitly so A->d1 visibly traverses the upper C4/C3 row.
 */
static int interpolate_e12_to_c8(
        int64_t offset, int64_t span, int *row, int *column) {
        int step;
        enum { LAST_STEP = 52 };
        if (!row || !column || span <= 0 ||
            offset < 0 || offset > span) {
                return 0;
        }
        step = route_step(offset, span, LAST_STEP);
        if (step <= 11) {
                *row = 5;
                *column = 7 + step;
        } else if (step == 12) {
                *row = 3;
                *column = 18;
        } else {
                *row = 1;
                *column = 22 + (step - 13);
        }
        return 1;
}

/*
 * Known bends on the catalogued A-F routes. Every returned coordinate is a
 * visible '-', '/', '\\', '|', sensor label, or endpoint on the operator's
 * Track-D drawing. This prevents a fast prediction from cutting across
 * whitespace while retaining smooth motion on the long horizontal rails.
 */
static int interpolate_known_track_path(
        const tc2_dispatch_projection_waypoint *first,
        const tc2_dispatch_projection_waypoint *second,
        int64_t offset, int64_t span, int *row, int *column) {
        int step;
        if (!first || !second || !row || !column || span <= 0) {
                return 0;
        }
        if (first->sensor_index == 22 &&
            second->sensor_index == 9) {           /* B7 -> A10 */
                step = route_step(offset, span, 7);
                *row = 9;
                if (step == 0) {
                        *column = 105;
                } else if (step <= 6) {
                        *column = 105 - step;
                } else {
                        *column = 98;
                }
                return 1;
        }
        if (first->sensor_index == 9 &&
            second->sensor_index == 38) {          /* A10 -> C7 */
                step = route_step(offset, span, 29);
                if (step == 0) {
                        *row = 9;
                        *column = 98;
                } else if (step <= 4) {
                        *row = 9;
                        *column = 93 - step;
                } else if (step == 5) {
                        *row = 7;
                        *column = 88;
                } else if (step == 6) {
                        *row = 5;
                        *column = 86;
                } else if (step == 7) {
                        *row = 3;
                        *column = 84;
                } else if (step == 8) {
                        *row = 1;
                        *column = 82;
                } else {
                        *row = 1;
                        *column = 90 - step;
                }
                return 1;
        }
        if (first->sensor_index == 38 &&
            second->sensor_index == 74) {          /* C7 -> E11 */
                step = route_step(offset, span, 60);
                if (step == 0) {
                        *row = 1;
                        *column = 61;
                } else if (step <= 13) {
                        *row = 1;
                        *column = 61 - step;
                } else if (step == 14) {
                        *row = 3;
                        *column = 52;
                } else if (step == 15) {
                        *row = 5;
                        *column = 52;
                } else {
                        *row = 5;
                        *column = 67 - step;
                }
                return 1;
        }
        if (first->sensor_index == 74 &&
            second->sensor_index == 57) {          /* E11 -> D10 */
                step = route_step(offset, span, 6);
                if (step == 0) {
                        *row = 5;
                        *column = 7;
                } else if (step <= 3) {
                        *row = 5;
                        *column = 7 - step;
                } else if (step == 4) {
                        *row = 7;
                        *column = 3;
                } else if (step == 5) {
                        *row = 9;
                        *column = 0;
                } else {
                        *row = 11;
                        *column = 0;
                }
                return 1;
        }
        if (first->sensor_index == 57 &&
            second->sensor_index == 55) {          /* D10 -> D8 */
                step = route_step(offset, span, 11);
                if (step == 0) {
                        *row = 11;
                        *column = 0;
                } else if (step < 11) {
                        *row = 11 + 2 * step;
                        *column = 0;
                } else {
                        *row = 33;
                        *column = 4;
                }
                return 1;
        }
        if (first->sensor_index == 55 &&
            second->sensor_index == 71) {          /* D8 -> E8 */
                step = route_step(offset, span, 21);
                if (step == 0) {
                        *row = 33;
                        *column = 4;
                } else if (step <= 4) {
                        *row = 33;
                        *column = 4 + step;
                } else if (step == 5) {
                        *row = 35;
                        *column = 3;
                } else if (step < 21) {
                        *row = 35;
                        *column = step - 2;
                } else {
                        *row = 35;
                        *column = 23;
                }
                return 1;
        }
        if (first->sensor_index == 71 &&
            second->sensor_index == 45) {          /* E8 -> C14 */
                step = route_step(offset, span, 42);
                *row = 35;
                if (step == 0) {
                        *column = 23;
                } else if (step < 42) {
                        *column = 23 + step;
                } else {
                        *column = 71;
                }
                return 1;
        }
        if (first->sensor_index == 70 &&
            second->sensor_index == 54) {          /* E7 -> D7 */
                step = route_step(offset, span, 25);
                if (step <= 16) {
                        *row = 35;
                        *column = 19 - step;
                } else if (step == 17) {
                        *row = 33;
                        *column = 8;
                } else {
                        *row = 33;
                        *column = 25 - step;
                }
                return 1;
        }
        if (first->sensor_index == 54 &&
            second->sensor_index == 56) {          /* D7 -> D9 */
                step = route_step(offset, span, 11);
                *row = 33 - 2 * step;
                *column = 0;
                return 1;
        }
        if (first->sensor_index == 54 &&
            second->sensor_index == 73) {          /* D7 -> E10 */
                step = route_step(offset, span, 10);
                if (step <= 8) {
                        *row = 33 - 2 * step;
                        *column = 0;
                } else if (step == 9) {
                        *row = 15;
                        *column = 3;
                } else {
                        *row = 13;
                        *column = 4;
                }
                return 1;
        }
        if (first->sensor_index == 56 &&
            second->sensor_index == 75) {          /* D9 -> E12 */
                step = route_step(offset, span, 6);
                if (step == 0) {
                        *row = 11;
                        *column = 0;
                } else if (step == 1) {
                        *row = 9;
                        *column = 0;
                } else if (step == 2) {
                        *row = 7;
                        *column = 3;
                } else {
                        *row = 5;
                        *column = step + 1;
                }
                return 1;
        }
        if (first->sensor_index == 75 &&
            second->sensor_index == 39) {          /* E12 -> C8 */
                return interpolate_e12_to_c8(
                        offset, span, row, column);
        }
        if (first->sensor_index == 39 &&
            second->sensor_index == 11) {          /* C8 -> A12 */
                step = route_step(offset, span, 27);
                if (step <= 21) {
                        *row = 1;
                        *column = 61 + step;
                } else {
                        static const int bend_rows[6] =
                                {3, 5, 7, 9, 11, 13};
                        static const int bend_columns[6] =
                                {84, 86, 88, 89, 91, 93};
                        *row = bend_rows[step - 22];
                        *column = bend_columns[step - 22];
                }
                return 1;
        }
        if (first->sensor_index == 11 &&
            second->kind == TC2_ROUTE_WAYPOINT_DESTINATION &&
            second->destination_index == 0) {      /* A12 -> d1 */
                static const int rows[5] = {13, 15, 17, 19, 21};
                static const int columns[5] = {93, 94, 95, 95, 94};
                step = route_step(offset, span, 4);
                *row = rows[step];
                *column = columns[step];
                return 1;
        }
        if (first->sensor_index == 73 &&
            second->sensor_index == 76) {          /* E10 -> E13 */
                step = route_step(offset, span, 6);
                if (step == 0) {
                        *row = 13;
                        *column = 4;
                } else if (step == 1) {
                        *row = 11;
                        *column = 10;
                } else {
                        *row = 9;
                        *column = step + 9;
                }
                return 1;
        }
        if (first->sensor_index == 76 &&
            second->sensor_index == 62) {          /* E13 -> D15 */
                step = route_step(offset, span, 16);
                if (step <= 15) {
                        *row = 9;
                        *column = 15 + step;
                } else {
                        *row = 11;
                        *column = 29;
                }
                return 1;
        }
        if (first->sensor_index == 62 &&
            second->sensor_index == 28) {          /* D15 -> B13 */
                static const int rows[3] = {11, 13, 15};
                static const int columns[3] = {29, 33, 29};
                step = route_step(offset, span, 2);
                *row = rows[step];
                *column = columns[step];
                return 1;
        }
        if (first->sensor_index == 28 &&
            second->sensor_index == 49) {          /* B13 -> D2 */
                static const int rows[7] =
                        {15, 17, 19, 21, 23, 25, 27};
                static const int columns[7] =
                        {29, 36, 41, 41, 41, 36, 31};
                step = route_step(offset, span, 6);
                *row = rows[step];
                *column = columns[step];
                return 1;
        }
        if (first->sensor_index == 49 &&
            second->sensor_index == 67) {          /* D2 -> E4 */
                static const int rows[3] = {27, 29, 31};
                static const int columns[3] = {31, 35, 29};
                step = route_step(offset, span, 2);
                *row = rows[step];
                *column = columns[step];
                return 1;
        }
        if (first->sensor_index == 67 &&
            second->sensor_index == 68) {          /* E4 -> E5 */
                step = route_step(offset, span, 14);
                if (step == 0) {
                        *row = 31;
                        *column = 29;
                } else if (step == 1) {
                        *row = 33;
                        *column = 31;
                } else {
                        *row = 33;
                        *column = 32 - step;
                }
                return 1;
        }
        if (first->sensor_index == 68 &&
            second->sensor_index == 53) {          /* E5 -> D6 */
                step = route_step(offset, span, 12);
                if (step <= 10) {
                        *row = 33;
                        *column = 18 - step;
                } else if (step == 11) {
                        *row = 31;
                        *column = 5;
                } else {
                        *row = 29;
                        *column = 4;
                }
                return 1;
        }
        if (first->sensor_index == 40 &&
            second->sensor_index == 30) {          /* C9 -> B15 */
                step = route_step(offset, span, 12);
                if (step <= 10) {
                        *row = 9;
                        *column = 66 + step;
                } else if (step == 11) {
                        *row = 11;
                        *column = 77;
                } else {
                        *row = 13;
                        *column = 75;
                }
                return 1;
        }
        if (first->sensor_index == 30 &&
            second->sensor_index == 2) {           /* B15 -> A3 */
                static const int rows[9] =
                        {13, 15, 17, 19, 21, 23, 25, 27, 29};
                static const int columns[9] =
                        {75, 80, 81, 81, 80, 81, 81, 80, 76};
                step = route_step(offset, span, 8);
                *row = rows[step];
                *column = columns[step];
                return 1;
        }
        if (first->sensor_index == 2 &&
            second->sensor_index == 42) {          /* A3 -> C11 */
                step = route_step(offset, span, 14);
                if (step == 0) {
                        *row = 29;
                        *column = 76;
                } else if (step == 1) {
                        *row = 31;
                        *column = 77;
                } else if (step == 2) {
                        *row = 33;
                        *column = 79;
                } else {
                        *row = 33;
                        *column = 81 - step;
                }
                return 1;
        }
        if (first->sensor_index == 45 &&
            second->sensor_index == 14) {          /* C14 -> A15 */
                step = route_step(offset, span, 22);
                if (step == 0) {
                        *row = 35;
                        *column = 71;
                } else if (step <= 19) {
                        *row = 35;
                        *column = 71 + step;
                } else if (step == 20) {
                        *row = 33;
                        *column = 91;
                } else if (step == 21) {
                        *row = 31;
                        *column = 93;
                } else {
                        *row = 29;
                        *column = 94;
                }
                return 1;
        }
        if (first->sensor_index == 14 &&
            second->kind == TC2_ROUTE_WAYPOINT_DESTINATION &&
            second->destination_index == 0) {      /* A15 -> d1 */
                static const int rows[5] = {29, 27, 25, 23, 21};
                static const int columns[5] = {94, 94, 95, 95, 94};
                step = route_step(offset, span, 4);
                *row = rows[step];
                *column = columns[step];
                return 1;
        }
        return 0;
}

int Tc2LiveUiInterpolateTrackCell(
        const tc2_dispatch_projection_waypoint *first,
        const tc2_dispatch_projection_waypoint *second,
        int64_t offset_um, int64_t span_um,
        int *row, int *column) {
        if (!first || !second || !row || !column ||
            offset_um < 0) {
                return 0;
        }
        if (span_um <= 0) {
                *row = first->ui_row;
                *column = first->ui_column;
                return Tc2TrackDLayoutCellIsOccupied(*row, *column);
        }
        if (offset_um > span_um) return 0;
        if (Tc2UiPathGeometryInterpolate(
                    first, second, offset_um, span_um,
                    row, column) ||
            interpolate_known_track_path(
                    first, second, offset_um, span_um,
                    row, column)) {
                return Tc2TrackDLayoutCellIsOccupied(*row, *column);
        }
        *row = interpolate_component(
                first->ui_row, second->ui_row,
                offset_um, span_um);
        *column = interpolate_component(
                first->ui_column, second->ui_column,
                offset_um, span_um);
        return Tc2TrackDLayoutCellIsOccupied(*row, *column);
}

/*
 * An exact CURRENT replacement can arrive after the one-cell localization
 * placeholder has already retained the physical marker.  If that marker is
 * visibly on the replacement route's initial rail segment, it is sufficient
 * evidence to rebase the UI trajectory immediately: the dispatcher-provided
 * pre-origin prefix keeps the first reverse sensor at its real positive
 * distance.  A marker that cannot be aligned remains fail-closed behind the
 * physical sensor gate.
 *
 * The ASCII map has fewer than ROWS+COLUMNS distinct steps on any initial
 * visual segment.  Sampling that many normalized offsets visits every cell
 * produced by both the generic interpolation and the catalogued paths,
 * without iterating once per micrometre.
 */
static int retained_current_cell_maps_to_initial_route(
        const tc2_dispatch_projection_waypoint *waypoints,
        int waypoint_count, int retained_row, int retained_column,
        int64_t *mapped_distance_um) {
        const tc2_dispatch_projection_waypoint *first;
        const tc2_dispatch_projection_waypoint *second = 0;
        int64_t span_um;
        int samples = TC2_TRACK_D_LAYOUT_ROWS +
                TC2_TRACK_D_LAYOUT_MAP_COLUMNS;

        if (mapped_distance_um) *mapped_distance_um = 0;
        if (!waypoints || waypoint_count < 2 ||
            !Tc2TrackDLayoutCellIsOccupied(
                    retained_row, retained_column)) {
                return 0;
        }
        first = &waypoints[0];
        for (int index = 1; index < waypoint_count; ++index) {
                if (waypoint_has_visual_cell(&waypoints[index])) {
                        second = &waypoints[index];
                        break;
                }
        }
        if (!second) return 0;
        span_um = second->distance_um - first->distance_um;
        if (span_um <= 0 || samples < 1) return 0;

        for (int sample = 0; sample <= samples; ++sample) {
                int row;
                int column;
                int64_t quotient = span_um / samples;
                int64_t remainder = span_um % samples;
                int64_t offset_um = quotient * sample +
                        (remainder * sample) / samples;
                if (Tc2LiveUiInterpolateTrackCell(
                            first, second, offset_um, span_um,
                            &row, &column) &&
                    row == retained_row &&
                    column == retained_column) {
                        if (mapped_distance_um) {
                                *mapped_distance_um =
                                        first->distance_um + offset_um;
                        }
                        return 1;
                }
        }
        return 0;
}

static int update_predicted_position(
        tc2_live_ui *ui, tc2_live_ui_train *train,
        const tc2_dispatch_job_snapshot *job,
        uint32_t now_tick) {
        int previous = -1;
        int next = -1;
        int64_t progress_um;
        int velocity_um_per_tick;
        int row;
        int column;
        const tc2_ui_train_overlay *shown;

        if (!ui || !train || !job ||
            !job_has_physical_motion(job, train)) {
                return 0;
        }
        if (train->awaiting_current_sensor) {
                return 0;
        }
        velocity_um_per_tick =
                job->prediction_velocity_um_per_tick;
        if (velocity_um_per_tick <= 0) return 0;
        progress_um = train->anchor_distance_um +
                (int64_t)(now_tick - train->anchor_tick) *
                        velocity_um_per_tick;
        if (progress_um < train->anchor_distance_um) {
                progress_um = train->anchor_distance_um;
        }
        if (progress_um > job->prediction_physical_destination_distance_um &&
            job->prediction_physical_destination_distance_um > 0) {
                progress_um =
                        job->prediction_physical_destination_distance_um;
        }
        if (job->traffic_hold_active &&
            job->traffic_hold_distance_um >= 0 &&
            progress_um >
                    job->traffic_hold_distance_um) {
                progress_um =
                        job->traffic_hold_distance_um;
        }
        /*
         * Prediction is monotonic, but physical detectors are one-way gates.
         * '@' may animate along every '-', '/', '\\', or '|' cell leading to
         * the next sensor; it must then wait on the last pre-sensor distance
         * until that sensor's attributed rising edge flashes. This prevents a
         * fast command-120 projection from visually passing A1, C13, or E7
         * before the locomotive has actually reached them.
         */
        if (progress_um < train->displayed_distance_um) {
                progress_um = train->displayed_distance_um;
        }
        int64_t next_sensor_distance_um =
                next_unconfirmed_sensor_distance_um(train);
        if (next_sensor_distance_um > 0) {
                int64_t pre_sensor_hold_um =
                        next_sensor_distance_um - 1;
                if (pre_sensor_hold_um >=
                            train->displayed_distance_um &&
                    progress_um > pre_sensor_hold_um) {
                        progress_um = pre_sensor_hold_um;
                }
        }

        for (int index = 0; index < train->waypoint_count; ++index) {
                const tc2_dispatch_projection_waypoint *waypoint =
                        &train->waypoints[index];
                if (!waypoint_has_visual_cell(waypoint)) continue;
                if (waypoint->distance_um <= progress_um) {
                        previous = index;
                        continue;
                }
                next = index;
                break;
        }
        if (previous < 0) {
                for (int index = 0;
                     index < train->waypoint_count; ++index) {
                        if (waypoint_has_visual_cell(
                                    &train->waypoints[index])) {
                                previous = index;
                                break;
                        }
                }
        }
        if (previous < 0) return 0;
        if (next < 0) next = previous;
        const tc2_dispatch_projection_waypoint *first =
                &train->waypoints[previous];
        const tc2_dispatch_projection_waypoint *second =
                &train->waypoints[next];
        int64_t span = second->distance_um - first->distance_um;
        int64_t offset = progress_um - first->distance_um;
        if (!Tc2LiveUiInterpolateTrackCell(
                    first, second, offset, span,
                    &row, &column)) {
                /*
                 * Unknown geometry must never create the old midpoint snap
                 * or move a train onto a different rail. Ordinary A-F/d1-d8
                 * plans are exhaustively covered by the matrix test; this
                 * branch is only a fail-closed guard for corrupt/new data.
                 */
                shown = Tc2UiOverlayFindTrain(
                        &ui->overlay, train->train);
                if (shown &&
                    Tc2TrackDLayoutCellIsOccupied(
                            shown->row, shown->column)) {
                        row = shown->row;
                        column = shown->column;
                } else {
                        row = first->ui_row;
                        column = first->ui_column;
                }
        }
        if (row < 0 || row >= TC2_TRACK_D_LAYOUT_ROWS ||
            column < 0 ||
            column >= TC2_TRACK_D_LAYOUT_MAP_COLUMNS) {
                return -1;
        }
        train->displayed_distance_um = progress_um;
        shown = Tc2UiOverlayFindTrain(
                &ui->overlay, train->train);
        if (shown && shown->row == row &&
            shown->column == column &&
            shown->position_kind ==
                    TC2_UI_POSITION_PREDICTED) {
                return 0;
        }
        ++train->position_revision;
        if (train->position_revision == 0) {
                ++train->position_revision;
        }
        return Tc2UiOverlayUpdatePredictedCell(
                &ui->overlay, train->train, row, column,
                train->plan_generation,
                train->position_revision, now_tick);
}

static int update_arrived_position(
        tc2_live_ui *ui, tc2_live_ui_train *train,
        const tc2_dispatch_job_snapshot *job,
        uint32_t now_tick) {
        const tc2_ui_train_overlay *shown;
        if (!ui || !train || !job ||
            job->state != TC2_JOB_ARRIVED) {
                return 0;
        }
        /*
         * ARRIVED is only renderable at d1-d8 after the dispatcher has
         * confirmed the interior endpoint offset.  An estimated/unconfirmed
         * terminal state can occur during a CURRENT handoff or failed sensor
         * sequence; teleporting '@' would hide the physical failure.
         */
        if (job->remaining_distance_mm > 0 ||
            !job->destination_offset_confirmed) {
                return 0;
        }
        shown = Tc2UiOverlayFindTrain(
                &ui->overlay, train->train);
        if (shown && shown->position_kind ==
                    TC2_UI_POSITION_DESTINATION) {
                return 0;
        }
        ++train->position_revision;
        if (train->position_revision == 0) {
                ++train->position_revision;
        }
        return Tc2UiOverlayUpdateAtDestination(
                &ui->overlay, train->train,
                train->destination_index,
                train->plan_generation,
                train->position_revision,
                TC2_UI_QUALITY_ESTIMATED, now_tick);
}

int Tc2LiveUiSync(
        tc2_live_ui *ui, const tc2_dispatch_snapshot *dispatch,
        const train_sensor_snapshot_t *sensors, uint32_t now_tick) {
        if (!ui || !ui->initialized || !dispatch || !sensors) return -1;
        ui->tick = now_tick;
        for (int slot = 0; slot < TC2_UI_MAX_TRAINS; ++slot) {
                tc2_live_ui_train *train = &ui->trains[slot];
                if (!train->active) continue;
                const tc2_dispatch_job_snapshot *job =
                        find_job(dispatch, train->train,
                                 train->plan_generation);
                if (!job) {
                        /*
                         * A snapshot and projection are two separately
                         * published records. If either side is one
                         * generation ahead, preserve the visible marker
                         * until a matching pair is observed; removing it
                         * would poison the overlay high-water mark and
                         * create a visible disappear/reappear race.
                         */
                        if (find_job_for_train(
                                    dispatch, train->train)) {
                                if (train->last_job_state !=
                                            TC2_LIVE_UI_JOB_STATE_GENERATION_HANDOFF) {
                                        train->last_job_state =
                                                TC2_LIVE_UI_JOB_STATE_GENERATION_HANDOFF;
                                        train->anchor_tick = now_tick;
                                } else if ((uint32_t)(
                                                   now_tick -
                                                   train->anchor_tick) >
                                           TC2_LIVE_UI_GENERATION_HANDOFF_TICKS) {
                                        /*
                                         * Snapshot/projection publication is
                                         * asynchronous, but a stale route is
                                         * not valid forever. Keep the last
                                         * physical marker visible and stop
                                         * advertising/predicting the old plan
                                         * until its replacement projection is
                                         * accepted.
                                         */
                                        train->localization_placeholder = 1;
                                        train->awaiting_current_sensor = 1;
                                        train->current_speed = 0;
                                        if (Tc2UiOverlaySetTrainSpeed(
                                                    &ui->overlay,
                                                    train->train, 0,
                                                    train->plan_generation,
                                                    now_tick) < 0) {
                                                return -1;
                                        }
                                }
                                continue;
                        }
                        (void)Tc2UiOverlayRemoveTrain(
                                &ui->overlay, train->train);
                        zero_bytes(train, sizeof(*train));
                        continue;
                }
                int next_current_speed =
                        job_visible_speed(job, train);
                if (job_state_supports_physical_prediction(job->state) &&
                    (!job_state_supports_physical_prediction(
                             train->last_job_state) ||
                     (train->current_speed == 0 &&
                      next_current_speed > 0))) {
                        train->anchor_distance_um =
                                train->displayed_distance_um;
                        train->anchor_tick = now_tick;
                }
                train->current_speed = next_current_speed;
                if (Tc2UiOverlaySetTrainSpeed(
                            &ui->overlay, train->train,
                            train->current_speed,
                            train->plan_generation,
                            now_tick) < 0) {
                        return -1;
                }
                int sensor_changed = apply_new_sensor_events(
                        ui, train, sensors, now_tick);
                if (sensor_changed < 0) return -1;
                /*
                 * Sensor labels retain their independent flash overlay.
                 * Always redraw the train from its monotonic route distance
                 * in the same refresh so the flash cannot teleport '@'.
                 */
                if (update_predicted_position(
                            ui, train, job, now_tick) < 0) {
                        return -1;
                }
                if (update_arrived_position(
                            ui, train, job, now_tick) < 0) {
                        return -1;
                }
                train->last_job_state = job->state;
        }
        return Tc2UiOverlayExpireSensorFlashes(
                &ui->overlay, now_tick) < 0 ? -1 : 0;
}

int Tc2LiveUiRender(
        tc2_live_ui *ui, const char *prompt_restore,
        size_t prompt_restore_length,
        tc2_live_ui_render_result *result) {
        tc2_live_ui_render_result local;
        zero_bytes(&local, sizeof(local));
        if (!ui || !ui->initialized || !prompt_restore ||
            prompt_restore_length == 0) {
                return -1;
        }
        if (Tc2UiTxPumpNeedsFullRedraw(&ui->pump)) {
                Tc2UiRendererInvalidate(&ui->renderer);
        }
        uint32_t generation =
                next_nonzero(&ui->next_render_generation);
        int status = Tc2UiTxPumpBeginFrame(&ui->pump, generation);
        if (status == TC2_UI_TX_BUSY) return 2;
        if (status != TC2_UI_TX_OK) return -1;
        (void)Tc2UiOverlayAdvanceRefresh(&ui->overlay);
        status = Tc2UiRendererRender(
                &ui->renderer, &ui->overlay, ui->tick,
                Tc2UiTxPumpStageWrite, &ui->pump, &local.renderer);
        if (status != 0) {
                Tc2UiTxPumpAbortFrame(&ui->pump);
                Tc2UiRendererInvalidate(&ui->renderer);
                return -1;
        }
        if (local.renderer.emitted_bytes == 0 &&
            Tc2UiTxPumpStageWrite(&ui->pump, "\033[0m", 4) != 0) {
                Tc2UiTxPumpAbortFrame(&ui->pump);
                return -1;
        }
        int kind = local.renderer.full_redraw ?
                TC2_UI_TX_FRAME_FULL : TC2_UI_TX_FRAME_DELTA;
        status = Tc2UiTxPumpCommitFrame(
                &ui->pump, (tc2_ui_tx_frame_kind)kind,
                prompt_restore, prompt_restore_length);
        if (status == TC2_UI_TX_NEEDS_FULL_REDRAW) return 3;
        if (status != TC2_UI_TX_OK) return -1;
        local.generation = generation;
        local.frame_kind = kind;
        local.queued = 1;
        if (result) copy_bytes(result, &local, sizeof(*result));
        return 0;
}

int Tc2LiveUiDrain(
        tc2_live_ui *ui, tc2_ui_tx_write_fn write,
        void *write_context, unsigned int max_chunks,
        tc2_ui_tx_drain_result *result) {
        if (!ui || !ui->initialized || !write || max_chunks == 0) return -1;
        int status = Tc2UiTxPumpDrain(
                &ui->pump, write, write_context, max_chunks, result);
        if (status == TC2_UI_TX_OK || status == TC2_UI_TX_IDLE) return 0;
        if (status == TC2_UI_TX_BACKPRESSURE) return 2;
        if (status == TC2_UI_TX_NEEDS_FULL_REDRAW) {
                Tc2UiRendererInvalidate(&ui->renderer);
                return 3;
        }
        Tc2UiRendererInvalidate(&ui->renderer);
        return -1;
}

int Tc2LiveUiHasPendingOutput(const tc2_live_ui *ui) {
        return ui && ui->initialized ?
                Tc2UiTxPumpHasWork(&ui->pump) : 0;
}

int Tc2LiveUiNeedsFullRedraw(const tc2_live_ui *ui) {
        return ui && ui->initialized ?
                Tc2UiTxPumpNeedsFullRedraw(&ui->pump) : 1;
}

#endif
