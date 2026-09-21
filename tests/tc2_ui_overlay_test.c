#include <stdio.h>
#include <string.h>
#include <limits.h>

#include "../tc2_ui_overlay.h"

static int fail(const char *message) {
        fprintf(stderr, "tc2_ui_overlay_test: %s\n", message);
        return 1;
}

static int source_and_color_test(void) {
        tc2_ui_overlay overlay;
        int colors[TC2_UI_COLOR_COUNT] = {0};

        Tc2UiOverlayInit(&overlay, TC2_UI_SOURCE_OFFLINE_SIMULATION);
        if (strcmp(Tc2UiOverlaySourceName(overlay.source),
                   "OFFLINE SIMULATION / NO PHYSICAL MOTION") != 0 ||
            Tc2UiOverlayActiveTrainCount(&overlay) != 0) {
                return fail("offline source was not explicit");
        }

        for (int slot = 0; slot < TC2_UI_MAX_TRAINS; ++slot) {
                int train = 14 + slot;
                const tc2_ui_train_overlay *entry;
                if (Tc2UiOverlayUpsertAtStart(
                            &overlay, train, slot % 6,
                            slot % 8, 1u,
                            TC2_UI_QUALITY_PREDICTED,
                            100 + slot) != 1) {
                        return fail("could not allocate sixteen trains");
                }
                entry = Tc2UiOverlayFindTrain(&overlay, train);
                if (!entry || entry->color_slot < 0 ||
                    entry->color_slot >= TC2_UI_COLOR_COUNT ||
                    (train == 14 && entry->color_slot != 0) ||
                    (train == 15 && entry->color_slot != 2) ||
                    (train == 17 && entry->color_slot != 3) ||
                    (train == 18 && entry->color_slot != 1) ||
                    colors[entry->color_slot]++ ||
                    Tc2UiOverlayXtermColor(entry->color_slot) < 0) {
                        return fail("active train colors are not unique");
                }
        }
        if (Tc2UiOverlayActiveTrainCount(&overlay) != 16 ||
            Tc2UiOverlayUpsertAtStart(
                    &overlay, 200, 0, 0, 1u,
                    TC2_UI_QUALITY_PREDICTED, 200) >= 0) {
                return fail("train capacity boundary failed");
        }

        {
                const tc2_ui_train_overlay *before =
                        Tc2UiOverlayFindTrain(&overlay, 14);
                int stable_color = before ? before->color_slot : -1;
                if (Tc2UiOverlayUpsertAtStart(
                            &overlay, 14, 5, 7, 2u,
                            TC2_UI_QUALITY_ESTIMATED, 300) != 1 ||
                    !Tc2UiOverlayFindTrain(&overlay, 14) ||
                    Tc2UiOverlayFindTrain(&overlay, 14)->color_slot !=
                            stable_color) {
                        return fail("train color changed after update");
                }
        }

        if (Tc2UiOverlayRemoveTrain(&overlay, 20) < 0 ||
            Tc2UiOverlayUpsertAtStart(
                    &overlay, 200, 0, 0, 1u,
                    TC2_UI_QUALITY_PREDICTED, 400) < 0 ||
            Tc2UiOverlayActiveTrainCount(&overlay) != 16) {
                return fail("released color/slot was not reusable");
        }

        if (Tc2UiOverlaySetSource(
                    &overlay, TC2_UI_SOURCE_LIVE_CAN) < 0 ||
            strcmp(Tc2UiOverlaySourceName(overlay.source),
                   "LIVE CAN") != 0 ||
            Tc2UiOverlaySetSource(&overlay, 99) >= 0 ||
            strcmp(Tc2UiOverlayQualityName(
                           TC2_UI_QUALITY_SENSOR_CONFIRMED),
                   "sensor-confirmed") != 0) {
                return fail("source/quality labels are invalid");
        }

        {
                const tc2_ui_train_overlay *before =
                        Tc2UiOverlayFindTrain(&overlay, 14);
                int row = before ? before->row : -1;
                int column = before ? before->column : -1;
                unsigned int revision = before ?
                        before->position_revision : 0;
                if (Tc2UiOverlayFlashSensor(
                            &overlay, 14, 9, 2u, 1u,
                            TC2_UI_QUALITY_SENSOR_CONFIRMED, 498) != 1 ||
                    !(before = Tc2UiOverlayFindTrain(&overlay, 14)) ||
                    before->row != row ||
                    before->column != column ||
                    before->position_kind != TC2_UI_POSITION_START ||
                    before->position_revision != revision ||
                    !Tc2UiOverlaySensor(&overlay, 9)->active) {
                        return fail(
                                "sensor-only flash moved the train marker");
                }
        }

        if (Tc2UiOverlayUpdateAtSensor(
                    &overlay, 14, 10, 2u, 1u, 1u,
                    TC2_UI_QUALITY_PREDICTED, 499) >= 0 ||
            Tc2UiOverlayFindTrain(&overlay, 14)->position_kind !=
                    TC2_UI_POSITION_START ||
            Tc2UiOverlaySensor(&overlay, 10)->active) {
                return fail(
                        "predicted crossing was accepted as live evidence");
        }
        if (Tc2UiOverlayUpdateAtSensor(
                    &overlay, 14, 10, 2u, 1u, 2u,
                    TC2_UI_QUALITY_SENSOR_CONFIRMED, 500) < 0) {
                return fail("live sensor confirmation was rejected");
        }
        if (Tc2UiOverlaySensor(&overlay, 10)->evidence_source !=
                    TC2_UI_SOURCE_LIVE_CAN ||
            Tc2UiOverlaySensor(&overlay, 10)->evidence_quality !=
                    TC2_UI_QUALITY_SENSOR_CONFIRMED ||
            Tc2UiOverlaySensor(&overlay, 10)->plan_generation != 2u) {
                return fail("live flash did not retain its provenance");
        }
        if (Tc2UiOverlayUpdateAtDestination(
                    &overlay, 14, 7, 2u, 2u,
                    TC2_UI_QUALITY_SENSOR_CONFIRMED, 500) >= 0 ||
            Tc2UiOverlayFindTrain(&overlay, 14)->sensor_index != 10 ||
            Tc2UiOverlayFindTrain(&overlay, 14)->position_revision != 1u) {
                return fail("live virtual destination accepted confirmation");
        }
        if (Tc2UiOverlaySetSource(
                    &overlay,
                    TC2_UI_SOURCE_OFFLINE_SIMULATION) < 0 ||
            Tc2UiOverlayFindTrain(&overlay, 14)->position_quality !=
                    TC2_UI_QUALITY_PREDICTED ||
            Tc2UiOverlaySensor(&overlay, 10)->active ||
            Tc2UiOverlayUpdateAtDestination(
                    &overlay, 14, 7, 2u, 2u,
                    TC2_UI_QUALITY_SENSOR_CONFIRMED, 501) >= 0) {
                return fail("offline mode retained a live confirmation");
        }
        return 0;
}

static int named_train_color_test(void) {
        static const int orders[4][4] = {
                {18, 14, 15, 17},
                {17, 15, 14, 18},
                {15, 18, 17, 14},
                {14, 17, 18, 15}
        };
        static const int trains[4] = {18, 14, 15, 17};
        static const int expected_slot[4] = {1, 0, 2, 3};
        static const char *const expected_name[4] = {
                "GREEN", "RED", "BLUE", "YELLOW"
        };

        for (int order = 0; order < 4; ++order) {
                tc2_ui_overlay overlay;
                Tc2UiOverlayInit(
                        &overlay,
                        TC2_UI_SOURCE_OFFLINE_SIMULATION);
                for (int index = 0; index < 4; ++index) {
                        if (Tc2UiOverlayUpsertAtStart(
                                    &overlay,
                                    orders[order][index],
                                    index, index, 1u,
                                    TC2_UI_QUALITY_PREDICTED,
                                    (unsigned int)index) != 1) {
                                return fail(
                                        "named train allocation failed");
                        }
                }
                for (int index = 0; index < 4; ++index) {
                        const tc2_ui_train_overlay *entry =
                                Tc2UiOverlayFindTrain(
                                        &overlay, trains[index]);
                        if (!entry ||
                            entry->color_slot !=
                                    expected_slot[index] ||
                            strcmp(Tc2UiOverlayColorName(
                                           entry->color_slot),
                                   expected_name[index]) != 0) {
                                return fail(
                                        "named train color changed with "
                                        "dispatch order");
                        }
                }
                if (Tc2UiOverlayRemoveTrain(&overlay, 18) < 0 ||
                    Tc2UiOverlayUpsertAtStart(
                            &overlay, 18, 5, 7, 2u,
                            TC2_UI_QUALITY_PREDICTED, 20u) != 1 ||
                    !Tc2UiOverlayFindTrain(&overlay, 18) ||
                    Tc2UiOverlayFindTrain(
                            &overlay, 18)->color_slot != 1) {
                        return fail(
                                "named train color changed after recreate");
                }
        }

        {
                tc2_ui_overlay overlay;
                Tc2UiOverlayInit(
                        &overlay,
                        TC2_UI_SOURCE_OFFLINE_SIMULATION);
                for (int train = 1; train <= 12; ++train) {
                        if (Tc2UiOverlayUpsertAtStart(
                                    &overlay, train,
                                    train % 6, train % 8, 1u,
                                    TC2_UI_QUALITY_PREDICTED,
                                    (unsigned int)train) != 1) {
                                return fail(
                                        "generic color saturation failed");
                        }
                }
                if (Tc2UiOverlayUpsertAtStart(
                            &overlay, 200, 0, 0, 1u,
                            TC2_UI_QUALITY_PREDICTED, 20u) != 1 ||
                    !Tc2UiOverlayFindTrain(&overlay, 200) ||
                    Tc2UiOverlayFindTrain(
                            &overlay, 200)->color_slot != 0 ||
                    Tc2UiOverlayFlashSensor(
                            &overlay, 200, 10, 1u, 1u,
                            TC2_UI_QUALITY_ESTIMATED, 21u) != 1 ||
                    !Tc2UiOverlaySensor(&overlay, 10)->active ||
                    Tc2UiOverlaySensor(
                            &overlay, 10)->color_slot != 0) {
                        return fail(
                                "reserved color borrow setup failed");
                }
                if (Tc2UiOverlayUpsertAtStart(
                            &overlay, 14, 0, 0, 1u,
                            TC2_UI_QUALITY_PREDICTED, 22u) != 1 ||
                    !Tc2UiOverlayFindTrain(&overlay, 14) ||
                    Tc2UiOverlayFindTrain(
                            &overlay, 14)->color_slot != 0 ||
                    !Tc2UiOverlayFindTrain(&overlay, 200) ||
                    Tc2UiOverlayFindTrain(
                            &overlay, 200)->color_slot != 1 ||
                    Tc2UiOverlaySensor(
                            &overlay, 10)->color_slot != 1) {
                        return fail(
                                "fixed color relocation split train/flash");
                }
        }
        return 0;
}

static int directional_sensor_flash_test(void) {
        tc2_ui_overlay overlay;
        const tc2_ui_train_overlay *train14;
        const tc2_ui_sensor_overlay *a11;
        const tc2_ui_sensor_overlay *a12;

        Tc2UiOverlayInit(&overlay, TC2_UI_SOURCE_LIVE_CAN);
        if (Tc2UiOverlayUpsertAtStart(
                    &overlay, 14, 0, 6, 1u,
                    TC2_UI_QUALITY_PREDICTED, 0) < 0 ||
            Tc2UiOverlayUpsertAtStart(
                    &overlay, 13, 1, 0, 1u,
                    TC2_UI_QUALITY_PREDICTED, 0) < 0) {
                return fail("could not create sensor test trains");
        }

        if (Tc2UiOverlayUpdateAtSensor(
                    &overlay, 14, 10, 1u, 1u, 1u,
                    TC2_UI_QUALITY_SENSOR_CONFIRMED, 100) != 1 ||
            Tc2UiOverlayUpdateAtSensor(
                    &overlay, 13, 11, 1u, 1u, 1u,
                    TC2_UI_QUALITY_SENSOR_CONFIRMED, 100) != 1) {
                return fail("A11/A12 events were not accepted");
        }
        a11 = Tc2UiOverlaySensor(&overlay, 10);
        a12 = Tc2UiOverlaySensor(&overlay, 11);
        if (!a11 || !a12 || !a11->active || !a12->active ||
            a11->row != a12->row ||
            a11->column != a12->column ||
            a11->train != 14 || a12->train != 13 ||
            a11->evidence_source != TC2_UI_SOURCE_LIVE_CAN ||
            a12->evidence_source != TC2_UI_SOURCE_LIVE_CAN ||
            a11->evidence_quality !=
                    TC2_UI_QUALITY_SENSOR_CONFIRMED ||
            a12->evidence_quality !=
                    TC2_UI_QUALITY_SENSOR_CONFIRMED ||
            a11->plan_generation != 1u ||
            a12->plan_generation != 1u) {
                return fail(
                        "co-located directions lost identity or physical cell");
        }
        train14 = Tc2UiOverlayFindTrain(&overlay, 14);
        if (!train14 ||
            train14->position_kind != TC2_UI_POSITION_SENSOR ||
            train14->position_quality !=
                    TC2_UI_QUALITY_SENSOR_CONFIRMED ||
            train14->row != a11->row ||
            train14->column != a11->column) {
                return fail("train marker did not jump to A11");
        }

        if (!Tc2UiOverlaySensorVisible(a11, 100) ||
            Tc2UiOverlaySensorVisible(a11, 110) ||
            !Tc2UiOverlaySensorVisible(a11, 120) ||
            Tc2UiOverlaySensorVisible(a11, 130) ||
            Tc2UiOverlayExpireSensorFlashes(&overlay, 129) != 0 ||
            Tc2UiOverlayExpireSensorFlashes(&overlay, 130) != 2 ||
            Tc2UiOverlaySensor(&overlay, 10)->active ||
            Tc2UiOverlaySensor(&overlay, 11)->active) {
                return fail("300ms sensor flash timing is wrong");
        }

        if (Tc2UiOverlayUpdateAtSensor(
                    &overlay, 14, 12, 1u, 5u, 5u,
                    TC2_UI_QUALITY_SENSOR_CONFIRMED, 200) != 1 ||
            Tc2UiOverlayUpdateAtSensor(
                    &overlay, 14, 13, 1u, 5u, 5u,
                    TC2_UI_QUALITY_SENSOR_CONFIRMED, 201) != 0 ||
            Tc2UiOverlayFindTrain(&overlay, 14)->sensor_index != 12) {
                return fail("duplicate sequence moved a train");
        }

        if (Tc2UiOverlayUpdateAtSensor(
                    &overlay, 14, 14, 1u, 6u, 0u,
                    TC2_UI_QUALITY_SENSOR_CONFIRMED, 202) >= 0) {
                return fail("reserved zero sensor sequence was accepted");
        }

        if (Tc2UiOverlayUpdateAtSensor(
                    &overlay, 14, 16, 1u, 100u, 100u,
                    TC2_UI_QUALITY_SENSOR_CONFIRMED, 300) != 1 ||
            Tc2UiOverlayUpdateAtSensor(
                    &overlay, 13, 16, 1u, 50u, 50u,
                    TC2_UI_QUALITY_SENSOR_CONFIRMED, 301) != 0 ||
            Tc2UiOverlaySensor(&overlay, 16)->train != 14 ||
            Tc2UiOverlaySensor(&overlay, 16)->sequence != 100 ||
            Tc2UiOverlaySensor(&overlay, 16)->started_at_tick != 300 ||
            Tc2UiOverlayFindTrain(&overlay, 13)->sensor_index != 11 ||
            Tc2UiOverlayFindTrain(&overlay, 13)->position_revision != 1u) {
                return fail("late cross-train event moved marker/flash");
        }
        return 0;
}

static int wrapping_tick_test(void) {
        tc2_ui_overlay overlay;
        const tc2_ui_sensor_overlay *sensor;
        unsigned int start = UINT_MAX - 9u;

        Tc2UiOverlayInit(&overlay, TC2_UI_SOURCE_LIVE_CAN);
        if (Tc2UiOverlayUpsertAtStart(
                    &overlay, 14, 0, 6, 1u,
                    TC2_UI_QUALITY_PREDICTED, start) < 0) {
                return fail("could not register wrapping test train");
        }
        if (Tc2UiOverlayUpdateAtSensor(
                    &overlay, 14, 10, 1u, UINT_MAX, UINT_MAX,
                    TC2_UI_QUALITY_SENSOR_CONFIRMED, start) != 1) {
                return fail("could not create wrapping sensor flash");
        }
        sensor = Tc2UiOverlaySensor(&overlay, 10);
        if (!sensor ||
            !Tc2UiOverlaySensorVisible(sensor, start) ||
            Tc2UiOverlaySensorVisible(sensor, start + 10u) ||
            !Tc2UiOverlaySensorVisible(sensor, start + 20u) ||
            Tc2UiOverlaySensorVisible(sensor, start + 30u) ||
            Tc2UiOverlayExpireSensorFlashes(
                    &overlay, start + 29u) != 0 ||
            Tc2UiOverlayExpireSensorFlashes(
                    &overlay, start + 30u) != 1) {
                return fail("sensor flash failed across tick wrap");
        }

        if (Tc2UiOverlayUpdateAtSensor(
                    &overlay, 14, 11, 1u, 1u, 1u,
                    TC2_UI_QUALITY_SENSOR_CONFIRMED,
                    start + 31u) != 1 ||
            Tc2UiOverlayFindTrain(&overlay, 14)->sensor_index != 11) {
                return fail("sensor sequence failed across uint32 wrap");
        }
        return 0;
}

static int marker_and_bounds_test(void) {
        tc2_ui_overlay overlay;
        const tc2_ui_train_overlay *entry;

        Tc2UiOverlayInit(&overlay, TC2_UI_SOURCE_OFFLINE_SIMULATION);
        if (Tc2UiOverlayMarkerPhase(&overlay) != 0 ||
            Tc2UiOverlayAdvanceRefresh(&overlay) != 1 ||
            Tc2UiOverlayMarkerPhase(&overlay) != 1 ||
            Tc2UiOverlayAdvanceRefresh(&overlay) != 2 ||
            Tc2UiOverlayMarkerPhase(&overlay) != 0) {
                return fail("train marker refresh phase did not toggle");
        }

        if (Tc2UiOverlayUpsertAtStart(
                    &overlay, 14, 0, 6, 1u,
                    TC2_UI_QUALITY_PREDICTED, 0) < 0 ||
            Tc2UiOverlayUpdatePredictedCell(
                    &overlay, 14, 7, 29, 1u, 1u, 10) != 1 ||
            !(entry = Tc2UiOverlayFindTrain(&overlay, 14)) ||
            entry->position_kind != TC2_UI_POSITION_PREDICTED ||
            entry->position_quality != TC2_UI_QUALITY_PREDICTED ||
            entry->row != 7 || entry->column != 29 ||
            Tc2UiOverlayUpdateAtDestination(
                    &overlay, 14, 6, 1u, 2u,
                    TC2_UI_QUALITY_ESTIMATED, 20) < 0 ||
            Tc2UiOverlayFindTrain(&overlay, 14)->position_kind !=
                    TC2_UI_POSITION_DESTINATION) {
                return fail("predicted/destination marker update failed");
        }

        if (Tc2UiOverlayUpdatePredictedCell(
                    &overlay, 14, -1, 0, 1u, 3u, 20) >= 0 ||
            Tc2UiOverlayUpdatePredictedCell(
                    &overlay, 14, TC2_TRACK_D_LAYOUT_ROWS,
                    0, 1u, 3u, 20) >= 0 ||
            Tc2UiOverlayUpdateAtSensor(
                    &overlay, 14, 80, 1u, 3u, 3u,
                    TC2_UI_QUALITY_ESTIMATED, 20) >= 0 ||
            Tc2UiOverlayUpsertAtStart(
                    &overlay, 0, 0, 0, 1u,
                    TC2_UI_QUALITY_PREDICTED, 20) >= 0 ||
            Tc2UiOverlayXtermColor(-1) >= 0 ||
            Tc2UiOverlayXtermColor(16) >= 0) {
                return fail("invalid overlay boundary accepted");
        }
        return 0;
}

static int lifecycle_generation_test(void) {
        tc2_ui_overlay overlay;
        const tc2_ui_train_overlay *train;
        const tc2_ui_sensor_overlay *sensor;
        int stable_color;

        Tc2UiOverlayInit(&overlay, TC2_UI_SOURCE_OFFLINE_SIMULATION);
        if (Tc2UiOverlayUpsertAtStart(
                    &overlay, 14, 0, 6, 10u,
                    TC2_UI_QUALITY_PREDICTED, 1) != 1) {
                return fail("could not register lifecycle test train");
        }
        stable_color = Tc2UiOverlayFindTrain(&overlay, 14)->color_slot;
        if (Tc2UiOverlayUpdatePredictedCell(
                    &overlay, 14, 7, 29, 10u, 1u, 2) != 1 ||
            Tc2UiOverlayUpsertAtStart(
                    &overlay, 14, 5, 7, 10u,
                    TC2_UI_QUALITY_PREDICTED, 3) != 0 ||
            !(train = Tc2UiOverlayFindTrain(&overlay, 14)) ||
            train->position_kind != TC2_UI_POSITION_PREDICTED ||
            train->row != 7 || train->column != 29 ||
            Tc2UiOverlayUpsertAtStart(
                    &overlay, 14, 4, 5, 9u,
                    TC2_UI_QUALITY_PREDICTED, 4) != 0 ||
            Tc2UiOverlayFindTrain(&overlay, 14)->plan_generation !=
                    10u) {
                return fail("duplicate/older plan reset active marker");
        }

        if (Tc2UiOverlayUpsertAtStart(
                    &overlay, 14, 1, 0, 11u,
                    TC2_UI_QUALITY_ESTIMATED, 5) != 1 ||
            !(train = Tc2UiOverlayFindTrain(&overlay, 14)) ||
            train->plan_generation != 11u ||
            train->position_kind != TC2_UI_POSITION_START ||
            train->position_revision != 0 ||
            train->color_slot != stable_color ||
            Tc2UiOverlayUpdatePredictedCell(
                    &overlay, 14, 8, 30, 10u, 2u, 6) >= 0 ||
            Tc2UiOverlayUpdateAtDestination(
                    &overlay, 14, 0, 10u, 2u,
                    TC2_UI_QUALITY_ESTIMATED, 6) >= 0 ||
            Tc2UiOverlayUpdateAtSensor(
                    &overlay, 14, 20, 10u, 2u, 2u,
                    TC2_UI_QUALITY_ESTIMATED, 6) >= 0) {
                return fail("stale plan generation changed new trip");
        }

        if (Tc2UiOverlayUpdateAtSensor(
                    &overlay, 14, 20, 11u, 20u, 20u,
                    TC2_UI_QUALITY_ESTIMATED, 100) != 1 ||
            Tc2UiOverlaySensor(&overlay, 20)->evidence_source !=
                    TC2_UI_SOURCE_OFFLINE_SIMULATION ||
            Tc2UiOverlaySensor(&overlay, 20)->evidence_quality !=
                    TC2_UI_QUALITY_ESTIMATED ||
            Tc2UiOverlaySensor(&overlay, 20)->plan_generation != 11u ||
            Tc2UiOverlayExpireSensorFlashes(&overlay, 130) != 1 ||
            (sensor = Tc2UiOverlaySensor(&overlay, 20))->active ||
            sensor->sequence != 20u ||
            Tc2UiOverlayUpsertAtStart(
                    &overlay, 13, 2, 2, 1u,
                    TC2_UI_QUALITY_PREDICTED, 131) < 0 ||
            Tc2UiOverlayUpdateAtSensor(
                    &overlay, 13, 20, 1u, 1u, 19u,
                    TC2_UI_QUALITY_ESTIMATED, 132) != 0 ||
            Tc2UiOverlaySensor(&overlay, 20)->active ||
            Tc2UiOverlaySensor(&overlay, 20)->sequence != 20u ||
            Tc2UiOverlayFindTrain(&overlay, 13)->position_kind !=
                    TC2_UI_POSITION_START ||
            Tc2UiOverlayUpdateAtSensor(
                    &overlay, 13, 20, 1u, 2u, 21u,
                    TC2_UI_QUALITY_ESTIMATED, 133) != 1 ||
            !Tc2UiOverlaySensor(&overlay, 20)->active ||
            Tc2UiOverlaySensor(&overlay, 20)->train != 13) {
                return fail("expired sensor lost its sequence high-water");
        }

        if (Tc2UiOverlayUpdateAtSensor(
                    &overlay, 14, 21, 11u, 30u, 30u,
                    TC2_UI_QUALITY_ESTIMATED, 140) != 1 ||
            !Tc2UiOverlaySensor(&overlay, 21)->active ||
            Tc2UiOverlayRemoveTrain(&overlay, 14) < 0 ||
            Tc2UiOverlayFindTrain(&overlay, 14) ||
            Tc2UiOverlaySensor(&overlay, 21)->active ||
            Tc2UiOverlaySensor(&overlay, 21)->sequence != 30u ||
            Tc2UiOverlayUpdateAtSensor(
                    &overlay, 14, 22, 11u, 31u, 31u,
                    TC2_UI_QUALITY_ESTIMATED, 141) >= 0) {
                return fail("removed train left a ghost marker/flash");
        }

        if (Tc2UiOverlayUpsertAtStart(
                    &overlay, 14, 0, 6, 11u,
                    TC2_UI_QUALITY_PREDICTED, 142) != 0 ||
            Tc2UiOverlayUpsertAtStart(
                    &overlay, 14, 0, 6, 10u,
                    TC2_UI_QUALITY_PREDICTED, 143) != 0 ||
            Tc2UiOverlayFindTrain(&overlay, 14)) {
                return fail(
                        "equal/older generation recreated removed train");
        }

        if (Tc2UiOverlayUpsertAtStart(
                    &overlay, 14, 0, 6, 12u,
                    TC2_UI_QUALITY_PREDICTED, 150) != 1 ||
            Tc2UiOverlayUpdateAtSensor(
                    &overlay, 14, 22, 11u, 31u, 31u,
                    TC2_UI_QUALITY_ESTIMATED, 151) >= 0 ||
            Tc2UiOverlayUpdateAtSensor(
                    &overlay, 14, 22, 12u, 5u, 40u,
                    TC2_UI_QUALITY_ESTIMATED, 152) != 1 ||
            Tc2UiOverlayUpdateAtDestination(
                    &overlay, 14, 6, 12u, 4u,
                    TC2_UI_QUALITY_ESTIMATED, 153) != 0 ||
            Tc2UiOverlayFindTrain(&overlay, 14)->position_kind !=
                    TC2_UI_POSITION_SENSOR ||
            Tc2UiOverlayUpdateAtDestination(
                    &overlay, 14, 6, 12u, 6u,
                    TC2_UI_QUALITY_ESTIMATED, 154) != 1 ||
            Tc2UiOverlayFindTrain(&overlay, 14)->position_kind !=
                    TC2_UI_POSITION_DESTINATION) {
                return fail("position sequence/generation guard failed");
        }
        return 0;
}

static int removed_generation_wrap_test(void) {
        tc2_ui_overlay overlay;

        Tc2UiOverlayInit(
                &overlay, TC2_UI_SOURCE_OFFLINE_SIMULATION);
        if (Tc2UiOverlayUpsertAtStart(
                    &overlay, -1, 0, 6, 1u,
                    TC2_UI_QUALITY_PREDICTED, 0) != -1 ||
            Tc2UiOverlayUpsertAtStart(
                    &overlay, 256, 0, 6, 1u,
                    TC2_UI_QUALITY_PREDICTED, 0) != -1) {
                return fail(
                        "invalid train id bypassed tombstone bounds");
        }
        if (Tc2UiOverlayUpsertAtStart(
                    &overlay, 14, 0, 6, ~0u,
                    TC2_UI_QUALITY_PREDICTED, 1) != 1 ||
            Tc2UiOverlayRemoveTrain(&overlay, 14) < 0 ||
            Tc2UiOverlayUpsertAtStart(
                    &overlay, 14, 0, 6, ~0u,
                    TC2_UI_QUALITY_PREDICTED, 2) != 0 ||
            Tc2UiOverlayFindTrain(&overlay, 14) ||
            Tc2UiOverlayUpsertAtStart(
                    &overlay, 14, 0, 6, 1u,
                    TC2_UI_QUALITY_PREDICTED, 3) != 1 ||
            !Tc2UiOverlayFindTrain(&overlay, 14) ||
            Tc2UiOverlayFindTrain(
                    &overlay, 14)->plan_generation != 1u) {
                return fail(
                        "removed generation tombstone broke wrap ordering");
        }
        return 0;
}

static int source_transition_clears_flash_test(void) {
        tc2_ui_overlay overlay;

        Tc2UiOverlayInit(
                &overlay, TC2_UI_SOURCE_OFFLINE_SIMULATION);
        if (Tc2UiOverlayUpsertAtStart(
                    &overlay, 14, 0, 6, 1u,
                    TC2_UI_QUALITY_PREDICTED, 0) < 0 ||
            Tc2UiOverlayUpdateAtSensor(
                    &overlay, 14, 10, 1u, 1u, 1u,
                    TC2_UI_QUALITY_ESTIMATED, 10) < 0 ||
            !Tc2UiOverlaySensor(&overlay, 10)->active ||
            Tc2UiOverlaySetSource(
                    &overlay, TC2_UI_SOURCE_LIVE_CAN) < 0 ||
            Tc2UiOverlaySensor(&overlay, 10)->active ||
            Tc2UiOverlaySensor(&overlay, 10)->sequence != 1u) {
                return fail(
                        "offline flash survived a source transition");
        }
        return 0;
}

int main(void) {
        if (source_and_color_test() != 0) return 1;
        if (named_train_color_test() != 0) return 1;
        if (directional_sensor_flash_test() != 0) return 1;
        if (wrapping_tick_test() != 0) return 1;
        if (marker_and_bounds_test() != 0) return 1;
        if (lifecycle_generation_test() != 0) return 1;
        if (removed_generation_wrap_test() != 0) return 1;
        if (source_transition_clears_flash_test() != 0) return 1;
        printf("tc2_ui_overlay_test: PASS "
               "(16 stable colors, 80 directed sensor cells, "
               "300ms flash, generation-safe offline markers)\n");
        return 0;
}
