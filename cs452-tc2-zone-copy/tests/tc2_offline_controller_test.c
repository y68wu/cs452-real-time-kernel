#include "../tc2_offline_controller.h"

#ifdef MODE_TC2

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "../track_data.h"

#define CAPTURE_CAPACITY \
        (TC2_UI_TX_FRAME_CAPACITY + TC2_UI_TX_RESTORE_CAPACITY + 64u)

typedef struct {
        char bytes[CAPTURE_CAPACITY];
        size_t length;
        unsigned int calls;
        unsigned int block_on_call;
        unsigned int fail_on_call;
        size_t maximum_chunk;
} capture_sink;

static tc2_offline_controller controller;
static track_node test_track[TRACK_MAX];
static capture_sink sink;

static int report_failure(
        const char *test, int line, const char *condition) {
        (void)fprintf(stderr, "FAIL %s:%d: %s\n",
                      test, line, condition);
        return 0;
}

#define CHECK(condition) \
        do { \
                if (!(condition)) { \
                        return report_failure( \
                                __func__, __LINE__, #condition); \
                } \
        } while (0)

static void reset_sink(void) {
        (void)memset(&sink, 0, sizeof(sink));
}

static int capture_write(
        void *context, const char *bytes, size_t length) {
        capture_sink *capture = context;
        ++capture->calls;
        if (capture->block_on_call != 0 &&
            capture->calls == capture->block_on_call) {
                return TC2_UI_TX_SINK_WOULD_BLOCK;
        }
        if (capture->fail_on_call != 0 &&
            capture->calls == capture->fail_on_call) {
                return TC2_UI_TX_SINK_FAILED;
        }
        if (!bytes || length == 0 ||
            length > TC2_UI_TX_CHUNK_CAPACITY ||
            capture->length > CAPTURE_CAPACITY ||
            length > CAPTURE_CAPACITY - capture->length) {
                return TC2_UI_TX_SINK_FAILED;
        }
        if (length > capture->maximum_chunk) {
                capture->maximum_chunk = length;
        }
        (void)memcpy(
                capture->bytes + capture->length,
                bytes, length);
        capture->length += length;
        return TC2_UI_TX_SINK_ACCEPTED;
}

static int drain_to_idle(capture_sink *capture) {
        unsigned int guard = 0;
        while (Tc2OfflineControllerHasPendingOutput(
                       &controller)) {
                tc2_ui_tx_drain_result result;
                int status = Tc2OfflineControllerDrain(
                        &controller, capture_write, capture,
                        128u, &result);
                if (status != TC2_OFFLINE_CONTROLLER_OK) {
                        return status;
                }
                if (++guard > 1024u) {
                        return TC2_OFFLINE_CONTROLLER_TX_ERROR;
                }
        }
        return TC2_OFFLINE_CONTROLLER_OK;
}

static int ends_with(
        const char *bytes, size_t length,
        const char *suffix, size_t suffix_length) {
        if (!bytes || !suffix || length < suffix_length) return 0;
        return memcmp(
                       bytes + length - suffix_length,
                       suffix, suffix_length) == 0;
}

static int test_initialization_and_argument_contract(void) {
        tc2_offline_controller_snapshot snapshot;
        tc2_offline_train_snapshot train;

        (void)memset(&controller, 0, sizeof(controller));
        CHECK(Tc2OfflineControllerGetSnapshot(
                      &controller, &snapshot) ==
              TC2_OFFLINE_CONTROLLER_NOT_INITIALIZED);
        CHECK(Tc2OfflineControllerInitialize(
                      0, 10u) ==
              TC2_OFFLINE_CONTROLLER_INVALID_ARGUMENT);
        CHECK(Tc2OfflineControllerInitialize(
                      &controller, 10u) ==
              TC2_OFFLINE_CONTROLLER_OK);
        CHECK(Tc2OfflineControllerGetSnapshot(
                      &controller, &snapshot) ==
              TC2_OFFLINE_CONTROLLER_OK);
        CHECK(snapshot.initialized == 1);
        CHECK(snapshot.failed_closed == 0);
        CHECK(snapshot.predicted_only == 1);
        CHECK(snapshot.is_live == 0);
        CHECK(snapshot.source ==
              TC2_UI_SOURCE_OFFLINE_SIMULATION);
        CHECK(snapshot.tick == 10u);
        CHECK(snapshot.runtime.tick == 10u);
        CHECK(snapshot.runtime.train_count == 0);
        CHECK(snapshot.overlay_train_count == 0);
        CHECK(snapshot.full_redraw_required == 1);
        CHECK(strcmp(
                      Tc2OfflineControllerEvidenceLabel(),
                      "OFFLINE SIMULATION / PREDICTED / "
                      "NO CAN / NO PHYSICAL MOTION") == 0);

        CHECK(Tc2OfflineControllerStageTrip(
                      &controller, 0, 14, 0, 20, 6) ==
              TC2_OFFLINE_CONTROLLER_INVALID_ARGUMENT);
        CHECK(Tc2OfflineControllerStageTrip(
                      &controller, test_track, 0, 0, 20, 6) ==
              TC2_OFFLINE_CONTROLLER_INVALID_ARGUMENT);
        CHECK(Tc2OfflineControllerStageTrip(
                      &controller, test_track, 14, -1, 20, 6) ==
              TC2_OFFLINE_CONTROLLER_INVALID_ARGUMENT);
        CHECK(Tc2OfflineControllerStageTrip(
                      &controller, test_track, 14, 6, 20, 6) ==
              TC2_OFFLINE_CONTROLLER_INVALID_ARGUMENT);
        CHECK(Tc2OfflineControllerStageTrip(
                      &controller, test_track, 14, 0, 0, 6) ==
              TC2_OFFLINE_CONTROLLER_INVALID_ARGUMENT);
        CHECK(Tc2OfflineControllerStageTrip(
                      &controller, test_track, 14, 0, 121, 6) ==
              TC2_OFFLINE_CONTROLLER_INVALID_ARGUMENT);
        CHECK(Tc2OfflineControllerStageTrip(
                      &controller, test_track, 14, 0, 20, -1) ==
              TC2_OFFLINE_CONTROLLER_INVALID_ARGUMENT);
        CHECK(Tc2OfflineControllerStageTrip(
                      &controller, test_track, 14, 0, 20, 8) ==
              TC2_OFFLINE_CONTROLLER_INVALID_ARGUMENT);
        CHECK(Tc2OfflineControllerStartTrip(
                      &controller, 14) ==
              TC2_OFFLINE_CONTROLLER_NOT_FOUND);
        CHECK(Tc2OfflineControllerRemoveTrip(
                      &controller, 14) ==
              TC2_OFFLINE_CONTROLLER_NOT_FOUND);
        CHECK(Tc2OfflineControllerGetTrainSnapshot(
                      &controller, 14, &train) ==
              TC2_OFFLINE_CONTROLLER_NOT_FOUND);
        CHECK(Tc2OfflineControllerStartAll(
                      &controller) ==
              TC2_OFFLINE_CONTROLLER_OK);
        CHECK(Tc2OfflineControllerStep(
                      &controller, 10u) ==
              TC2_OFFLINE_CONTROLLER_OK);
        return 1;
}

static int test_complete_a_f_d1_d8_speed_matrix(void) {
        static const int speeds[] = {1, 20, 80, 100, 120};
        uint32_t previous_generation = 0;
        uint64_t previous_publication = 0;
        uint32_t previous_epoch = 0;

        Tc2OfflineControllerReset(&controller, 100u);
        for (int start = 0;
             start < TC2_TRACK_D_START_COUNT; ++start) {
                for (int destination = 0;
                     destination <
                             TC2_TRACK_D_DESTINATION_COUNT;
                     ++destination) {
                        for (size_t speed_index = 0;
                             speed_index <
                                     sizeof(speeds) /
                                             sizeof(speeds[0]);
                             ++speed_index) {
                                tc2_offline_train_snapshot runtime;
                                const tc2_ui_train_overlay *train;
                                const int speed =
                                        speeds[speed_index];
                                const int stage_status =
                                        Tc2OfflineControllerStageTrip(
                                                &controller,
                                                test_track, 14,
                                                start, speed,
                                                destination);
                                if (stage_status !=
                                            TC2_OFFLINE_CONTROLLER_OK) {
                                        fprintf(stderr,
                                                "matrix stage failed: "
                                                "start=%d destination=%d "
                                                "speed=%d status=%d "
                                                "subsystem=%d\n",
                                                start, destination, speed,
                                                stage_status,
                                                controller.
                                                        last_subsystem_error);
                                        return 0;
                                }
                                CHECK(Tc2OfflineControllerGetTrainSnapshot(
                                              &controller, 14,
                                              &runtime) ==
                                      TC2_OFFLINE_CONTROLLER_OK);
                                CHECK(runtime.state ==
                                      TC2_OFFLINE_TRAIN_READY);
                                CHECK(runtime.train == 14);
                                CHECK(runtime.start_index == start);
                                CHECK(runtime.speed == speed);
                                CHECK(runtime.destination_index ==
                                      destination);
                                CHECK(runtime.plan_generation != 0);
                                CHECK(runtime.publication_serial != 0);
                                CHECK(runtime.launch_epoch != 0);
                                CHECK(runtime.plan_generation >
                                      previous_generation);
                                CHECK(runtime.publication_serial >
                                      previous_publication);
                                CHECK(runtime.launch_epoch >
                                      previous_epoch);
                                previous_generation =
                                        runtime.plan_generation;
                                previous_publication =
                                        runtime.publication_serial;
                                previous_epoch =
                                        runtime.launch_epoch;

                                train = Tc2UiOverlayFindTrain(
                                        Tc2OfflineControllerOverlay(
                                                &controller),
                                        14);
                                CHECK(train != 0);
                                CHECK(train->start_index == start);
                                CHECK(train->destination_index ==
                                      destination);
                                CHECK(train->position_kind ==
                                      TC2_UI_POSITION_START);
                                CHECK(train->position_quality ==
                                      TC2_UI_QUALITY_PREDICTED);
                                CHECK(Tc2OfflineControllerRemoveTrip(
                                              &controller, 14) ==
                                      TC2_OFFLINE_CONTROLLER_OK);
                                CHECK(Tc2UiOverlayFindTrain(
                                              Tc2OfflineControllerOverlay(
                                                      &controller),
                                              14) == 0);
                        }
                }
        }
        return 1;
}

static int test_capacity_batch_and_color_identity(void) {
        int color_seen[TC2_UI_COLOR_COUNT] = {0};
        tc2_offline_controller_snapshot snapshot;
        const tc2_ui_overlay *overlay;

        Tc2OfflineControllerReset(&controller, 0u);
        for (int slot = 0;
             slot < TC2_OFFLINE_CONTROLLER_MAX_TRAINS; ++slot) {
                CHECK(Tc2OfflineControllerStageTrip(
                              &controller, test_track,
                              slot + 1,
                              slot % TC2_TRACK_D_START_COUNT,
                              1 + ((slot * 17) % 120),
                              slot %
                                      TC2_TRACK_D_DESTINATION_COUNT) ==
                      TC2_OFFLINE_CONTROLLER_OK);
        }
        CHECK(Tc2OfflineControllerStageTrip(
                      &controller, test_track, 200,
                      0, 20, 0) ==
              TC2_OFFLINE_CONTROLLER_CAPACITY);
        CHECK(Tc2OfflineControllerGetSnapshot(
                      &controller, &snapshot) ==
              TC2_OFFLINE_CONTROLLER_OK);
        CHECK(snapshot.runtime.train_count ==
              TC2_OFFLINE_CONTROLLER_MAX_TRAINS);
        CHECK(snapshot.overlay_train_count ==
              TC2_OFFLINE_CONTROLLER_MAX_TRAINS);
        overlay = Tc2OfflineControllerOverlay(&controller);
        CHECK(overlay != 0);
        for (int train = 1;
             train <= TC2_OFFLINE_CONTROLLER_MAX_TRAINS;
             ++train) {
                const tc2_ui_train_overlay *marker =
                        Tc2UiOverlayFindTrain(overlay, train);
                CHECK(marker != 0);
                CHECK(marker->color_slot >= 0);
                CHECK(marker->color_slot <
                      TC2_UI_COLOR_COUNT);
                CHECK(color_seen[marker->color_slot] == 0);
                color_seen[marker->color_slot] = 1;
        }
        CHECK(Tc2OfflineControllerStartAll(
                      &controller) ==
              TC2_OFFLINE_CONTROLLER_OK);
        CHECK(Tc2OfflineControllerStep(
                      &controller, 1u) ==
              TC2_OFFLINE_CONTROLLER_OK);
        CHECK(Tc2OfflineControllerGetSnapshot(
                      &controller, &snapshot) ==
              TC2_OFFLINE_CONTROLLER_OK);
        CHECK(snapshot.runtime.running_count +
                      snapshot.runtime.waiting_count >
              0);
        for (int train = 1;
             train <= TC2_OFFLINE_CONTROLLER_MAX_TRAINS;
             ++train) {
                CHECK(Tc2OfflineControllerRemoveTrip(
                              &controller, train) ==
                      TC2_OFFLINE_CONTROLLER_OK);
        }
        return 1;
}

static int test_predicted_replay_and_wrap(void) {
        tc2_offline_train_snapshot runtime;
        tc2_offline_controller_snapshot snapshot;
        int saw_sensor_position = 0;
        int saw_sensor_flash = 0;
        uint32_t tick = UINT32_MAX - 2u;

        Tc2OfflineControllerReset(&controller, tick);
        CHECK(Tc2OfflineControllerStageTrip(
                      &controller, test_track, 14,
                      0, 100, 6) ==
              TC2_OFFLINE_CONTROLLER_OK);
        CHECK(Tc2OfflineControllerStartTrip(
                      &controller, 14) ==
              TC2_OFFLINE_CONTROLLER_OK);
        {
                const tc2_ui_train_overlay *started =
                        Tc2UiOverlayFindTrain(
                                Tc2OfflineControllerOverlay(
                                        &controller),
                                14);
                CHECK(started != 0);
                CHECK(started->speed == 100);
        }
        for (unsigned int step = 0;
             step < 100000u; ++step) {
                const tc2_ui_overlay *overlay;
                const tc2_ui_train_overlay *marker;
                ++tick;
                CHECK(Tc2OfflineControllerStep(
                              &controller, tick) ==
                      TC2_OFFLINE_CONTROLLER_OK);
                overlay =
                        Tc2OfflineControllerOverlay(
                                &controller);
                marker = Tc2UiOverlayFindTrain(
                        overlay, 14);
                CHECK(marker != 0);
                CHECK(marker->position_quality !=
                      TC2_UI_QUALITY_SENSOR_CONFIRMED);
                if (marker->position_kind ==
                    TC2_UI_POSITION_SENSOR) {
                        saw_sensor_position = 1;
                }
                for (int sensor = 0;
                     sensor < TC2_UI_SENSOR_COUNT;
                     ++sensor) {
                        const tc2_ui_sensor_overlay *flash =
                                Tc2UiOverlaySensor(
                                        overlay, sensor);
                        if (!flash || !flash->active) continue;
                        CHECK(flash->evidence_source ==
                              TC2_UI_SOURCE_OFFLINE_SIMULATION);
                        CHECK(flash->evidence_quality ==
                              TC2_UI_QUALITY_PREDICTED);
                        saw_sensor_flash = 1;
                }
                CHECK(Tc2OfflineControllerGetTrainSnapshot(
                              &controller, 14,
                              &runtime) ==
                      TC2_OFFLINE_CONTROLLER_OK);
                if (runtime.state ==
                    TC2_OFFLINE_TRAIN_ARRIVED) {
                        break;
                }
                CHECK(runtime.state ==
                              TC2_OFFLINE_TRAIN_RUNNING ||
                      runtime.state ==
                              TC2_OFFLINE_TRAIN_WAIT_CONFLICT ||
                      runtime.state ==
                              TC2_OFFLINE_TRAIN_READY);
        }
        CHECK(runtime.state ==
              TC2_OFFLINE_TRAIN_ARRIVED);
        CHECK(runtime.progress_um ==
              runtime.destination_distance_um);
        CHECK(saw_sensor_position);
        CHECK(saw_sensor_flash);
        CHECK(Tc2OfflineControllerGetSnapshot(
                      &controller, &snapshot) ==
              TC2_OFFLINE_CONTROLLER_OK);
        CHECK(snapshot.predicted_only == 1);
        CHECK(snapshot.is_live == 0);
        CHECK(snapshot.source ==
              TC2_UI_SOURCE_OFFLINE_SIMULATION);
        CHECK(snapshot.runtime.arrived_count == 1);
        {
                const tc2_ui_train_overlay *marker =
                        Tc2UiOverlayFindTrain(
                                Tc2OfflineControllerOverlay(
                                        &controller),
                                14);
                CHECK(marker != 0);
                CHECK(marker->position_kind ==
                      TC2_UI_POSITION_DESTINATION);
                CHECK(marker->position_quality ==
                      TC2_UI_QUALITY_ESTIMATED);
        }
        return 1;
}

static int test_atomic_render_backpressure_and_failure(void) {
        static const char prompt[] =
                "\033[0m\033[?25h\033[38;1H> "
                "dispatch 14 A 80 d7";
        static const char renderer_evidence[] =
                "TRACK D";
        tc2_offline_controller_render_result rendered;
        tc2_ui_tx_drain_result drained;
        tc2_offline_controller_snapshot snapshot;
        size_t before_block;

        Tc2OfflineControllerReset(&controller, 0u);
        CHECK(Tc2OfflineControllerStageTrip(
                      &controller, test_track, 14,
                      0, 80, 6) ==
              TC2_OFFLINE_CONTROLLER_OK);
        CHECK(Tc2OfflineControllerRender(
                      &controller, prompt,
                      sizeof(prompt) - 1u,
                      &rendered) ==
              TC2_OFFLINE_CONTROLLER_OK);
        CHECK(rendered.queued == 1);
        CHECK(rendered.frame_kind ==
              TC2_UI_TX_FRAME_FULL);
        CHECK(rendered.renderer.full_redraw == 1);
        CHECK(Tc2OfflineControllerHasPendingOutput(
                      &controller));

        reset_sink();
        sink.block_on_call = 1u;
        before_block = sink.length;
        CHECK(Tc2OfflineControllerDrain(
                      &controller, capture_write, &sink,
                      1u, &drained) ==
              TC2_OFFLINE_CONTROLLER_BACKPRESSURE);
        CHECK(sink.length == before_block);
        CHECK(Tc2OfflineControllerHasPendingOutput(
                      &controller));
        sink.block_on_call = 0u;
        CHECK(drain_to_idle(&sink) ==
              TC2_OFFLINE_CONTROLLER_OK);
        CHECK(sink.maximum_chunk <=
              TC2_UI_TX_CHUNK_CAPACITY);
        CHECK(sink.length < CAPTURE_CAPACITY);
        sink.bytes[sink.length] = '\0';
        CHECK(strstr(sink.bytes, renderer_evidence) != 0);
        CHECK(ends_with(
                      sink.bytes, sink.length,
                      prompt, sizeof(prompt) - 1u));
        CHECK(!Tc2OfflineControllerHasPendingOutput(
                      &controller));

        CHECK(Tc2OfflineControllerRender(
                      &controller, prompt,
                      sizeof(prompt) - 1u,
                      &rendered) ==
              TC2_OFFLINE_CONTROLLER_OK);
        reset_sink();
        sink.fail_on_call = 1u;
        CHECK(Tc2OfflineControllerDrain(
                      &controller, capture_write, &sink,
                      1u, &drained) ==
              TC2_OFFLINE_CONTROLLER_TX_ERROR);
        CHECK(Tc2OfflineControllerNeedsFullRedraw(
                      &controller));
        CHECK(Tc2OfflineControllerGetSnapshot(
                      &controller, &snapshot) ==
              TC2_OFFLINE_CONTROLLER_OK);
        CHECK(snapshot.output_failed == 1);
        CHECK(snapshot.output_pending == 0);

        CHECK(Tc2OfflineControllerRender(
                      &controller, prompt,
                      sizeof(prompt) - 1u,
                      &rendered) ==
              TC2_OFFLINE_CONTROLLER_OK);
        CHECK(rendered.frame_kind ==
              TC2_UI_TX_FRAME_FULL);
        reset_sink();
        CHECK(drain_to_idle(&sink) ==
              TC2_OFFLINE_CONTROLLER_OK);
        CHECK(Tc2OfflineControllerGetSnapshot(
                      &controller, &snapshot) ==
              TC2_OFFLINE_CONTROLLER_OK);
        CHECK(snapshot.output_failed == 0);
        CHECK(snapshot.full_redraw_required == 0);
        return 1;
}

static int test_registry_tamper_fails_closed_and_reset_recovers(void) {
        tc2_offline_controller_snapshot snapshot;
        tc2_ui_train_overlay *marker = 0;

        Tc2OfflineControllerReset(&controller, 0u);
        CHECK(Tc2OfflineControllerStageTrip(
                      &controller, test_track, 14,
                      0, 20, 6) ==
              TC2_OFFLINE_CONTROLLER_OK);
        for (int slot = 0;
             slot < TC2_UI_MAX_TRAINS; ++slot) {
                if (controller.overlay.trains[slot].active &&
                    controller.overlay.trains[slot].train == 14) {
                        marker =
                                &controller.overlay.trains[slot];
                        break;
                }
        }
        CHECK(marker != 0);
        ++marker->plan_generation;
        CHECK(Tc2OfflineControllerStep(
                      &controller, 1u) ==
              TC2_OFFLINE_CONTROLLER_FAILED_CLOSED);
        CHECK(Tc2OfflineControllerStageTrip(
                      &controller, test_track, 15,
                      1, 20, 7) ==
              TC2_OFFLINE_CONTROLLER_FAILED_CLOSED);
        CHECK(Tc2OfflineControllerGetSnapshot(
                      &controller, &snapshot) ==
              TC2_OFFLINE_CONTROLLER_OK);
        CHECK(snapshot.failed_closed == 1);
        CHECK(snapshot.last_error ==
              TC2_OFFLINE_CONTROLLER_FAILED_CLOSED);

        Tc2OfflineControllerReset(&controller, 55u);
        CHECK(Tc2OfflineControllerGetSnapshot(
                      &controller, &snapshot) ==
              TC2_OFFLINE_CONTROLLER_OK);
        CHECK(snapshot.failed_closed == 0);
        CHECK(snapshot.tick == 55u);
        CHECK(Tc2OfflineControllerStageTrip(
                      &controller, test_track, 15,
                      1, 20, 7) ==
              TC2_OFFLINE_CONTROLLER_OK);
        return 1;
}

int main(void) {
        int passed = 0;
        const int total = 6;

        init_trackb(test_track);
        passed += test_initialization_and_argument_contract();
        passed += test_complete_a_f_d1_d8_speed_matrix();
        passed += test_capacity_batch_and_color_identity();
        passed += test_predicted_replay_and_wrap();
        passed += test_atomic_render_backpressure_and_failure();
        passed +=
                test_registry_tamper_fails_closed_and_reset_recovers();

        if (passed != total) {
                (void)fprintf(stderr,
                              "%d/%d controller tests passed\n",
                              passed, total);
                return 1;
        }
        (void)printf(
                "tc2 offline controller tests passed (%d/%d): "
                "A-F/d1-d8 matrix, speeds 1..120, batch, "
                "predicted provenance, render pump, fail-closed\n",
                passed, total);
        return 0;
}

#else

int main(void) {
        return 0;
}

#endif
