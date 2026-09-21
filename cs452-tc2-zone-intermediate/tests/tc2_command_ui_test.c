#include <stdio.h>
#include <string.h>

#define MODE_TC2 1
#include "../train_control.c"

static char output[131072];
static int output_length;
static int scheduler_reply_ok = 1;
static int scheduler_healthy = 1;
static int managed_job;
static int managed_job_state = TC2_JOB_RUNNING;
static int stage_calls;
static int calibration_stage_calls;
static int current_stage_calls;
static int current_stage_result;
static int reroute_stage_calls;
static int reroute_stage_result;
static int provisional_snapshot;
static int turnout_plan_snapshot;
static int go_calls;
static int cancel_calls;
static int remove_calls;
static int block_calls;
static int unblock_calls;
static int blocked_snapshot_calls;
static int reserve_calls;
static int release_calls;
static int speed_calls;
static int switch_calls;
static int switch_result;
static int last_train;
static int last_start;
static int last_speed;
static int last_destination;
static int last_node;
static int fake_time_tick = 700;
static int try_getc_calls;
static int try_getc_result = TERMINAL_TRY_GETC_EMPTY;
static int terminal_try_write_calls;
static int terminal_try_write_result = TERMINAL_TRY_WRITE_ACCEPTED;
static int terminal_try_write_last_tid;
static int terminal_try_write_last_length;
static char terminal_try_write_last_bytes[TERMINAL_TRY_WRITE_CAPACITY];
static int offline_initialize_calls;
static int offline_reset_calls;
static int offline_stage_calls;
static int offline_stage_accepts;
static int offline_start_trip_calls;
static int offline_start_all_calls;
static int offline_remove_calls;
static int offline_step_calls;
static int offline_render_calls;
static int offline_drain_calls;
static int offline_snapshot_calls;
static int offline_train_snapshot_calls;
static int offline_pending_calls;
static int offline_full_redraw_calls;
static int offline_evidence_calls;
static int offline_last_train;
static int offline_last_start;
static int offline_last_speed;
static int offline_last_destination;
static uint32_t offline_last_tick;
static int offline_stage_forced_status = TC2_OFFLINE_CONTROLLER_OK;
static int offline_start_all_status = TC2_OFFLINE_CONTROLLER_OK;
static int offline_remove_status = TC2_OFFLINE_CONTROLLER_OK;
static int offline_snapshot_status = TC2_OFFLINE_CONTROLLER_OK;
static int offline_snapshot_total = 7;
static int offline_snapshot_running = 3;
static int offline_snapshot_waiting = 2;
static int offline_snapshot_arrived = 2;

static int equal_fold(const char *left, const char *right) {
        while (*left && *right) {
                char a = *left++;
                char b = *right++;
                if (a >= 'a' && a <= 'z') a = (char)(a - 'a' + 'A');
                if (b >= 'a' && b <= 'z') b = (char)(b - 'a' + 'A');
                if (a != b) return 0;
        }
        return *left == 0 && *right == 0;
}

static void reset_actions(void) {
        output_length = 0;
        output[0] = 0;
        stage_calls = 0;
        calibration_stage_calls = 0;
        current_stage_calls = 0;
        current_stage_result = 0;
        reroute_stage_calls = 0;
        reroute_stage_result = 0;
        go_calls = 0;
        cancel_calls = 0;
        remove_calls = 0;
        block_calls = 0;
        unblock_calls = 0;
        blocked_snapshot_calls = 0;
        reserve_calls = 0;
        release_calls = 0;
        speed_calls = 0;
        switch_calls = 0;
        switch_result = 0;
        last_train = 0;
        last_start = 0;
        last_speed = 0;
        last_destination = 0;
        last_node = -1;
}

static void reset_offline_actions(void) {
        output_length = 0;
        output[0] = 0;
        try_getc_calls = 0;
        try_getc_result = TERMINAL_TRY_GETC_EMPTY;
        terminal_try_write_calls = 0;
        terminal_try_write_result = TERMINAL_TRY_WRITE_ACCEPTED;
        terminal_try_write_last_tid = -1;
        terminal_try_write_last_length = 0;
        memset(terminal_try_write_last_bytes, 0,
               sizeof(terminal_try_write_last_bytes));
        offline_initialize_calls = 0;
        offline_reset_calls = 0;
        offline_stage_calls = 0;
        offline_stage_accepts = 0;
        offline_start_trip_calls = 0;
        offline_start_all_calls = 0;
        offline_remove_calls = 0;
        offline_step_calls = 0;
        offline_render_calls = 0;
        offline_drain_calls = 0;
        offline_snapshot_calls = 0;
        offline_train_snapshot_calls = 0;
        offline_pending_calls = 0;
        offline_full_redraw_calls = 0;
        offline_evidence_calls = 0;
        offline_last_train = 0;
        offline_last_start = TC2_DISPATCH_START_INVALID;
        offline_last_speed = 0;
        offline_last_destination = -1;
        offline_last_tick = 0;
        offline_stage_forced_status = TC2_OFFLINE_CONTROLLER_OK;
        offline_start_all_status = TC2_OFFLINE_CONTROLLER_OK;
        offline_remove_status = TC2_OFFLINE_CONTROLLER_OK;
        offline_snapshot_status = TC2_OFFLINE_CONTROLLER_OK;
        offline_snapshot_total = 7;
        offline_snapshot_running = 3;
        offline_snapshot_waiting = 2;
        offline_snapshot_arrived = 2;
        tc2_ui_live = 0;
        stage_calls = 0;
        current_stage_calls = 0;
        reroute_stage_calls = 0;
        go_calls = 0;
        remove_calls = 0;
        speed_calls = 0;
        switch_calls = 0;
}

static void run_line(const char *text) {
        char line[160];
        size_t length = strlen(text);
        if (length >= sizeof(line)) length = sizeof(line) - 1;
        memcpy(line, text, length);
        line[length] = 0;
        run_command(1, 3, line);
}

static int run_offline_line(const char *text, int emit) {
        char line[160];
        size_t length = strlen(text);
        if (length >= sizeof(line)) length = sizeof(line) - 1;
        memcpy(line, text, length);
        line[length] = 0;
        return tc2_handle_offline_command(line, emit, 1);
}

static int output_contains(const char *needle) {
        return strstr(output, needle) != 0;
}

static int fail(const char *message) {
        fprintf(stderr, "%s\noutput:\n%s\n", message, output);
        return -1;
}

static int test_dispatch_aliases_and_bounds(void) {
        char command[80];
        const int supported_trains[] = {14, 15, 17, 18};
        scheduler_reply_ok = 1;
        scheduler_healthy = 1;
        managed_job = 0;
        reset_actions();

        for (int start = 0; start < 6; ++start) {
                for (int destination = 0; destination < 8; ++destination) {
                        int case_index = start * 8 + destination;
                        int train = supported_trains[
                            case_index % (int)(sizeof(supported_trains) /
                                               sizeof(supported_trains[0]))];
                        snprintf(command, sizeof(command),
                                 "dispatch %d %c %d d%d",
                                 train,
                                 'A' + start, destination + 1,
                                 destination + 1);
                        run_line(command);
                        if (stage_calls != start * 8 + destination + 1 ||
                            last_train != train ||
                            last_start != start ||
                            last_speed != destination + 1 ||
                            last_destination != destination) {
                                return fail("A-F/d1-d8 dispatch mapping failed");
                        }
                }
        }

        run_line("dispatch 18 current 120 d8");
        if (current_stage_calls != 1 || last_train != 18 ||
            reroute_stage_calls != 0 ||
            last_speed != 120 || last_destination != 7) {
                return fail("CURRENT/lowercase dispatch alias failed");
        }

        int accepted = stage_calls + current_stage_calls;
        const char *invalid[] = {
                "dispatch 0 A 1 d1",
                "dispatch 13 A 1 d1",
                "dispatch 255 A 1 d1",
                "dispatch 256 A 1 d1",
                "dispatch 14 G 1 d1",
                "dispatch 14 A 0 d1",
                "dispatch 14 A 121 d1",
                "dispatch 14 A 20 d0",
                "dispatch 14 A 20 d9",
                "dispatch 14 A 20 d1 trailing",
                "dispatch 14 A 20 d1x",
                "dispatch 14A 20 d1",
                "dispatchx 14 A 20 d1",
                "dispatch 14 A 20 D1"
        };
        for (unsigned int i = 0;
             i < sizeof(invalid) / sizeof(invalid[0]); ++i) {
                run_line(invalid[i]);
        }
        if (stage_calls + current_stage_calls != accepted) {
                return fail("invalid/trailing dispatch command was accepted");
        }
        reset_actions();
        run_line("dispatch");
        if (!output_contains(
                    "dispatch <train 14|15|17|18> <A-F|CURRENT> <speed> <d1-d8>")) {
                return fail("dispatch usage omitted CURRENT origin");
        }
        return 0;
}

static int test_provisional_caldispatch(void) {
        scheduler_reply_ok = 1;
        scheduler_healthy = 1;
        managed_job = 0;
        provisional_snapshot = 0;
        reset_actions();

        run_line("caldispatch 14 A 1 d7");
        run_line("caldispatch 14 a 120 d7");
        if (calibration_stage_calls != 2 ||
            last_train != 14 || last_start != 0 ||
            last_speed != 120 || last_destination != 6 ||
            !output_contains("PROVISIONAL")) {
                return fail("valid supervised caldispatch was not staged");
        }

        int accepted = calibration_stage_calls;
        const char *invalid[] = {
                "caldispatch 13 A 20 d7",
                "caldispatch 15 A 20 d7",
                "caldispatch 14 B 20 d7",
                "caldispatch 14 CURRENT 20 d7",
                "caldispatch 14 A 0 d7",
                "caldispatch 14 A 121 d7",
                "caldispatch 14 A 20 d6",
                "caldispatch 14 A 20 d8",
                "caldispatch 14 A 20 d7 trailing",
                "caldispatch 14A 20 d7",
                "caldispatchx 14 A 20 d7",
                "caldispatch 14 A 20 D7"
        };
        for (unsigned int index = 0;
             index < sizeof(invalid) / sizeof(invalid[0]); ++index) {
                run_line(invalid[index]);
        }
        if (calibration_stage_calls != accepted) {
                return fail("out-of-scope caldispatch was accepted");
        }

        reset_actions();
        run_line("caldispatch");
        run_line("help");
        if (!output_contains(
                    "caldispatch 14 A <speed 1..120> d7") ||
            !output_contains("supervised provisional")) {
                return fail("caldispatch usage/help is missing");
        }

        managed_job = 1;
        provisional_snapshot = 1;
        reset_actions();
        run_line("trips");
        const char *diagnostics[] = {
                "prediction=PROVISIONAL_T14_A_D7",
                "velocity=396um/10ms",
                "stop_distance=10243um(rounded_display=11mm)",
                "prediction_anchor=A1",
                "predicted_request_tick=300",
                "actual_request_tick=305",
                "confirmed_tick=307",
                "request_delta_ticks=+5",
                "confirm_delay_ticks=+2",
                "trigger=prediction-timer"
        };
        for (unsigned int index = 0;
             index < sizeof(diagnostics) / sizeof(diagnostics[0]);
             ++index) {
                if (!output_contains(diagnostics[index])) {
                        managed_job = 0;
                        provisional_snapshot = 0;
                        return fail(
                                "provisional prediction diagnostic missing");
                }
        }

        reset_actions();
        print_observation_tick(
                1, 1, (int)0x80000010u);
        tc_puts(1, "|");
        print_optional_tick_delta(
                1, 1,
                (int)0x80000015u,
                (int)0x80000010u);
        tc_puts(1, "|");
        print_optional_tick_delta(
                1, 1,
                (int)0x00000010u,
                (int)0x80000010u);
        if (!output_contains("2147483664|+5|ambiguous")) {
                managed_job = 0;
                provisional_snapshot = 0;
                return fail(
                        "provisional timing display was not wrap-safe");
        }

        managed_job = 0;
        provisional_snapshot = 0;
        return 0;
}

static int test_reroute_command_contract(void) {
        scheduler_reply_ok = 1;
        scheduler_healthy = 1;
        managed_job = 1;
        managed_job_state = TC2_JOB_STOPPED;
        reset_actions();

        run_line("reroute 15 60 d5");
        if (reroute_stage_calls != 1 || current_stage_calls != 0 ||
            last_train != 15 ||
            last_start != TC2_DISPATCH_START_CURRENT ||
            last_speed != 60 || last_destination != 4 ||
            !output_contains(
                    "reroute staged: train=15 from=CURRENT speed=60 destination=d5") ||
            strstr(tc2_ui_status,
                   "reroute staged; enter go to replace the current route") == 0) {
                managed_job_state = TC2_JOB_RUNNING;
                managed_job = 0;
                return fail("valid reroute did not stage an exact CURRENT replacement");
        }

        int accepted = reroute_stage_calls;
        const char *malformed[] = {
                "reroute",
                "reroute 15",
                "reroute 15 60",
                "reroute 15 60 d5 trailing",
                "reroute fifteen 60 d5",
                "reroute 15 sixty d5"
        };
        for (unsigned int index = 0;
             index < sizeof(malformed) / sizeof(malformed[0]); ++index) {
                output_length = 0;
                output[0] = 0;
                run_line(malformed[index]);
                if (reroute_stage_calls != accepted ||
                    !output_contains(
                            "usage: reroute <train 14|15|17|18> <speed 1..120> <d1-d8>")) {
                        managed_job_state = TC2_JOB_RUNNING;
                        managed_job = 0;
                        return fail("malformed reroute did not fail with exact usage");
                }
        }

        const char *out_of_range[] = {
                "reroute 0 60 d5",
                "reroute 256 60 d5",
                "reroute 15 0 d5",
                "reroute 15 121 d5",
                "reroute 15 60 d0",
                "reroute 15 60 d9",
                "reroute 15 60 D5"
        };
        for (unsigned int index = 0;
             index < sizeof(out_of_range) / sizeof(out_of_range[0]);
             ++index) {
                output_length = 0;
                output[0] = 0;
                run_line(out_of_range[index]);
                if (reroute_stage_calls != accepted ||
                    !output_contains(
                            "reroute rejected: train must be under managed control and physically stopped")) {
                        managed_job_state = TC2_JOB_RUNNING;
                        managed_job = 0;
                        return fail("out-of-range reroute reached the dispatcher");
                }
        }

        output_length = 0;
        output[0] = 0;
        managed_job = 0;
        reroute_stage_result = -1;
        run_line("reroute 15 60 d5");
        if (reroute_stage_calls != accepted + 1 ||
            !output_contains(
                    "reroute rejected: train must be under managed control and physically stopped") ||
            strstr(tc2_ui_status,
                   "reroute rejected; train must be managed and physically stopped") == 0) {
                managed_job_state = TC2_JOB_RUNNING;
                return fail("unmanaged reroute rejection was not visible");
        }

        output_length = 0;
        output[0] = 0;
        managed_job = 1;
        managed_job_state = TC2_JOB_RUNNING;
        run_line("reroute 15 60 d5");
        if (reroute_stage_calls != accepted + 2 ||
            !output_contains(
                    "reroute rejected: train must be under managed control and physically stopped")) {
                managed_job = 0;
                return fail("moving managed train reroute rejection was not visible");
        }

        reroute_stage_result = 0;
        output_length = 0;
        output[0] = 0;
        managed_job_state = TC2_JOB_STOPPED;
        run_line("dispatch 17 CURRENT 40 d3");
        if (current_stage_calls != 1 ||
            reroute_stage_calls != accepted + 2 ||
            last_train != 17 ||
            last_start != TC2_DISPATCH_START_CURRENT ||
            last_speed != 40 || last_destination != 2) {
                managed_job = 0;
                return fail(
                        "ordinary CURRENT dispatch did not use the non-reroute staging API");
        }

        managed_job = 0;
        return 0;
}

/*
 * `next_sensor` and `next_turnout` are intentionally independent public
 * cursors.  The operator needs the former to audit detector attribution and
 * the latter two to audit the immutable per-train SW/S/C plan.  Keep the
 * trips text contract explicit so a future UI cleanup cannot accidentally
 * relabel a sensor as a turnout (the original field failure).
 */
static int test_turnout_plan_trips_contract(void) {
        scheduler_reply_ok = 1;
        scheduler_healthy = 1;
        managed_job = 1;
        managed_job_state = TC2_JOB_RUNNING;
        provisional_snapshot = 0;
        turnout_plan_snapshot = 1;
        reset_actions();

        run_line("trips");
        if (!output_contains("next_sensor=C7") ||
            !output_contains(
                    "turnout_plan: steps=10 actions=10 | "
                    "next_turnout=SW2C | next_next_turnout=SW3C")) {
                turnout_plan_snapshot = 0;
                managed_job = 0;
                return fail(
                        "trips did not expose independent sensor and "
                        "turnout S/C cursors");
        }

        /*
         * Successful lifecycle cleanup publishes a canonical empty logical
         * plan.  This is the observable contract used after cancel/remove,
         * before a CURRENT replacement installs its new plan, and after the
         * final arrival cleanup.
         */
        turnout_plan_snapshot = 0;
        reset_actions();
        run_line("trips");
        if (!output_contains(
                    "turnout_plan: steps=0 actions=0 | "
                    "next_turnout=none | next_next_turnout=none")) {
                managed_job = 0;
                return fail("trips did not publish an empty cleared plan");
        }

        managed_job = 0;
        return 0;
}

static int test_boot_diagnostic_lines_are_silent(void) {
        reset_actions();
        run_line("xHC ver: 256 HCS: 05000420 fc000031 00e70004 HCC: 002841eb");
        run_line("XHC ports 5 slots 32 intrs 4");
        run_line("   xHC ports 5 slots 32 intrs 4");
        if (output_length != 0 || stage_calls != 0 ||
            current_stage_calls != 0 || go_calls != 0 ||
            cancel_calls != 0 || remove_calls != 0 ||
            speed_calls != 0 || switch_calls != 0) {
                return fail("startup xHC/XHC diagnostics were treated as commands");
        }

        run_line("xHCgarbage");
        if (!output_contains("unknown command")) {
                return fail("xHC diagnostic filter accepted a non-token prefix");
        }
        return 0;
}

static int test_exact_commands_and_blocks(void) {
        scheduler_reply_ok = 1;
        scheduler_healthy = 1;
        managed_job = 0;
        reset_actions();

        run_line("go");
        run_line("go trailing");
        if (go_calls != 1) return fail("go did not require an exact command");

        run_line("block a5");
        run_line("block A5 trailing");
        run_line("unblock A5");
        run_line("unblock A5 trailing");
        if (block_calls != 1 || unblock_calls != 1 || last_node != 4) {
                return fail("block/unblock parsing or uppercase mapping failed");
        }

        run_line("blocks");
        run_line("blocks trailing");
        if (blocked_snapshot_calls != 1) {
                return fail("blocks did not require an exact command");
        }

        const char *trailing[] = {
                "cancel 14 trailing",
                "remove 14 trailing",
                "reserve 14 A1 A5 trailing",
                "release 14 trailing",
                "tr 14 20 trailing",
                "sw 8 S trailing",
                "route A5 trailing",
                "stop 14 A5 30 trailing"
        };
        for (unsigned int i = 0;
             i < sizeof(trailing) / sizeof(trailing[0]); ++i) {
                run_line(trailing[i]);
        }
        if (cancel_calls || remove_calls || reserve_calls || release_calls ||
            speed_calls || switch_calls) {
                return fail("a TC2 command accepted trailing arguments");
        }

        output_length = 0;
        output[0] = 0;
        run_line("help trailing");
        if (!output_contains("unknown command")) {
                return fail("no-argument command accepted trailing garbage");
        }
        return 0;
}

static int assert_manual_calls_blocked(const char *context) {
        reset_actions();
        run_line("tr 14 20");
        run_line("sw 8 S");
        run_line("route A5");
        run_line("reserve 14 A1 A5");
        run_line("release 14");
        run_line("stop 14 A5 30");
        run_line("demo");
        if (speed_calls || switch_calls || reserve_calls || release_calls) {
                return fail(context);
        }
        if (!output_contains("manual control disabled in TC2")) {
                return fail("disabled TC2 manual command did not explain the safe alternative");
        }
        return 0;
}

static int test_manual_controls_always_disabled(void) {
        scheduler_reply_ok = 0;
        scheduler_healthy = 0;
        managed_job = 0;
        if (assert_manual_calls_blocked(
                    "manual command ran while scheduler unavailable") < 0) {
                return -1;
        }

        scheduler_reply_ok = 1;
        scheduler_healthy = 0;
        if (assert_manual_calls_blocked(
                    "manual command ran while scheduler unhealthy") < 0) {
                return -1;
        }

        scheduler_healthy = 1;
        managed_job = 1;
        if (assert_manual_calls_blocked(
                    "manual command ran while a managed job existed") < 0) {
                return -1;
        }

        managed_job = 0;
        if (assert_manual_calls_blocked(
                    "manual command bypassed an idle healthy TC2 scheduler") < 0) {
                return -1;
        }
        return 0;
}

static int test_track_diagram_and_destination_visibility(void) {
        scheduler_reply_ok = 1;
        scheduler_healthy = 1;
        managed_job = 1;
        reset_actions();
        run_line("trackview");
        const char *required[] = {
                "TC2 DISPATCH TRACK D",
                "C4/C3",
                "C8/C7",
                "E11/E12",
                "D11/D12",
                "C15/C16",
                "A1/A2",
                "d1={A12,A15}",
                "d2={A4,B15}",
                "d3={C11,B6}",
                "d4={C10,B2}",
                "d5={E5,E6}",
                "d6={E13,E14}",
                "d7={E9,D6}",
                "d8={C15,D11}",
                "d7=305/305",
                "d1=CAN-side 24064",
                "all endpoint sides remain TRIAL until repeated",
                "T14 from=A speed=0 destination=d1",
                "anchor_sensor=A12",
                "user_endpoint_offset=180mm(PROVISIONAL;base=0mm speed_correction=0mm)",
                "route_offset=current:0 anchor:0 endpoint-ceiling:1",
                "state=RUNNING"
        };
        for (unsigned int index = 0;
             index < sizeof(required) / sizeof(required[0]);
             ++index) {
                if (!output_contains(required[index])) {
                        managed_job = 0;
                        return fail(
                                "track diagram/current destination field missing");
                }
        }
        managed_job_state = TC2_JOB_STOP_UNCONFIRMED;
        reset_actions();
        run_line("trips");
        if (!output_contains("state=STOP_UNCONFIRMED") ||
            !output_contains(
                    "wait/reason=destination unconfirmed; "
                    "cancel then remove")) {
                managed_job_state = TC2_JOB_RUNNING;
                managed_job = 0;
                return fail(
                        "unconfirmed destination stop was not explicit");
        }
        managed_job_state = TC2_JOB_RUNNING;
        managed_job = 0;
        return 0;
}

static int test_live_dashboard_does_not_capture_commands(void) {
        char command[] = "help";
        char resume_command[] = "uion";
        char cancel_command[] = "cancel 14";
        char remove_command[] = "remove 14";
        char dispatch_command[] = "dispatch 14 A 100 d1";
        char go_command[] = "go";
        scheduler_reply_ok = 1;
        scheduler_healthy = 1;
        managed_job = 1;
        reset_actions();
        tc2_ui_active = 1;
        tc2_ui_live = 1;
        tc2_execute_live_dashboard_command(1, 3, command);
        managed_job = 0;
        if (tc2_ui_active || !tc2_ui_live ||
            !output_contains("\033[H\033[2J\033[3J\033[H") ||
            !output_contains("show this help") ||
            !output_contains("> ")) {
                return fail(
                        "live dashboard erased verbose command output");
        }
        tc2_offline_controller_ready = 1;
        run_command(1, 3, resume_command);
        if (!tc2_ui_active || !tc2_ui_live) {
                return fail("uion did not restore the live dashboard");
        }
        tc2_execute_live_dashboard_command(
                1, 3, cancel_command);
        if (cancel_calls != 1 ||
            strstr(tc2_ui_status, "cancel accepted") == 0) {
                return fail(
                        "live dashboard lost cancel command or status");
        }
        tc2_execute_live_dashboard_command(
                1, 3, remove_command);
        if (remove_calls != 1 ||
            strstr(tc2_ui_status, "remove accepted") == 0) {
                return fail(
                        "live dashboard lost remove command or status");
        }
        tc2_execute_live_dashboard_command(
                1, 3, dispatch_command);
        if (stage_calls != 1 ||
            last_train != 14 || last_start != 0 ||
            last_speed != 100 || last_destination != 0 ||
            strstr(tc2_ui_status, "dispatch staged") == 0) {
                return fail(
                        "live dashboard did not accept redispatch after remove");
        }
        tc2_execute_live_dashboard_command(
                1, 3, go_command);
        if (go_calls != 1 || !tc2_ui_active || !tc2_ui_live ||
            tc2_ui_status[0] != 0) {
                return fail(
                        "live dashboard did not start redispatched train");
        }
        return 0;
}

static int offline_live_or_can_was_called(void) {
        return stage_calls != 0 ||
               current_stage_calls != 0 ||
               go_calls != 0 ||
               remove_calls != 0 ||
               speed_calls != 0 ||
               switch_calls != 0;
}

static int test_offline_dispatch_aliases_bounds_and_isolation(void) {
        char command[80];
        tc2_offline_controller_ready = 1;
        reset_offline_actions();

        /*
         * Exercise the public command loop, not only the parser helper.
         * Every A-F x d1-d8 combination must preserve the exact indexes.
         */
        for (int start = 0; start < 6; ++start) {
                for (int destination = 0; destination < 8; ++destination) {
                        int expected = start * 8 + destination + 1;
                        snprintf(command, sizeof(command),
                                 "simdispatch %d %c %d d%d",
                                 expected, 'A' + start,
                                 destination + 1, destination + 1);
                        run_line(command);
                        if (offline_stage_accepts != expected ||
                            offline_last_train != expected ||
                            offline_last_start != start ||
                            offline_last_speed != destination + 1 ||
                            offline_last_destination != destination) {
                                return fail(
                                        "offline A-F/d1-d8 mapping failed");
                        }
                }
        }
        run_line("simdispatch 255 f 120 d8");
        if (offline_stage_accepts != 49 ||
            offline_last_train != 255 ||
            offline_last_start != 5 ||
            offline_last_speed != 120 ||
            offline_last_destination != 7) {
                return fail("offline lowercase/speed-bound mapping failed");
        }
        if (offline_live_or_can_was_called()) {
                return fail(
                        "offline simdispatch reached live dispatcher/CAN");
        }
        if (!output_contains(
                    TC2_OFFLINE_CONTROLLER_EVIDENCE_LABEL) ||
            !output_contains("staged predicted trip T255 F -> d8 speed=120")) {
                return fail(
                        "offline simdispatch omitted evidence/status text");
        }

        int accepted = offline_stage_accepts;
        const char *invalid[] = {
                "simdispatch 0 A 1 d1",
                "simdispatch 256 A 1 d1",
                "simdispatch 14 CURRENT 1 d1",
                "simdispatch 14 G 1 d1",
                "simdispatch 14 A 0 d1",
                "simdispatch 14 A 121 d1",
                "simdispatch 14 A 20 d0",
                "simdispatch 14 A 20 d9",
                "simdispatch 14 A 20 d1 trailing",
                "simdispatch 14 A 20 d1x",
                "simdispatch 14A 20 d1",
                "simdispatch 14 A 20 D1"
        };
        for (unsigned int index = 0;
             index < sizeof(invalid) / sizeof(invalid[0]); ++index) {
                run_line(invalid[index]);
        }
        if (offline_stage_accepts != accepted ||
            offline_live_or_can_was_called()) {
                return fail(
                        "invalid offline dispatch was accepted or went live");
        }

        if (run_offline_line("simdispatchx 14 A 20 d1", 1) != 0) {
                return fail("offline namespace accepted simdispatch prefix");
        }

        output_length = 0;
        output[0] = 0;
        offline_stage_forced_status =
                TC2_OFFLINE_CONTROLLER_CAPACITY;
        run_line("simdispatch 200 A 20 d1");
        if (!output_contains("offline capacity reached (max 16)") ||
            !output_contains(
                    TC2_OFFLINE_CONTROLLER_EVIDENCE_LABEL)) {
                return fail("offline capacity rejection lacked evidence");
        }

        int attempts = offline_stage_calls;
        tc2_offline_controller_ready = 0;
        output_length = 0;
        output[0] = 0;
        run_line("simdispatch 201 A 20 d1");
        if (offline_stage_calls != attempts ||
            !output_contains("offline controller is not ready") ||
            !output_contains(
                    TC2_OFFLINE_CONTROLLER_EVIDENCE_LABEL)) {
                return fail("offline not-ready gate was not fail-closed");
        }

        tc2_offline_controller_ready = 1;
        offline_stage_forced_status = TC2_OFFLINE_CONTROLLER_OK;
        output_length = 0;
        output[0] = 0;
        if (run_offline_line("simdispatch 202 B 1 d2", 0) != 1 ||
            offline_stage_accepts != accepted + 1 ||
            output_length != 0) {
                return fail("silent offline handler contract failed");
        }
        if (offline_live_or_can_was_called()) {
                return fail("silent offline handler reached live/CAN path");
        }
        return 0;
}

static int test_offline_exact_lifecycle_commands(void) {
        tc2_offline_controller_ready = 1;
        reset_offline_actions();

        run_line("simgo");
        run_line("simgo trailing");
        if (offline_start_all_calls != 1 ||
            !tc2_ui_active ||
            tc2_ui_shutdown_requested ||
            tc2_ui_shutdown_frame_queued ||
            tc2_ui_next_render_tick != 0 ||
            !output_contains("all staged predicted trips started together")) {
                return fail("simgo exact command/UI activation failed");
        }

        run_line("simtrips");
        run_line("simtrips trailing");
        if (offline_snapshot_calls != 1 ||
            !output_contains(
                    "predicted trips total=7 running=3 waiting=2 arrived=2")) {
                return fail("simtrips exact snapshot output failed");
        }

        run_line("simremove 14");
        run_line("simremove 14 trailing");
        run_line("simremove 0");
        if (offline_remove_calls != 2 ||
            offline_last_train != 0 ||
            !output_contains("predicted trip removed")) {
                return fail("simremove parsing/status failed");
        }
        offline_remove_status = TC2_OFFLINE_CONTROLLER_NOT_FOUND;
        run_line("simremove 15");
        if (offline_remove_calls != 3 ||
            !output_contains("simremove rejected: train not found")) {
                return fail("simremove not-found result was hidden");
        }

        fake_time_tick = 123456;
        run_line("simreset");
        run_line("simreset trailing");
        if (offline_reset_calls != 1 ||
            offline_last_tick != 123456u ||
            !output_contains(
                    "offline simulation reset; no physical state changed")) {
                return fail("simreset exact command/tick failed");
        }

        tc2_ui_active = 0;
        tc2_ui_shutdown_requested = 1;
        tc2_ui_shutdown_frame_queued = 1;
        tc2_ui_next_render_tick = 99;
        run_line("uion");
        run_line("uion trailing");
        if (!tc2_ui_active ||
            tc2_ui_shutdown_requested ||
            tc2_ui_shutdown_frame_queued ||
            tc2_ui_next_render_tick != 0 ||
            !output_contains(
                    "dynamic predicted Track D dashboard enabled")) {
                return fail("uion exact command/state transition failed");
        }

        tc2_ui_shutdown_requested = 0;
        run_line("uioff trailing");
        if (tc2_ui_shutdown_requested) {
                return fail("uioff accepted trailing arguments");
        }
        run_line("uioff");
        if (!tc2_ui_shutdown_requested ||
            tc2_ui_shutdown_frame_queued ||
            !output_contains(
                    "dynamic dashboard stopping after final complete frame")) {
                return fail("uioff exact shutdown request failed");
        }

        if (run_offline_line("simunknown", 1) != 0) {
                return fail("unknown sim command claimed offline namespace");
        }
        if (offline_live_or_can_was_called()) {
                return fail("offline lifecycle command reached live/CAN path");
        }
        if (!output_contains(
                    TC2_OFFLINE_CONTROLLER_EVIDENCE_LABEL) ||
            offline_evidence_calls == 0) {
                return fail("offline lifecycle output omitted evidence label");
        }
        return 0;
}

static int test_nonblocking_terminal_stub_contract(void) {
        int terminal_tid = 3;
        size_t restore_length;
        reset_offline_actions();
        try_getc_result = 'Q';
        if (TryGetc(3) != 'Q' || try_getc_calls != 1 ||
            TryGetc(0) != TERMINAL_TRY_GETC_ERROR ||
            try_getc_calls != 1) {
                return fail("TryGetc strict stub contract failed");
        }

        terminal_try_write_result =
                TERMINAL_TRY_WRITE_WOULD_BLOCK;
        if (TerminalTryWrite(3, "abc", 3) !=
                    TERMINAL_TRY_WRITE_WOULD_BLOCK ||
            terminal_try_write_calls != 1 ||
            terminal_try_write_last_tid != 3 ||
            terminal_try_write_last_length != 3 ||
            memcmp(terminal_try_write_last_bytes, "abc", 3) != 0) {
                return fail("TerminalTryWrite mapping/atomic attempt failed");
        }
        if (TerminalTryWrite(0, "x", 1) !=
                    TERMINAL_TRY_WRITE_ERROR ||
            TerminalTryWrite(3, 0, 1) !=
                    TERMINAL_TRY_WRITE_ERROR ||
            TerminalTryWrite(3, "", 0) !=
                    TERMINAL_TRY_WRITE_ERROR ||
            TerminalTryWrite(3, terminal_try_write_last_bytes,
                             TERMINAL_TRY_WRITE_CAPACITY + 1) !=
                    TERMINAL_TRY_WRITE_ERROR ||
            terminal_try_write_calls != 1) {
                return fail("TerminalTryWrite accepted invalid arguments");
        }

        terminal_try_write_result = TERMINAL_TRY_WRITE_ACCEPTED;
        if (tc2_terminal_sink(&terminal_tid, "ok", 2) !=
                    TC2_UI_TX_SINK_ACCEPTED ||
            terminal_try_write_calls != 2) {
                return fail("terminal sink did not map ACCEPTED");
        }
        terminal_try_write_result =
                TERMINAL_TRY_WRITE_WOULD_BLOCK;
        if (tc2_terminal_sink(&terminal_tid, "wait", 4) !=
                    TC2_UI_TX_SINK_WOULD_BLOCK) {
                return fail("terminal sink did not map WOULD_BLOCK");
        }
        terminal_try_write_result = TERMINAL_TRY_WRITE_ERROR;
        if (tc2_terminal_sink(&terminal_tid, "fail", 4) !=
                    TC2_UI_TX_SINK_FAILED ||
            tc2_terminal_sink(0, "x", 1) !=
                    TC2_UI_TX_SINK_FAILED ||
            tc2_terminal_sink(&terminal_tid, 0, 1) !=
                    TC2_UI_TX_SINK_FAILED ||
            tc2_terminal_sink(&terminal_tid, "", 0) !=
                    TC2_UI_TX_SINK_FAILED) {
                return fail("terminal sink failure validation failed");
        }

        if (!tc2_tick_due(10u, 10u) ||
            tc2_tick_due(10u, 11u) ||
            !tc2_tick_due(1u, UINT32_MAX) ||
            tc2_tick_due(UINT32_MAX, 1u)) {
                return fail("dashboard tick comparison was not wrap-safe");
        }

        tc2_status_set("predicted UI healthy");
        restore_length = tc2_build_prompt_restore("simgo", 5);
        if (restore_length == 0 ||
            strstr(tc2_ui_restore,
                   "\033[0m\033[37;1H\033[2Kpredicted UI healthy"
                   "\033[38;1H\033[2K> simgo\033[?25h") == 0) {
                return fail(
                        "prompt restore omitted status or input state");
        }
        tc2_ui_live = 1;
        tc2_live_ui_state.renderer.initialized = 1;
        restore_length = tc2_build_prompt_restore("simgo", 5);
        if (restore_length == 0 ||
            strstr(tc2_ui_restore, "predicted UI healthy") != 0 ||
            strstr(tc2_ui_restore,
                   "\033[38;1H\033[2K> simgo\033[?25h") == 0) {
                return fail(
                        "unchanged live status was repainted and could flicker");
        }
        tc2_ui_live = 0;
        Tc2LiveUiInitialize(&tc2_live_ui_state, 0);
        tc2_live_ui_state.trains[0].active = 1;
        tc2_live_ui_state.trains[0].train = 14;
        tc2_live_ui_state.trains[0].current_speed = 80;
        if (Tc2UiOverlayUpsertAtStart(
                    &tc2_live_ui_state.overlay, 14, 0, 0, 1U,
                    TC2_UI_QUALITY_PREDICTED, 0U) < 0) {
                return fail("could not prepare compact T14 status");
        }
        tc2_ui_live = 1;
        tc2_status_set("physical dispatch active");
        restore_length = tc2_build_prompt_restore("", 0);
        tc2_ui_live = 0;
        if (restore_length == 0 ||
            strstr(tc2_ui_restore,
                   "\033[0m\033[37;1H\033[2Kphysical dispatch active"
                   "\033[38;1H\033[2K> \033[?25h") == 0 ||
            strstr(tc2_ui_restore, "T14 RED") != 0 ||
            strstr(tc2_ui_restore, "\033[45;") != 0 ||
            strstr(tc2_ui_restore, "\033[46;") != 0) {
                return fail(
                        "compact live prompt could scroll the terminal");
        }
        return 0;
}

int Putc(int tid, unsigned char ch) {
        (void)tid;
        if (output_length + 1 < (int)sizeof(output)) {
                output[output_length++] = (char)ch;
                output[output_length] = 0;
        }
        return 0;
}

int Getc(int tid) {
        (void)tid;
        return -1;
}

int Time(void) {
        return fake_time_tick;
}

int TryGetc(int tid) {
        if (tid < 1) return TERMINAL_TRY_GETC_ERROR;
        try_getc_calls++;
        return try_getc_result;
}

int TerminalTryWrite(int tid, const char *bytes, int length) {
        if (tid < 1 || !bytes ||
            length < 1 ||
            length > TERMINAL_TRY_WRITE_CAPACITY) {
                return TERMINAL_TRY_WRITE_ERROR;
        }
        terminal_try_write_calls++;
        terminal_try_write_last_tid = tid;
        terminal_try_write_last_length = length;
        memcpy(terminal_try_write_last_bytes, bytes, (size_t)length);
        return terminal_try_write_result;
}

void Yield(void) {}
void Exit(void) {}
int WhoIs(const char *name) {
        (void)name;
        return 1;
}

int Tc2OfflineControllerInitialize(
        tc2_offline_controller *controller, uint32_t initial_tick) {
        if (controller != &tc2_offline_controller_state) {
                return TC2_OFFLINE_CONTROLLER_INVALID_ARGUMENT;
        }
        offline_initialize_calls++;
        offline_last_tick = initial_tick;
        controller->initialized = 1;
        controller->tick = initial_tick;
        return TC2_OFFLINE_CONTROLLER_OK;
}

void Tc2OfflineControllerReset(
        tc2_offline_controller *controller, uint32_t initial_tick) {
        if (controller != &tc2_offline_controller_state) return;
        offline_reset_calls++;
        offline_last_tick = initial_tick;
        controller->initialized = 1;
        controller->tick = initial_tick;
}

int Tc2OfflineControllerStageTrip(
        tc2_offline_controller *controller,
        track_node track[TRACK_MAX],
        int train, int start_index, int speed,
        int destination_index) {
        offline_stage_calls++;
        offline_last_train = train;
        offline_last_start = start_index;
        offline_last_speed = speed;
        offline_last_destination = destination_index;
        if (controller != &tc2_offline_controller_state ||
            track != tc2_track ||
            train < 1 || train > 255 ||
            start_index < 0 || start_index >= TC2_DISPATCH_START_COUNT ||
            speed < 1 || speed > 120 ||
            destination_index < 0 ||
            destination_index >= TC2_DISPATCH_DESTINATION_COUNT) {
                return TC2_OFFLINE_CONTROLLER_INVALID_ARGUMENT;
        }
        if (offline_stage_forced_status !=
            TC2_OFFLINE_CONTROLLER_OK) {
                return offline_stage_forced_status;
        }
        offline_stage_accepts++;
        return TC2_OFFLINE_CONTROLLER_OK;
}

int Tc2OfflineControllerStartTrip(
        tc2_offline_controller *controller, int train) {
        if (controller != &tc2_offline_controller_state ||
            train < 1 || train > 255) {
                return TC2_OFFLINE_CONTROLLER_INVALID_ARGUMENT;
        }
        offline_start_trip_calls++;
        offline_last_train = train;
        return TC2_OFFLINE_CONTROLLER_OK;
}

int Tc2OfflineControllerStartAll(
        tc2_offline_controller *controller) {
        if (controller != &tc2_offline_controller_state) {
                return TC2_OFFLINE_CONTROLLER_INVALID_ARGUMENT;
        }
        offline_start_all_calls++;
        return offline_start_all_status;
}

int Tc2OfflineControllerRemoveTrip(
        tc2_offline_controller *controller, int train) {
        if (controller != &tc2_offline_controller_state) {
                return TC2_OFFLINE_CONTROLLER_INVALID_ARGUMENT;
        }
        offline_remove_calls++;
        offline_last_train = train;
        if (train < 1 || train > 255) {
                return TC2_OFFLINE_CONTROLLER_INVALID_ARGUMENT;
        }
        return offline_remove_status;
}

int Tc2OfflineControllerStep(
        tc2_offline_controller *controller, uint32_t now_tick) {
        if (controller != &tc2_offline_controller_state) {
                return TC2_OFFLINE_CONTROLLER_INVALID_ARGUMENT;
        }
        offline_step_calls++;
        offline_last_tick = now_tick;
        return TC2_OFFLINE_CONTROLLER_OK;
}

int Tc2OfflineControllerRender(
        tc2_offline_controller *controller,
        const char *prompt_restore, size_t prompt_restore_length,
        tc2_offline_controller_render_result *result) {
        if (controller != &tc2_offline_controller_state ||
            (!prompt_restore && prompt_restore_length != 0) ||
            !result) {
                return TC2_OFFLINE_CONTROLLER_INVALID_ARGUMENT;
        }
        offline_render_calls++;
        memset(result, 0, sizeof(*result));
        return TC2_OFFLINE_CONTROLLER_OK;
}

int Tc2OfflineControllerDrain(
        tc2_offline_controller *controller,
        tc2_ui_tx_write_fn write, void *write_context,
        unsigned int max_chunks,
        tc2_ui_tx_drain_result *result) {
        if (controller != &tc2_offline_controller_state ||
            !write || !write_context || max_chunks == 0 || !result) {
                return TC2_OFFLINE_CONTROLLER_INVALID_ARGUMENT;
        }
        offline_drain_calls++;
        memset(result, 0, sizeof(*result));
        result->idle_after = 1;
        return TC2_OFFLINE_CONTROLLER_NO_CHANGE;
}

int Tc2OfflineControllerGetTrainSnapshot(
        const tc2_offline_controller *controller, int train,
        tc2_offline_train_snapshot *snapshot) {
        if (controller != &tc2_offline_controller_state ||
            train < 1 || train > 255 || !snapshot) {
                return TC2_OFFLINE_CONTROLLER_INVALID_ARGUMENT;
        }
        offline_train_snapshot_calls++;
        memset(snapshot, 0, sizeof(*snapshot));
        snapshot->train = train;
        return TC2_OFFLINE_CONTROLLER_OK;
}

int Tc2OfflineControllerGetSnapshot(
        const tc2_offline_controller *controller,
        tc2_offline_controller_snapshot *snapshot) {
        if (controller != &tc2_offline_controller_state || !snapshot) {
                return TC2_OFFLINE_CONTROLLER_INVALID_ARGUMENT;
        }
        offline_snapshot_calls++;
        if (offline_snapshot_status !=
            TC2_OFFLINE_CONTROLLER_OK) {
                return offline_snapshot_status;
        }
        memset(snapshot, 0, sizeof(*snapshot));
        snapshot->runtime.train_count = offline_snapshot_total;
        snapshot->runtime.running_count = offline_snapshot_running;
        snapshot->runtime.waiting_count = offline_snapshot_waiting;
        snapshot->runtime.arrived_count = offline_snapshot_arrived;
        return TC2_OFFLINE_CONTROLLER_OK;
}

const tc2_ui_overlay *Tc2OfflineControllerOverlay(
        const tc2_offline_controller *controller) {
        if (controller != &tc2_offline_controller_state) return 0;
        return &controller->overlay;
}

int Tc2OfflineControllerHasPendingOutput(
        const tc2_offline_controller *controller) {
        if (controller != &tc2_offline_controller_state) return 0;
        offline_pending_calls++;
        return 0;
}

int Tc2OfflineControllerNeedsFullRedraw(
        const tc2_offline_controller *controller) {
        if (controller != &tc2_offline_controller_state) return 0;
        offline_full_redraw_calls++;
        return 0;
}

const char *Tc2OfflineControllerEvidenceLabel(void) {
        offline_evidence_calls++;
        return TC2_OFFLINE_CONTROLLER_EVIDENCE_LABEL;
}

void init_trackb(track_node *track) {
        memset(track, 0, sizeof(track_node) * TRACK_MAX);
        track[0].name = "A1";
        track[4].name = "A5";
        track[38].name = "C7";
        track[79].name = "E16";
}

int TrackFindNodeByName(track_node *track, const char *name) {
        (void)track;
        if (equal_fold(name, "A1")) return 0;
        if (equal_fold(name, "A5")) return 4;
        if (equal_fold(name, "C7")) return 38;
        if (equal_fold(name, "E16")) return 79;
        return -1;
}

int TrackFindShortestRouteWithReversals(
        track_node *track, int start, int destination, int penalty,
        track_route *route) {
        (void)track;
        (void)start;
        (void)destination;
        (void)penalty;
        (void)route;
        return -1;
}

int TrackBuildTurnoutPlan(
        track_node *track, const track_route *route, track_turnout_plan *plan) {
        (void)track;
        (void)route;
        (void)plan;
        return -1;
}

int TrackReservationServerReserve(
        int tid, const track_route *route, int train, int destination,
        track_reservation_conflict *conflict) {
        (void)tid;
        (void)route;
        (void)train;
        (void)destination;
        (void)conflict;
        reserve_calls++;
        return 0;
}

int TrackReservationServerRelease(int tid, int train) {
        (void)tid;
        (void)train;
        release_calls++;
        return 0;
}

int TrackReservationServerSnapshot(
        int tid, track_reservation_snapshot *snapshot) {
        (void)tid;
        memset(snapshot, 0, sizeof(*snapshot));
        for (int train = 0; train < 256; ++train) {
                snapshot->destination_by_train[train] = -1;
        }
        return 0;
}

int TrainSensorGetLatest(int tid, train_sensor_snapshot_t *snapshot) {
        (void)tid;
        memset(snapshot, 0, sizeof(*snapshot));
        for (int train = 0; train < 256; ++train) {
                snapshot->position_by_train[train] = -1;
        }
        return 0;
}

int TrainSensorLabel(int sensor, char *bank, int *number) {
        if (sensor < 0 || sensor >= 80 || !bank || !number) return -1;
        *bank = (char)('A' + sensor / 16);
        *number = sensor % 16 + 1;
        return 0;
}

int CanGetHealth(int tid, can_health_t *health) {
        (void)tid;
        memset(health, 0, sizeof(*health));
        health->hw_ready = 1;
        return 0;
}

int CanTrainSetSpeed(int tid, int train, int speed) {
        (void)tid;
        speed_calls++;
        last_train = train;
        last_speed = speed;
        return 0;
}

int CanSwitch(int tid, int switch_number, char direction) {
        (void)tid;
        (void)switch_number;
        (void)direction;
        switch_calls++;
        return switch_result;
}

int Tc2DispatchParseStart(const char *label) {
        if (equal_fold(label, "CURRENT")) return TC2_DISPATCH_START_CURRENT;
        if (label && label[0] && !label[1]) {
                char value = label[0];
                if (value >= 'a' && value <= 'f') {
                        value = (char)(value - 'a' + 'A');
                }
                if (value >= 'A' && value <= 'F') return value - 'A';
        }
        return TC2_DISPATCH_START_INVALID;
}

int Tc2DispatchParseDestination(const char *label) {
        if (!label || label[0] != 'd' ||
            label[1] < '1' || label[1] > '8' || label[2]) {
                return -1;
        }
        return label[1] - '1';
}

const char *Tc2DispatchStartLabel(int start) {
        static const char *const labels[] = {"A", "B", "C", "D", "E", "F"};
        return start >= 0 && start < 6 ? labels[start] : "CURRENT";
}

const char *Tc2DispatchStartNodeName(int start) {
        (void)start;
        return "EN5";
}

const char *Tc2DispatchDestinationLabel(int destination) {
        static const char *const labels[] = {
                "d1", "d2", "d3", "d4", "d5", "d6", "d7", "d8"
        };
        return destination >= 0 && destination < 8 ?
                labels[destination] : "?";
}

const char *Tc2DispatchDestinationSensorA(int destination) {
        (void)destination;
        return "A12";
}

const char *Tc2DispatchDestinationSensorB(int destination) {
        (void)destination;
        return "A11";
}

const char *Tc2DispatchStateName(int state) {
        if (state == TC2_JOB_RUNNING) return "RUNNING";
        if (state == TC2_JOB_STOP_UNCONFIRMED) {
                return "STOP_UNCONFIRMED";
        }
        return "STATE";
}

int Tc2DispatchStage(
        int tid, int train, int start, int speed, int destination) {
        (void)tid;
        stage_calls++;
        last_train = train;
        last_start = start;
        last_speed = speed;
        last_destination = destination;
        return 0;
}

int Tc2DispatchStageCalibration(
        int tid, int train, int start, int speed, int destination) {
        (void)tid;
        calibration_stage_calls++;
        last_train = train;
        last_start = start;
        last_speed = speed;
        last_destination = destination;
        return 0;
}

int Tc2DispatchStageFromCurrent(
        int tid, int train, int speed, int destination) {
        (void)tid;
        current_stage_calls++;
        last_train = train;
        last_start = TC2_DISPATCH_START_CURRENT;
        last_speed = speed;
        last_destination = destination;
        return current_stage_result;
}

int Tc2DispatchStageReroute(
        int tid, int train, int speed, int destination) {
        (void)tid;
        reroute_stage_calls++;
        last_train = train;
        last_start = TC2_DISPATCH_START_CURRENT;
        last_speed = speed;
        last_destination = destination;
        return reroute_stage_result;
}

int Tc2DispatchStartAll(int tid) {
        (void)tid;
        go_calls++;
        return 0;
}

int Tc2DispatchCancel(int tid, int train) {
        (void)tid;
        (void)train;
        cancel_calls++;
        return 0;
}

int Tc2DispatchRemove(int tid, int train) {
        (void)tid;
        (void)train;
        remove_calls++;
        return 0;
}

int Tc2DispatchBlockNode(int tid, int node) {
        (void)tid;
        block_calls++;
        last_node = node;
        return 0;
}

int Tc2DispatchUnblockNode(int tid, int node) {
        (void)tid;
        unblock_calls++;
        last_node = node;
        return 0;
}

int Tc2DispatchGetBlockedSnapshot(
        int tid, tc2_dispatch_blocked_snapshot *snapshot) {
        (void)tid;
        blocked_snapshot_calls++;
        memset(snapshot, 0, sizeof(*snapshot));
        return 0;
}

int Tc2DispatchIsTrainManaged(int tid, int train) {
        (void)tid;
        (void)train;
        return managed_job;
}

int Tc2DispatchGetSnapshot(int tid, tc2_dispatch_snapshot *snapshot) {
        (void)tid;
        if (!scheduler_reply_ok) return -1;
        memset(snapshot, 0, sizeof(*snapshot));
        snapshot->scheduler_healthy = scheduler_healthy;
        snapshot->batch_ready_at_tick = -1;
        if (managed_job) {
                snapshot->jobs[0].train = 14;
                snapshot->jobs[0].state = managed_job_state;
                snapshot->jobs[0].current_node = -1;
                snapshot->jobs[0].next_sensor_node = -1;
                snapshot->jobs[0].selected_destination_side = 0;
                snapshot->jobs[0].destination_offset_mm = 180;
                snapshot->jobs[0].destination_route_offset = 1;
                snapshot->jobs[0].destination_offset_confirmed = 0;
                if (turnout_plan_snapshot) {
                        snapshot->jobs[0].next_sensor_node =
                                TrackFindNodeByName(tc2_track, "C7");
                        snapshot->jobs[0].turnout_plan_step_count = 10;
                        snapshot->jobs[0].turnout_plan_action_count = 10;
                        snapshot->jobs[0].next_turnout_action_count = 1;
                        snapshot->jobs[0].next_turnout_switch[0] = 2;
                        snapshot->jobs[0].next_turnout_direction[0] = 'C';
                        snapshot->jobs[0]
                                .next_next_turnout_action_count = 1;
                        snapshot->jobs[0].next_next_turnout_switch[0] = 3;
                        snapshot->jobs[0]
                                .next_next_turnout_direction[0] = 'C';
                }
                if (provisional_snapshot) {
                        snapshot->jobs[0].provisional_prediction = 1;
                        snapshot->jobs[0]
                                .prediction_velocity_um_per_tick = 396;
                        snapshot->jobs[0]
                                .prediction_stop_distance_um = 10243;
                        snapshot->jobs[0]
                                .prediction_stop_distance_mm = 11;
                        snapshot->jobs[0].prediction_anchor_node = 0;
                        snapshot->jobs[0].prediction_anchor_tick = 100;
                        snapshot->jobs[0].prediction_timing_valid = 1;
                        snapshot->jobs[0]
                                .prediction_anchor_distance_mm = 469;
                        snapshot->jobs[0]
                                .prediction_command_at_tick = 300;
                        snapshot->jobs[0].stop_request_tick = 305;
                        snapshot->jobs[0].stop_confirmed_tick = 307;
                        snapshot->jobs[0].stop_timing_valid = 1;
                        snapshot->jobs[0].stop_trigger =
                                TC2_STOP_TRIGGER_TIMER;
                }
                snapshot->job_count = 1;
        }
        return 0;
}

int Tc2DispatchGetProjectionHeader(
        int tid, int train,
        tc2_dispatch_projection_header *header) {
        (void)tid;
        (void)train;
        if (header) memset(header, 0, sizeof(*header));
        return TC2_DISPATCH_PROJECTION_NOT_READY;
}

int Tc2DispatchGetProjectionPage(
        int tid, int train, uint64_t publication_serial,
        uint32_t plan_generation, uint32_t launch_epoch,
        int first_waypoint,
        tc2_dispatch_projection_page *page) {
        (void)tid;
        (void)train;
        (void)publication_serial;
        (void)plan_generation;
        (void)launch_epoch;
        (void)first_waypoint;
        if (page) memset(page, 0, sizeof(*page));
        return TC2_DISPATCH_PROJECTION_NOT_READY;
}

int Tc2MotionModelIndex(int speed) {
        return speed >= 1 && speed <= 120 ? 1 : -1;
}

int Tc2MotionVelocityMmPerTick(int speed) {
        return Tc2MotionModelIndex(speed) < 0 ? -1 : 12;
}

int Tc2MotionVelocityTenthsMmPerTick(int speed) {
        return Tc2MotionModelIndex(speed) < 0 ? -1 : 120;
}

int Tc2MotionStopDistanceMm(int speed) {
        return Tc2MotionModelIndex(speed) < 0 ? -1 : 80;
}

int Tc2MotionUncertaintyMm(int speed) {
        return Tc2MotionModelIndex(speed) < 0 ? -1 : 250;
}

int Tc2MotionBrakingDistanceMm(int speed) {
        return Tc2MotionModelIndex(speed) < 0 ? -1 : 330;
}

int Tc2MotionWatchdogMarginTicks(int speed) {
        return Tc2MotionModelIndex(speed) < 0 ? -1 : 100;
}

int Tc2MotionProvisionalVelocityUmPerTick(int speed) {
        return speed >= 1 && speed <= 120 ?
                (speed == 20 ? 396 : speed * 20) : -1;
}

int Tc2MotionProvisionalStopDistanceMm(int speed) {
        return speed >= 1 && speed <= 120 ?
                (speed == 20 ? 11 : speed) : -1;
}

int main(void) {
        init_trackb(tc2_track);
        tc_dispatch_server_tid = 2;
        tc_reservation_server_tid = 4;
        tc_sensor_server_tid = 5;
        tc_can_server_tid = 3;

        if (test_dispatch_aliases_and_bounds() < 0 ||
            test_provisional_caldispatch() < 0 ||
            test_reroute_command_contract() < 0 ||
            test_turnout_plan_trips_contract() < 0 ||
            test_boot_diagnostic_lines_are_silent() < 0 ||
            test_exact_commands_and_blocks() < 0 ||
            test_manual_controls_always_disabled() < 0 ||
            test_track_diagram_and_destination_visibility() < 0 ||
            test_live_dashboard_does_not_capture_commands() < 0 ||
            test_offline_dispatch_aliases_bounds_and_isolation() < 0 ||
            test_offline_exact_lifecycle_commands() < 0 ||
            test_nonblocking_terminal_stub_contract() < 0) {
                return 1;
        }
        printf("validated strict TC2 live/offline/reroute commands, silent boot diagnostics, provisional caldispatch diagnostics, both 48-way A-F/d1-d8 matrices, exact evidence labels, nonblocking terminal contracts, bounds, labelled UI, and complete live/CAN isolation\n");
        return 0;
}
