#!/bin/sh
set -eu

tc1_plan=$(make -B -n MODE=TC1 all)
tc2_plan=$(make -B -n MODE=TC2 all)
ambient_plan=$(env MODE=TC1 make -B -n all)
clock_plan=$(make -B -n MODE=CLOCK all)
k4_plan=$(make -B -n MODE=K4 all)
tc2_o0_plan=$(make -B -n MODE=TC2 OPT=-O0 all)
idle_preprocessor=${HOSTCC:-clang}
tc1_clock=$("$idle_preprocessor" -E -P -I. -DMODE_TC1 clock.c)
tc2_clock=$("$idle_preprocessor" -E -P -I. -DMODE_TC2 clock.c)
clock_clock=$("$idle_preprocessor" -E -P -I. -DMODE_CLOCK clock.c)
tc1_motion=$("$idle_preprocessor" -E -P -I. -DMODE_TC1 tc2_motion_model.c)
tc2_motion=$("$idle_preprocessor" -E -P -I. -DMODE_TC2 tc2_motion_model.c)
tc1_prediction=$("$idle_preprocessor" -E -P -I. -DMODE_TC1 tc2_prediction_model.c)
tc2_prediction=$("$idle_preprocessor" -E -P -I. -DMODE_TC2 tc2_prediction_model.c)
tc1_overlay=$("$idle_preprocessor" -E -P -I. -DMODE_TC1 tc2_ui_overlay.c)
tc2_overlay=$("$idle_preprocessor" -E -P -I. -DMODE_TC2 tc2_ui_overlay.c)
tc1_renderer=$("$idle_preprocessor" -E -P -I. -DMODE_TC1 tc2_ui_renderer.c)
tc2_renderer=$("$idle_preprocessor" -E -P -I. -DMODE_TC2 tc2_ui_renderer.c)
tc1_projection=$("$idle_preprocessor" -E -P -I. -DMODE_TC1 tc2_route_projection.c)
tc2_projection=$("$idle_preprocessor" -E -P -I. -DMODE_TC2 tc2_route_projection.c)
tc1_offline_runtime=$("$idle_preprocessor" -E -P -I. -DMODE_TC1 tc2_offline_runtime.c)
tc2_offline_runtime=$("$idle_preprocessor" -E -P -I. -DMODE_TC2 tc2_offline_runtime.c)
tc1_offline_planner=$("$idle_preprocessor" -E -P -I. -DMODE_TC1 tc2_offline_planner.c)
tc2_offline_planner=$("$idle_preprocessor" -E -P -I. -DMODE_TC2 tc2_offline_planner.c)
tc1_ui_tx_pump=$("$idle_preprocessor" -E -P -I. -DMODE_TC1 tc2_ui_tx_pump.c)
tc2_ui_tx_pump=$("$idle_preprocessor" -E -P -I. -DMODE_TC2 tc2_ui_tx_pump.c)
tc1_offline_controller=$("$idle_preprocessor" -E -P -I. -DMODE_TC1 tc2_offline_controller.c)
tc2_offline_controller=$("$idle_preprocessor" -E -P -I. -DMODE_TC2 tc2_offline_controller.c)
tc1_terminal=$("$idle_preprocessor" -E -P -I. -DMODE_TC1 terminal.c)
tc2_terminal=$("$idle_preprocessor" -E -P -I. -DMODE_TC2 terminal.c)

require_text() {
        text=$1
        expected=$2
        label=$3
        if ! printf '%s\n' "$text" | grep -Fq -- "$expected"; then
                echo "$label is missing: $expected" >&2
                exit 1
        fi
}

reject_text() {
        text=$1
        unexpected=$2
        label=$3
        if printf '%s\n' "$text" | grep -Fq -- "$unexpected"; then
                echo "$label unexpectedly contains: $unexpected" >&2
                exit 1
        fi
}

require_text "$tc1_plan" "build/TC1/" "TC1 build plan"
require_text "$tc1_plan" "/tasks.o" "TC1 build plan"
require_text "$tc1_plan" "-DMODE_TC1" "TC1 build plan"
require_text "$tc1_plan" "-mgeneral-regs-only" "TC1 build plan"
require_text "$tc1_plan" "-Werror" "TC1 build plan"
reject_text "$tc1_plan" "build/TC2/" "TC1 build plan"
reject_text "$tc1_clock" \
        "IdleTask: elapsed_us=" \
        "TC1 preprocessed idle task"
reject_text "$tc1_motion" \
        "Tc2MotionProvisionalVelocityUmPerTick" \
        "TC1 preprocessed motion model"
reject_text "$tc1_prediction" \
        "Tc2PredictionBuildPlan" \
        "TC1 preprocessed prediction model"
reject_text "$tc1_overlay" \
        "Tc2UiOverlayInit" \
        "TC1 preprocessed UI overlay"
reject_text "$tc1_renderer" \
        "Tc2UiRendererRender" \
        "TC1 preprocessed UI renderer"
reject_text "$tc1_projection" \
        "Tc2RouteProjectionBuild" \
        "TC1 preprocessed route projection"
reject_text "$tc1_offline_runtime" \
        "Tc2OfflineRuntimeStep" \
        "TC1 preprocessed offline runtime"
reject_text "$tc1_offline_planner" \
        "Tc2OfflinePlannerBuild" \
        "TC1 preprocessed offline planner"
reject_text "$tc1_ui_tx_pump" \
        "Tc2UiTxPumpDrain" \
        "TC1 preprocessed UI transmit pump"
reject_text "$tc1_offline_controller" \
        "Tc2OfflineControllerStep" \
        "TC1 preprocessed offline controller"
require_text "$tc1_terminal" \
        "TerminalTryWrite" \
        "TC1 preprocessed terminal transport"

require_text "$tc2_plan" "build/TC2/" "TC2 build plan"
require_text "$tc2_plan" "/tasks.o" "TC2 build plan"
require_text "$tc2_plan" "-DMODE_TC2" "TC2 build plan"
require_text "$tc2_plan" "-mgeneral-regs-only" "TC2 build plan"
require_text "$tc2_plan" "-Werror" "TC2 build plan"
reject_text "$tc2_plan" "build/TC1/" "TC2 build plan"
reject_text "$tc2_clock" \
        "IdleTask: elapsed_us=" \
        "TC2 preprocessed idle task"
require_text "$tc2_motion" \
        "Tc2MotionProvisionalVelocityUmPerTick" \
        "TC2 preprocessed motion model"
require_text "$tc2_prediction" \
        "Tc2PredictionBuildPlan" \
        "TC2 preprocessed prediction model"
require_text "$tc2_overlay" \
        "Tc2UiOverlayInit" \
        "TC2 preprocessed UI overlay"
require_text "$tc2_renderer" \
        "Tc2UiRendererRender" \
        "TC2 preprocessed UI renderer"
require_text "$tc2_projection" \
        "Tc2RouteProjectionBuild" \
        "TC2 preprocessed route projection"
require_text "$tc2_offline_runtime" \
        "Tc2OfflineRuntimeStep" \
        "TC2 preprocessed offline runtime"
require_text "$tc2_offline_planner" \
        "Tc2OfflinePlannerBuild" \
        "TC2 preprocessed offline planner"
require_text "$tc2_ui_tx_pump" \
        "Tc2UiTxPumpDrain" \
        "TC2 preprocessed UI transmit pump"
require_text "$tc2_offline_controller" \
        "Tc2OfflineControllerStep" \
        "TC2 preprocessed offline controller"
require_text "$tc2_terminal" \
        "TerminalTryWrite" \
        "TC2 preprocessed terminal transport"

require_text "$ambient_plan" "build/TC2/" "ambient MODE build plan"
require_text "$ambient_plan" "/tasks.o" "ambient MODE build plan"
require_text "$ambient_plan" "-DMODE_TC2" "ambient MODE build plan"
require_text "$ambient_plan" "-mgeneral-regs-only" "ambient MODE build plan"
require_text "$ambient_plan" "-Werror" "ambient MODE build plan"
reject_text "$ambient_plan" "build/TC1/" "ambient MODE build plan"

require_text "$clock_plan" "build/CLOCK/" "CLOCK build plan"
require_text "$clock_plan" "-DMODE_CLOCK" "CLOCK build plan"
require_text "$clock_clock" \
        "IdleTask: elapsed_us=" \
        "CLOCK preprocessed idle task"
require_text "$k4_plan" "build/K4/" "K4 build plan"
require_text "$k4_plan" "-DMODE_K4" "K4 build plan"

tc2_tasks=$(printf '%s\n' "$tc2_plan" | sed -n 's#.*-o \(build/TC2/[^/]*/tasks\.o\).*#\1#p' | head -n 1)
tc2_o0_tasks=$(printf '%s\n' "$tc2_o0_plan" | sed -n 's#.*-o \(build/TC2/[^/]*/tasks\.o\).*#\1#p' | head -n 1)
if [ -z "$tc2_tasks" ] || [ -z "$tc2_o0_tasks" ] || [ "$tc2_tasks" = "$tc2_o0_tasks" ]; then
        echo "TC2 -O3 and -O0 plans did not use distinct configuration trees" >&2
        exit 1
fi

if ! grep -Fq \
        'TC2_DISPATCH_TICKER_PRIORITY,' \
        tc2_dispatch.c; then
        echo "TC2 dispatcher ticker does not use the shared control-plane priority" >&2
        exit 1
fi

if ! grep -Fq \
        'Create(TC2_SENSOR_COURIER_PRIORITY,' \
        train_sensor.c; then
        echo "TC2 sensor courier does not use the shared control-plane priority" >&2
        exit 1
fi

if ! grep -Fq \
        'TC2_SENSOR_STARTUP_MAX_ATTEMPTS' \
        tc2_dispatch.c ||
   ! grep -Fq \
        'wait_for_sensor_startup(' \
        tc2_dispatch.c; then
        echo "TC2 dispatcher lacks the bounded sensor READY startup barrier" >&2
        exit 1
fi

if ! grep -Fq \
        'util_zero_bytes(dispatch_blocked_by_node,' \
        tc2_dispatch.c; then
        echo "TC2 dispatcher lacks the GCC-safe bounded block-map clear" >&2
        exit 1
fi

if ! grep -Fq \
        '#define TERMINAL_NOTIFIER_PRIORITY TC2_TERMINAL_NOTIFIER_PRIORITY' \
        terminal.c; then
        echo "TC2 terminal notifiers do not use the shared control-plane priority" >&2
        exit 1
fi

expected_marker='TC2 build marker: stage68-compound-zone-watchdog-latch'
for marker_file in tasks.h Makefile README-TC2; do
        if ! grep -Fq "$expected_marker" "$marker_file"; then
		echo "$marker_file lacks the exact stage68 build marker" >&2
                exit 1
        fi
done
if grep -Eq \
		'TC2 build marker: .*stage([3-9]|[1-5][0-9]|6[0-6])([^0-9]|$)' \
        tasks.h Makefile README-TC2; then
	echo "stale pre-stage68 TC2 build marker remains" >&2
        exit 1
fi

echo "validated isolated mode/config trees and ambient-MODE-safe TC2 default"
