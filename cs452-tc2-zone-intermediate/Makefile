FILENAME=iotest
BUILD_ROOT:=build
XDIR:=/u/cs452/public/xdev
TRIPLE=aarch64-none-elf
XBINDIR:=$(XDIR)/bin
CC:=$(XBINDIR)/$(TRIPLE)-gcc -ffreestanding
OBJCOPY:=$(XBINDIR)/$(TRIPLE)-objcopy
OBJDUMP:=$(XBINDIR)/$(TRIPLE)-objdump
STRINGS:=$(XBINDIR)/$(TRIPLE)-strings
HOSTCC?=clang
HOST_TEST_DIR?=/tmp/cs452_tc2_tests
HOST_TEST_FLAGS:=-std=c11 -Wall -Wextra -Werror -fsanitize=address,undefined -I.

MMU?=on
OPT?=-O3
CACHE?=both
# Ignore an ambient shell MODE while still allowing an explicit
# `make MODE=...` command-line selection.
MODE:=TC2
PERF_ORDER?=S
PERF_SIZE?=4
PERF_ITER?=1000

VALID_MODES:=TC1 TC2 PERF K4 CLOCK RPS
ifeq ($(filter $(MODE),$(VALID_MODES)),)
$(error unsupported MODE '$(MODE)'; expected one of: $(VALID_MODES))
endif

MODEFLAGS:=-DMODE_$(MODE)
ifeq ($(MODE),PERF)
MODEFLAGS+=-DPERF_ORDER_$(PERF_ORDER) -DPERF_MSG_SIZE=$(PERF_SIZE) -DPERF_ITERATIONS=$(PERF_ITER)
endif

# COMPILE OPTIONS
ifeq ($(MMU),on)
MMUFLAGS:=-DMMU
else
MMUFLAGS:=-mstrict-align
endif

CACHEFLAGS:=-DCACHE_$(CACHE)
REGISTERFLAGS:=-mgeneral-regs-only
SCALARFLAGS:=-fno-tree-vectorize -fno-tree-slp-vectorize -fno-tree-loop-distribute-patterns
WARNINGS:=-Wall -Wextra -Wpedantic -Werror -Wno-unused-const-variable
CFLAGS:=-g -pipe -static -march=armv8-a -mcpu=cortex-a72 $(OPT) $(MMUFLAGS) $(CACHEFLAGS) $(REGISTERFLAGS) $(SCALARFLAGS) $(WARNINGS) $(MODEFLAGS)

# -Wl,option tells gcc to pass 'option' to the linker with commas replaced by spaces
# doing this rather than calling the linker directly simplifies the compilation procedure
LDFLAGS:=-Wl,-nmagic -Wl,-Tlinker.ld -Wl,--no-warn-rwx-segments -nostartfiles

# Source files and include dirs
SOURCES := $(wildcard *.S) $(wildcard *.c)
# Keep each mode and complete build configuration in a distinct object tree.
# A content-derived directory is reliable even on GNU make 3.81 and
# one-second-mtime filesystems; it does not delete targets during dependency
# traversal or rely on command-line changes being visible to make.
CONFIG_ID := $(shell printf '%s\n' "SOURCES=$(SOURCES)" "CC=$(CC)" "CFLAGS=$(CFLAGS)" "LDFLAGS=$(LDFLAGS)" "OBJCOPY=$(OBJCOPY)" | cksum | awk '{print $$1 "-" $$2}')
ifeq ($(strip $(CONFIG_ID)),)
$(error failed to derive build configuration id)
endif
BUILD_DIR := $(BUILD_ROOT)/$(MODE)/$(CONFIG_ID)
BUILD_ELF := $(BUILD_DIR)/$(FILENAME).elf
BUILD_IMG := $(BUILD_DIR)/$(FILENAME).img
BUILD_CONFIG := $(BUILD_DIR)/.config
OBJECTS := $(patsubst %.S, $(BUILD_DIR)/%.o, $(patsubst %.c, $(BUILD_DIR)/%.o, $(SOURCES)))
DEPENDS := $(patsubst %.S, $(BUILD_DIR)/%.d, $(patsubst %.c, $(BUILD_DIR)/%.d, $(SOURCES)))
LEGACY_OBJECTS := $(patsubst %.S, %.o, $(patsubst %.c, %.o, $(SOURCES)))
LEGACY_DEPENDS := $(patsubst %.S, %.d, $(patsubst %.c, %.d, $(SOURCES)))

.PHONY: all clean test dispatch-matrix-report verify-build FORCE invalidate-output

all: $(BUILD_IMG) verify-build
	cp $(BUILD_ELF) $(FILENAME).elf
	cp $(BUILD_IMG) $(FILENAME).img
	@printf "Built and verified MODE=%s image: %s\n" "$(MODE)" "$(FILENAME).img"

invalidate-output:
	rm -f $(FILENAME).elf $(FILENAME).img

clean:
	rm -f $(LEGACY_OBJECTS) $(LEGACY_DEPENDS) $(FILENAME).elf $(FILENAME).img
	rm -rf build

test:
	mkdir -p $(HOST_TEST_DIR)
	$(HOSTCC) $(HOST_TEST_FLAGS) tests/util_test.c util.c -o $(HOST_TEST_DIR)/util_test
	$(HOST_TEST_DIR)/util_test
	$(HOSTCC) $(HOST_TEST_FLAGS) -DMODE_TC2 tests/first_user_priority_test.c main.c -o $(HOST_TEST_DIR)/first_user_priority_tc2_test
	$(HOST_TEST_DIR)/first_user_priority_tc2_test
	$(HOSTCC) $(HOST_TEST_FLAGS) -DMODE_TC1 tests/first_user_priority_test.c main.c -o $(HOST_TEST_DIR)/first_user_priority_tc1_test
	$(HOST_TEST_DIR)/first_user_priority_tc1_test
	$(HOSTCC) $(HOST_TEST_FLAGS) tests/tc2_bootstrap_topology_test.c -o $(HOST_TEST_DIR)/tc2_bootstrap_topology_test
	$(HOST_TEST_DIR)/tc2_bootstrap_topology_test
	sh tests/context_general_register_test.sh
	sh tests/bss_zero_init_test.sh
	$(HOSTCC) $(HOST_TEST_FLAGS) tests/context_frame_layout_test.c -o $(HOST_TEST_DIR)/context_frame_layout_test
	$(HOST_TEST_DIR)/context_frame_layout_test
	sh tests/build_mode_isolation_test.sh
	$(HOSTCC) $(HOST_TEST_FLAGS) tests/task_exit_ipc_test.c task.c -o $(HOST_TEST_DIR)/task_exit_ipc_test
	$(HOST_TEST_DIR)/task_exit_ipc_test
	$(HOSTCC) $(HOST_TEST_FLAGS) -DMODE_TC2 tests/tc2_motion_model_test.c tc2_motion_model.c -o $(HOST_TEST_DIR)/tc2_motion_model_test
	$(HOST_TEST_DIR)/tc2_motion_model_test
	$(HOSTCC) $(HOST_TEST_FLAGS) tests/tc2_traffic_safety_test.c tc2_traffic_safety.c -o $(HOST_TEST_DIR)/tc2_traffic_safety_test
	$(HOST_TEST_DIR)/tc2_traffic_safety_test
	$(HOSTCC) $(HOST_TEST_FLAGS) tests/tc2_conflict_zone_test.c tc2_conflict_zone.c track_route.c track_data.c -o $(HOST_TEST_DIR)/tc2_conflict_zone_test
	$(HOST_TEST_DIR)/tc2_conflict_zone_test
	$(HOSTCC) $(HOST_TEST_FLAGS) tests/tc2_track_model_test.c tc2_track_model.c track_route.c track_data.c -o $(HOST_TEST_DIR)/tc2_track_model_test
	$(HOST_TEST_DIR)/tc2_track_model_test
	$(HOSTCC) $(HOST_TEST_FLAGS) -DMODE_TC2 tests/tc2_prediction_model_test.c tc2_prediction_model.c tc2_track_model.c tc2_motion_model.c track_route.c track_data.c -o $(HOST_TEST_DIR)/tc2_prediction_model_test
	$(HOST_TEST_DIR)/tc2_prediction_model_test
	$(HOSTCC) $(HOST_TEST_FLAGS) tests/track_d_distance_corrections_test.c track_data.c -o $(HOST_TEST_DIR)/track_d_distance_corrections_test
	$(HOST_TEST_DIR)/track_d_distance_corrections_test
	$(HOSTCC) $(HOST_TEST_FLAGS) -DMODE_TC2 tests/tc2_track_d_matrix_test.c tc2_prediction_model.c tc2_track_model.c tc2_motion_model.c tc2_route_monitor.c track_route.c track_data.c -o $(HOST_TEST_DIR)/tc2_track_d_matrix_test
	$(HOST_TEST_DIR)/tc2_track_d_matrix_test
	$(HOSTCC) $(HOST_TEST_FLAGS) -DMODE_TC2 tests/tc2_route_projection_test.c tc2_route_projection.c tc2_prediction_model.c tc2_track_model.c tc2_track_d_layout.c tc2_motion_model.c track_route.c track_data.c -o $(HOST_TEST_DIR)/tc2_route_projection_test
	$(HOST_TEST_DIR)/tc2_route_projection_test
	$(HOSTCC) $(HOST_TEST_FLAGS) tests/track_route_advanced_test.c track_route.c track_data.c -o $(HOST_TEST_DIR)/track_route_advanced_test
	$(HOST_TEST_DIR)/track_route_advanced_test
	$(HOSTCC) $(HOST_TEST_FLAGS) tests/tc2_route_monitor_test.c tc2_route_monitor.c track_route.c track_data.c -o $(HOST_TEST_DIR)/tc2_route_monitor_test
	$(HOST_TEST_DIR)/tc2_route_monitor_test
	$(HOSTCC) $(HOST_TEST_FLAGS) tests/track_reservation_window_test.c track_reservation.c track_route.c track_data.c -o $(HOST_TEST_DIR)/track_reservation_window_test
	$(HOST_TEST_DIR)/track_reservation_window_test
	$(HOSTCC) $(HOST_TEST_FLAGS) tests/train_sensor_latch_test.c train_sensor.c -o $(HOST_TEST_DIR)/train_sensor_latch_test
	$(HOST_TEST_DIR)/train_sensor_latch_test
	$(HOSTCC) $(HOST_TEST_FLAGS) tests/train_sensor_journal_test.c train_sensor.c -o $(HOST_TEST_DIR)/train_sensor_journal_test
	$(HOST_TEST_DIR)/train_sensor_journal_test
	$(HOSTCC) $(HOST_TEST_FLAGS) tests/terminal_transport_test.c terminal.c -o $(HOST_TEST_DIR)/terminal_transport_test
	$(HOST_TEST_DIR)/terminal_transport_test
	$(HOSTCC) $(HOST_TEST_FLAGS) -DMODE_TC2 tests/terminal_transport_test.c terminal.c -o $(HOST_TEST_DIR)/terminal_transport_tc2_test
	$(HOST_TEST_DIR)/terminal_transport_tc2_test
	$(HOSTCC) $(HOST_TEST_FLAGS) tests/can_transport_state_test.c can.c -o $(HOST_TEST_DIR)/can_transport_state_test
	$(HOST_TEST_DIR)/can_transport_state_test
	$(HOSTCC) $(HOST_TEST_FLAGS) tests/spi_timeout_test.c -o $(HOST_TEST_DIR)/spi_timeout_test
	$(HOST_TEST_DIR)/spi_timeout_test
	$(HOSTCC) $(HOST_TEST_FLAGS) tests/mcp2515_transport_test.c mcp2515.c -o $(HOST_TEST_DIR)/mcp2515_transport_test
	$(HOST_TEST_DIR)/mcp2515_transport_test
	$(HOSTCC) $(HOST_TEST_FLAGS) -DMODE_TC2 tests/tc2_track_d_layout_test.c tc2_track_d_layout.c -o $(HOST_TEST_DIR)/tc2_track_d_layout_test
	$(HOST_TEST_DIR)/tc2_track_d_layout_test
	$(HOSTCC) $(HOST_TEST_FLAGS) -DMODE_TC2 tests/tc2_ui_overlay_test.c tc2_ui_overlay.c tc2_track_d_layout.c -o $(HOST_TEST_DIR)/tc2_ui_overlay_test
	$(HOST_TEST_DIR)/tc2_ui_overlay_test
	$(HOSTCC) $(HOST_TEST_FLAGS) -DMODE_TC2 tests/tc2_ui_path_geometry_test.c tc2_ui_path_geometry.c tc2_track_d_layout.c -o $(HOST_TEST_DIR)/tc2_ui_path_geometry_test
	$(HOST_TEST_DIR)/tc2_ui_path_geometry_test
	$(HOSTCC) $(HOST_TEST_FLAGS) -DMODE_TC2 tests/tc2_ui_route_geometry_matrix_test.c tc2_live_ui.c tc2_ui_path_geometry.c tc2_ui_renderer.c tc2_ui_overlay.c tc2_ui_tx_pump.c tc2_track_d_layout.c tc2_motion_model.c tc2_offline_planner.c tc2_route_projection.c tc2_prediction_model.c tc2_track_model.c track_route.c track_data.c -o $(HOST_TEST_DIR)/tc2_ui_route_geometry_matrix_test
	$(HOST_TEST_DIR)/tc2_ui_route_geometry_matrix_test
	$(HOSTCC) $(HOST_TEST_FLAGS) -O3 -DMODE_TC2 tests/tc2_ui_renderer_test.c tc2_ui_renderer.c tc2_ui_overlay.c tc2_track_d_layout.c -o $(HOST_TEST_DIR)/tc2_ui_renderer_test
	$(HOST_TEST_DIR)/tc2_ui_renderer_test
	$(HOSTCC) $(HOST_TEST_FLAGS) -DMODE_TC2 tests/tc2_ui_tx_pump_test.c tc2_ui_tx_pump.c -o $(HOST_TEST_DIR)/tc2_ui_tx_pump_test
	$(HOST_TEST_DIR)/tc2_ui_tx_pump_test
	$(HOSTCC) $(HOST_TEST_FLAGS) -DMODE_TC2 tests/tc2_live_ui_test.c tc2_live_ui.c tc2_ui_path_geometry.c tc2_ui_renderer.c tc2_ui_overlay.c tc2_ui_tx_pump.c tc2_track_d_layout.c tc2_motion_model.c -o $(HOST_TEST_DIR)/tc2_live_ui_test
	$(HOST_TEST_DIR)/tc2_live_ui_test
	$(HOSTCC) $(HOST_TEST_FLAGS) -DMODE_TC2 tests/tc2_command_ui_test.c tc2_live_ui.c tc2_ui_path_geometry.c tc2_ui_renderer.c tc2_ui_overlay.c tc2_ui_tx_pump.c tc2_track_d_layout.c -o $(HOST_TEST_DIR)/tc2_command_ui_test
	$(HOST_TEST_DIR)/tc2_command_ui_test
	$(HOSTCC) $(HOST_TEST_FLAGS) tests/tc2_alias_route_test.c tc2_dispatch.c tc2_track_model.c track_route.c track_data.c util.c -o $(HOST_TEST_DIR)/tc2_alias_route_test
	$(HOST_TEST_DIR)/tc2_alias_route_test
	$(HOSTCC) $(HOST_TEST_FLAGS) -DMODE_TC2 -DTC2_CONFLICT_ZONE_INTEGRATION_TEST tests/tc2_dispatch_core_test.c tc2_conflict_zone.c tc2_track_model.c tc2_track_d_layout.c tc2_prediction_model.c tc2_route_projection.c track_data.c track_route.c tc2_route_monitor.c tc2_motion_model.c tc2_traffic_safety.c track_reservation.c util.c -o $(HOST_TEST_DIR)/tc2_conflict_zone_integration_test
	$(HOST_TEST_DIR)/tc2_conflict_zone_integration_test
	$(HOSTCC) $(HOST_TEST_FLAGS) -DMODE_TC2 tests/tc2_dispatch_core_test.c tc2_conflict_zone.c tc2_track_model.c tc2_track_d_layout.c tc2_prediction_model.c tc2_route_projection.c track_data.c track_route.c tc2_route_monitor.c tc2_motion_model.c tc2_traffic_safety.c track_reservation.c util.c -o $(HOST_TEST_DIR)/tc2_dispatch_core_test
	$(HOST_TEST_DIR)/tc2_dispatch_core_test
	$(HOSTCC) $(HOST_TEST_FLAGS) -DMODE_TC2 tests/tc2_offline_runtime_test.c tc2_offline_runtime.c tc2_motion_model.c tc2_track_d_layout.c tc2_ui_overlay.c -o $(HOST_TEST_DIR)/tc2_offline_runtime_test
	$(HOST_TEST_DIR)/tc2_offline_runtime_test
	$(HOSTCC) $(HOST_TEST_FLAGS) -DMODE_TC2 tests/tc2_offline_planner_test.c tc2_offline_planner.c tc2_route_projection.c tc2_prediction_model.c tc2_track_model.c tc2_track_d_layout.c tc2_motion_model.c track_route.c track_data.c -o $(HOST_TEST_DIR)/tc2_offline_planner_test
	$(HOST_TEST_DIR)/tc2_offline_planner_test
	$(HOSTCC) $(HOST_TEST_FLAGS) -DMODE_TC2 tests/tc2_physical_turnout_oracle_test.c tc2_offline_planner.c tc2_route_projection.c tc2_prediction_model.c tc2_track_model.c tc2_track_d_layout.c tc2_motion_model.c track_route.c track_data.c -o $(HOST_TEST_DIR)/tc2_physical_turnout_oracle_test
	$(HOST_TEST_DIR)/tc2_physical_turnout_oracle_test
	$(HOSTCC) $(HOST_TEST_FLAGS) -DMODE_TC2 tests/tc2_offline_controller_test.c tc2_offline_controller.c tc2_offline_planner.c tc2_offline_runtime.c tc2_ui_overlay.c tc2_ui_renderer.c tc2_ui_tx_pump.c tc2_track_d_layout.c tc2_route_projection.c tc2_prediction_model.c tc2_track_model.c tc2_motion_model.c track_route.c track_data.c -o $(HOST_TEST_DIR)/tc2_offline_controller_test
	$(HOST_TEST_DIR)/tc2_offline_controller_test

dispatch-matrix-report:
	mkdir -p $(HOST_TEST_DIR)
	$(HOSTCC) $(HOST_TEST_FLAGS) -DMODE_TC2 tests/tc2_dispatch_matrix_report.c tc2_offline_planner.c tc2_route_projection.c tc2_prediction_model.c tc2_track_model.c tc2_track_d_layout.c tc2_motion_model.c track_route.c track_data.c -o $(HOST_TEST_DIR)/tc2_dispatch_matrix_report
	$(HOST_TEST_DIR)/tc2_dispatch_matrix_report

verify-build: $(BUILD_ELF)
	@sh tests/no_simd_binary_test.sh "$(OBJDUMP)" "$<"
ifeq ($(MODE),TC2)
	@$(OBJDUMP) -t $< | grep -Fq "Tc2DispatchServerTask" || { printf "ERROR: MODE=TC2 image lacks Tc2DispatchServerTask\n" >&2; exit 1; }
	@$(OBJDUMP) -t $< | grep -Fq "Tc2DispatchTickerTask" || { printf "ERROR: MODE=TC2 image lacks Tc2DispatchTickerTask\n" >&2; exit 1; }
	@$(OBJDUMP) -t $< | grep -Fq "Tc2DispatchSafetyTask" || { printf "ERROR: MODE=TC2 image lacks Tc2DispatchSafetyTask\n" >&2; exit 1; }
	@$(OBJDUMP) -t $< | grep -Fq "Tc2PredictionBuildPlan" || { printf "ERROR: MODE=TC2 image lacks Tc2PredictionBuildPlan\n" >&2; exit 1; }
	@$(OBJDUMP) -t $< | grep -Fq "Tc2RouteProjectionBuild" || { printf "ERROR: MODE=TC2 image lacks Tc2RouteProjectionBuild\n" >&2; exit 1; }
	@$(OBJDUMP) -t $< | grep -Fq "Tc2OfflineRuntimeStep" || { printf "ERROR: MODE=TC2 image lacks Tc2OfflineRuntimeStep\n" >&2; exit 1; }
	@$(OBJDUMP) -t $< | grep -Fq "Tc2OfflinePlannerBuild" || { printf "ERROR: MODE=TC2 image lacks Tc2OfflinePlannerBuild\n" >&2; exit 1; }
	@$(OBJDUMP) -t $< | grep -Fq "Tc2UiRendererRender" || { printf "ERROR: MODE=TC2 image lacks Tc2UiRendererRender\n" >&2; exit 1; }
	@$(OBJDUMP) -t $< | grep -Fq "Tc2UiTxPumpDrain" || { printf "ERROR: MODE=TC2 image lacks Tc2UiTxPumpDrain\n" >&2; exit 1; }
	@$(OBJDUMP) -t $< | grep -Fq "Tc2OfflineControllerStep" || { printf "ERROR: MODE=TC2 image lacks Tc2OfflineControllerStep\n" >&2; exit 1; }
	@$(OBJDUMP) -t $< | grep -Fq "TerminalTryWrite" || { printf "ERROR: MODE=TC2 image lacks TerminalTryWrite\n" >&2; exit 1; }
	@$(STRINGS) $< | grep -Fq "TC2 Train Control Application" || { printf "ERROR: MODE=TC2 image lacks the TC2 banner\n" >&2; exit 1; }
	@$(STRINGS) $< | grep -Fq "TC2 build marker: stage68-compound-zone-watchdog-latch" || { printf "ERROR: MODE=TC2 image lacks the expected boot marker\n" >&2; exit 1; }
	@$(OBJDUMP) -t $< | grep -Fq "__bss_start" || { printf "ERROR: linked image lacks __bss_start\n" >&2; exit 1; }
	@$(OBJDUMP) -t $< | grep -Fq "__bss_end" || { printf "ERROR: linked image lacks __bss_end\n" >&2; exit 1; }
	@if $(STRINGS) $< | grep -Fq "TC1 Train Control Application"; then printf "ERROR: MODE=TC2 image contains the TC1 banner\n" >&2; exit 1; fi
else ifeq ($(MODE),TC1)
	@$(STRINGS) $< | grep -Fq "TC1 Train Control Application" || { printf "ERROR: MODE=TC1 image lacks the TC1 banner\n" >&2; exit 1; }
	@if $(OBJDUMP) -t $< | grep -Fq "Tc2DispatchServerTask"; then printf "ERROR: MODE=TC1 image contains the TC2 dispatcher\n" >&2; exit 1; fi
	@if $(OBJDUMP) -t $< | grep -Fq "Tc2MotionProvisional"; then printf "ERROR: MODE=TC1 image contains the TC2 provisional predictor\n" >&2; exit 1; fi
	@if $(OBJDUMP) -t $< | grep -Fq "Tc2PredictionBuildPlan"; then printf "ERROR: MODE=TC1 image contains the TC2 route-relative predictor\n" >&2; exit 1; fi
	@if $(OBJDUMP) -t $< | grep -Fq "Tc2RouteProjectionBuild"; then printf "ERROR: MODE=TC1 image contains the TC2 route projection layer\n" >&2; exit 1; fi
	@if $(OBJDUMP) -t $< | grep -Fq "Tc2OfflineRuntimeStep"; then printf "ERROR: MODE=TC1 image contains the TC2 offline runtime\n" >&2; exit 1; fi
	@if $(OBJDUMP) -t $< | grep -Fq "Tc2OfflinePlannerBuild"; then printf "ERROR: MODE=TC1 image contains the TC2 offline planner\n" >&2; exit 1; fi
	@if $(OBJDUMP) -t $< | grep -Fq "Tc2UiTxPumpDrain"; then printf "ERROR: MODE=TC1 image contains the TC2 UI transmit pump\n" >&2; exit 1; fi
	@if $(OBJDUMP) -t $< | grep -Fq "Tc2OfflineControllerStep"; then printf "ERROR: MODE=TC1 image contains the TC2 offline controller\n" >&2; exit 1; fi
else ifeq ($(MODE),PERF)
	@$(OBJDUMP) -t $< | grep -Fq "PerfReceiverTask" || { printf "ERROR: MODE=PERF image lacks PerfReceiverTask\n" >&2; exit 1; }
	@$(OBJDUMP) -t $< | grep -Fq "PerfSenderTask" || { printf "ERROR: MODE=PERF image lacks PerfSenderTask\n" >&2; exit 1; }
	@$(STRINGS) $< | grep -Fq "FirstUserTask: starting PERF batch test" || { printf "ERROR: MODE=PERF image lacks the PERF banner\n" >&2; exit 1; }
else ifeq ($(MODE),K4)
	@$(STRINGS) $< | grep -Fq "FirstUserTask: starting K4 terminal I/O test" || { printf "ERROR: MODE=K4 image lacks the K4 banner\n" >&2; exit 1; }
else ifeq ($(MODE),CLOCK)
	@$(STRINGS) $< | grep -Fq "FirstUserTask: creating clock clients" || { printf "ERROR: MODE=CLOCK image lacks the clock-client banner\n" >&2; exit 1; }
else ifeq ($(MODE),RPS)
	@$(STRINGS) $< | grep -Fq "FirstUserTask: creating RPS server" || { printf "ERROR: MODE=RPS image lacks the RPS banner\n" >&2; exit 1; }
endif

$(BUILD_IMG): $(BUILD_ELF)
	$(OBJCOPY) -S -O binary $< $@; sync

$(BUILD_ELF): $(OBJECTS) linker.ld
	$(CC) $(CFLAGS) $(filter-out %.ld, $^) -o $@ $(LDFLAGS)

$(BUILD_DIR):
	mkdir -p $@

$(BUILD_CONFIG): FORCE invalidate-output | $(BUILD_DIR)
	@{ \
		printf '%s\n' "SOURCES=$(SOURCES)" "CC=$(CC)" "CFLAGS=$(CFLAGS)" "LDFLAGS=$(LDFLAGS)" "OBJCOPY=$(OBJCOPY)"; \
	} > $@.tmp
	@if test -e $@ && ! cmp -s $@.tmp $@; then \
		printf "ERROR: build configuration checksum collision at %s\n" "$@" >&2; \
		rm -f $@.tmp; \
		exit 1; \
	elif test ! -e $@; then \
		mv $@.tmp $@; \
	else \
		rm -f $@.tmp; \
	fi

$(BUILD_DIR)/%.o: %.S Makefile $(BUILD_CONFIG) | $(BUILD_DIR)
	$(CC) $(CFLAGS) -MMD -MP -c $< -o $@

$(BUILD_DIR)/%.o: %.c Makefile $(BUILD_CONFIG) | $(BUILD_DIR)
	$(CC) $(CFLAGS) -MMD -MP -c $< -o $@

-include $(DEPENDS)
