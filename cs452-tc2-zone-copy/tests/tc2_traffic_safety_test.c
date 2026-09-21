#include <assert.h>
#include <stdio.h>

#include "tc2_traffic_safety.h"

static tc2_traffic_safety_input base_input(void) {
        tc2_traffic_safety_input input = {
                .train_a_route_scalar_mm = 1000,
                .train_b_route_scalar_mm = 1500,
                .train_a_effective_speed = 60,
                .train_b_effective_speed = 80,
                .train_a_braking_envelope_mm = 150,
                .train_b_braking_envelope_mm = 300,
                .gap_mm = 1000,
                .relation =
                        TC2_TRAFFIC_RELATION_SAME_DIRECTION_A_REAR,
                .rear_stop_latched = 0,
                .rear_has_movement_authority = 1,
        };
        return input;
}

static void test_head_on_always_stops_both(void) {
        tc2_traffic_safety_input input = base_input();
        input.relation = TC2_TRAFFIC_RELATION_HEAD_ON;

        input.gap_mm = TC2_TRAFFIC_EMERGENCY_CLEARANCE_MM - 1;
        assert(Tc2TrafficSafetyEvaluate(&input) ==
               TC2_TRAFFIC_ACTION_STOP_BOTH);
        input.gap_mm = TC2_TRAFFIC_EMERGENCY_CLEARANCE_MM;
        assert(Tc2TrafficSafetyEvaluate(&input) ==
               TC2_TRAFFIC_ACTION_STOP_BOTH);
        input.gap_mm = TC2_TRAFFIC_EMERGENCY_CLEARANCE_MM + 1;
        assert(Tc2TrafficSafetyEvaluate(&input) ==
               TC2_TRAFFIC_ACTION_STOP_BOTH);

        input.gap_mm = 5000;
        assert(Tc2TrafficSafetyEvaluate(&input) ==
               TC2_TRAFFIC_ACTION_STOP_BOTH);
}

static void test_following_350_mm_stops_only_rear(void) {
        tc2_traffic_safety_input input = base_input();

        input.gap_mm = TC2_TRAFFIC_FOLLOWING_CLEARANCE_MM;
        assert(Tc2TrafficSafetyEvaluate(&input) ==
               TC2_TRAFFIC_ACTION_STOP_REAR);
        input.gap_mm = TC2_TRAFFIC_FOLLOWING_CLEARANCE_MM + 1;
        assert(Tc2TrafficSafetyEvaluate(&input) ==
               TC2_TRAFFIC_ACTION_NONE);

        input.relation =
                TC2_TRAFFIC_RELATION_SAME_DIRECTION_B_REAR;
        input.train_a_effective_speed = 80;
        input.train_b_effective_speed = 60;
        input.train_a_braking_envelope_mm = 300;
        input.train_b_braking_envelope_mm = 150;
        input.gap_mm = TC2_TRAFFIC_FOLLOWING_CLEARANCE_MM;
        assert(Tc2TrafficSafetyEvaluate(&input) ==
               TC2_TRAFFIC_ACTION_STOP_REAR);
        input.gap_mm = TC2_TRAFFIC_FOLLOWING_CLEARANCE_MM + 1;
        assert(Tc2TrafficSafetyEvaluate(&input) ==
               TC2_TRAFFIC_ACTION_NONE);
}

static void test_equal_speed_still_rechecks_clearance(void) {
        tc2_traffic_safety_input input = base_input();
        input.train_a_effective_speed = 60;
        input.train_b_effective_speed = 60;
        input.train_a_braking_envelope_mm = 150;
        input.train_b_braking_envelope_mm = 150;

        input.gap_mm = TC2_TRAFFIC_FOLLOWING_CLEARANCE_MM;
        assert(Tc2TrafficSafetyEvaluate(&input) ==
               TC2_TRAFFIC_ACTION_STOP_REAR);
        input.gap_mm = TC2_TRAFFIC_FOLLOWING_CLEARANCE_MM + 1;
        assert(Tc2TrafficSafetyEvaluate(&input) ==
               TC2_TRAFFIC_ACTION_NONE);
}

static void test_stopped_front_stops_only_rear(void) {
        tc2_traffic_safety_input input = base_input();
        input.train_b_effective_speed = 0;
        input.train_b_braking_envelope_mm = 0;
        input.gap_mm = 2000;

        assert(Tc2TrafficSafetyEvaluate(&input) ==
               TC2_TRAFFIC_ACTION_STOP_REAR);

        input = base_input();
        input.relation =
                TC2_TRAFFIC_RELATION_SAME_DIRECTION_B_REAR;
        input.train_a_effective_speed = 0;
        input.train_a_braking_envelope_mm = 0;
        input.gap_mm = 2000;
        assert(Tc2TrafficSafetyEvaluate(&input) ==
               TC2_TRAFFIC_ACTION_STOP_REAR);
}

static void test_slower_front_rechecks_rear(void) {
        tc2_traffic_safety_input input = base_input();
        input.train_a_effective_speed = 80;
        input.train_b_effective_speed = 60;
        input.gap_mm = 2000;

        /* Current input exposes deceleration conservatively as front < rear. */
        assert(Tc2TrafficSafetyEvaluate(&input) ==
               TC2_TRAFFIC_ACTION_STOP_REAR);

        input = base_input();
        input.relation =
                TC2_TRAFFIC_RELATION_SAME_DIRECTION_B_REAR;
        input.train_a_effective_speed = 60;
        input.train_b_effective_speed = 80;
        input.gap_mm = 2000;
        assert(Tc2TrafficSafetyEvaluate(&input) ==
               TC2_TRAFFIC_ACTION_STOP_REAR);
}

static void test_front_localization_failure_stops_only_rear(void) {
        tc2_traffic_safety_input input = base_input();
        input.train_b_route_scalar_mm = -1;
        input.gap_mm = 2000;
        assert(Tc2TrafficSafetyEvaluate(&input) ==
               TC2_TRAFFIC_ACTION_STOP_REAR);

        input = base_input();
        input.train_b_braking_envelope_mm = 0;
        input.gap_mm = 2000;
        assert(Tc2TrafficSafetyEvaluate(&input) ==
               TC2_TRAFFIC_ACTION_STOP_REAR);

        input = base_input();
        input.relation =
                TC2_TRAFFIC_RELATION_SAME_DIRECTION_B_REAR;
        input.train_a_route_scalar_mm = -1;
        input.gap_mm = 2000;
        assert(Tc2TrafficSafetyEvaluate(&input) ==
               TC2_TRAFFIC_ACTION_STOP_REAR);
}

static void test_following_hold_requires_safe_front_and_clearance(void) {
        tc2_traffic_safety_input input = base_input();
        input.train_a_effective_speed = 0;
        input.train_a_braking_envelope_mm = 0;
        input.rear_stop_latched = 1;

        input.gap_mm = TC2_TRAFFIC_FOLLOWING_CLEARANCE_MM;
        assert(Tc2TrafficSafetyEvaluate(&input) ==
               TC2_TRAFFIC_ACTION_KEEP_HOLD);
        input.gap_mm = TC2_TRAFFIC_FOLLOWING_CLEARANCE_MM + 1;
        assert(Tc2TrafficSafetyEvaluate(&input) ==
               TC2_TRAFFIC_ACTION_RESUME_REAR);

        input.rear_has_movement_authority = 0;
        assert(Tc2TrafficSafetyEvaluate(&input) ==
               TC2_TRAFFIC_ACTION_KEEP_HOLD);
        input.rear_has_movement_authority = 1;
        input.train_b_effective_speed = 0;
        input.train_b_braking_envelope_mm = 0;
        input.gap_mm = 2000;
        assert(Tc2TrafficSafetyEvaluate(&input) ==
               TC2_TRAFFIC_ACTION_KEEP_HOLD);

        input.train_b_effective_speed = 80;
        input.train_b_braking_envelope_mm = 300;
        input.train_b_route_scalar_mm = -1;
        assert(Tc2TrafficSafetyEvaluate(&input) ==
               TC2_TRAFFIC_ACTION_KEEP_HOLD);
}

static void test_invalid_input_fails_closed(void) {
        tc2_traffic_safety_input input = base_input();
        assert(Tc2TrafficSafetyEvaluate(0) ==
               TC2_TRAFFIC_ACTION_STOP_BOTH);

        input.train_a_route_scalar_mm = -1;
        assert(Tc2TrafficSafetyEvaluate(&input) ==
               TC2_TRAFFIC_ACTION_STOP_BOTH);
        input = base_input();
        input.train_a_effective_speed = -1;
        assert(Tc2TrafficSafetyEvaluate(&input) ==
               TC2_TRAFFIC_ACTION_STOP_BOTH);
        input = base_input();
        input.gap_mm = -1;
        assert(Tc2TrafficSafetyEvaluate(&input) ==
               TC2_TRAFFIC_ACTION_STOP_BOTH);
        input = base_input();
        input.rear_stop_latched = 2;
        assert(Tc2TrafficSafetyEvaluate(&input) ==
               TC2_TRAFFIC_ACTION_STOP_BOTH);
        input = base_input();
        input.rear_has_movement_authority = -1;
        assert(Tc2TrafficSafetyEvaluate(&input) ==
               TC2_TRAFFIC_ACTION_STOP_BOTH);
        input = base_input();
        input.relation = (tc2_traffic_relation)99;
        assert(Tc2TrafficSafetyEvaluate(&input) ==
               TC2_TRAFFIC_ACTION_STOP_BOTH);
}

int main(void) {
        assert(TC2_TRAFFIC_EMERGENCY_CLEARANCE_MM == 250);
        assert(TC2_TRAFFIC_FOLLOWING_CLEARANCE_MM == 350);

        test_head_on_always_stops_both();
        test_following_350_mm_stops_only_rear();
        test_equal_speed_still_rechecks_clearance();
        test_stopped_front_stops_only_rear();
        test_slower_front_rechecks_rear();
        test_front_localization_failure_stops_only_rear();
        test_following_hold_requires_safe_front_and_clearance();
        test_invalid_input_fails_closed();

        puts("tc2_traffic_safety_test: PASS");
        return 0;
}
