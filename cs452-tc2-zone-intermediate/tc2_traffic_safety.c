#include "tc2_traffic_safety.h"

#include <limits.h>

static int canonical_boolean(int value) {
        return value == 0 || value == 1;
}

static int valid_vehicle(
        int route_scalar_mm,
        int effective_speed,
        int braking_envelope_mm) {
        if (route_scalar_mm < 0 ||
            effective_speed < 0 ||
            braking_envelope_mm < 0) {
                return 0;
        }

        /*
         * A non-zero effective speed without any braking envelope would let
         * the policy declare an unsafe gap clear.
         */
        return effective_speed == 0 || braking_envelope_mm > 0;
}

static int saturated_add(int left, int right) {
        if (left > INT_MAX - right) return INT_MAX;
        return left + right;
}

static int following_stop_threshold_mm(int rear_envelope_mm) {
        return saturated_add(
                rear_envelope_mm,
                TC2_TRAFFIC_FOLLOWING_STOP_CLEARANCE_MM);
}

tc2_traffic_action Tc2TrafficSafetyEvaluate(
        const tc2_traffic_safety_input *input) {
        if (input == 0 ||
            !valid_vehicle(
                    input->train_a_route_scalar_mm,
                    input->train_a_effective_speed,
                    input->train_a_braking_envelope_mm) ||
            !valid_vehicle(
                    input->train_b_route_scalar_mm,
                    input->train_b_effective_speed,
                    input->train_b_braking_envelope_mm) ||
            input->gap_mm < 0 ||
            !canonical_boolean(input->rear_stop_latched) ||
            !canonical_boolean(
                    input->rear_has_movement_authority)) {
                return TC2_TRAFFIC_ACTION_STOP_BOTH;
        }

        if (input->relation == TC2_TRAFFIC_RELATION_HEAD_ON) {
                int threshold = saturated_add(
                        input->train_a_braking_envelope_mm,
                        input->train_b_braking_envelope_mm);
                threshold = saturated_add(
                        threshold,
                        TC2_TRAFFIC_HEAD_ON_EXTRA_CLEARANCE_MM);
                return input->gap_mm <= threshold
                               ? TC2_TRAFFIC_ACTION_STOP_BOTH
                               : TC2_TRAFFIC_ACTION_NONE;
        }

        int rear_speed;
        int rear_envelope;
        if (input->relation ==
            TC2_TRAFFIC_RELATION_SAME_DIRECTION_A_REAR) {
                rear_speed = input->train_a_effective_speed;
                rear_envelope =
                        input->train_a_braking_envelope_mm;
        } else if (input->relation ==
                   TC2_TRAFFIC_RELATION_SAME_DIRECTION_B_REAR) {
                rear_speed = input->train_b_effective_speed;
                rear_envelope =
                        input->train_b_braking_envelope_mm;
        } else {
                return TC2_TRAFFIC_ACTION_STOP_BOTH;
        }

        if (input->rear_stop_latched) {
                if (rear_speed != 0 ||
                    !input->rear_has_movement_authority ||
                    input->gap_mm <
                            TC2_TRAFFIC_FOLLOWING_RESUME_CLEARANCE_MM) {
                        return TC2_TRAFFIC_ACTION_KEEP_HOLD;
                }
                return TC2_TRAFFIC_ACTION_RESUME_REAR;
        }

        return input->gap_mm <=
                               following_stop_threshold_mm(
                                       rear_envelope)
                       ? TC2_TRAFFIC_ACTION_STOP_REAR
                       : TC2_TRAFFIC_ACTION_NONE;
}
