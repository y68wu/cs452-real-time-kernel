#include "tc2_traffic_safety.h"

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

tc2_traffic_action Tc2TrafficSafetyEvaluate(
        const tc2_traffic_safety_input *input) {
        if (input == 0) {
                return TC2_TRAFFIC_ACTION_STOP_BOTH;
        }
        if (input->gap_mm < 0 ||
            !canonical_boolean(input->rear_stop_latched) ||
            !canonical_boolean(
                    input->rear_has_movement_authority)) {
                return TC2_TRAFFIC_ACTION_STOP_BOTH;
        }

        if (input->relation == TC2_TRAFFIC_RELATION_HEAD_ON) {
                if (!valid_vehicle(
                            input->train_a_route_scalar_mm,
                            input->train_a_effective_speed,
                            input->train_a_braking_envelope_mm) ||
                    !valid_vehicle(
                            input->train_b_route_scalar_mm,
                            input->train_b_effective_speed,
                            input->train_b_braking_envelope_mm)) {
                        return TC2_TRAFFIC_ACTION_STOP_BOTH;
                }
                /*
                 * A HEAD_ON sample is emitted only after the caller has
                 * proved that both physical bodies occupy one common
                 * directed corridor in opposite directions.  Distance is a
                 * diagnostic severity value, not permission for either body
                 * to keep entering that single-track corridor.  Both remain
                 * stopped until cancel/remove or a completed reroute removes
                 * the relation.
                 */
                return TC2_TRAFFIC_ACTION_STOP_BOTH;
        }

        int rear_route_scalar;
        int rear_speed;
        int rear_braking_envelope;
        int front_route_scalar;
        int front_speed;
        int front_braking_envelope;
        if (input->relation ==
            TC2_TRAFFIC_RELATION_SAME_DIRECTION_A_REAR) {
                rear_route_scalar =
                        input->train_a_route_scalar_mm;
                rear_speed = input->train_a_effective_speed;
                rear_braking_envelope =
                        input->train_a_braking_envelope_mm;
                front_route_scalar =
                        input->train_b_route_scalar_mm;
                front_speed = input->train_b_effective_speed;
                front_braking_envelope =
                        input->train_b_braking_envelope_mm;
        } else if (input->relation ==
                   TC2_TRAFFIC_RELATION_SAME_DIRECTION_B_REAR) {
                rear_route_scalar =
                        input->train_b_route_scalar_mm;
                rear_speed = input->train_b_effective_speed;
                rear_braking_envelope =
                        input->train_b_braking_envelope_mm;
                front_route_scalar =
                        input->train_a_route_scalar_mm;
                front_speed = input->train_a_effective_speed;
                front_braking_envelope =
                        input->train_a_braking_envelope_mm;
        } else {
                return TC2_TRAFFIC_ACTION_STOP_BOTH;
        }

        /*
         * If the rear train itself cannot be localized or stopped, the
         * requested rear/front ordering is no longer trustworthy.  Preserve
         * the module's global fail-closed result for that case.  A bad front
         * observation is different: the topology has already identified the
         * rear, so only that rear train must be prevented from advancing.
         */
        if (!valid_vehicle(
                    rear_route_scalar, rear_speed,
                    rear_braking_envelope)) {
                return TC2_TRAFFIC_ACTION_STOP_BOTH;
        }
        int front_valid = valid_vehicle(
                front_route_scalar, front_speed,
                front_braking_envelope);

        if (input->rear_stop_latched) {
                if (rear_speed != 0 ||
                    !input->rear_has_movement_authority ||
                    !front_valid ||
                    front_speed == 0 ||
                    input->gap_mm <=
                            TC2_TRAFFIC_FOLLOWING_CLEARANCE_MM) {
                        return TC2_TRAFFIC_ACTION_KEEP_HOLD;
                }
                return TC2_TRAFFIC_ACTION_RESUME_REAR;
        }

        /*
         * Every same-direction sample rechecks the identified rear train.
         * The current public input has no previous-speed field, so the
         * observable conservative deceleration condition is that the leader
         * is now slower than the follower.  A stopped or unlocalized leader
         * is fail-closed in exactly the same rear-only direction.  Equality
         * at 350 mm is unsafe; only a strictly larger clearance is clear.
         */
        if (!front_valid ||
            front_speed == 0 ||
            front_speed < rear_speed ||
            input->gap_mm <=
                    TC2_TRAFFIC_FOLLOWING_CLEARANCE_MM) {
                return TC2_TRAFFIC_ACTION_STOP_REAR;
        }
        return TC2_TRAFFIC_ACTION_NONE;
}
