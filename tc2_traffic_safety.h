#ifndef _tc2_traffic_safety_h_
#define _tc2_traffic_safety_h_ 1

/*
 * Pure dynamic-traffic policy.  This module owns no train, route, turnout, or
 * reservation state and performs no I/O.  Callers supply a topology-aware gap
 * and latch the returned stop/hold state in their runtime.
 */
/* Diagnostic threshold for an already-invalid physical proximity. */
#define TC2_TRAFFIC_EMERGENCY_CLEARANCE_MM 250
#define TC2_TRAFFIC_FOLLOWING_CLEARANCE_MM 350

typedef enum {
        TC2_TRAFFIC_RELATION_SAME_DIRECTION_A_REAR = 0,
        TC2_TRAFFIC_RELATION_SAME_DIRECTION_B_REAR,
        TC2_TRAFFIC_RELATION_HEAD_ON
} tc2_traffic_relation;

typedef enum {
        TC2_TRAFFIC_ACTION_NONE = 0,
        TC2_TRAFFIC_ACTION_STOP_REAR,
        TC2_TRAFFIC_ACTION_STOP_BOTH,
        TC2_TRAFFIC_ACTION_KEEP_HOLD,
        TC2_TRAFFIC_ACTION_RESUME_REAR
} tc2_traffic_action;

typedef struct {
        /*
         * Directed route scalars are retained so callers cannot accidentally
         * evaluate an unlocalized train.  They need not share an origin:
         * gap_mm is the caller's topology-aware physical separation.
         */
        int train_a_route_scalar_mm;
        int train_b_route_scalar_mm;
        int train_a_effective_speed;
        int train_b_effective_speed;
        int train_a_braking_envelope_mm;
        int train_b_braking_envelope_mm;
        int gap_mm;
        tc2_traffic_relation relation;

        /*
         * rear_stop_latched identifies a prior same-direction following stop.
         * A stopped rear train may resume only with movement authority.
         * Both fields must be canonical booleans (zero or one).
         */
        int rear_stop_latched;
        int rear_has_movement_authority;
} tc2_traffic_safety_input;

/*
 * Boundary rules:
 *
 * - Complete next-authority remains the primary route exclusion mechanism.
 * - A topology-confirmed opposing/common-corridor relation stops both trains;
 *   250 mm remains the emergency proximity reported by the audit, not an
 *   authorization boundary.
 * - Same-direction clearance at or below 350 mm stops only the rear train; the
 *   leader is never stopped merely because a follower is close.
 * - A following-latched rear stays held until 350 mm is restored and its
 *   next physical authority is still valid.
 * - Invalid input returns STOP_BOTH (fail closed).
 */
tc2_traffic_action Tc2TrafficSafetyEvaluate(
        const tc2_traffic_safety_input *input);

#endif
