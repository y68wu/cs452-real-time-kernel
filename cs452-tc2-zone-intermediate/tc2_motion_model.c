#include "tc2_motion_model.h"

/*
 * Historical values are centimetres/second, numerically identical to tenths
 * of a millimetre per 10 ms tick:
 *
 *   1 cm/s = 0.1 mm/10 ms
 */
static const int tc2_velocity_tenths_mm_per_tick[15] = {
        0, 12, 18, 24, 31,
        38, 45, 52, 59, 66,
        73, 80, 87, 94, 101
};

/*
 * Track-D train 14 command-to-rest measurements, supplied directly in
 * millimetres. Linear interpolation keeps every accepted command speed
 * 1..120 monotone between the measured knots.
 */
static const int tc2_measured_speed_knots[] = {
        0, 20, 40, 60, 70, 80, 90, 100, 120
};

static const int tc2_measured_stop_um[] = {
        0, 50000, 110000, 240000, 390000,
        530000, 740000, 875000, 1435000
};

/*
 * Never shrink an already deployed collision/reservation envelope merely
 * because a later physical run produced a shorter point stopping distance.
 * These legacy values are safety floors only; they are not used to choose
 * the actual speed-zero command point.
 */
static const int tc2_safety_floor_speed_knots[] = {
        0, 14, 20, 30, 40, 60, 80, 100, 120
};

static const int tc2_safety_floor_stop_um[] = {
        0, 38100, 50800, 63500, 127000,
        342900, 812800, 1168400, 1524000
};

enum {
        TC2_MEASURED_STOP_KNOT_COUNT =
                sizeof(tc2_measured_speed_knots) /
                sizeof(tc2_measured_speed_knots[0]),
        TC2_SAFETY_FLOOR_KNOT_COUNT =
                sizeof(tc2_safety_floor_speed_knots) /
                sizeof(tc2_safety_floor_speed_knots[0])
};

static int interpolate_curve_um(
        int speed, const int *speed_knots,
        const int *distance_um, int knot_count) {
        if (speed < 0 || speed > 120) return -1;
        if (speed == 0) return 0;
        int upper = 1;
        while (upper < knot_count &&
               speed > speed_knots[upper]) {
                ++upper;
        }
        if (upper >= knot_count) return -1;
        int lower = upper - 1;
        int speed_span =
                speed_knots[upper] -
                speed_knots[lower];
        int speed_offset =
                speed - speed_knots[lower];
        int stop_low = distance_um[lower];
        int stop_delta =
                distance_um[upper] - stop_low;
        return stop_low +
                (stop_delta * speed_offset +
                 speed_span - 1) / speed_span;
}

static int interpolate_measured_stop_um(int speed) {
        return interpolate_curve_um(
                speed, tc2_measured_speed_knots,
                tc2_measured_stop_um,
                TC2_MEASURED_STOP_KNOT_COUNT);
}

static int interpolate_safety_floor_stop_mm(int speed) {
        int distance_um = interpolate_curve_um(
                speed, tc2_safety_floor_speed_knots,
                tc2_safety_floor_stop_um,
                TC2_SAFETY_FLOOR_KNOT_COUNT);
        return distance_um < 0 ? -1 :
                (distance_um + 999) / 1000;
}

int Tc2MotionModelIndex(int speed) {
        if (speed < 1 || speed > 120) return -1;

        int index = (speed * 14 + 60) / 120;
        if (index < 1) index = 1;
        if (index > 14) index = 14;
        return index;
}

int Tc2MotionVelocityMmPerTick(int speed) {
        int velocity = Tc2MotionVelocityTenthsMmPerTick(speed);
        return velocity < 0 ? -1 : (velocity + 9) / 10;
}

int Tc2MotionVelocityTenthsMmPerTick(int speed) {
        int index = Tc2MotionModelIndex(speed);
        return index < 0 ? -1 :
                tc2_velocity_tenths_mm_per_tick[index];
}

int Tc2MotionFastVelocityTenthsMmPerTick(int speed) {
        int velocity = Tc2MotionVelocityTenthsMmPerTick(speed);
        if (velocity < 0) return -1;
        return velocity + (velocity + 4) / 5;
}

int Tc2MotionSlowVelocityTenthsMmPerTick(int speed) {
        int velocity = Tc2MotionVelocityTenthsMmPerTick(speed);
        if (velocity < 0) return -1;
        velocity /= 2;
        return velocity > 0 ? velocity : 1;
}

static int travel_ticks(int distance_mm, int velocity_tenths) {
        if (distance_mm < 0 || velocity_tenths <= 0 ||
            distance_mm > 0x0ccccccc) {
                return -1;
        }
        return (distance_mm * 10 + velocity_tenths - 1) /
                velocity_tenths;
}

int Tc2MotionNominalTravelTicks(int speed, int distance_mm) {
        return travel_ticks(
                distance_mm,
                Tc2MotionVelocityTenthsMmPerTick(speed));
}

int Tc2MotionFastTravelTicks(int speed, int distance_mm) {
        return travel_ticks(
                distance_mm,
                Tc2MotionFastVelocityTenthsMmPerTick(speed));
}

int Tc2MotionSlowTravelTicks(int speed, int distance_mm) {
        return travel_ticks(
                distance_mm,
                Tc2MotionSlowVelocityTenthsMmPerTick(speed));
}

int Tc2MotionStopDistanceMm(int speed) {
        if (speed < 1 || speed > 120) return -1;
        int distance_um = interpolate_measured_stop_um(speed);
        return distance_um < 0 ? -1 :
                (distance_um + 999) / 1000;
}

int Tc2MotionUncertaintyMm(int speed) {
        int stop_distance = Tc2MotionStopDistanceMm(speed);
        if (stop_distance < 0) return -1;

        int quarter_stop = stop_distance / 4;
        return quarter_stop > 250 ? quarter_stop : 250;
}

int Tc2MotionBrakingDistanceMm(int speed) {
        int stop_distance = Tc2MotionStopDistanceMm(speed);
        int uncertainty = Tc2MotionUncertaintyMm(speed);
        int safety_floor_stop =
                interpolate_safety_floor_stop_mm(speed);
        if (stop_distance < 0 || uncertainty < 0 ||
            safety_floor_stop < 0) {
                return -1;
        }
        int safety_floor_uncertainty =
                safety_floor_stop / 4;
        if (safety_floor_uncertainty < 250) {
                safety_floor_uncertainty = 250;
        }
        int measured_envelope =
                stop_distance + uncertainty;
        int safety_floor_envelope =
                safety_floor_stop +
                safety_floor_uncertainty;
        return measured_envelope > safety_floor_envelope ?
                measured_envelope : safety_floor_envelope;
}

int Tc2MotionStopSettleTicks(int speed) {
        int braking_distance = Tc2MotionBrakingDistanceMm(speed);
        int settle_ticks;
        if (braking_distance < 0) return -1;
        /*
         * The slow bound is half nominal velocity. Traversing the complete
         * stop-plus-uncertainty envelope at that bound is at least the
         * constant-deceleration 2d/v seed estimate and is deliberately more
         * conservative than the old fixed one-second delay.
         */
        settle_ticks = Tc2MotionSlowTravelTicks(
                speed, braking_distance);
        if (settle_ticks < 0 ||
            settle_ticks > 0x7fffffff - 20) {
                return -1;
        }
        return settle_ticks + 20;
}

int Tc2MotionWatchdogMarginTicks(int speed) {
        if (Tc2MotionModelIndex(speed) < 0) return -1;
        return speed <= 20 ? 100 : 60;
}

#ifdef MODE_TC2

/*
 * Keep the original curve knots explicit so interpolation does not inherit
 * the nearest-neighbour quantization used by the conservative safety model.
 */
static const int tc2_provisional_speed_knots[15] = {
        0, 9, 17, 26, 34,
        43, 51, 60, 69, 77,
        86, 94, 103, 111, 120
};

#define TC2_PROVISIONAL_ANCHOR_SEED_TENTHS 20
#define TC2_PROVISIONAL_ANCHOR_VELOCITY_UM_PER_TICK 478

int Tc2MotionProvisionalVelocityUmPerTick(int speed) {
        if (speed < 1 || speed > 120) return -1;

        int upper = 1;
        while (upper < 15 &&
               speed > tc2_provisional_speed_knots[upper]) {
                ++upper;
        }
        if (upper >= 15) return -1;

        int lower = upper - 1;
        int speed_span =
                tc2_provisional_speed_knots[upper] -
                tc2_provisional_speed_knots[lower];
        int speed_offset =
                speed - tc2_provisional_speed_knots[lower];
        int seed_low =
                tc2_velocity_tenths_mm_per_tick[lower];
        int seed_delta =
                tc2_velocity_tenths_mm_per_tick[upper] -
                seed_low;
        int seed_numerator =
                seed_low * speed_span +
                seed_delta * speed_offset;
        int denominator =
                speed_span *
                TC2_PROVISIONAL_ANCHOR_SEED_TENTHS;
        int numerator =
                seed_numerator *
                TC2_PROVISIONAL_ANCHOR_VELOCITY_UM_PER_TICK;
        return (numerator + denominator - 1) / denominator;
}

int Tc2MotionObservedVelocityUmPerTick(
        int distance_mm, int elapsed_ticks) {
        if (distance_mm <= 0 || elapsed_ticks <= 0 ||
            distance_mm > 0x7fffffff / 1000) {
                return -1;
        }
        unsigned long long distance_um =
                (unsigned long long)(unsigned int)distance_mm * 1000u;
        unsigned long long velocity =
                (distance_um + (unsigned int)elapsed_ticks / 2u) /
                (unsigned int)elapsed_ticks;
        return velocity > 0 && velocity <= 0x7fffffffu ?
                (int)velocity : -1;
}

int Tc2MotionTravelTicksUmAtVelocity(
        int velocity_um_per_tick, int distance_um) {
        if (velocity_um_per_tick < 1 || distance_um < 0) return -1;
        unsigned long long ticks =
                ((unsigned long long)(unsigned int)distance_um +
                 (unsigned int)velocity_um_per_tick - 1u) /
                (unsigned int)velocity_um_per_tick;
        return ticks <= 0x7fffffffu ? (int)ticks : -1;
}

int Tc2MotionStopDistanceForVelocityUm(
        int velocity_um_per_tick) {
        if (velocity_um_per_tick < 1) return -1;
        int previous_velocity =
                Tc2MotionProvisionalVelocityUmPerTick(1);
        int previous_stop = interpolate_measured_stop_um(1);
        if (previous_velocity < 1 || previous_stop < 0) return -1;
        if (velocity_um_per_tick <= previous_velocity) {
                return (int)(((unsigned long long)previous_stop *
                              (unsigned int)velocity_um_per_tick +
                              (unsigned int)previous_velocity - 1u) /
                             (unsigned int)previous_velocity);
        }
        for (int speed = 2; speed <= 120; ++speed) {
                int velocity =
                        Tc2MotionProvisionalVelocityUmPerTick(speed);
                int stop = interpolate_measured_stop_um(speed);
                if (velocity < previous_velocity || stop < previous_stop) {
                        return -1;
                }
                if (velocity_um_per_tick <= velocity) {
                        if (velocity == previous_velocity) return stop;
                        int span = velocity - previous_velocity;
                        int offset =
                                velocity_um_per_tick -
                                previous_velocity;
                        return previous_stop +
                                (int)(((unsigned long long)
                                              (unsigned int)
                                                      (stop -
                                                       previous_stop) *
                                      (unsigned int)offset +
                                      (unsigned int)span - 1u) /
                                     (unsigned int)span);
                }
                previous_velocity = velocity;
                previous_stop = stop;
        }
        unsigned long long observed =
                (unsigned int)velocity_um_per_tick;
        unsigned long long calibrated =
                (unsigned int)previous_velocity;
        if (observed >
                    0xffffffffffffffffull /
                            observed /
                            (unsigned int)previous_stop) {
                return -1;
        }
        unsigned long long extrapolated =
                (unsigned long long)(unsigned int)previous_stop *
                observed * observed /
                (calibrated * calibrated);
        return extrapolated <= 0x7fffffffu ?
                (int)extrapolated : -1;
}

static int provisional_travel_ticks(
        int speed, int distance_mm, int velocity_scale_numerator,
        int velocity_scale_denominator) {
        int velocity = Tc2MotionProvisionalVelocityUmPerTick(speed);
        if (velocity < 1 || distance_mm < 0 ||
            velocity_scale_numerator < 1 ||
            velocity_scale_denominator < 1) {
                return -1;
        }
        unsigned long long scaled_velocity =
                (unsigned long long)velocity *
                (unsigned int)velocity_scale_numerator;
        unsigned long long distance_um_scaled =
                (unsigned long long)(unsigned int)distance_mm *
                1000u *
                (unsigned int)velocity_scale_denominator;
        unsigned long long ticks =
                (distance_um_scaled + scaled_velocity - 1u) /
                scaled_velocity;
        return ticks <= 0x7fffffffu ? (int)ticks : -1;
}

int Tc2MotionProvisionalFastTravelTicks(
        int speed, int distance_mm) {
        return provisional_travel_ticks(speed, distance_mm, 6, 5);
}

int Tc2MotionProvisionalNominalTravelTicks(
        int speed, int distance_mm) {
        return provisional_travel_ticks(speed, distance_mm, 1, 1);
}

int Tc2MotionProvisionalNominalTravelTicksUm(
        int speed, int distance_um) {
        int velocity = Tc2MotionProvisionalVelocityUmPerTick(speed);
        return Tc2MotionTravelTicksUmAtVelocity(velocity, distance_um);
}

int Tc2MotionProvisionalSlowTravelTicks(
        int speed, int distance_mm) {
        return provisional_travel_ticks(speed, distance_mm, 1, 2);
}

int Tc2MotionMeasuredStopDistanceUm(int speed) {
        if (speed < 1 || speed > 120) return -1;
        return interpolate_measured_stop_um(speed);
}

int Tc2MotionProvisionalStopDistanceUm(int speed) {
        return Tc2MotionMeasuredStopDistanceUm(speed);
}

int Tc2MotionProvisionalStopDistanceMm(int speed) {
        int distance_um =
                Tc2MotionProvisionalStopDistanceUm(speed);
        return distance_um < 0 ? -1 :
                (distance_um + 999) / 1000;
}

int Tc2MotionD7EndpointCorrectionMm(int speed) {
        return Tc2MotionProvisionalVelocityUmPerTick(speed) < 0 ?
                -1 : 0;
}

int Tc2MotionTransferredEndpointCorrectionMm(int speed) {
        return Tc2MotionD7EndpointCorrectionMm(speed);
}

int Tc2MotionRouteEndpointCorrectionMm(
        int train, int start_index, int destination_index, int speed) {
        if (train < 1 || train > 255 ||
            start_index < 0 || start_index >= 6 ||
            destination_index < 0 || destination_index >= 8 ||
            speed < 1 || speed > 120) {
                return -1;
        }
        return Tc2MotionTransferredEndpointCorrectionMm(speed);
}

#endif
