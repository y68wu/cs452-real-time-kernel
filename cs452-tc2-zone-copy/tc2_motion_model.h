#ifndef _tc2_motion_model_h_
#define _tc2_motion_model_h_ 1

/*
 * Conservative fallback model for a 10 ms clock tick.  Velocity is stored in
 * tenths of a millimetre per tick so the historical 12..101 seed table is not
 * accidentally interpreted as the physically impossible 12..101 mm/tick.
 * The values are mirrored in calibration/tc2_seed_model.csv.
 */
int Tc2MotionModelIndex(int speed);
int Tc2MotionVelocityTenthsMmPerTick(int speed);
int Tc2MotionFastVelocityTenthsMmPerTick(int speed);
int Tc2MotionSlowVelocityTenthsMmPerTick(int speed);
int Tc2MotionNominalTravelTicks(int speed, int distance_mm);
int Tc2MotionFastTravelTicks(int speed, int distance_mm);
int Tc2MotionSlowTravelTicks(int speed, int distance_mm);

/* Compatibility/display helper: nominal velocity rounded up to whole mm. */
int Tc2MotionVelocityMmPerTick(int speed);
int Tc2MotionStopDistanceMm(int speed);
int Tc2MotionUncertaintyMm(int speed);
int Tc2MotionBrakingDistanceMm(int speed);
/*
 * Conservative fallback time from an acknowledged speed-zero command until
 * the train may be treated as stationary. Physical calibration must replace
 * the seed table for each train/direction used on Track D.
 */
int Tc2MotionStopSettleTicks(int speed);
int Tc2MotionWatchdogMarginTicks(int speed);

#ifdef MODE_TC2

/*
 * Track-D train-14 prediction model.
 *
 * The velocity curve preserves the monotone shape of the historical seed
 * table and is normalized to the measured E7-to-D7 speed-20 interval:
 *
 *   376 mm / 787 ticks = 478 um per 10 ms tick
 *
 * Point stopping distances interpolate the measured command-to-rest values
 * at speeds 14, 20, 30, 40, 60, 80, 100, and 120.  These functions remain
 * separate from the larger reservation safety envelope.
 */
int Tc2MotionProvisionalVelocityUmPerTick(int speed);
/*
 * Runtime point-model helpers.  A valid ordered sensor interval supplies a
 * train/track-specific velocity observation; callers retain the provisional
 * speed curve only until such an observation exists.
 */
int Tc2MotionObservedVelocityUmPerTick(
        int distance_mm, int elapsed_ticks);
int Tc2MotionTravelTicksUmAtVelocity(
        int velocity_um_per_tick, int distance_um);
int Tc2MotionStopDistanceForVelocityUm(
        int velocity_um_per_tick);
int Tc2MotionProvisionalFastTravelTicks(int speed, int distance_mm);
int Tc2MotionProvisionalNominalTravelTicks(int speed, int distance_mm);
int Tc2MotionProvisionalNominalTravelTicksUm(
        int speed, int distance_um);
int Tc2MotionProvisionalSlowTravelTicks(int speed, int distance_mm);
/* Exact micrometre interpolation of the measured command-to-rest curve. */
int Tc2MotionMeasuredStopDistanceUm(int speed);
int Tc2MotionProvisionalStopDistanceUm(int speed);
int Tc2MotionProvisionalStopDistanceMm(int speed);
/* Retained compatibility hook; endpoint corrections are now zero because
 * d1-d8 geometry is represented directly by measured directed offsets. */
int Tc2MotionD7EndpointCorrectionMm(int speed);
/*
 * Generic TC2 prediction uses the same measured curve as a transferred
 * prior. This wrapper makes that evidence transfer explicit at call sites;
 * it does not claim that other trains/routes are calibrated.
 */
int Tc2MotionTransferredEndpointCorrectionMm(int speed);
/*
 * Route/train-specific residual layered on the transferred prior.  A
 * physical run may refine one tuple without moving the immutable operator
 * endpoint geometry or silently changing every other route.
 */
int Tc2MotionRouteEndpointCorrectionMm(
        int train, int start_index, int destination_index, int speed);

#endif

#endif
