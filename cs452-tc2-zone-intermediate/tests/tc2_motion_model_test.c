#include <assert.h>
#include <stdio.h>

#include "tc2_motion_model.h"

int main(void) {
        assert(Tc2MotionModelIndex(0) == -1);
        assert(Tc2MotionModelIndex(121) == -1);
        assert(Tc2MotionModelIndex(1) == 1);
        assert(Tc2MotionModelIndex(120) == 14);

        int previous_velocity = 0;
        int previous_fast = 0;
        int previous_slow = 0;
        int previous_stop = 0;
        int previous_braking = 0;
        for (int speed = 1; speed <= 120; ++speed) {
                int velocity =
                        Tc2MotionVelocityTenthsMmPerTick(speed);
                int fast =
                        Tc2MotionFastVelocityTenthsMmPerTick(speed);
                int slow =
                        Tc2MotionSlowVelocityTenthsMmPerTick(speed);
                int stop = Tc2MotionStopDistanceMm(speed);
                int uncertainty = Tc2MotionUncertaintyMm(speed);
                int braking = Tc2MotionBrakingDistanceMm(speed);

                assert(velocity > 0);
                assert(slow > 0);
                assert(slow <= velocity);
                assert(velocity <= fast);
                assert(stop > 0);
                assert(uncertainty >= 250);
                assert(braking >= stop + uncertainty);
                assert(velocity >= previous_velocity);
                assert(fast >= previous_fast);
                assert(slow >= previous_slow);
                assert(stop >= previous_stop);
                assert(braking >= previous_braking);
                assert(Tc2MotionWatchdogMarginTicks(speed) >= 60);

                previous_velocity = velocity;
                previous_fast = fast;
                previous_slow = slow;
                previous_stop = stop;
                previous_braking = braking;
        }

        assert(Tc2MotionVelocityTenthsMmPerTick(120) == 101);
        assert(Tc2MotionVelocityMmPerTick(120) == 11);
        assert(Tc2MotionFastVelocityTenthsMmPerTick(120) == 122);
        assert(Tc2MotionSlowVelocityTenthsMmPerTick(120) == 50);
        assert(Tc2MotionFastTravelTicks(120, 1000) == 82);
        assert(Tc2MotionNominalTravelTicks(120, 1000) == 100);
        assert(Tc2MotionSlowTravelTicks(120, 1000) == 200);
        assert(Tc2MotionFastTravelTicks(0, 1000) == -1);
        assert(Tc2MotionSlowTravelTicks(120, -1) == -1);
        assert(Tc2MotionStopDistanceMm(20) == 50);
        assert(Tc2MotionStopDistanceMm(40) == 110);
        assert(Tc2MotionStopDistanceMm(60) == 240);
        assert(Tc2MotionStopDistanceMm(70) == 390);
        assert(Tc2MotionStopDistanceMm(80) == 530);
        assert(Tc2MotionStopDistanceMm(90) == 740);
        assert(Tc2MotionStopDistanceMm(100) == 875);
        assert(Tc2MotionStopDistanceMm(120) == 1435);
        assert(Tc2MotionBrakingDistanceMm(120) == 1905);
        assert(Tc2MotionStopSettleTicks(120) ==
               Tc2MotionSlowTravelTicks(120, 1905) + 20);
        assert(Tc2MotionStopSettleTicks(120) > 100);
        assert(Tc2MotionStopSettleTicks(0) == -1);

        assert(Tc2MotionProvisionalVelocityUmPerTick(0) == -1);
        assert(Tc2MotionProvisionalVelocityUmPerTick(121) == -1);
        assert(Tc2MotionProvisionalStopDistanceUm(0) == -1);
        assert(Tc2MotionProvisionalStopDistanceMm(0) == -1);
        assert(Tc2MotionD7EndpointCorrectionMm(0) == -1);
        assert(Tc2MotionD7EndpointCorrectionMm(121) == -1);
        assert(Tc2MotionProvisionalNominalTravelTicks(20, -1) == -1);
        assert(Tc2MotionProvisionalNominalTravelTicksUm(20, -1) == -1);
        assert(Tc2MotionProvisionalFastTravelTicks(0, 1000) == -1);
        assert(Tc2MotionProvisionalSlowTravelTicks(121, 1000) == -1);
        assert(Tc2MotionProvisionalNominalTravelTicks(20, 0) == 0);
        assert(Tc2MotionObservedVelocityUmPerTick(100, 250) == 400);
        assert(Tc2MotionObservedVelocityUmPerTick(0, 250) == -1);
        assert(Tc2MotionObservedVelocityUmPerTick(100, 0) == -1);
        assert(Tc2MotionTravelTicksUmAtVelocity(400, 100001) == 251);
        assert(Tc2MotionTravelTicksUmAtVelocity(0, 1000) == -1);
        assert(Tc2MotionStopDistanceForVelocityUm(478) == 50000);
        assert(Tc2MotionStopDistanceForVelocityUm(0x7fffffff) == -1);

        int previous_provisional_velocity = 0;
        int previous_provisional_stop = 0;
        int previous_provisional_stop_um = 0;
        for (int speed = 1; speed <= 120; ++speed) {
                int velocity =
                        Tc2MotionProvisionalVelocityUmPerTick(speed);
                int stop =
                        Tc2MotionProvisionalStopDistanceMm(speed);
                int stop_um =
                        Tc2MotionProvisionalStopDistanceUm(speed);
                int fast =
                        Tc2MotionProvisionalFastTravelTicks(
                                speed, 1000);
                int nominal =
                        Tc2MotionProvisionalNominalTravelTicks(
                                speed, 1000);
                int slow =
                        Tc2MotionProvisionalSlowTravelTicks(
                                speed, 1000);
                int endpoint_correction =
                        Tc2MotionD7EndpointCorrectionMm(speed);
                assert(velocity > 0);
                assert(stop > 0);
                assert(stop_um > 0);
                assert(velocity >= previous_provisional_velocity);
                assert(stop >= previous_provisional_stop);
                assert(stop_um >= previous_provisional_stop_um);
                assert(stop == (stop_um + 999) / 1000);
                assert(fast > 0);
                assert(fast <= nominal);
                assert(nominal <= slow);
                assert(endpoint_correction == 0);
                /*
                 * Every supported speed must issue a micrometre-precision
                 * point command that reaches a 2000 mm scalar endpoint
                 * reference and overshoots by less than one velocity tick.
                 */
                int point_ticks =
                        Tc2MotionProvisionalNominalTravelTicksUm(
                                speed, 2000000 - stop_um);
                unsigned long long point_endpoint_um =
                        (unsigned long long)(unsigned int)point_ticks *
                                (unsigned int)velocity +
                        (unsigned int)stop_um;
                assert(point_ticks >= 0);
                assert(point_endpoint_um >= 2000000);
                assert(point_endpoint_um - 2000000 <
                       (unsigned int)velocity);
                /*
                 * The same runtime helpers must accept an observed velocity
                 * at every supported command speed.  This is deliberately
                 * not a speed-20 branch.
                 */
                assert(Tc2MotionObservedVelocityUmPerTick(
                               velocity, 1000) == velocity);
                assert(Tc2MotionStopDistanceForVelocityUm(velocity) ==
                       stop_um);
                previous_provisional_velocity = velocity;
                previous_provisional_stop = stop;
                previous_provisional_stop_um = stop_um;
        }
        assert(Tc2MotionProvisionalVelocityUmPerTick(1) == 32);
        assert(Tc2MotionProvisionalStopDistanceMm(1) == 3);
        assert(Tc2MotionProvisionalVelocityUmPerTick(20) == 478);
        assert(Tc2MotionProvisionalStopDistanceUm(20) == 50000);
        assert(Tc2MotionMeasuredStopDistanceUm(20) == 50000);
        assert(Tc2MotionProvisionalStopDistanceMm(20) == 50);
        assert(Tc2MotionD7EndpointCorrectionMm(1) == 0);
        assert(Tc2MotionD7EndpointCorrectionMm(20) == 0);
        assert(Tc2MotionD7EndpointCorrectionMm(40) == 0);
        assert(Tc2MotionD7EndpointCorrectionMm(60) == 0);
        assert(Tc2MotionD7EndpointCorrectionMm(80) == 0);
        assert(Tc2MotionD7EndpointCorrectionMm(100) == 0);
        assert(Tc2MotionD7EndpointCorrectionMm(120) == 0);
        assert(Tc2MotionTransferredEndpointCorrectionMm(20) == 0);
        assert(Tc2MotionTransferredEndpointCorrectionMm(100) == 0);
        assert(Tc2MotionTransferredEndpointCorrectionMm(120) == 0);
        assert(Tc2MotionRouteEndpointCorrectionMm(
                       14, 0, 0, 80) == 0);
        assert(Tc2MotionRouteEndpointCorrectionMm(
                       14, 0, 0, 100) == 0);
        assert(Tc2MotionRouteEndpointCorrectionMm(
                       14, 0, 1, 80) == 0);
        assert(Tc2MotionRouteEndpointCorrectionMm(
                       15, 0, 0, 80) == 0);
        assert(Tc2MotionRouteEndpointCorrectionMm(
                       14, 0, 0, 0) == -1);
        assert(Tc2MotionProvisionalFastTravelTicks(20, 1000) == 1744);
        assert(Tc2MotionProvisionalNominalTravelTicks(20, 1000) == 2093);
        assert(Tc2MotionProvisionalSlowTravelTicks(20, 1000) == 4185);
        assert(Tc2MotionProvisionalVelocityUmPerTick(120) == 2414);
        assert(Tc2MotionProvisionalStopDistanceUm(120) == 1435000);
        assert(Tc2MotionMeasuredStopDistanceUm(120) == 1435000);
        assert(Tc2MotionProvisionalStopDistanceMm(120) == 1435);
        /*
         * E7 -> the physical D7 anchor is 376 mm. Whole-mm rounding issued
         * speed zero two ticks too early; micrometre arithmetic must cross
         * that anchor by a small, bounded amount under the point model.
         */
        int e7_d7_ticks =
                Tc2MotionProvisionalNominalTravelTicksUm(
                        20, 376000 - 50000);
        assert(e7_d7_ticks == 683);
        int predicted_endpoint_um =
                e7_d7_ticks * 478 + 50000;
        assert(predicted_endpoint_um == 376474);
        assert(predicted_endpoint_um >= 376000);
        assert(predicted_endpoint_um - 376000 < 478);

        puts("validated conservative TC2 safety bounds and provisional 1..120 point predictions");
        return 0;
}
