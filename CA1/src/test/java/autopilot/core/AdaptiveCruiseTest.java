
package autopilot.core;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class AdaptiveCruiseTest {
        
    @BeforeEach
    void setUp() {
        AdaptiveCruise cruise = new AdaptiveCruise(100.0);
    }

    private static final double DELTA = 1e-9;

    @Test
    void Should_ReturnExpectedSafeDistance_When_GivenValidSpeed() {
        assertEquals(40.0, cruise.safeDistanceFor(80.0), DELTA);
    }

    @Test
    void Should_ReturnZero_When_SpeedIsZero() {
        assertEquals(0.0, cruise.safeDistanceFor(0.0), DELTA);
    }

    @Test
    void Should_ThrowException_When_SpeedIsInfinite() {
        assertThrows(
                IllegalArgumentException.class,
                () -> cruise.safeDistanceFor(Double.POSITIVE_INFINITY)
        );
    }

    @Test
    void Should_ThrowException_When_SpeedIsNegative() {
        assertThrows(
                IllegalArgumentException.class,
                () -> cruise.safeDistanceFor(-10.0)
        );
    }

    // --------------------------------------------------
    // update: validates speed and distance
    // Tests validateSpeed and validateDistance indirectly
    // --------------------------------------------------

    @Test
    void Should_ThrowException_When_UpdateSpeedIsInfinite() {
        assertThrows(
                IllegalArgumentException.class,
                () -> cruise.update(
                        Double.POSITIVE_INFINITY,
                        30.0
                )
        );
    }

    @Test
    void Should_ThrowException_When_UpdateSpeedIsNegative() {
        assertThrows(
                IllegalArgumentException.class,
                () -> cruise.update(-10.0, 30.0)
        );
    }

    @Test
    void Should_ThrowException_When_UpdateDistanceIsInfinite() {
        assertThrows(
                IllegalArgumentException.class,
                () -> cruise.update(
                        50.0,
                        Double.POSITIVE_INFINITY
                )
        );
    }

    @Test
    void Should_ThrowException_When_UpdateDistanceIsNegative() {
        assertThrows(
                IllegalArgumentException.class,
                () -> cruise.update(50.0, -10.0)
        );
    }

    // --------------------------------------------------
    // update: EMERGENCY_BRAKE
    // Also verifies braking remains active
    // --------------------------------------------------

    @Test
    void Should_ReturnEmergencyBrake_When_DistanceIsAtEmergencyThreshold() {
        // safeDistance = 40
        // emergencyDistance = 20
        AdaptiveCruise.Decision decision =cruise.update(80.0, 20.0);

        assertEquals(
                AdaptiveCruise.Mode.EMERGENCY_BRAKE,
                decision.mode()
        );
        assertEquals(100.0, decision.amount(), DELTA);
    }

    @Test
    void Should_ContinueBraking_When_DistanceIsBelowReleaseThreshold() {
        cruise.update(80.0, 20.0);
        // safeDistance = 40
        // releaseDistance = 48
        AdaptiveCruise.Decision decision =cruise.update(80.0, 45.0);

        assertEquals(AdaptiveCruise.Mode.BRAKE, decision.mode());
        assertEquals(10.0, decision.amount(), DELTA);
    }

    // --------------------------------------------------
    // update: proportionalBrake through update
    // --------------------------------------------------

    @Test
    void Should_ReturnProportionalBrake_When_DistanceIsBelowSafeDistance() {
        // safeDistance = 40
        // brake = (40 - 30) / 40 * 100 = 25
        AdaptiveCruise.Decision decision =cruise.update(80.0, 30.0);

        assertEquals(AdaptiveCruise.Mode.BRAKE, decision.mode());
        assertEquals(25.0, decision.amount(), DELTA);
    }

    @Test
    void Should_ReturnEmergencyBrake_When_SpeedAndDistanceAreZero() {
        // safeDistance = 0, emergencyDistance = 0
        // Emergency branch is reached before proportionalBrake.
        AdaptiveCruise.Decision decision =cruise.update(0.0, 0.0);

        assertEquals(
                AdaptiveCruise.Mode.EMERGENCY_BRAKE,
                decision.mode()
        );
        assertEquals(100.0, decision.amount(), DELTA);
    }

    // --------------------------------------------------
    // update: existing braking and hysteresis
    // --------------------------------------------------

    @Test
    void Should_ReturnProportionalBrake_When_AlreadyBrakingAndDistanceIsBelowSafeDistance() {
        cruise.update(80.0, 20.0);

        AdaptiveCruise.Decision decision = cruise.update(80.0, 30.0);

        assertEquals(AdaptiveCruise.Mode.BRAKE, decision.mode());
        assertEquals(25.0, decision.amount(), DELTA);
    }

    @Test
    void Should_ApplyHysteresisBrakePressure_When_BrakingAndDistanceIsBetweenSafeAndReleaseThresholds() {
        cruise.update(80.0, 20.0);

        // 40 < 45 < 48
        AdaptiveCruise.Decision decision = cruise.update(80.0, 45.0);

        assertEquals(AdaptiveCruise.Mode.BRAKE, decision.mode());
        assertEquals(10.0, decision.amount(), DELTA);
    }

    @Test
    void Should_ReleaseBraking_When_DistanceReachesReleaseThreshold() {
        cruise.update(80.0, 20.0);

        // At 48 meters, braking is released.
        AdaptiveCruise.Decision decision =cruise.update(80.0, 48.0);

        assertEquals(
                AdaptiveCruise.Mode.ACCELERATE,
                decision.mode()
        );
        assertEquals(5.0, decision.amount(), DELTA);
    }

    // --------------------------------------------------
    // update: ACCELERATE
    // --------------------------------------------------

    @Test
    void Should_AccelerateByMaximumStep_When_SpeedIsBelowTargetByMoreThanFive() {
        AdaptiveCruise.Decision decision = cruise.update(80.0, 41.0);

        assertEquals(
                AdaptiveCruise.Mode.ACCELERATE,
                decision.mode()
        );
        assertEquals(5.0, decision.amount(), DELTA);
    }

    @Test
    void Should_AccelerateByRemainingDifference_When_TargetSpeedDifferenceIsBelowFive() {
        AdaptiveCruise cruise = new AdaptiveCruise(82.0);

        AdaptiveCruise.Decision decision =
                cruise.update(80.0, 41.0);

        assertEquals(
                AdaptiveCruise.Mode.ACCELERATE,
                decision.mode()
        );
        assertEquals(2.0, decision.amount(), DELTA);
    }

    // --------------------------------------------------
    // update: HOLD
    // --------------------------------------------------

    @Test
    void Should_ReturnHold_When_TargetSpeedIsReached() {
        AdaptiveCruise.Decision decision = cruise.update(100.0, 60.0);

        assertEquals(AdaptiveCruise.Mode.HOLD, decision.mode());
        assertEquals(0.0, decision.amount(), DELTA);
    }

    @Test
    void Should_ReturnHold_When_DistanceEqualsSafeDistanceAndBrakingIsInactive() {
        // distance == safeDistance, so neither < nor > is true.
        AdaptiveCruise.Decision decision = cruise.update(80.0, 40.0);

        assertEquals(AdaptiveCruise.Mode.HOLD, decision.mode());
        assertEquals(0.0, decision.amount(), DELTA);
    }
}
