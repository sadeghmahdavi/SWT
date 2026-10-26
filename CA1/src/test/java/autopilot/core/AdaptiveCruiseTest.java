package autopilot.core;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class AdaptiveCruiseTest {

    private AdaptiveCruise cruise;

    private static final double DELTA = 1e-9;

    @BeforeEach
    void setUp() {
        cruise = new AdaptiveCruise(100.0);
    }

    private AdaptiveCruise.Decision updateCruise(double speed, double distance) {
        return cruise.update(speed, distance);
    }

    private void assertDecision(
            AdaptiveCruise.Decision decision,
            AdaptiveCruise.Mode expectedMode,
            double expectedAmount
    ) {
        assertEquals(expectedMode, decision.mode());
        assertEquals(expectedAmount, decision.amount(), DELTA);
    }

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

    @Test
    void Should_ThrowException_When_UpdateSpeedIsInfinite() {
        assertThrows(
                IllegalArgumentException.class,
                () -> updateCruise(Double.POSITIVE_INFINITY, 30.0)
        );
    }

    @Test
    void Should_ThrowException_When_UpdateSpeedIsNegative() {
        assertThrows(
                IllegalArgumentException.class,
                () -> updateCruise(-10.0, 30.0)
        );
    }

    @Test
    void Should_ThrowException_When_UpdateDistanceIsInfinite() {
        assertThrows(
                IllegalArgumentException.class,
                () -> updateCruise(50.0, Double.POSITIVE_INFINITY)
        );
    }

    @Test
    void Should_ThrowException_When_UpdateDistanceIsNegative() {
        assertThrows(
                IllegalArgumentException.class,
                () -> updateCruise(50.0, -10.0)
        );
    }

    @Test
    void Should_ReturnEmergencyBrake_When_DistanceIsAtEmergencyThreshold() {
        AdaptiveCruise.Decision decision = updateCruise(80.0, 20.0);

        assertDecision(
                decision,
                AdaptiveCruise.Mode.EMERGENCY_BRAKE,
                100.0
        );
    }

    @Test
    void Should_ContinueBraking_When_DistanceIsBelowReleaseThreshold() {
        updateCruise(80.0, 20.0);

        AdaptiveCruise.Decision decision = updateCruise(80.0, 45.0);

        assertDecision(decision, AdaptiveCruise.Mode.BRAKE, 10.0);
    }

    @Test
    void Should_ReturnProportionalBrake_When_DistanceIsBelowSafeDistance() {
        AdaptiveCruise.Decision decision = updateCruise(80.0, 30.0);

        assertDecision(decision, AdaptiveCruise.Mode.BRAKE, 25.0);
    }

    @Test
    void Should_ReturnEmergencyBrake_When_SpeedAndDistanceAreZero() {
        AdaptiveCruise.Decision decision = updateCruise(0.0, 0.0);

        assertDecision(
                decision,
                AdaptiveCruise.Mode.EMERGENCY_BRAKE,
                100.0
        );
    }

    @Test
    void Should_ReturnProportionalBrake_When_AlreadyBrakingAndDistanceIsBelowSafeDistance() {
        updateCruise(80.0, 20.0);

        AdaptiveCruise.Decision decision = updateCruise(80.0, 30.0);

        assertDecision(decision, AdaptiveCruise.Mode.BRAKE, 25.0);
    }

    @Test
    void Should_ApplyHysteresisBrakePressure_When_BrakingAndDistanceIsBetweenSafeAndReleaseThresholds() {
        updateCruise(80.0, 20.0);

        AdaptiveCruise.Decision decision = updateCruise(80.0, 45.0);

        assertDecision(decision, AdaptiveCruise.Mode.BRAKE, 10.0);
    }

    @Test
    void Should_ReleaseBraking_When_DistanceReachesReleaseThreshold() {
        updateCruise(80.0, 20.0);

        AdaptiveCruise.Decision decision = updateCruise(80.0, 48.0);

        assertDecision(decision, AdaptiveCruise.Mode.ACCELERATE, 5.0);
    }

    @Test
    void Should_AccelerateByMaximumStep_When_SpeedIsBelowTargetByMoreThanFive() {
        AdaptiveCruise.Decision decision = updateCruise(80.0, 41.0);

        assertDecision(decision, AdaptiveCruise.Mode.ACCELERATE, 5.0);
    }

    @Test
    void Should_AccelerateByRemainingDifference_When_TargetSpeedDifferenceIsBelowFive() {
        cruise = new AdaptiveCruise(82.0);

        AdaptiveCruise.Decision decision = updateCruise(80.0, 41.0);

        assertDecision(decision, AdaptiveCruise.Mode.ACCELERATE, 2.0);
    }

    @Test
    void Should_ReturnHold_When_TargetSpeedIsReached() {
        AdaptiveCruise.Decision decision = updateCruise(100.0, 60.0);

        assertDecision(decision, AdaptiveCruise.Mode.HOLD, 0.0);
    }

    @Test
    void Should_ReturnHold_When_DistanceEqualsSafeDistanceAndBrakingIsInactive() {
        AdaptiveCruise.Decision decision = updateCruise(80.0, 40.0);

        assertDecision(decision, AdaptiveCruise.Mode.HOLD, 0.0);
    }
}