
package autopilot.core;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.CsvFileSource;
import org.junit.jupiter.params.provider.ValueSource;

import static org.junit.jupiter.api.Assertions.*;

class AdaptiveCruiseTest {

    private static final double DELTA = 1e-9;

    private static final double TARGET_SPEED_KMH = 100.0;
    private static final double TEST_SPEED_KMH = 80.0;

    private static final double SAFE_DISTANCE_METERS = 40.0;
    private static final double EMERGENCY_DISTANCE_METERS = 20.0;
    private static final double RELEASE_DISTANCE_METERS = 48.0;
    private static final double INTERMEDIATE_DISTANCE_METERS = 45.0;

    private AdaptiveCruise cruise;

    @BeforeEach
    void setUp() {
        cruise = new AdaptiveCruise(TARGET_SPEED_KMH);
    }


    @Test
    void Should_ReturnExpectedSafeDistance_When_SpeedIsPositive() {

        double actual = cruise.safeDistanceFor(TEST_SPEED_KMH);

        assertEquals(SAFE_DISTANCE_METERS, actual, DELTA);
    }

    @Test
    void Should_ReturnZeroSafeDistance_When_SpeedIsZero() {

        double actual = cruise.safeDistanceFor(0.0);

        assertEquals(0.0, actual, DELTA);
    }


    @ParameterizedTest
    @ValueSource(doubles = {
            -10.0,
            Double.NaN,
            Double.POSITIVE_INFINITY,
            Double.NEGATIVE_INFINITY
    })
    void Should_ThrowIllegalArgumentException_When_SafeDistanceSpeedIsInvalid(
            double invalidSpeed) {

        assertThrows(
                IllegalArgumentException.class,
                () -> cruise.safeDistanceFor(invalidSpeed)
        );
    }


    @ParameterizedTest
    @ValueSource(doubles = {
            -10.0,
            Double.NaN,
            Double.POSITIVE_INFINITY,
            Double.NEGATIVE_INFINITY
    })
    void Should_RejectInvalidSpeedAndPreserveBraking_When_UpdateSpeedIsInvalid(
            double invalidSpeed) {

        cruise.update(
                TEST_SPEED_KMH,
                EMERGENCY_DISTANCE_METERS
        );

        assertThrows(
                IllegalArgumentException.class,
                () -> cruise.update(invalidSpeed, 50.0)
        );

        AdaptiveCruise.Decision decision = cruise.update(
                TEST_SPEED_KMH,
                INTERMEDIATE_DISTANCE_METERS
        );

        assertDecision(decision, AdaptiveCruise.Mode.BRAKE, 10.0);
    }


    @ParameterizedTest
    @ValueSource(doubles = {
            -10.0,
            Double.NaN,
            Double.POSITIVE_INFINITY,
            Double.NEGATIVE_INFINITY
    })
    void Should_RejectInvalidDistanceAndPreserveBraking_When_UpdateDistanceIsInvalid(
            double invalidDistance) {

        cruise.update(
                TEST_SPEED_KMH,
                EMERGENCY_DISTANCE_METERS
        );

        assertThrows(
                IllegalArgumentException.class,
                () -> cruise.update(TEST_SPEED_KMH, invalidDistance)
        );

        AdaptiveCruise.Decision decision = cruise.update(
                TEST_SPEED_KMH,
                INTERMEDIATE_DISTANCE_METERS
        );

        assertDecision(decision, AdaptiveCruise.Mode.BRAKE, 10.0);
    }


    @Test
    void Should_ReturnProportionalBrake_When_BrakingContinuesBelowSafeDistance() {

        cruise.update(
                TEST_SPEED_KMH,
                EMERGENCY_DISTANCE_METERS
        );

        AdaptiveCruise.Decision decision = cruise.update(
                TEST_SPEED_KMH,
                30.0
        );

        assertDecision(decision, AdaptiveCruise.Mode.BRAKE, 25.0);
    }


    @Test
    void Should_ClearBrakingState_When_DistanceReachesReleaseThreshold() {

        cruise.update(
                TEST_SPEED_KMH,
                EMERGENCY_DISTANCE_METERS
        );

        AdaptiveCruise.Decision beforeRelease = cruise.update(
                TEST_SPEED_KMH,
                INTERMEDIATE_DISTANCE_METERS
        );

        assertDecision(beforeRelease, AdaptiveCruise.Mode.BRAKE, 10.0);

        AdaptiveCruise.Decision atRelease = cruise.update(
                TEST_SPEED_KMH,
                RELEASE_DISTANCE_METERS
        );

        assertDecision(atRelease, AdaptiveCruise.Mode.ACCELERATE, 5.0);

        AdaptiveCruise.Decision afterRelease = cruise.update(
                TEST_SPEED_KMH,
                INTERMEDIATE_DISTANCE_METERS
        );

        assertDecision(afterRelease, AdaptiveCruise.Mode.ACCELERATE, 5.0);
    }


    @Test
    void Should_RestoreHysteresis_When_BrakingStartsAgainAfterRelease() {

        cruise.update(
                TEST_SPEED_KMH,
                EMERGENCY_DISTANCE_METERS
        );

        cruise.update(
                TEST_SPEED_KMH,
                RELEASE_DISTANCE_METERS
        );

        AdaptiveCruise.Decision newBrake = cruise.update(
                TEST_SPEED_KMH,
                30.0
        );

        assertDecision(newBrake, AdaptiveCruise.Mode.BRAKE, 25.0);

        AdaptiveCruise.Decision hysteresis = cruise.update(
                TEST_SPEED_KMH,
                INTERMEDIATE_DISTANCE_METERS
        );

        assertDecision(hysteresis, AdaptiveCruise.Mode.BRAKE, 10.0);
    }


    @ParameterizedTest(
            name = "{index}: speed={0}, distance={1}, wasBraking={2}"
    )
    @CsvFileSource(
            resources = "/autopilot/core/adaptive-cruise-cases.csv",
            numLinesToSkip = 1
    )
    void Should_ReturnExpectedModeAndAmount_When_CruiseConditionsVary(
            double speed,
            double distance,
            boolean wasBraking,
            AdaptiveCruise.Mode expectedMode,
            double expectedAmount) {

        if (wasBraking) {
            cruise.update(speed, 0.0);
        }

        AdaptiveCruise.Decision decision =
                cruise.update(speed, distance);

        assertDecision(decision, expectedMode, expectedAmount);
    }

    private void assertDecision(
            AdaptiveCruise.Decision decision,
            AdaptiveCruise.Mode expectedMode,
            double expectedAmount) {

        assertAll(
                () -> assertEquals(expectedMode, decision.mode()),
                () -> assertEquals(
                        expectedAmount,
                        decision.amount(),
                        DELTA
                )
        );
    }
}
