
package autopilot.core;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.CsvFileSource;
import org.junit.jupiter.params.provider.CsvSource;
import org.junit.jupiter.params.provider.ValueSource;

import static org.junit.jupiter.api.Assertions.*;

class AdaptiveCruiseTest {

    private static final double DELTA = 1e-9;

    private static final double TARGET_SPEED_KMH = 100.0;
    private static final double TEST_SPEED_KMH = 80.0;
    private static final double EMERGENCY_DISTANCE_METERS = 20.0;
    private static final double RELEASE_DISTANCE_METERS = 48.0;
    private static final double INTERMEDIATE_DISTANCE_METERS = 45.0;
    private static final double MODERATE_BRAKE_DISTANCE_METERS = 30.0;
    private static final double HYSTERESIS_BRAKE_PRESSURE = 10.0;
    private static final double MAX_ACCELERATION_STEP_KMH = 5.0;

    private AdaptiveCruise cruise;

    @BeforeEach
    void setUp() {
        cruise = new AdaptiveCruise(TARGET_SPEED_KMH);
    }

    @ParameterizedTest(name = "speed={0}, safeDistance={1}")
    @CsvSource({
            "0, 0",
            "0.2, 0.1",
            "10, 5",
            "80, 40",
            "200, 100"
    })
    void Should_ReturnExpectedSafeDistance_When_SpeedIsValid(
            double speed,
            double expectedDistance) {

        assertEquals(
                expectedDistance,
                cruise.safeDistanceFor(speed),
                DELTA
        );
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

    @ParameterizedTest(name = "speed={0}, distance={1}, wasBraking={2}")
    @CsvSource({
            "-0.001, 50, false, ACCELERATE, 5",
            "NaN, 50, false, ACCELERATE, 5",
            "Infinity, 50, false, ACCELERATE, 5",
            "-Infinity, 50, false, ACCELERATE, 5",

            "-0.001, 50, true, BRAKE, 10",
            "NaN, 50, true, BRAKE, 10",
            "Infinity, 50, true, BRAKE, 10",
            "-Infinity, 50, true, BRAKE, 10",

            "80, -0.001, false, ACCELERATE, 5",
            "80, NaN, false, ACCELERATE, 5",
            "80, Infinity, false, ACCELERATE, 5",
            "80, -Infinity, false, ACCELERATE, 5",

            "80, -0.001, true, BRAKE, 10",
            "80, NaN, true, BRAKE, 10",
            "80, Infinity, true, BRAKE, 10",
            "80, -Infinity, true, BRAKE, 10"
    })
    void Should_RejectInvalidUpdateAndPreserveState_When_InputIsInvalid(
            double speed,
            double distance,
            boolean wasBraking,
            AdaptiveCruise.Mode expectedMode,
            double expectedAmount) {

        if (wasBraking) {
            cruise.update(
                    TEST_SPEED_KMH,
                    EMERGENCY_DISTANCE_METERS
            );
        }

        assertThrows(
                IllegalArgumentException.class,
                () -> cruise.update(speed, distance)
        );

        AdaptiveCruise.Decision afterRejection =
                cruise.update(
                        TEST_SPEED_KMH,
                        INTERMEDIATE_DISTANCE_METERS
                );

        assertDecision(
                afterRejection,
                expectedMode,
                expectedAmount
        );
    }

    @Test
    void Should_ReturnProportionalBrake_When_BrakingContinuesBelowSafeDistance() {

        cruise.update(
                TEST_SPEED_KMH,
                EMERGENCY_DISTANCE_METERS
        );

        AdaptiveCruise.Decision decision = cruise.update(
                TEST_SPEED_KMH,
                MODERATE_BRAKE_DISTANCE_METERS
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

        assertDecision(
                beforeRelease,
                AdaptiveCruise.Mode.BRAKE,
                HYSTERESIS_BRAKE_PRESSURE
        );

        AdaptiveCruise.Decision atRelease = cruise.update(
                TEST_SPEED_KMH,
                RELEASE_DISTANCE_METERS
        );

        assertDecision(
                atRelease,
                AdaptiveCruise.Mode.ACCELERATE,
                MAX_ACCELERATION_STEP_KMH
        );

        AdaptiveCruise.Decision afterRelease = cruise.update(
                TEST_SPEED_KMH,
                INTERMEDIATE_DISTANCE_METERS
        );

        assertDecision(
                afterRelease,
                AdaptiveCruise.Mode.ACCELERATE,
                MAX_ACCELERATION_STEP_KMH
        );
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
                MODERATE_BRAKE_DISTANCE_METERS
        );

        assertDecision(
                newBrake,
                AdaptiveCruise.Mode.BRAKE,
                25.0
        );

        AdaptiveCruise.Decision hysteresis = cruise.update(
                TEST_SPEED_KMH,
                INTERMEDIATE_DISTANCE_METERS
        );

        assertDecision(
                hysteresis,
                AdaptiveCruise.Mode.BRAKE,
                HYSTERESIS_BRAKE_PRESSURE
        );
    }

    @ParameterizedTest(
            name = "speed={0}, distance={1}, wasBraking={2}"
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
