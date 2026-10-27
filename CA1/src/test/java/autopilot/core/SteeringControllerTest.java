
package autopilot.core;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.CsvSource;
import org.junit.jupiter.params.provider.ValueSource;

import static org.junit.jupiter.api.Assertions.*;

class SteeringControllerTest {

    private static final double DELTA = 1e-9;
    private static final double ZERO_SPEED = 0.0;
    private static final double NORMAL_SPEED = 50.0;
    private static final double SAFE_SPEED = 100.0;
    private static final double HIGH_SPEED = 121.0;
    private static final double ZERO_ANGLE = 0.0;
    private static final double MAX_STEP_DEGREES = 60.0;
    private static final double HIGH_SPEED_LIMIT_DEGREES = 90.0;
    private static final double INITIAL_VALID_ANGLE = 120.0;
    private static final double TARGET_ANGLE = 150.0;
    private static final double LARGE_ANGLE = 300.0;
    private static final double SELF_CENTER_START_ANGLE = 180.0;
    private static final int TICKS_TO_LARGE_ANGLE = 5;
    private static final int TICKS_TO_CENTER_START = 3;

    private SteeringController steering;

    @BeforeEach
    void setUp() {
        steering = new SteeringController();
    }

    @ParameterizedTest(name = "request={0}, speed={1}")
    @CsvSource({
            "0, 0, 0, 0",
            "20, 0, 20, 20",
            "-20, 0, -20, -20",
            "60, 0, 60, 60",
            "-60, 0, -60, -60",
            "61, 0, 60, 61",
            "-61, 0, -60, -61",
            "450, 0, 60, 450",
            "-450, 0, -60, -450",
            "500, 0, 60, 450",
            "-500, 0, -60, -450",
            "500, 120, 60, 450",
            "500, 120.0001, 60, 90",
            "-500, 120.0001, -60, -90",
            "20, 130, 20, 20",
            "1.0E308, 130, 60, 90"
    })
    void Should_SetExpectedTargetAndMoveTowardIt_When_InputsAreValid(
            double requestedAngle, double speed,
            double expectedCurrent, double expectedTarget) {

        double actual = steering.update(requestedAngle, speed);

        assertAll(
                () -> assertEquals(expectedCurrent, actual, DELTA),
                () -> assertEquals(expectedTarget,
                        steering.getTargetAngleDegrees(), DELTA)
        );
    }

    @ParameterizedTest(name = "invalid angle={0}, speed={1}")
    @CsvSource({
            "NaN, 50",
            "Infinity, 50",
            "-Infinity, 50",
            "30, -0.01",
            "30, NaN",
            "30, Infinity",
            "30, -Infinity"
    })
    void Should_ThrowAndPreserveState_When_InputsAreInvalid(
            double requestedAngle, double speed) {

        steering.update(INITIAL_VALID_ANGLE, ZERO_SPEED);

        assertThrows(
                IllegalArgumentException.class,
                () -> steering.update(requestedAngle, speed)
        );

        assertAll(
                () -> assertEquals(MAX_STEP_DEGREES,
                        steering.getCurrentAngleDegrees(), DELTA),
                () -> assertEquals(INITIAL_VALID_ANGLE,
                        steering.getTargetAngleDegrees(), DELTA)
        );
    }

    @ParameterizedTest(name = "ticks={0}, expectedAngle={1}")
    @CsvSource({
            "1, 60",
            "2, 120",
            "3, 150",
            "4, 150"
    })
    void Should_ReachTargetGradually_When_UpdateIsRepeated(
            int ticks, double expectedAngle) {

        double actual = ZERO_ANGLE;

        for (int i = 0; i < ticks; i++) {
            actual = steering.update(TARGET_ANGLE, NORMAL_SPEED);
        }

        assertEquals(expectedAngle, actual, DELTA);
    }

    @ParameterizedTest(name = "reverseTicks={0}, expectedAngle={1}")
    @CsvSource({
            "1, 60",
            "2, 0",
            "3, -60",
            "4, -90"
    })
    void Should_ReverseDirectionGradually_When_TargetChangesSide(
            int reverseTicks, double expectedAngle) {

        steering.update(INITIAL_VALID_ANGLE, ZERO_SPEED);
        steering.update(INITIAL_VALID_ANGLE, ZERO_SPEED);

        double actual = ZERO_ANGLE;

        for (int i = 0; i < reverseTicks; i++) {
            actual = steering.update(
                    -HIGH_SPEED_LIMIT_DEGREES, ZERO_SPEED
            );
        }

        assertEquals(expectedAngle, actual, DELTA);
    }

    @ParameterizedTest(name = "requested={0}, highSpeedTicks={1}")
    @CsvSource({
            "300, 1, 240, 90",
            "300, 2, 180, 90",
            "300, 3, 120, 90",
            "300, 4, 90, 90",
            "-300, 1, -240, -90",
            "-300, 2, -180, -90",
            "-300, 3, -120, -90",
            "-300, 4, -90, -90"
    })
    void Should_MoveGraduallyToReducedLimit_When_SpeedBecomesHigh(
            double requestedAngle,
            int highSpeedTicks,
            double expectedCurrent,
            double expectedTarget) {

        for (int i = 0; i < TICKS_TO_LARGE_ANGLE; i++) {
            steering.update(requestedAngle, SAFE_SPEED);
        }

        assertEquals(
                requestedAngle,
                steering.getCurrentAngleDegrees(),
                DELTA
        );

        double actual = ZERO_ANGLE;

        for (int i = 0; i < highSpeedTicks; i++) {
            actual = steering.update(requestedAngle, HIGH_SPEED);
        }

        assertEquals(expectedCurrent, actual, DELTA);
        assertEquals(
                expectedTarget,
                steering.getTargetAngleDegrees(),
                DELTA
        );
    }

    @ParameterizedTest(name = "initialAngle={0}")
    @ValueSource(doubles = {
            -60.0, -15.0, 0.0, 15.0, 60.0
    })
    void Should_ReachZeroInOneStep_When_SelfCenterIsCalledWithinSingleStepRange(
            double initialAngle) {

        steering.update(initialAngle, ZERO_SPEED);

        double actual = steering.selfCenter();

        assertEquals(ZERO_ANGLE, actual, DELTA);
        assertEquals(
                ZERO_ANGLE,
                steering.getTargetAngleDegrees(),
                DELTA
        );
    }

    @Test
    void Should_ReachZeroGradually_When_SelfCenterIsRepeatedFromPositiveAngle() {

        for (int i = 0; i < TICKS_TO_CENTER_START; i++) {
            steering.update(SELF_CENTER_START_ANGLE, ZERO_SPEED);
        }

        assertEquals(
                SELF_CENTER_START_ANGLE,
                steering.getCurrentAngleDegrees(),
                DELTA
        );

        assertEquals(120.0, steering.selfCenter(), DELTA);
        assertEquals(60.0, steering.selfCenter(), DELTA);
        assertEquals(ZERO_ANGLE, steering.selfCenter(), DELTA);
        assertEquals(ZERO_ANGLE, steering.selfCenter(), DELTA);

        assertEquals(
                ZERO_ANGLE,
                steering.getTargetAngleDegrees(),
                DELTA
        );
    }

    @Test
    void Should_ReachZeroGradually_When_SelfCenterIsRepeatedFromNegativeAngle() {

        for (int i = 0; i < TICKS_TO_CENTER_START; i++) {
            steering.update(-SELF_CENTER_START_ANGLE, ZERO_SPEED);
        }

        assertEquals(
                -SELF_CENTER_START_ANGLE,
                steering.getCurrentAngleDegrees(),
                DELTA
        );

        assertEquals(-120.0, steering.selfCenter(), DELTA);
        assertEquals(-60.0, steering.selfCenter(), DELTA);
        assertEquals(ZERO_ANGLE, steering.selfCenter(), DELTA);
        assertEquals(ZERO_ANGLE, steering.selfCenter(), DELTA);

        assertEquals(
                ZERO_ANGLE,
                steering.getTargetAngleDegrees(),
                DELTA
        );
    }
}
