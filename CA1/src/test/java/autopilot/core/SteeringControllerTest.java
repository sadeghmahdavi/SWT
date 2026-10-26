
package autopilot.core;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.CsvSource;
import org.junit.jupiter.params.provider.ValueSource;

import static org.junit.jupiter.api.Assertions.*;

class SteeringControllerTest {

    private static final double DELTA = 1e-9;
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
            double requestedAngle,
            double speed,
            double expectedCurrent,
            double expectedTarget) {

        double actual = steering.update(requestedAngle, speed);

        assertEquals(expectedCurrent, actual, DELTA);
        assertEquals(
                expectedTarget,
                steering.getTargetAngleDegrees(),
                DELTA
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

        steering.update(120.0, 0.0);

        assertThrows(
                IllegalArgumentException.class,
                () -> steering.update(requestedAngle, speed)
        );

        assertEquals(
                60.0,
                steering.getCurrentAngleDegrees(),
                DELTA
        );
        assertEquals(
                120.0,
                steering.getTargetAngleDegrees(),
                DELTA
        );
    }


    @Test
    void Should_ReachTargetGradually_When_UpdateIsRepeated() {

        assertEquals(60.0, steering.update(150.0, 50.0), DELTA);
        assertEquals(120.0, steering.update(150.0, 50.0), DELTA);
        assertEquals(150.0, steering.update(150.0, 50.0), DELTA);
        assertEquals(150.0, steering.update(150.0, 50.0), DELTA);
    }


    @Test
    void Should_ReverseDirectionGradually_When_TargetChangesSide() {

        steering.update(150.0, 0.0);
        steering.update(150.0, 0.0);

        assertEquals(60.0, steering.update(-90.0, 0.0), DELTA);
        assertEquals(0.0, steering.update(-90.0, 0.0), DELTA);
        assertEquals(-60.0, steering.update(-90.0, 0.0), DELTA);
        assertEquals(-90.0, steering.update(-90.0, 0.0), DELTA);
    }


    @Test
    void Should_MoveTowardReducedLimit_When_SpeedBecomesHigh() {

        for (int i = 0; i < 5; i++) {
            steering.update(300.0, 100.0);
        }

        assertEquals(300.0, steering.getCurrentAngleDegrees(), DELTA);

        assertEquals(240.0, steering.update(300.0, 121.0), DELTA);
        assertEquals(90.0, steering.getTargetAngleDegrees(), DELTA);

        assertEquals(180.0, steering.update(300.0, 121.0), DELTA);
        assertEquals(120.0, steering.update(300.0, 121.0), DELTA);
        assertEquals(90.0, steering.update(300.0, 121.0), DELTA);
    }


    @ParameterizedTest(name = "initial angle={0}")
    @ValueSource(doubles = {
        -180.0, -60.0, -15.0,
        15.0, 60.0, 180.0
    })
    void Should_MoveTowardZero_When_SelfCenterIsCalled(
            double initialAngle) {

        for (int i = 0; i < 3; i++) {
            steering.update(initialAngle, 0.0);
        }

        double expected = 0.0;

        if (initialAngle > 60.0) {
            expected = initialAngle - 60.0;
        } else if (initialAngle < -60.0) {
            expected = initialAngle + 60.0;
        }

        double actual = steering.selfCenter();

        assertEquals(expected, actual, DELTA);
        assertEquals(
                0.0,
                steering.getTargetAngleDegrees(),
                DELTA
        );
    }


    @Test
    void Should_RemainAtZero_When_SelfCenterIsCalledFromZero() {

        assertEquals(0.0, steering.selfCenter(), DELTA);
        assertEquals(0.0, steering.getTargetAngleDegrees(), DELTA);
    }


    @Test
    void Should_ReachZeroGradually_When_SelfCenterIsRepeated() {

        for (int i = 0; i < 3; i++) {
            steering.update(180.0, 0.0);
        }

        assertEquals(120.0, steering.selfCenter(), DELTA);
        assertEquals(60.0, steering.selfCenter(), DELTA);
        assertEquals(0.0, steering.selfCenter(), DELTA);
        assertEquals(0.0, steering.selfCenter(), DELTA);
    }
}
