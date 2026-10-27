
package autopilot.core;

import autopilot.core.GearBox.Gear;
import autopilot.exception.IllegalGearChangeException;
import autopilot.exception.SensorFailureException;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.EnumSource;
import org.junit.jupiter.params.provider.MethodSource;

import java.util.stream.Stream;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.params.provider.Arguments.arguments;

class GearBoxTest {

    private double speedKmh;
    private GearBox gearBox;

    @BeforeEach
    void setUp() {
        speedKmh = 0.0;
        gearBox = new GearBox(() -> speedKmh);
    }

    private void startIn(Gear initialGear) {
        if (initialGear == Gear.P) return;

        gearBox.changeTo(Gear.R);

        if (initialGear == Gear.N) {
            gearBox.changeTo(Gear.N);
        }

        if (initialGear == Gear.D) {
            gearBox.changeTo(Gear.D);
        }
    }

    static Stream<Arguments> allowedTransitions() {
        return Stream.of(
                arguments(Gear.P, Gear.R, 0.0),
                arguments(Gear.P, Gear.R, 4.999),
                arguments(Gear.R, Gear.N, 0.0),
                arguments(Gear.R, Gear.N, 25.0),
                arguments(Gear.N, Gear.D, 0.0),
                arguments(Gear.N, Gear.D, 25.0),
                arguments(Gear.R, Gear.D, 0.0),
                arguments(Gear.D, Gear.R, 0.0)
        );
    }

    @ParameterizedTest(name = "{index}: {0} to {1} at {2} km/h")
    @MethodSource("allowedTransitions")
    void Should_ChangeGear_When_TransitionIsAllowed(
            Gear initialGear,
            Gear targetGear,
            double speed) {

        startIn(initialGear);
        speedKmh = speed;

        gearBox.changeTo(targetGear);

        assertEquals(targetGear, gearBox.getCurrentGear());
    }

    static Stream<Arguments> forbiddenTransitions() {
        return Stream.of(
                arguments(Gear.P, Gear.R, 5.0),
                arguments(Gear.P, Gear.R, 5.001),
                arguments(Gear.P, Gear.N, 0.0),
                arguments(Gear.P, Gear.D, 0.0),
                arguments(Gear.R, Gear.P, 0.0),
                arguments(Gear.N, Gear.P, 0.0),
                arguments(Gear.N, Gear.R, 0.0),
                arguments(Gear.D, Gear.P, 0.0),
                arguments(Gear.D, Gear.N, 0.0),
                arguments(Gear.R, Gear.D, 0.001),
                arguments(Gear.D, Gear.R, 0.001)
        );
    }

    @ParameterizedTest(name = "{index}: reject {0} to {1} at {2} km/h")
    @MethodSource("forbiddenTransitions")
    void Should_RejectTransitionAndKeepGear_When_TransitionIsForbidden(
            Gear initialGear,
            Gear targetGear,
            double speed) {

        startIn(initialGear);
        speedKmh = speed;

        assertThrows(
                IllegalGearChangeException.class,
                () -> gearBox.changeTo(targetGear)
        );

        assertEquals(initialGear, gearBox.getCurrentGear());
    }

    static Stream<Double> invalidSpeeds() {
        return Stream.of(
                -0.001,
                Double.NaN,
                Double.POSITIVE_INFINITY,
                Double.NEGATIVE_INFINITY
        );
    }

    @ParameterizedTest(name = "{index}: reject speed {0}")
    @MethodSource("invalidSpeeds")
    void Should_RejectSpeedAndKeepGear_When_SensorReturnsInvalidSpeed(
            double speed) {

        speedKmh = speed;

        assertThrows(
                IllegalArgumentException.class,
                () -> gearBox.changeTo(Gear.R)
        );

        assertEquals(Gear.P, gearBox.getCurrentGear());
    }

    @Test
    void Should_ThrowNullPointerException_When_TargetGearIsNull() {

        assertThrows(
                NullPointerException.class,
                () -> gearBox.changeTo(null)
        );

        assertEquals(Gear.P, gearBox.getCurrentGear());
    }

    @ParameterizedTest
    @EnumSource(Gear.class)
    void Should_SkipSensorReading_When_TargetEqualsCurrentGear(
            Gear currentGear) {

        startIn(currentGear);

        gearBox.setSpeedSensor(() -> {
            throw new AssertionError("The sensor must not be read");
        });

        gearBox.changeTo(currentGear);

        assertEquals(currentGear, gearBox.getCurrentGear());
    }

    @Test
    void Should_PreserveGearAndPropagateException_When_SpeedSensorFails() {

        startIn(Gear.D);

        gearBox.setSpeedSensor(() -> {
            throw new SensorFailureException("Speed sensor failed");
        });

        assertThrows(
                SensorFailureException.class,
                () -> gearBox.changeTo(Gear.R)
        );

        assertEquals(Gear.D, gearBox.getCurrentGear());
    }
}
