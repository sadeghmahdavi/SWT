
package autopilot.core;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.EnumSource;
import org.junit.jupiter.params.provider.ValueSource;

import static org.junit.jupiter.api.Assertions.*;

class BrakeControllerTest {

    private static final double DELTA = 1e-9;

    private static final double NORMAL_PRESSURE = 50.0;
    private static final double HIGH_PRESSURE = 95.0;
    private static final double MAX_PRESSURE = 100.0;

    private static final double ABS_PRESSURE_THRESHOLD = 80.0;
    private static final double ABS_SPEED_THRESHOLD = 30.0;
    private static final double ABS_SPEED = 40.0;
    private static final double LOW_SPEED = 20.0;

    private static final double ABS_HIGH_PULSE = 90.0;
    private static final double ABS_LOW_PULSE = 60.0;

    private BrakeController controller;

    @BeforeEach
    void setUp() {
        controller = new BrakeController();
    }


    @ParameterizedTest
    @ValueSource(doubles = {
            Double.POSITIVE_INFINITY,
            Double.NEGATIVE_INFINITY,
            Double.NaN,
            -10.0
    })
    void Should_RejectInvalidPressureAndPreserveState_When_RequestedPressureIsInvalid(
            double invalidPressure) {

        controller.apply(NORMAL_PRESSURE, LOW_SPEED, GearBox.Gear.D);

        assertThrows(
                IllegalArgumentException.class,
                () -> controller.apply(invalidPressure, ABS_SPEED, GearBox.Gear.D)
        );

        assertEquals(NORMAL_PRESSURE, controller.getPressure(), DELTA);
    }


    @ParameterizedTest
    @ValueSource(doubles = {
            Double.POSITIVE_INFINITY,
            Double.NEGATIVE_INFINITY,
            Double.NaN,
            -10.0
    })
    void Should_RejectInvalidSpeedAndPreserveState_When_SpeedIsInvalid(
            double invalidSpeed) {

        controller.apply(NORMAL_PRESSURE, LOW_SPEED, GearBox.Gear.D);

        assertThrows(
                IllegalArgumentException.class,
                () -> controller.apply(NORMAL_PRESSURE, invalidSpeed, GearBox.Gear.D)
        );

        assertEquals(NORMAL_PRESSURE, controller.getPressure(), DELTA);
    }


    @ParameterizedTest
    @ValueSource(doubles = {0.0, 50.0})
    void Should_ReturnRequestedPressure_When_AbsIsNotActivated(
            double requestedPressure) {

        double actual = controller.apply(
                requestedPressure, ABS_SPEED, GearBox.Gear.D
        );

        assertEquals(requestedPressure, actual, DELTA);
    }


    @Test
    void Should_ReleaseBrake_When_ZeroPressureIsRequestedAfterBraking() {

        controller.apply(NORMAL_PRESSURE, LOW_SPEED, GearBox.Gear.D);

        double actual = controller.apply(
                0.0, LOW_SPEED, GearBox.Gear.D
        );

        assertEquals(0.0, actual, DELTA);
    }


    @Test
    void Should_RejectNullGearAndPreserveState_When_ApplyingBrake() {

        controller.apply(NORMAL_PRESSURE, LOW_SPEED, GearBox.Gear.D);

        NullPointerException exception = assertThrows(
                NullPointerException.class,
                () -> controller.apply(NORMAL_PRESSURE, LOW_SPEED, null)
        );

        assertEquals("gear must not be null", exception.getMessage());
        assertEquals(NORMAL_PRESSURE, controller.getPressure(), DELTA);
    }


    @Test
    void Should_RejectParkGearAndPreserveState_When_ApplyingBrake() {

        controller.apply(NORMAL_PRESSURE, LOW_SPEED, GearBox.Gear.D);

        assertThrows(
                IllegalStateException.class,
                () -> controller.apply(NORMAL_PRESSURE, LOW_SPEED, GearBox.Gear.P)
        );

        assertEquals(NORMAL_PRESSURE, controller.getPressure(), DELTA);
    }


    @Test
    void Should_RejectEmergencyBrake_When_GearIsPark() {

        controller.apply(NORMAL_PRESSURE, LOW_SPEED, GearBox.Gear.D);

        assertThrows(
                IllegalStateException.class,
                () -> controller.emergencyBrake(GearBox.Gear.P)
        );

        assertEquals(NORMAL_PRESSURE, controller.getPressure(), DELTA);
        assertFalse(controller.isWarningActive());
    }


    @Test
    void Should_RejectEmergencyBrake_When_GearIsNull() {

        controller.apply(NORMAL_PRESSURE, LOW_SPEED, GearBox.Gear.D);

        NullPointerException exception = assertThrows(
                NullPointerException.class,
                () -> controller.emergencyBrake(null)
        );

        assertEquals("gear must not be null", exception.getMessage());
        assertEquals(NORMAL_PRESSURE, controller.getPressure(), DELTA);
        assertFalse(controller.isWarningActive());
    }


    @Test
    void Should_AlternateAbsPulses_When_BothThresholdsAreExceeded() {

        double firstPulse = controller.apply(
                HIGH_PRESSURE, ABS_SPEED, GearBox.Gear.D
        );

        double secondPulse = controller.apply(
                HIGH_PRESSURE, ABS_SPEED, GearBox.Gear.D
        );

        double thirdPulse = controller.apply(
                HIGH_PRESSURE, ABS_SPEED, GearBox.Gear.D
        );

        assertEquals(ABS_HIGH_PULSE, firstPulse, DELTA);
        assertEquals(ABS_LOW_PULSE, secondPulse, DELTA);
        assertEquals(ABS_HIGH_PULSE, thirdPulse, DELTA);
        assertFalse(controller.isWarningActive());
    }


    @Test
    void Should_NotActivateAbs_When_PressureEqualsThreshold() {

        double actual = controller.apply(
                ABS_PRESSURE_THRESHOLD, ABS_SPEED, GearBox.Gear.D
        );

        assertEquals(ABS_PRESSURE_THRESHOLD, actual, DELTA);
    }


    @Test
    void Should_NotActivateAbs_When_SpeedEqualsThreshold() {

        double actual = controller.apply(
                HIGH_PRESSURE, ABS_SPEED_THRESHOLD, GearBox.Gear.D
        );

        assertEquals(HIGH_PRESSURE, actual, DELTA);
    }


    @Test
    void Should_NotActivateAbs_When_SpeedIsBelowThreshold() {

        double actual = controller.apply(
                HIGH_PRESSURE, 25.0, GearBox.Gear.D
        );

        assertEquals(HIGH_PRESSURE, actual, DELTA);
    }


    @Test
    void Should_ActivateAbs_When_PressureIsJustAboveThreshold() {

        double actual = controller.apply(
                ABS_PRESSURE_THRESHOLD + 0.0001,
                ABS_SPEED,
                GearBox.Gear.D
        );

        assertEquals(ABS_HIGH_PULSE, actual, DELTA);
    }


    @Test
    void Should_ActivateAbs_When_SpeedIsJustAboveThreshold() {

        double actual = controller.apply(
                HIGH_PRESSURE,
                ABS_SPEED_THRESHOLD + 0.0001,
                GearBox.Gear.D
        );

        assertEquals(ABS_HIGH_PULSE, actual, DELTA);
    }


    @Test
    void Should_CapPressureAtMaximum_When_RequestExceedsLimit() {

        double actual = controller.apply(
                120.0, LOW_SPEED, GearBox.Gear.D
        );

        assertEquals(MAX_PRESSURE, actual, DELTA);
    }


    @Test
    void Should_ResetAbsPulseSequence_When_PressureDropsBelowThreshold() {

        double firstPulse = controller.apply(
                HIGH_PRESSURE, ABS_SPEED, GearBox.Gear.D
        );

        assertEquals(ABS_HIGH_PULSE, firstPulse, DELTA);

        double normalPressure = controller.apply(
                70.0, ABS_SPEED, GearBox.Gear.D
        );

        assertEquals(70.0, normalPressure, DELTA);

        double pulseAfterReset = controller.apply(
                HIGH_PRESSURE, ABS_SPEED, GearBox.Gear.D
        );

        assertEquals(ABS_HIGH_PULSE, pulseAfterReset, DELTA);
    }


    @Test
    void Should_ResetAbsPulseSequence_When_SpeedDropsToThreshold() {

        double firstPulse = controller.apply(
                HIGH_PRESSURE, ABS_SPEED, GearBox.Gear.D
        );

        assertEquals(ABS_HIGH_PULSE, firstPulse, DELTA);

        double normalPressure = controller.apply(
                HIGH_PRESSURE, ABS_SPEED_THRESHOLD, GearBox.Gear.D
        );

        assertEquals(HIGH_PRESSURE, normalPressure, DELTA);

        double pulseAfterReset = controller.apply(
                HIGH_PRESSURE, ABS_SPEED, GearBox.Gear.D
        );

        assertEquals(ABS_HIGH_PULSE, pulseAfterReset, DELTA);
    }


    @ParameterizedTest
    @EnumSource(
            value = GearBox.Gear.class,
            names = {"R", "N", "D"}
    )
    void Should_ActivateWarningAndResetAbs_When_EmergencyBrakeUsesAllowedGear(
            GearBox.Gear gear) {

        double firstPulse = controller.apply(
                HIGH_PRESSURE, ABS_SPEED, gear
        );

        assertEquals(ABS_HIGH_PULSE, firstPulse, DELTA);

        controller.emergencyBrake(gear);

        assertEquals(MAX_PRESSURE, controller.getPressure(), DELTA);
        assertTrue(controller.isWarningActive());

        double pulseAfterReset = controller.apply(
                HIGH_PRESSURE, ABS_SPEED, gear
        );

        assertEquals(ABS_HIGH_PULSE, pulseAfterReset, DELTA);
    }
}
