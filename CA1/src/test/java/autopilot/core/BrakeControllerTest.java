package autopilot.core;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.EnumSource;
import org.junit.jupiter.params.provider.ValueSource;

import static org.junit.jupiter.api.Assertions.*;

class BrakeControllerTest {

    private BrakeController controller;
    private static final double DELTA = 1e-9;

    @BeforeEach
    void setUp() {
        controller = new BrakeController();
    }

    @ParameterizedTest
    @ValueSource(doubles = {Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY, Double.NaN, -10.0})
    void Should_ThrowIllegalArgumentException_When_RequestedPressureIsInvalid(double invalidPressure) {
        assertThrows(
                IllegalArgumentException.class,
                () -> controller.apply(invalidPressure, 40.0, GearBox.Gear.D)
        );
    }

    @Test
    void Should_AcceptRequestedPressure_When_PressureIsValid() {
        double pressure = controller.apply(50.0, 20.0, GearBox.Gear.D);

        assertEquals(50.0, pressure, DELTA);
    }

    @ParameterizedTest
    @ValueSource(doubles = {Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY, Double.NaN, -10.0})
    void Should_ThrowIllegalArgumentException_When_SpeedIsInvalid(double invalidSpeed) {
        assertThrows(
                IllegalArgumentException.class,
                () -> controller.apply(50.0, invalidSpeed, GearBox.Gear.D)
        );
    }

    @Test
    void Should_ThrowNullPointerException_When_GearIsNull() {
        NullPointerException exception = assertThrows(
                NullPointerException.class,
                () -> controller.apply(50.0, 20.0, null)
        );

        assertEquals("gear must not be null", exception.getMessage());
    }

    @Test
    void Should_ThrowIllegalStateException_When_GearIsPark() {
        assertThrows(
                IllegalStateException.class,
                () -> controller.apply(50.0, 20.0, GearBox.Gear.P)
        );
    }

    @Test
    void Should_SetMaximumPressureAndActivateWarning_When_EmergencyBrakeIsAllowed() {
        controller.emergencyBrake(GearBox.Gear.D);

        assertEquals(100.0, controller.getPressure(), DELTA);
        assertTrue(controller.isWarningActive());
    }

    @Test
    void Should_ThrowIllegalStateException_When_EmergencyBrakeIsUsedInPark() {
        assertThrows(
                IllegalStateException.class,
                () -> controller.emergencyBrake(GearBox.Gear.P)
        );
    }

    @Test
    void Should_ThrowNullPointerException_When_EmergencyBrakeGearIsNull() {
        NullPointerException exception = assertThrows(
                NullPointerException.class,
                () -> controller.emergencyBrake(null)
        );

        assertEquals("gear must not be null", exception.getMessage());
    }

    @Test
    void Should_ReturnHighAbsPulse_When_PressureAndSpeedExceedThresholdsInitially() {
        double pressure = controller.apply(95.0, 40.0, GearBox.Gear.D);

        assertEquals(90.0, pressure, DELTA);
    }

    @Test
    void Should_ReturnLowAbsPulse_When_PreviousAbsPulseWasHigh() {
        controller.apply(95.0, 40.0, GearBox.Gear.D);
        double pressure = controller.apply(95.0, 40.0, GearBox.Gear.D);

        assertEquals(60.0, pressure, DELTA);
    }

    @Test
    void Should_AlternateAbsPulses_When_PressureAndSpeedExceedThresholds() {
        double firstPressure = controller.apply(95.0, 40.0, GearBox.Gear.D);
        double secondPressure = controller.apply(95.0, 40.0, GearBox.Gear.D);
        double thirdPressure = controller.apply(95.0, 40.0, GearBox.Gear.D);

        assertEquals(90.0, firstPressure, DELTA);
        assertEquals(60.0, secondPressure, DELTA);
        assertEquals(90.0, thirdPressure, DELTA);
    }

    @Test
    void Should_ReturnBoundedPressure_When_PressureEqualsAbsThreshold() {
        double pressure = controller.apply(80.0, 40.0, GearBox.Gear.D);

        assertEquals(80.0, pressure, DELTA);
    }

    @Test
    void Should_ReturnBoundedPressure_When_SpeedEqualsAbsThreshold() {
        double pressure = controller.apply(95.0, 30.0, GearBox.Gear.D);

        assertEquals(95.0, pressure, DELTA);
    }

    @Test
    void Should_ReturnBoundedPressure_When_PressureIsBelowAbsThreshold() {
        double pressure = controller.apply(70.0, 40.0, GearBox.Gear.D);

        assertEquals(70.0, pressure, DELTA);
    }

    @Test
    void Should_ReturnBoundedPressure_When_SpeedIsBelowAbsThreshold() {
        double pressure = controller.apply(95.0, 25.0, GearBox.Gear.D);

        assertEquals(95.0, pressure, DELTA);
    }

    @Test
    void Should_CapPressureAtMaximum_When_RequestedPressureExceedsMaximumAtLowSpeed() {
        double pressure = controller.apply(120.0, 20.0, GearBox.Gear.D);

        assertEquals(100.0, pressure, DELTA);
    }

    @Test
    void Should_ResetAbsPulseSequence_When_AbsConditionsAreNotMet() {
        double firstAbsPulse = controller.apply(95.0, 40.0, GearBox.Gear.D);
        assertEquals(90.0, firstAbsPulse, DELTA);

        controller.apply(70.0, 40.0, GearBox.Gear.D);

        double pulseAfterReset = controller.apply(95.0, 40.0, GearBox.Gear.D);
        assertEquals(90.0, pulseAfterReset, DELTA);
    }

    @ParameterizedTest
    @EnumSource(
        value = GearBox.Gear.class,
        names = {"R", "N", "D"}
    )
    void Should_ResetAbsAndActivateWarning_When_EmergencyBrakeUsesNonParkGear(
            GearBox.Gear gear) {

        controller.apply(95.0, 40.0, gear);

        controller.emergencyBrake(gear);

        assertEquals(100.0, controller.getPressure(), DELTA);
        assertTrue(controller.isWarningActive());

        double nextPulse = controller.apply(95.0, 40.0, gear);

        assertEquals(90.0, nextPulse, DELTA);
    }
}