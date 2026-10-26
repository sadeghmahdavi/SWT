package autopilot.core;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class BrakeControllerTest {

    private BrakeController controller;

    @BeforeEach
    void setUp() {
        controller = new BrakeController();
    }

    private static final double DELTA = 1e-9;

    // --------------------------------------------------
    // apply: validates requested pressure and speed
    // --------------------------------------------------

    @Test
    void Should_ThrowIllegalArgumentException_When_RequestedPressureIsInfinite() {
        assertThrows(
                IllegalArgumentException.class,
                () -> controller.apply(
                        Double.POSITIVE_INFINITY,
                        40.0,
                        GearBox.Gear.D
                )
        );
    }

    @Test
    void Should_ThrowIllegalArgumentException_When_RequestedPressureIsNegative() {
        assertThrows(
                IllegalArgumentException.class,
                () -> controller.apply(-10.0, 40.0, GearBox.Gear.D)
        );
    }

    @Test
    void Should_AcceptRequestedPressure_When_PressureIsValid() {
        double pressure = controller.apply(50.0, 20.0, GearBox.Gear.D);

        assertEquals(50.0, pressure, DELTA);
    }

    @Test
    void Should_ThrowIllegalArgumentException_When_SpeedIsInfinite() {
        assertThrows(
                IllegalArgumentException.class,
                () -> controller.apply(
                        50.0,
                        Double.POSITIVE_INFINITY,
                        GearBox.Gear.D
                )
        );
    }

    @Test
    void Should_ThrowIllegalArgumentException_When_SpeedIsNegative() {        
        assertThrows(
                IllegalArgumentException.class,
                () -> controller.apply(50.0, -10.0, GearBox.Gear.D)
        );
    }

    // --------------------------------------------------
    // apply: braking must be allowed
    // --------------------------------------------------

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

    // --------------------------------------------------
    // emergencyBrake
    // --------------------------------------------------

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

    // --------------------------------------------------
    // apply: ABS pulse behavior
    // --------------------------------------------------

    @Test
    void Should_ReturnHighAbsPulse_When_PressureAndSpeedExceedThresholdsInitially() {
        double pressure = controller.apply(90.0, 40.0, GearBox.Gear.D);

        assertEquals(90.0, pressure, DELTA);
    }

    @Test
    void Should_ReturnLowAbsPulse_When_PreviousAbsPulseWasHigh() {
        controller.apply(90.0, 40.0, GearBox.Gear.D);
        double pressure = controller.apply(90.0, 40.0, GearBox.Gear.D);

        assertEquals(60.0, pressure, DELTA);
    }

    @Test
    void Should_AlternateAbsPulses_When_PressureAndSpeedExceedThresholds() {
        double firstPressure =
                controller.apply(90.0, 40.0, GearBox.Gear.D);
        double secondPressure =
                controller.apply(90.0, 40.0, GearBox.Gear.D);
        double thirdPressure =
                controller.apply(90.0, 40.0, GearBox.Gear.D);

        assertEquals(90.0, firstPressure, DELTA);
        assertEquals(60.0, secondPressure, DELTA);
        assertEquals(90.0, thirdPressure, DELTA);
    }

    // --------------------------------------------------
    // apply: ABS threshold boundaries
    // --------------------------------------------------

    @Test
    void Should_ReturnBoundedPressure_When_PressureEqualsAbsThreshold() {
        double pressure = controller.apply(80.0, 40.0, GearBox.Gear.D);

        assertEquals(80.0, pressure, DELTA);
    }

    @Test
    void Should_ReturnBoundedPressure_When_SpeedEqualsAbsThreshold() {
        double pressure = controller.apply(90.0, 30.0, GearBox.Gear.D);

        assertEquals(90.0, pressure, DELTA);
    }

    @Test
    void Should_ReturnBoundedPressure_When_PressureIsBelowAbsThreshold() {
        double pressure = controller.apply(70.0, 40.0, GearBox.Gear.D);

        assertEquals(70.0, pressure, DELTA);
    }

    @Test
    void Should_ReturnBoundedPressure_When_SpeedIsBelowAbsThreshold() {
        double pressure = controller.apply(90.0, 25.0, GearBox.Gear.D);

        assertEquals(90.0, pressure, DELTA);
    }

    // --------------------------------------------------
    // apply: pressure is capped at MAX_PRESSURE
    // --------------------------------------------------

    @Test
    void Should_CapPressureAtMaximum_When_RequestedPressureExceedsMaximumAtLowSpeed() {
        double pressure = controller.apply(120.0, 20.0, GearBox.Gear.D);

        assertEquals(100.0, pressure, DELTA);
    }

    // --------------------------------------------------
    // apply: ABS pulse sequence resets when ABS is not active
    // --------------------------------------------------

    @Test
    void Should_ResetAbsPulseSequence_When_AbsConditionsAreNotMet() {
        controller.apply(90.0, 40.0, GearBox.Gear.D); // High pulse
        controller.apply(90.0, 40.0, GearBox.Gear.D); // Low pulse

        controller.apply(70.0, 40.0, GearBox.Gear.D); // ABS inactive
        double pressure = controller.apply(90.0, 40.0, GearBox.Gear.D);

        assertEquals(90.0, pressure, DELTA);
    }
}
