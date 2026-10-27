package autopilot.core;

import autopilot.exception.SensorFailureException;
import autopilot.sensor.LaneSensor;
import autopilot.sensor.ObstacleSensor;
import autopilot.sensor.SpeedSensor;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class AutoDriverTest {

    private static final double TARGET_SPEED_KMH = 80.0;
    private static final double CURRENT_SPEED_KMH = 80.0;
    private static final double SAFE_OBSTACLE_DISTANCE_METERS = 100.0;
    private static final double CENTERED_LANE_OFFSET_METERS = 0.0;
    private static final double POSITIVE_LANE_OFFSET_METERS = 1.0;
    private static final double NEGATIVE_LANE_OFFSET_METERS = -1.0;
    private static final double LANE_THRESHOLD_METERS = 0.5;
    private static final double LANE_CORRECTION_DEGREES = 45.0;
    private static final double NO_BRAKE_PRESSURE = 0.0;
    private static final double NO_STEERING_ANGLE = 0.0;
    private static final double SAFE_MODE_BRAKE_PRESSURE = 30.0;
    private static final double DELTA = 1e-9;

    private static final String SPEED_SENSOR_FAILURE_MESSAGE = "Speed sensor failed";
    private static final String OBSTACLE_SENSOR_FAILURE_MESSAGE = "Obstacle sensor failed";
    private static final String LANE_SENSOR_FAILURE_MESSAGE = "Lane sensor failed";

    private SpeedSensor speedSensor;
    private ObstacleSensor obstacleSensor;
    private LaneSensor laneSensor;
    private AutoDriver autoDriver;

    @BeforeEach
    void setUp() {
        speedSensor = speedSensorReturning(CURRENT_SPEED_KMH);
        obstacleSensor = obstacleSensorReturning(SAFE_OBSTACLE_DISTANCE_METERS);
        laneSensor = laneSensorReturning(CENTERED_LANE_OFFSET_METERS);
        autoDriver = createAutoDriver();
    }

    @Test
    void Should_ThrowNullPointerException_When_SpeedSensorIsNull() {
        NullPointerException ex = assertThrows(
                NullPointerException.class,
                () -> new AutoDriver(null, obstacleSensor, laneSensor, TARGET_SPEED_KMH)
        );
        assertEquals("speedSensor must not be null", ex.getMessage());
    }

    @Test
    void Should_ThrowNullPointerException_When_ObstacleSensorIsNull() {
        NullPointerException ex = assertThrows(
                NullPointerException.class,
                () -> new AutoDriver(speedSensor, null, laneSensor, TARGET_SPEED_KMH)
        );
        assertEquals("obstacleSensor must not be null", ex.getMessage());
    }

    @Test
    void Should_ThrowNullPointerException_When_LaneSensorIsNull() {
        NullPointerException ex = assertThrows(
                NullPointerException.class,
                () -> new AutoDriver(speedSensor, obstacleSensor, null, TARGET_SPEED_KMH)
        );
        assertEquals("laneSensor must not be null", ex.getMessage());
    }

    @Test
    void Should_NotApplyBrake_When_ConditionsAreNormal() {
        AutoDriver.DriveResult result = autoDriver.tick(GearBox.Gear.D);

        assertEquals(NO_BRAKE_PRESSURE, result.brakePressure(), DELTA);
        assertEquals(AdaptiveCruise.Mode.HOLD, result.cruiseMode());
    }

    @Test
    void Should_NotEnterSafeMode_When_ConditionsAreNormal() {
        AutoDriver.DriveResult result = autoDriver.tick(GearBox.Gear.D);

        assertFalse(result.safeMode());
        assertFalse(result.warningActive());
    }

    @Test
    void Should_SteerToExpectedNegativeAngle_When_LaneOffsetExceedsPositiveThreshold() {
        autoDriver = createAutoDriverWithLaneOffset(POSITIVE_LANE_OFFSET_METERS);
        double expectedAngle = new SteeringController().update(-LANE_CORRECTION_DEGREES, CURRENT_SPEED_KMH);

        AutoDriver.DriveResult result = autoDriver.tick(GearBox.Gear.D);

        assertEquals(expectedAngle, result.steeringAngleDegrees(), DELTA);
    }

    @Test
    void Should_SteerToExpectedPositiveAngle_When_LaneOffsetExceedsNegativeThreshold() {
        autoDriver = createAutoDriverWithLaneOffset(NEGATIVE_LANE_OFFSET_METERS);
        double expectedAngle = new SteeringController().update(LANE_CORRECTION_DEGREES, CURRENT_SPEED_KMH);

        AutoDriver.DriveResult result = autoDriver.tick(GearBox.Gear.D);

        assertEquals(expectedAngle, result.steeringAngleDegrees(), DELTA);
    }

    @Test
    void Should_SelfCenterSteering_When_LaneOffsetIsAtPositiveThreshold() {
        autoDriver = createAutoDriverWithLaneOffset(LANE_THRESHOLD_METERS);

        AutoDriver.DriveResult result = autoDriver.tick(GearBox.Gear.D);

        assertEquals(NO_STEERING_ANGLE, result.steeringAngleDegrees(), DELTA);
    }

    @Test
    void Should_SelfCenterSteering_When_LaneOffsetIsAtNegativeThreshold() {
        autoDriver = createAutoDriverWithLaneOffset(-LANE_THRESHOLD_METERS);

        AutoDriver.DriveResult result = autoDriver.tick(GearBox.Gear.D);

        assertEquals(NO_STEERING_ANGLE, result.steeringAngleDegrees(), DELTA);
    }

    @Test
    void Should_ApplyProportionalBrake_When_ObstacleIsBelowSafeDistance() {
        // در سرعت 80، فاصله امن 40 متر است؛ در فاصله 30 متر باید 25% ترمز بگیرد
        obstacleSensor = obstacleSensorReturning(30.0);
        autoDriver = createAutoDriver();

        AutoDriver.DriveResult result = autoDriver.tick(GearBox.Gear.D);

        assertEquals(AdaptiveCruise.Mode.BRAKE, result.cruiseMode());
        assertEquals(25.0, result.brakePressure(), DELTA);
        assertFalse(result.warningActive());
    }

    @Test
    void Should_ApplyEmergencyBrakeAndActivateWarning_When_ObstacleIsAtEmergencyDistance() {
        // در سرعت 80، فاصله اضطراری 20 متر است
        obstacleSensor = obstacleSensorReturning(20.0);
        autoDriver = createAutoDriver();

        AutoDriver.DriveResult result = autoDriver.tick(GearBox.Gear.D);

        assertEquals(AdaptiveCruise.Mode.EMERGENCY_BRAKE, result.cruiseMode());
        assertEquals(100.0, result.brakePressure(), DELTA);
        assertTrue(result.warningActive());
    }

    @Test
    void Should_EnterSafeMode_When_SpeedSensorFails() {
        speedSensor = failingSpeedSensor();
        autoDriver = createAutoDriver();

        AutoDriver.DriveResult result = autoDriver.tick(GearBox.Gear.D);

        assertSafeModeResult(result);
    }

    @Test
    void Should_EnterSafeMode_When_ObstacleSensorFails() {
        obstacleSensor = failingObstacleSensor();
        autoDriver = createAutoDriver();

        AutoDriver.DriveResult result = autoDriver.tick(GearBox.Gear.D);

        assertSafeModeResult(result);
    }

    @Test
    void Should_EnterSafeMode_When_LaneSensorFails() {
        laneSensor = failingLaneSensor();
        autoDriver = createAutoDriver();

        AutoDriver.DriveResult result = autoDriver.tick(GearBox.Gear.D);

        assertSafeModeResult(result);
    }

    private void assertSafeModeResult(AutoDriver.DriveResult result) {
        assertAll(
                () -> assertTrue(result.safeMode()),
                () -> assertEquals(SAFE_MODE_BRAKE_PRESSURE, result.brakePressure(), DELTA),
                () -> assertEquals(NO_STEERING_ANGLE, result.steeringAngleDegrees(), DELTA),
                () -> assertEquals(AdaptiveCruise.Mode.BRAKE, result.cruiseMode()),
                () -> assertFalse(result.warningActive())
        );
    }

    private AutoDriver createAutoDriver() {
        return new AutoDriver(speedSensor, obstacleSensor, laneSensor, TARGET_SPEED_KMH);
    }

    private AutoDriver createAutoDriverWithLaneOffset(double laneOffsetMeters) {
        laneSensor = laneSensorReturning(laneOffsetMeters);
        return createAutoDriver();
    }

    private SpeedSensor speedSensorReturning(double speedKmh) {
        return () -> speedKmh;
    }

    private ObstacleSensor obstacleSensorReturning(double distanceMeters) {
        return () -> distanceMeters;
    }

    private LaneSensor laneSensorReturning(double laneOffsetMeters) {
        return () -> laneOffsetMeters;
    }

    private SpeedSensor failingSpeedSensor() {
        return () -> {
            throw new SensorFailureException(SPEED_SENSOR_FAILURE_MESSAGE);
        };
    }

    private ObstacleSensor failingObstacleSensor() {
        return () -> {
            throw new SensorFailureException(OBSTACLE_SENSOR_FAILURE_MESSAGE);
        };
    }

    private LaneSensor failingLaneSensor() {
        return () -> {
            throw new SensorFailureException(LANE_SENSOR_FAILURE_MESSAGE);
        };
    }
}