
package autopilot.core;

import autopilot.exception.SensorFailureException;
import autopilot.sensor.LaneSensor;
import autopilot.sensor.ObstacleSensor;
import autopilot.sensor.SpeedSensor;

import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.params.provider.CsvSource;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class AutoDriverTest {

    private static final double TARGET_SPEED_KMH = 80.0;
    private static final double CURRENT_SPEED_KMH = 80.0;
    private static final double SAFE_OBSTACLE_DISTANCE_METERS = 100.0;

    private static final double CENTERED_LANE_OFFSET_METERS = 0.0;
    private static final double POSITIVE_LANE_OFFSET_METERS = 1.0;
    private static final double NEGATIVE_LANE_OFFSET_METERS = -1.0;

    private static final double LANE_CORRECTION_DEGREES = 45.0;
    private static final double NO_STEERING_ANGLE = 0.0;
    private static final double NO_BRAKE_PRESSURE = 0.0;
    private static final double SAFE_MODE_BRAKE_PRESSURE = 30.0;
    private static final double DELTA = 1e-9;

    private static final String SPEED_SENSOR_FAILURE_MESSAGE =
            "Speed sensor failed";

    private static final String OBSTACLE_SENSOR_FAILURE_MESSAGE =
            "Obstacle sensor failed";

    private SpeedSensor speedSensor;
    private ObstacleSensor obstacleSensor;
    private LaneSensor laneSensor;
    private AutoDriver autoDriver;

    @BeforeEach
    void setUp() {
        speedSensor = () -> CURRENT_SPEED_KMH;
        obstacleSensor = () -> SAFE_OBSTACLE_DISTANCE_METERS;
        laneSensor = () -> CENTERED_LANE_OFFSET_METERS;

        autoDriver = createAutoDriver();
    }


    @Test
    void Should_HoldSpeedWithoutBraking_When_ConditionsAreNormal() {

        AutoDriver.DriveResult result =
                autoDriver.tick(GearBox.Gear.D);

        assertAll(
                () -> assertFalse(result.safeMode()),
                () -> assertEquals(
                        AdaptiveCruise.Mode.HOLD,
                        result.cruiseMode()
                ),
                () -> assertEquals(
                        NO_BRAKE_PRESSURE,
                        result.brakePressure(),
                        DELTA
                )
        );
    }


    @ParameterizedTest
    @CsvSource({
        "1.0, -45.0",
        "-1.0, 45.0"
    })
    void Should_ReturnExpectedSteeringAngle_When_LaneOffsetExceedsThreshold(
            double laneOffset, double expectedSteeringAngle) {
        laneSensor = () -> laneOffset;
        autoDriver = createAutoDriver();

        AutoDriver.DriveResult result = autoDriver.tick(GearBox.Gear.D);

        assertEquals(expectedSteeringAngle, result.steeringAngleDegrees(), DELTA);
    }


    @Test
    void Should_EnterSafeMode_When_SpeedSensorFails() {

        speedSensor = () -> {
            throw new SensorFailureException(
                    SPEED_SENSOR_FAILURE_MESSAGE
            );
        };

        autoDriver = createAutoDriver();

        AutoDriver.DriveResult result =
                autoDriver.tick(GearBox.Gear.D);

        assertSafeModeResult(result);
    }


    @Test
    void Should_EnterSafeMode_When_ObstacleSensorFails() {

        obstacleSensor = () -> {
            throw new SensorFailureException(
                    OBSTACLE_SENSOR_FAILURE_MESSAGE
            );
        };

        autoDriver = createAutoDriver();

        AutoDriver.DriveResult result =
                autoDriver.tick(GearBox.Gear.D);

        assertSafeModeResult(result);
    }


    private AutoDriver createAutoDriver() {
        return new AutoDriver(
                speedSensor,
                obstacleSensor,
                laneSensor,
                TARGET_SPEED_KMH
        );
    }

    private void assertSafeModeResult(AutoDriver.DriveResult result) {
        assertAll(
                () -> assertTrue(result.safeMode()),
                () -> assertEquals(
                        SAFE_MODE_BRAKE_PRESSURE,
                        result.brakePressure(),
                        DELTA
                ),
                () -> assertEquals(
                        NO_STEERING_ANGLE,
                        result.steeringAngleDegrees(),
                        DELTA
                ),
                () -> assertEquals(
                        AdaptiveCruise.Mode.BRAKE,
                        result.cruiseMode()
                ),
                () -> assertFalse(result.warningActive())
        );
    }
}
