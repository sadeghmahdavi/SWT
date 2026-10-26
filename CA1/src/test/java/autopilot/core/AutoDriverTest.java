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
    private static final double NO_BRAKE_PRESSURE = 0.0;
    private static final double NO_STEERING_ANGLE = 0.0;
    private static final double SAFE_MODE_BRAKE_PRESSURE = 30.0;       
    private static final String SPEED_SENSOR_FAILURE_MESSAGE = "Speed sensor failed";
    private static final String OBSTACLE_SENSOR_FAILURE_MESSAGE = "Obstacle sensor failed";

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
    void tick_withNormalConditions_doesNotApplyBrake() {
        AutoDriver.DriveResult result = autoDriver.tick(GearBox.Gear.D);

        assertEquals(NO_BRAKE_PRESSURE, result.brakePressure());
    }

    @Test
    void tick_withNormalConditions_doesNotEnterSafeMode() {
        AutoDriver.DriveResult result = autoDriver.tick(GearBox.Gear.D);

        assertFalse(result.safeMode());
    }

    @Test
    void tick_whenLaneOffsetIsPositive_steersToNegativeAngle() {
        autoDriver = createAutoDriverWithLaneOffset(POSITIVE_LANE_OFFSET_METERS);

        AutoDriver.DriveResult result = autoDriver.tick(GearBox.Gear.D);

        assertTrue(result.steeringAngleDegrees() < NO_STEERING_ANGLE);
    }

    @Test
    void tick_whenLaneOffsetIsNegative_steersToPositiveAngle() {
        autoDriver = createAutoDriverWithLaneOffset(NEGATIVE_LANE_OFFSET_METERS);

        AutoDriver.DriveResult result = autoDriver.tick(GearBox.Gear.D);

        assertTrue(result.steeringAngleDegrees() > NO_STEERING_ANGLE);
    }

    @Test
    void tick_whenSpeedSensorFails_entersSafeMode() {
        speedSensor = failingSpeedSensor();
        autoDriver = createAutoDriver();

        AutoDriver.DriveResult result = autoDriver.tick(GearBox.Gear.D);

        assertSafeModeResult(result);
    }

    @Test
    void tick_whenObstacleSensorFails_entersSafeMode() {
        obstacleSensor = failingObstacleSensor();
        autoDriver = createAutoDriver();

        AutoDriver.DriveResult result = autoDriver.tick(GearBox.Gear.D);

        assertSafeModeResult(result);
    }

    private void assertSafeModeResult(AutoDriver.DriveResult result) {
        assertAll(
                () -> assertTrue(result.safeMode()),
                () -> assertEquals(SAFE_MODE_BRAKE_PRESSURE, result.brakePressure()),
                () -> assertEquals(NO_STEERING_ANGLE, result.steeringAngleDegrees()),
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
        return new SpeedSensor() {
            @Override
            public double getSpeedKmh() {
                return speedKmh;
            }
        };
    }

    private ObstacleSensor obstacleSensorReturning(double distanceMeters) {
        return new ObstacleSensor() {
            @Override
            public double getDistanceMeters() {
                return distanceMeters;
            }
        };
    }

    private LaneSensor laneSensorReturning(double laneOffsetMeters) {
        return new LaneSensor() {
            @Override
            public double getLaneOffsetMeters() {
                return laneOffsetMeters;
            }
        };
    }

    private SpeedSensor failingSpeedSensor() {
        return new SpeedSensor() {
            @Override
            public double getSpeedKmh() {
                throw new SensorFailureException(SPEED_SENSOR_FAILURE_MESSAGE);
            }
        };
    }

    private ObstacleSensor failingObstacleSensor() {
        return new ObstacleSensor() {
            @Override
            public double getDistanceMeters() {
                throw new SensorFailureException(OBSTACLE_SENSOR_FAILURE_MESSAGE);
            }
        };
    }
}