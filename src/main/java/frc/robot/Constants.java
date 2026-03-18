// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import frc.robot.generated.TunerConstants;

/** Robot-wide configuration constants. */
public final class Constants {

    /** Override mode flag - when true, reverses direction of most mechanisms. */
    public static boolean overrideEnabled = false;

    public static class OperatorConstants {
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int OPERATOR_CONTROLLER_PORT = 1;
    }

    public static class DriveConstants {
        public static final double MAX_SPEED_METERS_PER_SECOND =
            TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
        public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND =
            RotationsPerSecond.of(0.75).in(RadiansPerSecond);
    }

    // Swerve system IDs (DO NOT USE THESE AGAIN):
    // Drive motors: 1, 4, 7, 10
    // Angle motors: 2, 5, 8, 11
    // Encoders: 3, 6, 9, 12

    public static final class ClimbConstants {
        private ClimbConstants() {}
        public static final int CLIMB_MOTOR = 31;
        public static final int UPPER_LIMIT_SWITCH = 6;
        public static final int LOWER_LIMIT_SWITCH = 0;
    }

    public static final class HoodConstants {
        private HoodConstants() {}
        public static final int HOOD_MOTOR = 25;
        public static final double HOOD_SPEED = 0.05;
        public static final double HOMING_STALL_AMPS = 10.0;
    }

    public static final class IndexConstants {
        private IndexConstants() {}
        public static final int INDEX_MOTOR = 20;
        public static final double INDEX_SPEED = 1.0;
    }

    public static final class ShooterConstants {
        private ShooterConstants() {}
        public static final int FLOOR_ONE = 21;
        public static final int FLOOR_TWO = 22;
        public static final int FLOOR_THREE = 23;

        // PID constants for velocity control
        public static final double KS = 0.1;
        public static final double KV = 0.12;
        public static final double KP = 0.11;
        public static final double KI = 0;
        public static final double KD = 0;

        public static final double SHOOTER_TARGET_RPS = 85.0;
        public static final double SHOOTER_DEFAULT_SPEED = 1.0;
    }

    public static final class FeederConstants {
        private FeederConstants() {}
        public static final int FEEDER = 28;
        public static final double FEEDER_SPEED = -1.0;
    }

    public static final class PivotConstants {
        private PivotConstants() {}
        public static final int PIVOT_MOTOR = 15;
        public static final double HOMING_SPEED = 0.2;
        public static final double HOMING_STALL_LOWER_AMPS = 8.0;
        public static final double HOMING_STALL_RAISE_AMPS = 65.0;
        public static final double PIVOT_SPEED = 0.6;
    }

    public static final class IntakeConstants {
        private IntakeConstants() {}
        public static final int INTAKE_MOTOR = 16;
        public static final double INTAKE_SPEED = -1.0;
    }

    public static final class VisionConstants {
        private VisionConstants() {}
        public static final String LIMELIGHT_RIGHT = "limelight-right";
        public static final String LIMELIGHT_LEFT = "limelight-left";
        public static final String LIMELIGHT_CLIMB = "limelight-climb";
        public static final String[] ALL_LIMELIGHTS = {LIMELIGHT_RIGHT, LIMELIGHT_LEFT, LIMELIGHT_CLIMB};

        public static final double TARGET_DISTANCE_METERS = 1.5;

        // Hub center positions (blue alliance origin coordinates)
        public static final double BLUE_GOAL_X_METERS = 4.612;
        public static final double BLUE_GOAL_Y_METERS = 4.021;
        public static final double RED_GOAL_X_METERS = 12.257;
        public static final double RED_GOAL_Y_METERS = 4.021;

        // Climb-mounted Limelight base pose (when climb is at zero)
        // TODO: Measure and set these values for your robot
        public static final double CLIMB_CAM_FORWARD = -0.3429;   // meters from robot center
        public static final double CLIMB_CAM_SIDE = 0.339725;      // meters (positive = right)
        public static final double CLIMB_CAM_UP_BASE = 0.4572;   // meters height when climb is down
        public static final double CLIMB_CAM_ROLL = 0.0;      // degrees
        public static final double CLIMB_CAM_PITCH = 0.0;     // degrees
        public static final double CLIMB_CAM_YAW = 0.0;       // degrees

        // Conversion from climb motor rotations to camera height change
        public static final double CLIMB_METERS_PER_ROTATION = 0.00257;
    }
}
