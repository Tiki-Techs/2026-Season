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

        // Calibration speeds — tune if motor overshoots a limit
        public static final double CALIB_SPEED_UP   = 0.60; // Duty cycle toward upper limit
        public static final double CALIB_SPEED_DOWN = 0.75; // Duty cycle toward lower limit
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

        public static final double SHOOTER_TARGET_RPS = 79.0;
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
        // Note: lower limit switch was removed from robot; DIO 8 kept here in case it is re-added
        public static final int LOWER_LIMIT_SWITCH_DIO = 8;

        // Calibration: drive slowly to each physical hard stop, detect stall via current spike
        public static final double AUTO_PIVOT_SPEED = 0.15;   // Speed for calibration moves and auto-raise (dump)
        public static final double STALL_CURRENT_AMPS = 18.0; // SparkMax output current threshold for stall
        public static final double STALL_DURATION_S   = 0.15; // Must exceed threshold for this long

        // Normal operation speeds
        public static final double RAISE_SPEED       = 0.35;  // Speed when raising arm
        public static final double LOWER_SPEED       = 0.30;  // Speed when lowering arm
        public static final double SLOW_ZONE_SPEED   = 0.12;  // Speed in the slow zone near each limit
        public static final double SLOW_ZONE_FRACTION = 0.30; // Fraction of total travel that is the slow zone

        // Legacy — kept for backward compatibility; not used in new code
        public static final double HOMING_SPEED = 0.15;
        public static final double PIVOT_SPEED  = 0.35;
    }

    public static final class DumpConstants {
        private DumpConstants() {}
        // Dump: raises pivot, runs feeder+shooter forward at low speed, reverses index
        public static final double DUMP_SHOOTER_RPS  = -20.0; // Negative = forward (same sign as normal shot)
        public static final double DUMP_FEEDER_SPEED = -1.0;  // Forward = same sign as FeederConstants.FEEDER_SPEED
        public static final double DUMP_INDEX_SPEED  = -1.0;  // Negative = reverse (push balls back to intake)
    }

    public static final class IntakeConstants {
        private IntakeConstants() {}
        public static final int INTAKE_LEFT_MOTOR = 33;
        public static final int INTAKE_RIGHT_MOTOR = 34;

        public static final double INTAKE_SPEED = 5.0;
    }

    public static final class VisionConstants {
        private VisionConstants() {}
        public static final String LIMELIGHT_RIGHT = "limelight-right";
        public static final String LIMELIGHT_LEFT = "limelight-left";
        public static final String LIMELIGHT_CLIMB = "limelight-climb";
        public static final String[] ALL_LIMELIGHTS = {LIMELIGHT_RIGHT, LIMELIGHT_LEFT, LIMELIGHT_CLIMB};

        public static final double TARGET_DISTANCE_METERS = 1.5;

        // Hub AprilTag IDs for TX-based aiming
        public static final int[] BLUE_HUB_TAG_IDS = {18, 19, 20, 21, 24, 25, 26, 27};
        public static final int[] RED_HUB_TAG_IDS = {2, 3, 4, 5, 8, 9, 10, 11};

        // Hub center positions (blue alliance origin coordinates)
        public static final double BLUE_GOAL_X_METERS = 4.612;
        public static final double BLUE_GOAL_Y_METERS = 4.021;
        public static final double RED_GOAL_X_METERS = 12.257;
        public static final double RED_GOAL_Y_METERS = 4.021;


        // Field dimensions for pose validation
        public static final double FIELD_LENGTH_METERS = 16.54;
        public static final double FIELD_WIDTH_METERS = 8.02;
        public static final double FIELD_BORDER_MARGIN = 0.5;
    }
}
