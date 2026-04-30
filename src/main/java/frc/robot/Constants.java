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

        /** Speed multiplier applied during auto-aim modes (hub aim, corner dump). Tune to balance control vs. brownout. */
        public static final double SLOW_DRIVE_MULTIPLIER = 0.75;
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
        public static final double SHOOTER_TEST_RPS = 50.0;
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
        public static final double AUTO_PIVOT_SPEED = 0.30;   // Speed for calibration moves only
        public static final double STALL_CURRENT_AMPS = 18.0; // SparkMax output current threshold for stall
        public static final double STALL_DURATION_S   = 0.15; // Must exceed threshold for this long

        // Normal operation speeds
        public static final double RAISE_SPEED       = 0.35;  // Speed when raising arm manually (d-pad)
        public static final double DUMP_RAISE_SPEED  = 0.20;  // Speed for raising during shooting intake assist — tune this if arm raises too fast/slow with a full hopper
        public static final double LOWER_SPEED       = 0.30;  // Speed when lowering arm
        public static final double FAST_LOWER_SPEED  = 0.70;  // Speed for rapid return-to-bottom after shooting
        public static final double SLOW_ZONE_SPEED   = 0.12;  // Speed in the slow zone near each limit
        public static final double SLOW_ZONE_FRACTION = 0.30; // Fraction of total travel that is the slow zone
        public static final double INTAKE_ASSIST_PIVOT_FRACTION = 0.75; // How far up (0.0–1.0) to raise during shooting; 0.75 = 3/4 of the way up

        // Legacy — kept for backward compatibility; not used in new code
        public static final double HOMING_SPEED = 0.15;
        public static final double PIVOT_SPEED  = 0.35;
    }

    public static final class CornerDumpConstants {
        private CornerDumpConstants() {}

        // ---- Corner target positions (meters, field coordinates) ----
        // These are inset 1.5m from the true field corners to keep balls in bounds.
        // Adjust CORNER_INSET_X and CORNER_INSET_Y to move the target point.
        // The correct X corner (near Red wall or near Blue wall) is chosen automatically
        // based on alliance. Y is chosen based on which half of the field the robot is on.

        /** How far inward from the alliance wall (X axis) the target sits. Tune this. */
        public static final double CORNER_INSET_X = 1.5;

        /** How far inward from the audience/scoring-table wall (Y axis) the target sits. Tune this. */
        public static final double CORNER_INSET_Y = 1.5;

        // Derived corner target X positions (calculated from field length and inset)
        public static final double RED_CORNER_X   = VisionConstants.FIELD_LENGTH_METERS - CORNER_INSET_X;
        public static final double BLUE_CORNER_X  = CORNER_INSET_X;

        // Derived corner target Y positions (audience side = high Y, scoring side = low Y)
        public static final double AUDIENCE_CORNER_Y = VisionConstants.FIELD_WIDTH_METERS - CORNER_INSET_Y;
        public static final double SCORING_CORNER_Y  = CORNER_INSET_Y;

        // Y midpoint of the field — used to decide which corner (audience vs scoring) to target
        public static final double FIELD_Y_MIDPOINT = VisionConstants.FIELD_WIDTH_METERS / 2.0;

        // ---- Heading tolerance for "aimed enough to shoot" ----
        /** Robot must be within this many degrees of the corner target before feeding begins. */
        public static final double HEADING_TOLERANCE_DEGREES = 5.0;

        /** Shooter must be within this many RPS of its distance-based target before feeding starts. */
        public static final double SHOOTER_SPEED_TOLERANCE = 5.0;

        /** Delay after shooting starts before raising pivot and running intake (seconds). */
        public static final double INTAKE_ASSIST_DELAY_S = 0.5;
    }

    public static final class IntakeConstants {
        private IntakeConstants() {}
        public static final int INTAKE_LEFT_MOTOR = 33;
        public static final int INTAKE_RIGHT_MOTOR = 34;

        public static final double INTAKE_SPEED = 40.0;
    }

    public static final class VisionConstants {
        private VisionConstants() {}
        public static final String LIMELIGHT_RIGHT = "limelight-right";
        public static final String LIMELIGHT_LEFT = "limelight-left";
        public static final String LIMELIGHT_PIVOT = "limelight-pivot";
        public static final String[] ALL_LIMELIGHTS = {LIMELIGHT_RIGHT, LIMELIGHT_LEFT, LIMELIGHT_PIVOT};

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
