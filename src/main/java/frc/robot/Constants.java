// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;
import frc.robot.generated.TunerConstants;
// import frc.robot.subsystems.Climb;

import static edu.wpi.first.units.Units.*;

/**
 * Constants class for robot-wide configuration values.
 * All constants should be declared as public static (or public static final).
 *
 * This class should only contain constant values - no functional code.
 * Use static imports to access these values conveniently in other classes.
 */
public final class Constants {

    // ==================== GLOBAL STATE ====================

    /**
     * Override mode flag - when true, reverses direction of most mechanisms.
     * Controlled by holding the Y button on the driver controller.
     * Useful for unjamming or manual adjustment.
     */
    public static boolean overrideEnabled = false;

    // ==================== OPERATOR INTERFACE ====================

    /**
     * Constants for operator interface devices (controllers, buttons, etc.)
     */
    public static class OperatorConstants {
        /** USB port for the primary driver Xbox controller */
        public static final int DRIVER_CONTROLLER_PORT = 0;
    }

    // ==================== DRIVE ====================

    /**
     * Constants for drivetrain configuration.
     */
    public static class DriveConstants {
        /** Maximum translational speed in meters per second */
        public static final double MAX_SPEED_METERS_PER_SECOND =
            TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);

        /** Maximum rotational speed in radians per second (0.75 rotations/sec) */
        public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND =
            RotationsPerSecond.of(0.75).in(RadiansPerSecond);
    }

    // Swerve system IDs, (DO NOT USE THESE AGAIN)
    // Drive motors 1, 4, 7, 10
    // Angle motors 2, 5, 8, 11
    // Encoders     3, 6, 9, 12

    public static final class ClimbConstants {
        private ClimbConstants() {
        }
        public static final int CLIMB_MOTOR = 31;
        public static final int LOWER_LIMIT_SWITCH = 4;
        public static final int UPPER_LIMIT_SWITCH = 5;
    }

    public static final class HoodConstants {
        private HoodConstants() {
        }
        public static final int HOOD_MOTOR = 25;
        public static final int LOWER_LIMIT_SWITCH = 6;
        public static final double HOOD_SPEED = 0.05;
    }

    public static final class IndexConstants {
        private IndexConstants() {
        }
        public static final int INDEX_MOTOR = 20;

        public static final double INDEX_SPEED = 1.0;
    }

    public static final class ShooterConstants {
        private ShooterConstants() {
        }
        public static final int CENTER_SHOOTER = 24;

        // PID constants for velocity control
        public static final double KS = 0.1;  // Static friction compensation (volts)
        public static final double KV = 0.12; // Velocity feedforward (volts per RPS)
        public static final double KP = 0.11; // Proportional gain (volts per RPS of error)
        public static final double KI = 0;    // Integral gain (disabled)
        public static final double KD = 0;    // Derivative gain (disabled)

        public static final double SHOOTER_TARGET_RPS = 95.0; // Target shooter speed in rotations per second
        public static final double SHOOTER_DEFAULT_SPEED = 1.0; // Target shooter speed in rotations per second


    }

    public static final class ShooterIntakeConstants {
        private ShooterIntakeConstants() {
        }
        public static final int SHOOTER_INTAKE = 21;
        public static final double SHOOTER_INTAKE_SPEED = -1.0;
    }

    public static final class IntakePivotConstants {
        private IntakePivotConstants() {
        }
        public static final int PIVOT_MOTOR = 15;
        public static final int LOWER_LIMIT_SWITCH = 3;
        public static final int UPPER_LIMIT_SWITCH = 2;

        public static final double PIVOT_SPEED = 0.5;
    }

    public static final class IntakeConstants {
        private IntakeConstants() {
        }
        public static final int INTAKE_MOTOR = 16;

        public static final double INTAKE_SPEED = -0.5;
    }

    public static final class VisionConstants {
        private VisionConstants() {
        }
        /** Limelight camera names - must match names configured in Limelight web interface */
        public static final String LIMELIGHT_ONE = "limelight-front";
        public static final String LIMELIGHT_TWO = "limelight-back";

        /** Array of all Limelight names for iteration */
        public static final String[] ALL_LIMELIGHTS = {LIMELIGHT_ONE, LIMELIGHT_TWO};

        /** Desired stopping distance from camera to AprilTag in meters - tune this */
        public static final double TARGET_DISTANCE_METERS = 1.5;

        /** Blue alliance goal position (blue alliance origin coordinates) */
        public static final double BLUE_GOAL_X_METERS = 4.5;
        public static final double BLUE_GOAL_Y_METERS = 4.0;

        /** Red alliance goal position (blue alliance origin coordinates) */
        public static final double RED_GOAL_X_METERS = 12.0;   // Roughly mirrored from blue goal
        public static final double RED_GOAL_Y_METERS = 4.0;
    }
}
