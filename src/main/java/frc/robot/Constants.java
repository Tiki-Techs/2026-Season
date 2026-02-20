// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climb;

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
        public static final int kDriverControllerPort = 0;
    }

    // ==================== DRIVE ====================

    /**
     * Constants for drivetrain configuration.
     */
    public static class DriveConstants {
        /** Maximum translational speed in meters per second */
        public static final double kMaxSpeedMetersPerSecond =
            TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);

        /** Maximum rotational speed in radians per second (0.75 rotations/sec) */
        public static final double kMaxAngularSpeedRadiansPerSecond =
            RotationsPerSecond.of(0.75).in(RadiansPerSecond);
    }

    // Swerve system IDs, (DO NOT USE THESE AGAIN)
    // Drive motors 1, 4, 7, 10
    // Angle motors 2, 5, 8, 11
    // Encoders     3, 6, 9, 12

    public static final class ClimbConstants {
        private ClimbConstants() {
        }
        public static final int climbMotor = 31;
        public static final int lowerLimitSwitch = 4; 
        public static final int upperLimitSwitch = 5;
    }

    public static final class HoodConstants {
        private HoodConstants() {
        }
        public static final int hoodMotor = 25;
        public static final int lowerLimitSwitch = 6; 
    }

    public static final class IndexConstants {
        private IndexConstants() {
        }
        public static final int indexMotor = 20;

        public static final double indexSpeed = 1;
    }

    public static final class ShooterConstants {
        private ShooterConstants() {
        }
        public static final int centerShooter = 22;
        
        // PID constants for velocity control
        public static final double kS = 0.1;  // Static friction compensation (volts)
        public static final double kV = 0.12; // Velocity feedforward (volts per RPS)
        public static final double kP = 0.11; // Proportional gain (volts per RPS of error)
        public static final double kI = 0;    // Integral gain (disabled)
        public static final double kD = 0;    // Derivative gain (disabled)

        public static final double shooterTargetRPS = 100.0; // Target shooter speed in rotations per second
        public static final double shooterDefaultSpeed = 1.0; // 

        
    }

    public static final class ShooterIntakeConstants {
        private ShooterIntakeConstants() {
        }
        public static final int shooterIntake = 21;
        public static final double shooterIntakeSpeed = -1.0;
    }

    public static final class IntakePivotConstants {
        private IntakePivotConstants() {
        }
        public static final int pivotMotor = 15;
        public static final int lowerLimitSwitch = 3; 
        public static final int upperLimitSwitch = 2;

        public static final double pivotSpeed = 0.25;
    }
    
    public static final class IntakeConstants {
        private IntakeConstants() {
        }
        public static final int intakeMotor = 16;

        public static final double intakeSpeed = -0.6;
    }
}

