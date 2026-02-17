// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;
import frc.robot.generated.TunerConstants;
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
}
