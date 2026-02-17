package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

/**
 * Vision subsystem that handles Limelight camera processing and pose estimation.
 * Provides proportional control outputs for aiming and ranging to targets.
 * Integrates with the drivetrain for vision-based pose correction using MegaTag.
 */
public class Vision extends SubsystemBase {

    // ==================== DEPENDENCIES ====================

    /** Reference to the swerve drivetrain for pose updates */
    private final SwerveSubsystem drivetrain;

    // ==================== TELEMETRY ====================

    /** 2D field visualization for SmartDashboard */
    private final Field2d m_field = new Field2d();

    /** NetworkTables publisher for AdvantageScope 3D visualization */
    private final StructPublisher<Pose2d> posePublisher;

    /**
     * Constructs the Vision subsystem.
     * Sets up field visualization and NetworkTables publishing for AdvantageScope.
     *
     * @param drivetrain The swerve drivetrain to send vision measurements to
     */
    public Vision(SwerveSubsystem drivetrain) {
        this.drivetrain = drivetrain;
        SmartDashboard.putData("Field", m_field);

        // Setup pose publisher for AdvantageScope 3D field visualization
        posePublisher = NetworkTableInstance.getDefault()
            .getStructTopic("RobotPose", Pose2d.struct).publish();
    }

    // ==================== PROPORTIONAL CONTROL ====================

    /**
     * Calculates angular velocity for aiming at a target using proportional control.
     * Returns a rotation rate (rad/s) proportional to the horizontal offset (TX).
     *
     * How it works:
     * - TX is the horizontal angle to target in degrees (+ = right, - = left)
     * - We convert TX to radians and multiply by kP to get rotation rate
     * - Higher kP = more aggressive turning, may cause oscillation
     * - Lower kP = slower response, may never reach target
     *
     * @return Angular velocity in radians/second to aim at target, or 0 if no target
     */
    public double limelight_aim_proportional() {
        // Proportional gain - tune this value for your robot
        // Higher = more aggressive, may oscillate; Lower = slower response
        double kP = 1.5;

        // Only aim if we have a valid target
        if (!LimelightHelpers.getTV("limelight")) {
            return 0.0;
        }

        double tx = LimelightHelpers.getTX("limelight");

        // Deadband to ignore small errors and prevent jitter
        if (Math.abs(tx) < 5.0) {
            return 0.0;
        }

        // Convert TX from degrees to radians for proper units
        double txRadians = Math.toRadians(tx);

        // Calculate target angular velocity (rad/s)
        double targetingAngularVelocity = txRadians * kP;

        return targetingAngularVelocity;
    }

    /**
     * Calculates forward velocity for ranging to a target using proportional control.
     * Returns a speed (m/s) proportional to the vertical offset (TY).
     *
     * How it works:
     * - TY is the vertical angle to target in degrees
     * - Negative TY = target below center = too far away = drive forward
     * - We use TY as an error signal with an offset to set desired stopping point
     *
     * Note: This works best when Limelight and target are at different heights.
     * If at similar heights, use "ta" (area) for ranging instead.
     *
     * @return Forward velocity in m/s to approach target, or 0 if no target
     */
    public double limelight_range_proportional() {
        // Proportional gain - tune for desired approach speed
        double kP = 0.2;

        // Only drive if we have a valid target
        if (!LimelightHelpers.getTV("limelight")) {
            return 0.0;
        }

        // TY offset determines where robot stops relative to target
        // Robot stops when TY = -6 degrees (adjust this for desired stopping distance)
        double ty = LimelightHelpers.getTY("limelight");
        double error = ty + 6.0;

        double targetingForwardSpeed = error * kP;

        // Invert to match robot coordinate system
        return targetingForwardSpeed * -1.0;
    }

    @Override
    public void periodic() {
        // ==================== TELEMETRY ====================

        // Update field visualization with current pose
        Pose2d currentPose = drivetrain.getState().Pose;
        m_field.setRobotPose(currentPose);
        posePublisher.set(currentPose);

        // Get Limelight data for debugging
        boolean tv = LimelightHelpers.getTV("limelight");
        double tx = LimelightHelpers.getTX("limelight");
        double ty = LimelightHelpers.getTY("limelight");

        System.out.println("TV: " + tv + " TX: " + tx + " TY: " + ty);

        // Visual feedback: blink LEDs when target is visible
        if (tv) {
            LimelightHelpers.setLEDMode_ForceBlink("limelight");
        } else {
            LimelightHelpers.setLEDMode_ForceOff("limelight");
        }

        // ==================== VISION POSE ESTIMATION ====================

        // Toggle between MegaTag1 and MegaTag2 algorithms
        boolean useMegaTag2 = true;
        boolean doRejectUpdate = false;

        if (!useMegaTag2) {
            // ========== MEGATAG 1 ==========
            // Single-frame pose estimation, works without gyro data
            // More prone to noise with single-tag views

            LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");

            // Quality checks for single-tag measurements
            if (mt1.tagCount == 1 && mt1.rawFiducials.length == 1) {
                // Reject if pose ambiguity is too high (>70% uncertain)
                if (mt1.rawFiducials[0].ambiguity > 0.7) {
                    doRejectUpdate = true;
                }
                // Reject if tag is too far away (>3 meters)
                if (mt1.rawFiducials[0].distToCamera > 3) {
                    doRejectUpdate = true;
                }
            }

            // Reject if no tags visible
            if (mt1.tagCount == 0) {
                doRejectUpdate = true;
            }

            // Apply vision measurement if it passes quality checks
            if (!doRejectUpdate) {
                // Standard deviations: [x, y, theta]
                // Lower values = more trust in vision
                // theta = 9999999 means we don't trust vision rotation
                drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(0.5, 0.5, 9999999));
                drivetrain.addVisionMeasurement(mt1.pose, mt1.timestampSeconds);
            }
        } else {
            // ========== MEGATAG 2 ==========
            // Uses gyro orientation for improved multi-tag pose solving
            // More accurate and stable than MegaTag1

            // Send current robot orientation to Limelight for better solving
            LimelightHelpers.SetRobotOrientation(
                "limelight",
                drivetrain.getState().Pose.getRotation().getDegrees(),
                0, 0, 0, 0, 0
            );

            LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");

            // Reject during fast rotation (>720 deg/s) - camera image will be blurry
            if (Math.abs(drivetrain.getPigeon2().getAngularVelocityZWorld().getValueAsDouble()) > 720) {
                doRejectUpdate = true;
            }

            // Reject if no tags visible
            if (mt2 == null || mt2.tagCount == 0) {
                doRejectUpdate = true;
            }

            // Apply vision measurement if it passes quality checks
            if (!doRejectUpdate) {
                // Slightly higher std devs than MegaTag1
                drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(0.7, 0.7, 9999999));
                drivetrain.addVisionMeasurement(mt2.pose, mt2.timestampSeconds);
            }
        }
    }
}
