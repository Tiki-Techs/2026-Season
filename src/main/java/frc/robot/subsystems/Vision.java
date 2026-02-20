package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
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

    // ==================== CACHED LIMELIGHT STATE ====================
    // Updated once per loop in periodic() to avoid redundant NetworkTables reads.
    // limelight_aim_proportional() and limelight_range_proportional() use these
    // cached values instead of reading NT themselves.

    /** Whether a valid target is currently visible */
    private boolean cachedTV = false;

    /** Horizontal offset to target in degrees (+ = right) */
    private double cachedTX = 0.0;

    /** Average distance from camera to visible AprilTags in meters (from MegaTag2) */
    private double cachedTagDist = 0.0;

    /** Last TV state - used to only write LED mode to NT on change */
    private boolean lastTV = false;

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
     * Uses cached TX from periodic() - no extra NetworkTables reads.
     *
     * A minimum output is applied outside the deadband to ensure the robot
     * actually rotates even when the error is small (overcomes static friction).
     *
     * @return Angular velocity in radians/second to aim at target, or 0 if no target
     */
    public double limelight_aim_proportional() {
        double kP = 1.5;
        double minOutput = 0.15; // rad/s - minimum to overcome static friction, tune if needed

        if (!cachedTV) {
            return 0.0;
        }

        // Deadband to ignore small errors and prevent jitter
        if (Math.abs(cachedTX) < 5.0) {
            return 0.0;
        }

        // Convert TX from degrees to radians for proper units
        double txRadians = Math.toRadians(cachedTX);
        double output = txRadians * kP;

        // Clamp to minimum output so the robot always moves when a correction is needed
        if (output > 0) output = Math.max(output, minOutput);
        else            output = Math.min(output, -minOutput);

        return output;
    }

    /**
     * Calculates forward velocity for ranging to a target using proportional control.
     * Uses cachedTagDist (actual meters to tag from MegaTag2) instead of TY angle,
     * which is more accurate and doesn't depend on camera mount geometry.
     * Uses cached values from periodic() - no extra NetworkTables reads.
     *
     * @return Forward velocity in m/s to approach target, or 0 if no target or at distance
     */
    public double limelight_range_proportional() {
        double kP = 0.4; // tune for desired approach speed (output in m/s per meter of error)

        if (!cachedTV || cachedTagDist == 0.0) {
            return 0.0;
        }

        // Error in meters: positive = too far, negative = too close
        double error = cachedTagDist - VisionConstants.TARGET_DISTANCE_METERS;

        // Deadband - stop within 10 cm of target distance
        if (Math.abs(error) < 0.1) {
            return 0.0;
        }

        return error * kP;
    }

    @Override
    public void periodic() {
        // ==================== TELEMETRY ====================

        // Update field visualization with current pose
        Pose2d currentPose = drivetrain.getState().Pose;
        m_field.setRobotPose(currentPose);
        posePublisher.set(currentPose);

        // Read Limelight NT values ONCE per loop and cache for control methods
        cachedTV = LimelightHelpers.getTV(VisionConstants.LIMELIGHT_NAME);
        cachedTX = LimelightHelpers.getTX(VisionConstants.LIMELIGHT_NAME);
        double ty = LimelightHelpers.getTY(VisionConstants.LIMELIGHT_NAME);

        SmartDashboard.putBoolean("Vision/TV", cachedTV);
        SmartDashboard.putNumber("Vision/TX", cachedTX);
        SmartDashboard.putNumber("Vision/TY", ty);

        // Visual feedback: blink LEDs when target is visible (only write on change)
        if (cachedTV != lastTV) {
            if (cachedTV) {
                LimelightHelpers.setLEDMode_ForceBlink(VisionConstants.LIMELIGHT_NAME);
            } else {
                LimelightHelpers.setLEDMode_ForceOff(VisionConstants.LIMELIGHT_NAME);
            }
            lastTV = cachedTV;
        }

        // ==================== VISION POSE ESTIMATION (MEGATAG 2) ====================
        // Uses gyro orientation for improved multi-tag pose solving.
        // More accurate and stable than MegaTag1.

        boolean doRejectUpdate = false;

        // Send current robot orientation to Limelight for better solving
        LimelightHelpers.SetRobotOrientation(
            VisionConstants.LIMELIGHT_NAME,
            drivetrain.getState().Pose.getRotation().getDegrees(),
            0, 0, 0, 0, 0
        );

        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(VisionConstants.LIMELIGHT_NAME);

        // Cache tag distance for range control whenever tags are visible,
        // even if we end up rejecting the pose estimate below
        if (mt2 != null && mt2.tagCount > 0) {
            cachedTagDist = mt2.avgTagDist;
            SmartDashboard.putNumber("Vision/TagDist", cachedTagDist);
        } else {
            cachedTagDist = 0.0;
        }

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
            // Scale trust based on tag distance and count: closer tags with more visibility = lower std devs = more trust
            double stdDev = 0.5 * mt2.avgTagDist / mt2.tagCount;
            drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(stdDev, stdDev, 9999999));
            drivetrain.addVisionMeasurement(mt2.pose, mt2.timestampSeconds);
        }
    }
}
